"""
print_monitor_additions.py — v7.2.3 Session 3: Execution Controls for Print Monitor.

This file contains the NEW methods and modifications to add to the existing
print_monitor.py. Apply these changes to the original file.

OVERVIEW:
    1. Add `start_requested` signal
    2. Add `receive_job()` method
    3. Add job queue management to context panel
    4. Add `on_print_progress()` alias for compatibility
    5. Modify `get_context_widget()` to include execution controls

INSTRUCTIONS:
    See each section below for where to insert code.
"""

# ═══════════════════════════════════════════════════════════════════
# STEP 1: Add new signal to PrintMonitorPage class definition
# ═══════════════════════════════════════════════════════════════════
#
# In the class definition, after the existing signals:
#     pause_requested = Signal()
#     resume_requested = Signal()
#     abort_requested = Signal()
#
# ADD:
#     start_requested = Signal(object)  # Emits PrintJob when user clicks Start

START_REQUESTED_SIGNAL = """
    # v7.2.3: Execution control signals
    start_requested = Signal(object)  # Emits PrintJob when user clicks Start
"""

# ═══════════════════════════════════════════════════════════════════
# STEP 2: Add new attributes in __init__
# ═══════════════════════════════════════════════════════════════════
#
# After the existing attributes (self._hardware_config = None), ADD:

INIT_ADDITIONS = """
        # v7.2.3: Job queue for execution
        self._job_queue: list = []  # List of PrintJob objects
        self._current_job = None    # Currently active/displayed job
        self._context_widget = None  # Cache for context panel
"""

# ═══════════════════════════════════════════════════════════════════
# STEP 3: Add new methods to PrintMonitorPage
# ═══════════════════════════════════════════════════════════════════
#
# Add these methods BEFORE the "Recording Browser" section.

NEW_METHODS = '''
    # ═══════════════════════════════════════════════════════════════
    #  v7.2.3: JOB QUEUE + EXECUTION CONTROLS
    # ═══════════════════════════════════════════════════════════════

    def receive_job(self, job) -> None:
        """
        v7.2.3: Receive a print job from PrintSetupPage via app.py.

        Adds the job to the queue and updates the context panel.
        If no job is currently loaded, sets it as the active job.
        """
        self._job_queue.append(job)
        logger.info(f"Job received: {job.name} "
                     f"({len(self._job_queue)} in queue)")

        # If this is the first/only job, make it active
        if self._current_job is None:
            self._current_job = job

        # Update context panel
        self._refresh_job_queue_ui()

        # Update progress labels
        if hasattr(self, '_progress_labels'):
            self._progress_labels["job_name"].setText(job.name)
            self._progress_labels["step_info"].setText(
                f"0 / {job.total_steps:,}")

        # Setup plate overview if job has well data
        if hasattr(job, 'well_setup') and job.well_setup:
            try:
                from SupportClasses.WellPlate import WellPlate
                plate = getattr(job, 'plate', None)
                if plate:
                    self.setup_plate(plate)
            except Exception:
                pass

    def _refresh_job_queue_ui(self) -> None:
        """Update the job queue list widget in context panel."""
        if not hasattr(self, '_queue_list'):
            return
        self._queue_list.clear()
        for i, job in enumerate(self._job_queue):
            prefix = "▶ " if job is self._current_job else "  "
            steps = getattr(job, 'total_steps', '?')
            self._queue_list.addItem(f"{prefix}{job.name}  [{steps} steps]")

        # Enable/disable start button
        if hasattr(self, '_ctx_btn_start'):
            has_job = self._current_job is not None
            idle = self._print_state in (PrintState.IDLE, PrintState.COMPLETED,
                                          PrintState.ABORTED, PrintState.ERROR)
            self._ctx_btn_start.setEnabled(has_job and idle)

    def _on_ctx_start_clicked(self) -> None:
        """v7.2.3: Start button clicked in context panel."""
        if self._current_job is None:
            logger.warning("No job to start")
            return
        if self._print_state not in (PrintState.IDLE, PrintState.COMPLETED,
                                      PrintState.ABORTED, PrintState.ERROR):
            logger.warning(f"Cannot start: state is {self._print_state}")
            return

        logger.info(f"Starting job: {self._current_job.name}")
        self.start_requested.emit(self._current_job)

    def _on_ctx_remove_job(self) -> None:
        """Remove selected job from queue."""
        if not hasattr(self, '_queue_list'):
            return
        row = self._queue_list.currentRow()
        if 0 <= row < len(self._job_queue):
            removed = self._job_queue.pop(row)
            if removed is self._current_job:
                self._current_job = self._job_queue[0] if self._job_queue else None
            self._refresh_job_queue_ui()
            logger.info(f"Removed job: {removed.name}")

    def _on_ctx_clear_queue(self) -> None:
        """Clear all jobs from queue."""
        self._job_queue.clear()
        self._current_job = None
        self._refresh_job_queue_ui()
        logger.info("Job queue cleared")

    def on_print_progress(self, step: int, total: int, message: str) -> None:
        """
        v7.2.3: Simplified progress callback (compatible with PrintManager).

        Wraps the existing on_progress_update() with sensible defaults.
        """
        self.on_progress_update(
            current_step=step,
            total_steps=total,
            message=message,
            job_name=self._current_job.name if self._current_job else "",
        )

    def _advance_queue(self) -> None:
        """
        v7.2.3: After a job completes, advance to next in queue.

        Called from on_print_state_changed when state becomes COMPLETED.
        """
        if self._current_job in self._job_queue:
            self._job_queue.remove(self._current_job)

        if self._job_queue:
            self._current_job = self._job_queue[0]
            self._refresh_job_queue_ui()
            logger.info(f"Queue advanced: next job = {self._current_job.name}")
        else:
            self._current_job = None
            self._refresh_job_queue_ui()
            logger.info("Job queue empty")
'''


# ═══════════════════════════════════════════════════════════════════
# STEP 4: Modify on_print_state_changed to advance queue
# ═══════════════════════════════════════════════════════════════════
#
# At the END of the existing on_print_state_changed method, BEFORE the
# final line, add:
#
#     # v7.2.3: Advance queue on completion
#     if state == PrintState.COMPLETED:
#         self._advance_queue()
#     self._refresh_job_queue_ui()


# ═══════════════════════════════════════════════════════════════════
# STEP 5: Modify get_context_widget() to add execution controls
# ═══════════════════════════════════════════════════════════════════
#
# Replace the existing get_context_widget() with this version.
# It adds execution controls ABOVE the recording browser.

GET_CONTEXT_WIDGET_REPLACEMENT = '''
    def get_context_widget(self) -> QWidget:
        """
        v7.2.3: Context panel with execution controls + recording browser.

        Layout:
            - Execution Controls (Start, queue management)
            - Recording Browser (load/replay/delete)
        """
        if self._context_widget is not None:
            return self._context_widget

        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # ═══════════════════════════════════════════════════════════
        #  v7.2.3: EXECUTION CONTROLS
        # ═══════════════════════════════════════════════════════════

        exec_title = QLabel("Execution")
        exec_title.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; color: {COLORS['text']};")
        layout.addWidget(exec_title)

        # Start button
        self._ctx_btn_start = QPushButton("▶ Start Print")
        self._ctx_btn_start.setMinimumHeight(36)
        self._ctx_btn_start.setStyleSheet(
            f"QPushButton {{ background-color: {COLORS['green']}; "
            f"color: {COLORS['crust']}; font-weight: bold; border-radius: 4px; "
            f"font-size: 13px; }}"
            f"QPushButton:hover {{ background-color: #b8e8b0; }}"
            f"QPushButton:disabled {{ background-color: {COLORS['surface1']}; "
            f"color: {COLORS['overlay0']}; }}")
        self._ctx_btn_start.setEnabled(False)
        self._ctx_btn_start.clicked.connect(self._on_ctx_start_clicked)
        layout.addWidget(self._ctx_btn_start)

        # Job queue
        queue_label = QLabel("Job Queue")
        queue_label.setStyleSheet(
            f"font-weight: bold; color: {COLORS['subtext0']}; "
            f"font-size: 10px; margin-top: 4px;")
        layout.addWidget(queue_label)

        self._queue_list = QListWidget()
        self._queue_list.setMaximumHeight(80)
        self._queue_list.setStyleSheet(
            f"QListWidget {{ background-color: {COLORS['surface0']}; "
            f"color: {COLORS['text']}; border: 1px solid {COLORS['surface1']}; "
            f"border-radius: 4px; font-size: 10pt; }}"
            f"QListWidget::item:selected {{ background-color: {COLORS['surface2']}; }}")
        layout.addWidget(self._queue_list)

        q_btn_row = QHBoxLayout()
        q_btn_row.setSpacing(3)
        btn_rm = QPushButton("Remove")
        btn_rm.setMaximumHeight(22)
        btn_rm.clicked.connect(self._on_ctx_remove_job)
        q_btn_row.addWidget(btn_rm)
        btn_clr = QPushButton("Clear")
        btn_clr.setMaximumHeight(22)
        btn_clr.clicked.connect(self._on_ctx_clear_queue)
        q_btn_row.addWidget(btn_clr)
        layout.addLayout(q_btn_row)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"color: {COLORS.get('surface1', '#45475a')};")
        layout.addWidget(sep)

        # ═══════════════════════════════════════════════════════════
        #  RECORDING BROWSER (existing, preserved from v7.1)
        # ═══════════════════════════════════════════════════════════

        title = QLabel("Print Recordings")
        title.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; color: {COLORS['text']};")
        layout.addWidget(title)

        btn_refresh = QPushButton("🔄 Refresh")
        btn_refresh.clicked.connect(self._refresh_recording_list)
        layout.addWidget(btn_refresh)

        self._recording_list = QListWidget()
        self._recording_list.setStyleSheet(
            f"QListWidget {{ background-color: {COLORS['surface0']}; "
            f"color: {COLORS['text']}; border: 1px solid {COLORS['surface1']}; "
            f"border-radius: 4px; font-size: 10pt; }}"
            f"QListWidget::item:selected {{ background-color: {COLORS['surface2']}; }}")
        self._recording_list.setMaximumHeight(200)
        self._recording_list.itemClicked.connect(self._on_recording_selected)
        layout.addWidget(self._recording_list)

        self._rec_info_label = QLabel("Select a recording to view")
        self._rec_info_label.setWordWrap(True)
        self._rec_info_label.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 10pt;")
        layout.addWidget(self._rec_info_label)

        replay_row = QHBoxLayout()
        self._btn_load_replay = QPushButton("📂 Load Replay")
        self._btn_load_replay.setToolTip(
            "Load recording and overlay on trajectory view")
        self._btn_load_replay.clicked.connect(self._load_replay)
        self._btn_load_replay.setEnabled(False)
        replay_row.addWidget(self._btn_load_replay)

        self._btn_clear_replay = QPushButton("✕ Clear")
        self._btn_clear_replay.setToolTip("Clear replay overlay")
        self._btn_clear_replay.clicked.connect(self._clear_replay)
        self._btn_clear_replay.setEnabled(False)
        replay_row.addWidget(self._btn_clear_replay)
        layout.addLayout(replay_row)

        self._replay_info = QLabel("")
        self._replay_info.setWordWrap(True)
        self._replay_info.setStyleSheet(
            f"color: {COLORS['green']}; font-size: 10pt;")
        layout.addWidget(self._replay_info)

        self._btn_delete = QPushButton("🗑 Delete Recording")
        self._btn_delete.setStyleSheet(
            f"QPushButton {{ color: {COLORS['red']}; }}")
        self._btn_delete.clicked.connect(self._delete_recording)
        self._btn_delete.setEnabled(False)
        layout.addWidget(self._btn_delete)

        layout.addStretch()

        # Initial refresh
        from PySide6.QtCore import QTimer
        QTimer.singleShot(500, self._refresh_recording_list)

        self._context_widget = widget
        return widget
'''
