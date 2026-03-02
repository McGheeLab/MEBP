"""
app_modifications.py — v7.2.3 Session 3: Required app.py changes.

This file documents the exact changes needed in app.py to complete the
v7.2.3 execution control relocation. Apply these as patches to your
existing app.py.

Changes:
    1. Wire PrintSetupPage.job_ready → PrintMonitorPage.receive_job
    2. Wire PrintSetupPage.navigate_to_page → MainWindow._switch_page
    3. Move PrintManager/PrintQueue creation to app.py (shared ownership)
    4. Wire PrintMonitorPage execution signals to PrintManager
"""

# ═══════════════════════════════════════════════════════════════════
# CHANGE 1: In _build_pages(), after creating all pages, add:
# ═══════════════════════════════════════════════════════════════════

def _build_pages_v723_additions(self):
    """
    Add these lines AFTER the existing page wiring block
    (after 'self._update_page_gating(...)').
    """

    # --- v7.2.3: Wire print job pipeline ---

    setup_page = self._page_widgets[4]   # PrintSetupPage
    monitor_page = self._page_widgets[5]  # PrintMonitorPage

    # Job pipeline: PrintSetup → app.py → PrintMonitor
    if hasattr(setup_page, 'job_ready'):
        setup_page.job_ready.connect(
            lambda job: self._send_job_to_monitor(job))

    # Navigation: "Edit Hardware Setup" → switch to page 0
    if hasattr(setup_page, 'navigate_to_page'):
        setup_page.navigate_to_page.connect(self._switch_page)

    # Execution control signals from Monitor → PrintManager
    if hasattr(monitor_page, 'start_requested'):
        monitor_page.start_requested.connect(self._on_monitor_start)
    if hasattr(monitor_page, 'pause_requested'):
        monitor_page.pause_requested.connect(self._on_monitor_pause)
    if hasattr(monitor_page, 'resume_requested'):
        monitor_page.resume_requested.connect(self._on_monitor_resume)
    if hasattr(monitor_page, 'abort_requested'):
        monitor_page.abort_requested.connect(self._on_monitor_abort)


# ═══════════════════════════════════════════════════════════════════
# CHANGE 2: Add these new methods to MainWindow
# ═══════════════════════════════════════════════════════════════════

def _send_job_to_monitor(self, job):
    """
    v7.2.3: Receive a PrintJob from PrintSetupPage and forward
    to PrintMonitorPage, switching to the monitor page.
    """
    monitor_page = self._page_widgets[5]  # PrintMonitorPage

    if hasattr(monitor_page, 'receive_job'):
        monitor_page.receive_job(job)

    # Auto-switch to Print Monitor page
    self._switch_page(5)


def _on_monitor_start(self, job):
    """v7.2.3: Monitor requested start → use PrintManager from setup page."""
    setup_page = self._page_widgets[4]
    if hasattr(setup_page, 'print_manager'):
        setup_page.print_manager.start(job)


def _on_monitor_pause(self):
    """v7.2.3: Monitor requested pause."""
    setup_page = self._page_widgets[4]
    if hasattr(setup_page, 'print_manager'):
        setup_page.print_manager.pause()


def _on_monitor_resume(self):
    """v7.2.3: Monitor requested resume."""
    setup_page = self._page_widgets[4]
    if hasattr(setup_page, 'print_manager'):
        setup_page.print_manager.resume()


def _on_monitor_abort(self):
    """v7.2.3: Monitor requested abort."""
    setup_page = self._page_widgets[4]
    if hasattr(setup_page, 'print_manager'):
        setup_page.print_manager.abort()


# ═══════════════════════════════════════════════════════════════════
# CHANGE 3: In _build_pages(), wire PrintManager callbacks to Monitor
# ═══════════════════════════════════════════════════════════════════

def _wire_print_manager_to_monitor(self):
    """
    Add this after creating pages. Wires the PrintManager's callbacks
    to update the PrintMonitorPage in real-time.
    """
    setup_page = self._page_widgets[4]
    monitor_page = self._page_widgets[5]

    if hasattr(setup_page, 'print_manager'):
        pm = setup_page.print_manager

        # Progress: step, total, message → Monitor
        original_progress = pm.on_progress
        def combined_progress(step, total, msg):
            if original_progress:
                original_progress(step, total, msg)
            if hasattr(monitor_page, 'on_print_progress'):
                monitor_page.on_print_progress(step, total, msg)
        pm.on_progress = combined_progress

        # State changes → Monitor
        original_state = pm.on_state_changed
        def combined_state(state):
            if original_state:
                original_state(state)
            if hasattr(monitor_page, 'on_print_state_changed'):
                monitor_page.on_print_state_changed(state)
        pm.on_state_changed = combined_state


# ═══════════════════════════════════════════════════════════════════
# SUMMARY: Minimal patch to _build_pages()
# ═══════════════════════════════════════════════════════════════════
#
# After the existing block that ends with:
#     self._update_page_gating(is_valid)
#
# Add:
#     # v7.2.3: Wire job pipeline and execution controls
#     setup = self._page_widgets[4]
#     monitor = self._page_widgets[5]
#
#     if hasattr(setup, 'job_ready'):
#         setup.job_ready.connect(lambda job: self._send_job_to_monitor(job))
#     if hasattr(setup, 'navigate_to_page'):
#         setup.navigate_to_page.connect(self._switch_page)
#
#     # Wire execution signals from Monitor
#     if hasattr(monitor, 'start_requested'):
#         monitor.start_requested.connect(self._on_monitor_start)
#     for sig_name in ('pause_requested', 'resume_requested', 'abort_requested'):
#         if hasattr(monitor, sig_name):
#             getattr(monitor, sig_name).connect(
#                 getattr(self, f'_on_monitor_{sig_name.split("_")[0]}'))
#
#     # Wire PrintManager callbacks → Monitor for real-time updates
#     self._wire_print_manager_to_monitor()
#
# Then add the methods defined above to the MainWindow class.
# """  # End of guide
