#!/usr/bin/env python3
"""
Integration Test — Verify all GUI pages load, context panels create,
and interface contracts are satisfied.

Run from project root:
    python -m gui.tests.test_integration

Requirements: PySide6 (headless OK with QT_QPA_PLATFORM=offscreen)
"""
import os
import sys
import logging

# Allow headless testing
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger("test_integration")

PASS = 0
FAIL = 0


def check(name: str, condition: bool, detail: str = ""):
    global PASS, FAIL
    if condition:
        PASS += 1
        logger.info(f"  ✓ {name}")
    else:
        FAIL += 1
        logger.error(f"  ✗ {name} — {detail}")


def main():
    global PASS, FAIL
    app = QApplication.instance() or QApplication(sys.argv)

    # ── Import test ───────────────────────────────────────────────
    logger.info("=== 1. Import Tests ===")

    try:
        from gui.styles import DARK_THEME, COLORS, MENU_SELECTED_STYLESHEET
        check("gui.styles imports", True)
    except Exception as e:
        check("gui.styles imports", False, str(e))

    try:
        from gui.ui_functions import UIFunctions, AppSettings
        check("gui.ui_functions imports", True)
    except Exception as e:
        check("gui.ui_functions imports", False, str(e))

    try:
        from gui.widgets.console_log import ConsoleLogWidget, QtLogHandler
        check("ConsoleLogWidget imports", True)
    except Exception as e:
        check("ConsoleLogWidget imports", False, str(e))

    try:
        from gui.widgets.xbox_mapping_editor import XboxMappingEditor
        check("XboxMappingEditor imports", True)
    except Exception as e:
        check("XboxMappingEditor imports", False, str(e))

    try:
        from gui.pages.dashboard import DashboardPage
        from gui.pages.jog_control import JogControlPage
        from gui.pages.calibration import CalibrationPage
        from gui.pages.print_setup import PrintSetupPage
        from gui.pages.settings_page import SettingsPage
        check("All page imports", True)
    except Exception as e:
        check("All page imports", False, str(e))

    # ── Controller setup (simulation mode) ────────────────────────
    logger.info("=== 2. Controller Setup (Simulation) ===")

    try:
        from SupportClasses.StageController import StageController
        from SupportClasses.Settings import Settings
        from SupportClasses.PrintHistory import PrintHistory

        settings = Settings()  # defaults
        controller = StageController(simulate_xy=True, simulate_zp=True)
        check("StageController created (sim mode)", True)
    except Exception as e:
        check("StageController created", False, str(e))
        logger.error("Cannot continue without controller")
        return

    # ── Page instantiation ────────────────────────────────────────
    logger.info("=== 3. Page Instantiation ===")

    pages = {}
    page_classes = {
        "Dashboard": (DashboardPage, {"controller": controller, "print_history": PrintHistory()}),
        "JogControl": (JogControlPage, {"controller": controller}),
        "Calibration": (CalibrationPage, {"controller": controller, "settings": settings}),
        "PrintSetup": (PrintSetupPage, {"controller": controller}),
        "Settings": (SettingsPage, {"controller": controller, "settings": settings}),
    }

    for name, (cls, kwargs) in page_classes.items():
        try:
            page = cls(**kwargs)
            pages[name] = page
            check(f"{name} instantiation", True)
        except Exception as e:
            check(f"{name} instantiation", False, str(e))

    # ── Interface contract ────────────────────────────────────────
    logger.info("=== 4. Interface Contract ===")

    for name, page in pages.items():
        # get_page_title
        has_title = hasattr(page, 'get_page_title')
        check(f"{name}.get_page_title exists", has_title)
        if has_title:
            title = page.get_page_title()
            check(f"{name}.get_page_title returns str", isinstance(title, str) and len(title) > 0, f"got: {title!r}")

        # get_context_widget
        has_ctx = hasattr(page, 'get_context_widget')
        check(f"{name}.get_context_widget exists", has_ctx)
        if has_ctx:
            ctx = page.get_context_widget()
            check(f"{name}.get_context_widget returns QWidget", ctx is not None, "returned None")

        # on_status_update
        has_update = hasattr(page, 'on_status_update')
        check(f"{name}.on_status_update exists", has_update)
        if has_update:
            try:
                page.on_status_update()
                check(f"{name}.on_status_update callable", True)
            except Exception as e:
                check(f"{name}.on_status_update callable", False, str(e))

    # ── Context widget caching ────────────────────────────────────
    logger.info("=== 5. Context Widget Caching ===")

    for name, page in pages.items():
        if hasattr(page, 'get_context_widget'):
            ctx1 = page.get_context_widget()
            ctx2 = page.get_context_widget()
            check(f"{name} context cached (same object)", ctx1 is ctx2,
                  f"id1={id(ctx1)} id2={id(ctx2)}")

    # ── Print Setup specific ──────────────────────────────────────
    logger.info("=== 6. Print Setup Specifics ===")

    if "PrintSetup" in pages:
        ps = pages["PrintSetup"]

        check("PrintSetup has canvas", hasattr(ps, 'canvas'))
        check("PrintSetup has source_tabs", hasattr(ps, 'source_tabs'))
        check("PrintSetup has print_manager", hasattr(ps, 'print_manager'))
        check("PrintSetup has print_queue", hasattr(ps, 'print_queue'))
        check("PrintSetup has resume_print", hasattr(ps, 'resume_print'))

        # Check canvas methods
        if hasattr(ps, 'canvas'):
            try:
                ps.canvas.set_path_segments([])
                ps.canvas.set_wells([])
                ps.canvas.set_current_position(0, 0)
                ps.canvas.clear_current_position()
                ps.canvas.set_progress(0, 100)
                ps.canvas.clear()
                check("PathPreviewCanvas methods work", True)
            except Exception as e:
                check("PathPreviewCanvas methods work", False, str(e))

    # ── Settings specific ─────────────────────────────────────────
    logger.info("=== 7. Settings Page Specifics ===")

    if "Settings" in pages:
        sp = pages["Settings"]

        check("Settings has chk_safety_enabled", hasattr(sp, 'chk_safety_enabled'))
        check("Settings has spin_xy_min_x", hasattr(sp, 'spin_xy_min_x'))
        check("Settings has chk_verbose", hasattr(sp, 'chk_verbose'))

        # Verify context sync
        if hasattr(sp, 'ctx_safety_chk'):
            sp.chk_safety_enabled.setChecked(False)
            check("Settings safety sync to context",
                  not sp.ctx_safety_chk.isChecked(),
                  "context checkbox not synced")

    # ── Console log ───────────────────────────────────────────────
    logger.info("=== 8. Console Log ===")

    try:
        console = ConsoleLogWidget()
        console.log("Test message", "info")
        console.log("Error test", "error")
        console.log("Success test", "success")
        console.clear()
        check("ConsoleLogWidget basic ops", True)
    except Exception as e:
        check("ConsoleLogWidget basic ops", False, str(e))

    try:
        handler = QtLogHandler(console)
        record = logging.LogRecord("test", logging.INFO, "", 0, "test msg", (), None)
        handler.emit(record)
        check("QtLogHandler emit", True)
    except Exception as e:
        check("QtLogHandler emit", False, str(e))

    # ── MainWindow (full integration) ─────────────────────────────
    logger.info("=== 9. MainWindow Full Integration ===")

    try:
        from gui.app import MainWindow
        window = MainWindow(controller, settings)
        check("MainWindow created", True)

        # Verify pages are loaded
        check("MainWindow has 5 pages",
              len(window._page_widgets) == 5,
              f"got {len(window._page_widgets)}")

        # Test page switching
        for idx in range(5):
            try:
                window._switch_page(idx)
                check(f"Switch to page {idx}", True)
            except Exception as e:
                check(f"Switch to page {idx}", False, str(e))

        # Test status update
        try:
            window._update_status()
            check("_update_status runs", True)
        except Exception as e:
            check("_update_status runs", False, str(e))

        # Test keyboard shortcuts
        from PySide6.QtCore import Qt
        from PySide6.QtGui import QKeyEvent
        try:
            event = QKeyEvent(QKeyEvent.Type.KeyPress, Qt.Key.Key_Escape, Qt.KeyboardModifier.NoModifier)
            window.keyPressEvent(event)
            check("Keyboard event handling", True)
        except Exception as e:
            check("Keyboard event handling", False, str(e))

        # Test save_settings
        try:
            window.save_settings()
            check("save_settings runs", True)
        except Exception as e:
            check("save_settings runs", False, str(e))

        window.close()

    except Exception as e:
        check("MainWindow created", False, str(e))
        import traceback
        traceback.print_exc()

    # ── Summary ───────────────────────────────────────────────────
    total = PASS + FAIL
    logger.info("")
    logger.info(f"{'='*50}")
    logger.info(f"Results: {PASS}/{total} passed, {FAIL} failed")
    logger.info(f"{'='*50}")

    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
