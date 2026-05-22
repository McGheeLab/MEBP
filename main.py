"""
MEBP Bioprinter Application — Entry Point

Supports two modes:
  --headless : Xbox controller only (no GUI)
  (default)  : Full PyDracula-style GUI application

Usage:
  python main.py                         # GUI mode, simulation
  python main.py --headless              # Headless with simulation
  python main.py --real-xy --real-zp     # Real hardware
  python main.py --headless --real-xy    # Headless, real XY, simulated ZP
"""

import argparse
import logging
import os
import signal
import sys
import time

# ── HiDPI support ───────────────────────────────────────────────────
# Let Qt6 handle DPI scaling natively instead of locking to 96 DPI.
# The gui/scaling.py module detects actual DPI and scales hardcoded
# pixel dimensions accordingly.  Override with MEBP_UI_SCALE=1.5 etc.
os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")

# ── PyInstaller frozen-app support ──────────────────────────────────
# When running as a bundled executable, change the working directory to
# the bundle's internal directory so that all relative paths
# (config/, settings.json, print_records/, etc.) resolve correctly.
if getattr(sys, "frozen", False):
    _bundle_dir = getattr(sys, "_MEIPASS", os.path.dirname(sys.executable))
    os.chdir(_bundle_dir)

from SupportClasses.StageController import StageController
from SupportClasses.Settings import Settings


def setup_logging(verbose=False):
    """Configure logging for the application."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def run_headless(controller: StageController, settings: Settings):
    """Run in headless mode (Xbox controller only, no GUI)."""
    print("=" * 50)
    print("MEBP Bioprinter - Headless Mode")
    print("=" * 50)
    print(f"XY: {'SIM' if controller.simulate_xy else 'REAL'}")
    print(f"ZP: {'SIM' if controller.simulate_zp else 'REAL'}")
    print("=" * 50)

    controller.connect_stages()
    # v7.4.2 hotfix: persist last-known-good ZP port so next launch
    # short-circuits the rediscovery scan
    zp_port = controller.zp_connected_port
    if zp_port:
        settings.set("zp_stage.last_port", zp_port)
        settings.save()
    # v7.3.2: Load stick calibration offsets for headless mode
    _stick_offsets = settings.get_section("xbox_stick_offsets")
    if _stick_offsets:
        _stick_offsets = {int(k): v for k, v in _stick_offsets.items()}
    controller.connect_xbox(stick_offsets=_stick_offsets or None)

    print("Ready. Press Ctrl+C to exit.\n")

    def signal_handler(sig, frame):
        print("\nShutting down...")
        controller.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()


def run_gui(controller: StageController, settings: Settings):
    """Run the full GUI application."""
    try:
        from PySide6.QtWidgets import QApplication
        from gui.app import MainWindow
        from gui.widgets.console_log import QtLogHandler
    except ImportError as e:
        print(f"GUI dependencies not available: {e}")
        print("Install PySide6: pip install PySide6")
        print("Or run in headless mode: python main.py --headless")
        sys.exit(1)

    app = QApplication(sys.argv)
    app.setApplicationName("MEBP Bioprinter")
    app.setOrganizationName("Lab")
    # v7.4.2: force the Fusion widget style across platforms so our QSS
    # (notably QComboBox dropdown menus) actually paints. macOS' default
    # native style ignores stylesheet rules on the popup view, which
    # leaves combo dropdowns rendering as light system menus over our
    # dark theme.
    from PySide6.QtWidgets import QStyleFactory
    fusion = QStyleFactory.create("Fusion")
    if fusion is not None:
        app.setStyle(fusion)

    from SupportClasses.PrintRecorder import PrintRecorder
    recorder = PrintRecorder()

    window = MainWindow(controller, settings, recorder=recorder)

    # Wire Python logging → console widget
    qt_handler = QtLogHandler(window.console)
    qt_handler.setFormatter(logging.Formatter("%(name)s: %(message)s"))
    logging.getLogger().addHandler(qt_handler)

    window.show()
    window.console.log("Application started", "success")

    def on_quit():
        window.save_settings()
        controller.shutdown()

    app.aboutToQuit.connect(on_quit)

    sys.exit(app.exec())


def main():
    parser = argparse.ArgumentParser(description="MEBP Bioprinter Application")
    parser.add_argument("--headless", action="store_true",
                        help="Run without GUI (Xbox controller only)")
    parser.add_argument("--real-xy", action="store_true",
                        help="Use real XY stage hardware (default: simulate)")
    parser.add_argument("--real-zp", action="store_true",
                        help="Use real ZP stage hardware")
    # v7.2.8s2: default simulate False — add explicit simulate flags
    parser.add_argument("--simulate-xy", action="store_true",
                        help="Force XY stage simulation")
    parser.add_argument("--simulate-zp", action="store_true",
                        help="Force ZP stage simulation")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable debug logging")
    parser.add_argument("--debug-xy", action="store_true",
                        help="Record raw XY serial data + jog commands to a CSV file")
    parser.add_argument("--settings", default="settings.json",
                        help="Path to settings file (default: settings.json)")
    args = parser.parse_args()

    settings = Settings(args.settings)
    settings.load()

    verbose = args.verbose or settings.get("logging.verbose", False)
    setup_logging(verbose)

    if getattr(args, "debug_xy", False) or settings.get("logging.debug_xy", False):
        from SupportClasses.XYDebugLogger import enable as _enable_xy_debug
        _enable_xy_debug()

    # v7.2.8s2: default simulate False — real hardware is the default
    simulate_xy = settings.get("simulation.simulate_xy", False)
    simulate_zp = settings.get("simulation.simulate_zp", False)

    # CLI overrides
    if args.real_xy:
        simulate_xy = False
    if args.real_zp:
        simulate_zp = False
    if getattr(args, "simulate_xy", False):
        simulate_xy = True
    if getattr(args, "simulate_zp", False):
        simulate_zp = True

    # v7.2.8: Pass controller_json from settings for hardware auto-detect
    controller_json = settings.get("controller.controller_json", "auto")

    poll_interval_s = settings.get("polling.position_interval_ms", 300) / 1000.0
    watchdog_interval_s = settings.get("polling.watchdog_interval_s", 2.0)

    controller = StageController(
        simulate_xy=simulate_xy,
        simulate_zp=simulate_zp,
        controller_json=controller_json,
        poll_interval=poll_interval_s,
        watchdog_interval=watchdog_interval_s,
    )

    saved_zero = settings.get_section("zero_position")
    if saved_zero:
        controller.zero_position.update(saved_zero)

    # v7.4.2 hotfix: cache last-known-good ZP serial port so the
    # rediscovery scan can short-circuit on first connect_stages().
    saved_zp_port = settings.get("zp_stage.last_port")
    if saved_zp_port:
        controller.set_preferred_zp_port(saved_zp_port)

    # v7.3.2: Load axis flip settings
    saved_flips = settings.get_section("axis_flip")
    if saved_flips and isinstance(saved_flips, dict):
        controller.set_axis_flips(saved_flips)

    # v7.3.5: Load ZP stage feedrate settings
    from SupportClasses.ZPStage import ZPStageManager
    _default_fr = ZPStageManager.DEFAULT_FEEDRATE
    controller._zp_retract_feedrate = settings.get(
        "zp_stage.retract_feedrate", _default_fr)
    controller._zp_insert_feedrate = settings.get(
        "zp_stage.insert_feedrate", _default_fr / 2)
    controller._zp_auto_save_position = bool(
        settings.get("zp_stage.auto_save_position", False))

    # v7.2.6: Load safety_limits from settings
    saved_limits = settings.get_section("safety_limits")
    if saved_limits:
        from SupportClasses.SafetyLimits import SafetyLimits
        controller.safety_limits = SafetyLimits.from_dict(saved_limits)
        import logging as _log
        _log.getLogger(__name__).info(
            f"Safety limits loaded from settings "
            f"(enabled={controller.safety_limits.enabled})")

    if args.headless:
        run_headless(controller, settings)
    else:
        run_gui(controller, settings)


if __name__ == "__main__":
    # Required for PyInstaller on macOS/Windows when using multiprocessing
    import multiprocessing
    multiprocessing.freeze_support()
    main()
