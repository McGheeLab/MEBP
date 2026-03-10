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

# PyDracula HiDPI fix — must be set before QApplication
os.environ["QT_FONT_DPI"] = "96"

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


def run_headless(controller: StageController):
    """Run in headless mode (Xbox controller only, no GUI)."""
    print("=" * 50)
    print("MEBP Bioprinter - Headless Mode")
    print("=" * 50)
    print(f"XY: {'SIM' if controller.simulate_xy else 'REAL'}")
    print(f"ZP: {'SIM' if controller.simulate_zp else 'REAL'}")
    print("=" * 50)

    controller.connect_stages()
    controller.connect_xbox()

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

    window = MainWindow(controller, settings)

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
    parser.add_argument("--settings", default="settings.json",
                        help="Path to settings file (default: settings.json)")
    args = parser.parse_args()

    settings = Settings(args.settings)
    settings.load()

    verbose = args.verbose or settings.get("logging.verbose", False)
    setup_logging(verbose)

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

    controller = StageController(
        simulate_xy=simulate_xy,
        simulate_zp=simulate_zp,
        controller_json=controller_json,
    )

    saved_zero = settings.get_section("zero_position")
    if saved_zero:
        controller.zero_position.update(saved_zero)

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
        run_headless(controller)
    else:
        run_gui(controller, settings)


if __name__ == "__main__":
    main()
