"""
MEBP Bioprinter Application — Entry Point

Supports two modes:
  --headles : Xbox controller only (no GUI)
  (default)  : Full PyDracula-style GUI application

GUI mode: stages start disconnected. Click Connect (real hardware) or
Simulate (built-in simulator) per stage on Hardware Setup → Device →
Connect Hardware. There is no longer a single global "simulation" flag
in settings.json — the choice is made per-connection at runtime.

Headless mode: stages are connected once at startup. The defaults are
real hardware for both axes. Use ``--simulate-xy`` / ``--simulate-zp``
to spin up simulators instead (handy when no hardware is attached).

Usage:
  python main.py                         # GUI mode, nothing connected yet
  python main.py --headless              # Headless, real XY + real ZP
  python main.py --headless --simulate-xy --simulate-zp   # Headless, simulators
  python main.py --headless --simulate-zp                  # Real XY, sim ZP
"""

import argparse
import faulthandler
import logging
import os
import signal
import sys
import time

# ── Crash diagnostics ───────────────────────────────────────────────
# Dump a full per-thread Python traceback on a hard crash (segfault / fatal
# error) — these print NOTHING by default, so an end-of-print "ZP retract"
# crash left no trail. Writes to stderr (the console) AND to logs/crash.log so
# the traceback survives even if the terminal scrolls away. Cheap + always-on.
try:
    os.makedirs("logs", exist_ok=True)
    _crash_log = open(os.path.join("logs", "crash.log"), "a", buffering=1)
    faulthandler.enable(file=_crash_log, all_threads=True)
except Exception:
    faulthandler.enable(all_threads=True)  # stderr fallback

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
    """Configure logging for the application.

    Console at the chosen level, plus a persistent rotating file at
    ``logs/app.log`` (always DEBUG) so a session's full log survives after the
    terminal scrolls away / the app exits — essential for diagnosing the
    intermittent ZP comms faults. The ZP serial link has its own dedicated
    trace file (``logs/zp_serial.log``, see ``SupportClasses/ZPSerialTrace``).
    """
    level = logging.DEBUG if verbose else logging.INFO
    fmt = logging.Formatter(
        "%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S")

    root = logging.getLogger()
    root.setLevel(logging.DEBUG)

    # v7.5.x: silence third-party DEBUG firehoses. ``comtypes`` logs ~10 lines
    # per COM call, and the microscope panel polls the Ti's COM object model
    # about once a second — that alone wrote ~35 000 lines (several MB) during a
    # 4-minute mosaic scan, rotating app.log every few minutes and burying the
    # app's own diagnostics. WARNING keeps real COM errors.
    for _noisy in ("comtypes", "comtypes.client", "comtypes.client._generate",
                   "comtypes.client._managing", "comtypes._post_coinit",
                   "comtypes._post_coinit.unknwn", "PIL", "matplotlib"):
        logging.getLogger(_noisy).setLevel(logging.WARNING)

    console = logging.StreamHandler()
    console.setLevel(level)
    console.setFormatter(fmt)
    root.addHandler(console)

    try:
        from logging.handlers import RotatingFileHandler
        os.makedirs("logs", exist_ok=True)
        fileh = RotatingFileHandler(
            os.path.join("logs", "app.log"), maxBytes=8_000_000,
            backupCount=5, encoding="utf-8")
        # Full-resolution date on the file (the console keeps the short time).
        fileh.setFormatter(logging.Formatter(
            "%(asctime)s [%(name)s] %(levelname)s: %(message)s"))
        fileh.setLevel(logging.DEBUG)
        root.addHandler(fileh)
        root.info("=== app.log opened (level=%s) ===",
                  logging.getLevelName(level))
    except Exception as e:  # never let logging setup break startup
        root.warning("Could not open logs/app.log: %s", e)


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
    # v7.5.x: build the same per-axis deadzones the GUI connect path uses
    # (previously headless fell back to the worker's global 0.2 deadzone
    # for the triggers too, diverging from the GUI's 0.05 default).
    _stick_dz = float(settings.get("xbox.deadzones.sticks", 0.20))
    _trig_dz = float(settings.get("xbox.deadzones.triggers", 0.05))
    controller.connect_xbox(
        stick_offsets=_stick_offsets or None,
        axis_deadzones={0: _stick_dz, 1: _stick_dz, 2: _stick_dz,
                        3: _stick_dz, 4: _trig_dz, 5: _trig_dz},
        reconnect_timeout=float(settings.get("xbox.reconnect_timeout_s", 30)),
        debug_mode=bool(settings.get("xbox.debug_mode", False)),
    )

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


def _ensure_machine_id(app) -> None:
    """Report which rig this is, before anything resolves a config path.

    Per-machine config (calibration, camera cal, mosaics, …) lives under
    ``config/hardware/<machine-id>/`` (see ``SupportClasses/MachineConfig.py``)
    so it can never collide with another rig's data when this repo is shared
    across several machines. The identity IS the active device profile's name,
    chosen on Hardware Setup → Device; a machine with no profile yet gets the
    onboarding wizard, which lands the operator on exactly that card.

    So there is nothing to prompt for here — this only logs, early, which
    bucket the session is using. It stays a named function because
    ``run_gui`` must not import ``gui.app`` before it (that import resolves
    five per-machine stores), and an AST test pins that ordering.
    """
    try:
        from SupportClasses import MachineConfig
        if MachineConfig.machine_id_is_configured():
            logging.getLogger(__name__).info(
                "Machine: %s  (config/hardware/%s/)",
                MachineConfig.machine_id(), MachineConfig.machine_id())
        else:
            logging.getLogger(__name__).warning(
                "No device profile selected yet — per-machine config is read "
                "in place and nothing is moved. Name this machine on "
                "Hardware Setup → Device.")
    except Exception as exc:   # never block startup over this
        logging.getLogger(__name__).warning(f"Machine id check skipped: {exc}")


def _apply_dark_palette(app) -> None:
    """Pin a Catppuccin-Mocha dark palette on the QApplication.

    The dark look is a QSS (``gui/styles.py::build_theme``), but the QSS only
    styles the widgets it names — it does NOT cover QGraphicsView / QScrollArea
    viewports or leave a base-widget background, so those fall back to the Qt
    palette. With no palette set, Fusion follows the OS theme, and a light-mode
    OS makes the palette Base white → white viewports/panels on many pages.
    Setting the palette here decouples the app's dark theme from the OS theme;
    the QSS still wins wherever it sets an explicit per-widget background.
    """
    try:
        from PySide6.QtGui import QPalette, QColor
    except Exception:
        return
    pal = QPalette()
    win = QColor("#1e1e2e")       # base    — window / general surface
    base = QColor("#181825")      # mantle  — text-entry + VIEWPORT background
    alt = QColor("#313244")       # surface0
    text = QColor("#cdd6f4")      # text
    dim = QColor("#a6adc8")       # subtext0 (placeholder)
    disabled = QColor("#6c7086")  # overlay0
    pal.setColor(QPalette.Window, win)
    pal.setColor(QPalette.WindowText, text)
    pal.setColor(QPalette.Base, base)
    pal.setColor(QPalette.AlternateBase, alt)
    pal.setColor(QPalette.ToolTipBase, win)
    pal.setColor(QPalette.ToolTipText, text)
    pal.setColor(QPalette.Text, text)
    pal.setColor(QPalette.Button, alt)
    pal.setColor(QPalette.ButtonText, text)
    pal.setColor(QPalette.BrightText, QColor("#f38ba8"))
    pal.setColor(QPalette.Link, QColor("#89b4fa"))
    pal.setColor(QPalette.Highlight, QColor("#cba6f7"))
    pal.setColor(QPalette.HighlightedText, win)
    try:
        pal.setColor(QPalette.PlaceholderText, dim)
    except Exception:
        pass  # PlaceholderText only exists on Qt 5.12+/Qt6
    for role in (QPalette.Text, QPalette.WindowText, QPalette.ButtonText):
        pal.setColor(QPalette.Disabled, role, disabled)
    app.setPalette(pal)


def run_gui(controller: StageController, settings: Settings):
    """Run the full GUI application."""
    try:
        from PySide6.QtWidgets import QApplication
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

    # v7.5.x: pin a dark palette to match the dark QSS. Fusion follows the
    # OS/system palette when none is set, so on a LIGHT-mode OS (or after a Qt
    # dark-mode change) the palette Base is white — and EVERY widget the QSS
    # doesn't explicitly background-fill renders white: QScrollArea + QGraphicsView
    # viewports (several calibration/plate/projection views set no background
    # brush) and transparent panels that bottom out at Base. This was surfacing
    # as "multiple pages have a white background." A dark palette makes those
    # default to the theme's dark surfaces regardless of the OS theme; the QSS
    # still overrides per-widget wherever it sets an explicit background.
    _apply_dark_palette(app)

    # v7.17.x: ask which physical rig this is (once) before anything else
    # touches per-machine config — see _ensure_machine_id's docstring.
    _ensure_machine_id(app)

    # ⚠ ORDER IS LOAD-BEARING — do NOT hoist these back up to the PySide6
    # import above. Importing ``gui.app`` transitively imports several
    # per-machine stores (objectives, needle-bore, fluorescence mosaics,
    # print timing …), and each resolves its config path AT IMPORT TIME. Done
    # before _ensure_machine_id, that resolution happens with no machine id
    # known, so a fresh rig would read/write the wrong bucket for the whole
    # session. MachineConfig also refuses to migrate while unconfigured, so
    # this ordering and that guard are belt-and-braces for the same hazard.
    try:
        from gui.app import MainWindow
        from gui.widgets.console_log import QtLogHandler
    except ImportError as e:
        print(f"GUI dependencies not available: {e}")
        print("Install PySide6: pip install PySide6")
        print("Or run in headless mode: python main.py --headless")
        sys.exit(1)

    # v7.16: arm the GUI-thread stall watchdog. faulthandler (above) catches a
    # hard crash but is blind to a HANG — the process is alive, the event loop
    # simply stops turning — which is what gets reported as "python freezes"
    # and which leaves no trail whatsoever. This dumps every thread's stack to
    # logs/freeze.log when the event loop stalls, so a freeze that only happens
    # on the rig still produces the frame it is stuck in.
    try:
        from SupportClasses.GuiWatchdog import start_watchdog
        start_watchdog()
    except Exception as exc:                          # never block startup
        logging.getLogger(__name__).debug(f"GUI watchdog unavailable: {exc}")

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
    # v7.4.2: GUI mode no longer auto-connects — Connect / Simulate
    # buttons on Hardware Setup → Device drive per-stage selection.
    # In headless mode the default is real hardware on both axes; pass
    # --simulate-xy / --simulate-zp to swap in the built-in simulators.
    parser.add_argument("--real-xy", action="store_true",
                        help="(Headless) Force real XY stage. Default is real "
                             "hardware, so this is only needed alongside the "
                             "deprecated --simulate-xy to win the override.")
    parser.add_argument("--real-zp", action="store_true",
                        help="(Headless) Force real ZP stage. See --real-xy.")
    parser.add_argument("--simulate-xy", action="store_true",
                        help="(Headless) Open the XY simulator instead of real hardware.")
    parser.add_argument("--simulate-zp", action="store_true",
                        help="(Headless) Open the ZP simulator instead of real hardware.")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable debug logging")
    parser.add_argument("--debug-xy", action="store_true",
                        help="Record raw XY serial data + jog commands to a CSV file")
    parser.add_argument("--settings", default="settings.json",
                        help="Path to settings file (default: settings.json)")
    args = parser.parse_args()

    # v7.17.x: the machine identity (= active device profile name, which names
    # this rig's config/hardware/<name>/ folder) is read straight from the
    # settings FILE by SupportClasses.MachineConfig, so a non-default
    # --settings must be pointed at BEFORE anything resolves a config path.
    # Otherwise one rig's settings would run against another rig's config.
    if args.settings != "settings.json":
        try:
            from SupportClasses.MachineConfig import set_settings_path
            set_settings_path(args.settings)
        except Exception as exc:
            logging.getLogger(__name__).warning(
                f"Could not point MachineConfig at {args.settings}: {exc}")

    settings = Settings(args.settings)
    settings.load()

    verbose = args.verbose or settings.get("logging.verbose", False)
    setup_logging(verbose)

    if getattr(args, "debug_xy", False) or settings.get("logging.debug_xy", False):
        from SupportClasses.XYDebugLogger import enable as _enable_xy_debug
        _enable_xy_debug()

    # v7.4.2: simulation is no longer a persisted setting. Real hardware
    # is the default; CLI flags swap in the simulator per axis (mainly
    # useful in headless mode, since GUI mode chooses per-connect from
    # the Device sub-page). --real-* wins when both flags are passed.
    simulate_xy = bool(getattr(args, "simulate_xy", False))
    simulate_zp = bool(getattr(args, "simulate_zp", False))
    if args.real_xy:
        simulate_xy = False
    if args.real_zp:
        simulate_zp = False

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

    # v7.5.x: restore the unified Z convention (per-machine up-direction +
    # needle-cam fiducial + standard plate offsets) and the well-plate
    # orientation (plate_flip_180) from the device profile.
    controller.apply_z_convention(
        z_up_sign=settings.get("device_profile.z_up_sign"),
        needle_cam_z=settings.get("device_profile.needle_cam_z"),
        plate_z_offsets=settings.get("device_profile.plate_z_offsets"),
        plate_flip_180=settings.get("device_profile.plate_flip_180"),
    )

    # v7.5.x: restore the per-pump plunger convention (datum + derived
    # dispense/aspirate direction from the captured Set Dispensed / Set
    # Aspirated extremes). Soft limits come from the safety_limits section
    # above; this re-establishes the direction the calibration owns.
    controller.apply_pump_convention(
        settings.get("device_profile.pump_setup"))

    # v7.5.x: restore the per-pump compliance / "pressure relief" values (µL)
    # measured by the Needle Location compliance calibration, and the global
    # backlash-compensation enable toggle (take-up on reversal + unload on stop).
    controller.apply_pump_relief(
        settings.get("device_profile.pump_compliance_uL"))
    controller.set_backlash_comp_enabled(
        bool(settings.get("device_profile.backlash_comp_enabled")))

    if args.headless:
        run_headless(controller, settings)
    else:
        run_gui(controller, settings)


if __name__ == "__main__":
    # Required for PyInstaller on macOS/Windows when using multiprocessing
    import multiprocessing
    multiprocessing.freeze_support()
    main()
