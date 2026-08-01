# -*- coding: utf-8 -*-
"""tools_capture_app_screens.py - screenshot the real MEBP GUI, page by page,
for use in tutorial material (see docs/videos/).

Usage:  python tools_capture_app_screens.py [-o OUTDIR] [--scale 1.5] [--list]
        (no argument = docs/videos/app-tour/screens)

Boots a full MainWindow with SIMULATED hardware, walks the navigation, and
writes one PNG per screen with QWidget.grab() - the same capture primitive
gui/pages/workflows/quick_print_report.py already uses for its HTML export.

Two things this deliberately does NOT do:

  * It never touches the operator's settings.json. The app auto-saves (the
    calibration page has a ~500 ms debounced write), so pointing a scripted
    boot at the real file risks corrupting a taught calibration. Everything
    runs against a throwaway COPY in a temp directory.

  * It does not use QT_QPA_PLATFORM=offscreen. The offscreen plugin has no
    font database on Windows, so every glyph renders as tofu - layout and
    icons look right and the text is silently unreadable. The real platform
    plugin is used instead and the window is parked off-screen so it does not
    disturb the desktop."""
import argparse
import os
import shutil
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(HERE, "docs", "videos", "app-tour", "screens")

# Park the capture window outside any plausible desktop area.
OFFSCREEN_POS = (-4000, -4000)
WINDOW_SIZE = (1920, 1080)

# (slug, page index, description, navigator) - navigator gets the MainWindow.
SCREENS = [
    ("hardware-device", 0, "Hardware Setup / Device",
     lambda w: _sub(w, 0, 0)),
    ("hardware-plate", 0, "Hardware Setup / Plate",
     lambda w: _sub(w, 0, 2)),
    ("hardware-needle", 0, "Hardware Setup / Needle",
     lambda w: _sub(w, 0, 5)),
    ("calibration-needle-location", 1, "Calibration / Needle Location",
     lambda w: _cal_tab(w, 0)),
    ("calibration-plate-location", 1, "Calibration / Plate Location",
     lambda w: _cal_tab(w, 2)),
    ("jog", 2, "Jog Control", lambda w: None),
    ("print-builder-sketch", 3, "Print Builder / Sketch",
     lambda w: _sub(w, 3, 0)),
    ("print-builder-prints", 3, "Print Builder / Prints",
     lambda w: _sub(w, 3, 2)),
    ("workflows-picker", 4, "Workflows / picker", lambda w: None),
    ("workflows-quick-print", 4, "Workflows / Quick Print",
     lambda w: _open_workflow(w, "quick_print")),
]


def _page(win, index):
    return win._page_stack.widget(index)


def _sub(win, page_index, sub_index):
    page = _page(win, page_index)
    if hasattr(page, "switch_to"):
        page.switch_to(sub_index)


def _cal_tab(win, tab_index):
    page = _page(win, 1)
    tabs = getattr(page, "_workflow_tabs", None)
    if tabs is not None:
        tabs.setCurrentIndex(tab_index)


def _open_workflow(win, workflow_id):
    mode = getattr(win, "_workflows_mode", None)
    if mode is not None and hasattr(mode, "open_workflow"):
        mode.open_workflow(workflow_id)


def _settle(app, cycles=25, pause=0.02):
    """Let queued paints, timers and lazy page builds finish."""
    for _ in range(cycles):
        app.processEvents()
        time.sleep(pause)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default=DEFAULT_OUT)
    ap.add_argument("--scale", default="1.0",
                    help="MEBP_UI_SCALE for the capture (default 1.0)")
    ap.add_argument("--list", action="store_true",
                    help="list the screens this would capture and exit")
    args = ap.parse_args(argv)

    if args.list:
        for slug, idx, desc, _ in SCREENS:
            print("%-32s page %d  %s" % (slug, idx, desc))
        return 0

    # Real platform plugin (offscreen has no fonts); fixed UI scale so the
    # captures are reproducible across machines/DPI.
    os.environ.pop("QT_QPA_PLATFORM", None)
    os.environ["MEBP_UI_SCALE"] = str(args.scale)

    sys.path.insert(0, HERE)
    os.chdir(HERE)

    work = tempfile.mkdtemp(prefix="mebp_capture_")
    settings_copy = os.path.join(work, "settings.json")
    real = os.path.join(HERE, "settings.json")
    if os.path.exists(real):
        shutil.copy2(real, settings_copy)

    from PySide6.QtWidgets import QApplication, QStyleFactory
    from SupportClasses.StageController import StageController
    from SupportClasses.Settings import Settings
    from SupportClasses.PrintRecorder import PrintRecorder
    import main as mebp_main

    app = QApplication(sys.argv[:1])
    fusion = QStyleFactory.create("Fusion")
    if fusion is not None:
        app.setStyle(fusion)
    mebp_main._apply_dark_palette(app)

    settings = Settings(settings_copy)
    settings.load()

    controller = StageController(simulate_xy=True, simulate_zp=True,
                                 controller_json="auto")
    saved_zero = settings.get_section("zero_position")
    if saved_zero:
        controller.zero_position.update(saved_zero)
    saved_limits = settings.get_section("safety_limits")
    if saved_limits:
        from SupportClasses.SafetyLimits import SafetyLimits
        controller.safety_limits = SafetyLimits.from_dict(saved_limits)
    controller.apply_z_convention(
        z_up_sign=settings.get("device_profile.z_up_sign"),
        needle_cam_z=settings.get("device_profile.needle_cam_z"),
        plate_z_offsets=settings.get("device_profile.plate_z_offsets"),
        plate_flip_180=settings.get("device_profile.plate_flip_180"),
    )

    from gui.app import MainWindow
    win = MainWindow(controller, settings, recorder=PrintRecorder())
    win.resize(*WINDOW_SIZE)
    win.move(*OFFSCREEN_POS)
    win.show()
    _settle(app, cycles=40)

    os.makedirs(args.out, exist_ok=True)
    written, failed = [], []

    for slug, page_index, desc, navigate in SCREENS:
        try:
            win._navigate_to(page_index)
            _settle(app, cycles=10)
            navigate(win)
            _settle(app, cycles=20)
            path = os.path.join(args.out, slug + ".png")
            pm = win.grab()
            pm.save(path)
            written.append((slug, pm.width(), pm.height()))
            print("  captured %-32s %dx%d  (%s)"
                  % (slug, pm.width(), pm.height(), desc))
        except Exception as exc:            # one bad screen must not abort
            failed.append((slug, repr(exc)))
            print("  FAILED    %-32s %s" % (slug, exc))

    print("\n%d screen(s) -> %s" % (len(written), args.out))
    if failed:
        print("%d failed: %s" % (len(failed), ", ".join(s for s, _ in failed)))

    try:
        controller.shutdown()
    except Exception:
        pass
    # Hard exit: background poller/camera threads otherwise keep the process up.
    sys.stdout.flush()
    os._exit(0 if written else 1)


if __name__ == "__main__":
    sys.exit(main())
