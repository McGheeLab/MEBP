"""
capture_screenshots.py — Render the real MEBP GUI offscreen (simulated
hardware) and grab a PNG of each page/sub-page used by the Quick Start Guide.

Run from the repo root:
    QT_QPA_PLATFORM=offscreen python quickstart_guide/capture_screenshots.py

Outputs PNGs into quickstart_guide/screenshots/.
"""
import os
import sys

# NOTE: we deliberately use the REAL "windows" platform (NOT offscreen) — the
# offscreen plugin on this build loads 0 system fonts, so all text renders as
# tofu boxes. The real platform loads fonts; WA_DontShowOnScreen keeps the
# window from ever appearing on the desktop while still laying it out so
# QWidget.grab() works.
os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "0")
os.environ.setdefault("QT_SCALE_FACTOR", "1")
os.environ.setdefault("MEBP_UI_SCALE", "1.0")

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
os.chdir(REPO)

OUT = os.path.join(REPO, "quickstart_guide", "screenshots")
os.makedirs(OUT, exist_ok=True)

from PySide6.QtWidgets import QApplication, QStyleFactory
from PySide6.QtCore import QElapsedTimer, Qt

from SupportClasses.StageController import StageController
from SupportClasses.Settings import Settings


WIN_W, WIN_H = 1680, 1000


def settle(app, ms=700):
    """Spin the event loop so layouts/animations/timers settle."""
    t = QElapsedTimer()
    t.start()
    while t.elapsed() < ms:
        app.processEvents()
        app.sendPostedEvents()


def grab(window, name):
    path = os.path.join(OUT, name + ".png")
    pix = window.grab()
    ok = pix.save(path, "PNG")
    print(f"  {'OK ' if ok else 'FAIL'} {name}  ({pix.width()}x{pix.height()})")
    return ok


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("MEBP Bioprinter")
    fusion = QStyleFactory.create("Fusion")
    if fusion:
        app.setStyle(fusion)

    settings = Settings("settings.json")
    settings.load()

    controller = StageController(simulate_xy=True, simulate_zp=True,
                                 controller_json="auto")
    saved_zero = settings.get_section("zero_position")
    if saved_zero:
        controller.zero_position.update(saved_zero)
    # Restore the Z / pump / plate conventions like main.py so pages render
    # with correct orientation + heights.
    try:
        controller.apply_z_convention(
            z_up_sign=settings.get("device_profile.z_up_sign"),
            needle_cam_z=settings.get("device_profile.needle_cam_z"),
            plate_z_offsets=settings.get("device_profile.plate_z_offsets"),
            plate_flip_180=settings.get("device_profile.plate_flip_180"),
        )
        controller.apply_pump_convention(settings.get("device_profile.pump_setup"))
    except Exception as e:
        print("convention restore warning:", e)

    controller.connect_stages()

    from PySide6.QtWidgets import QApplication as _QA  # noqa
    from gui.app import MainWindow
    try:
        from SupportClasses.PrintRecorder import PrintRecorder
        recorder = PrintRecorder()
    except Exception:
        recorder = None

    window = MainWindow(controller, settings, recorder=recorder)
    # Render fully but never appear on the real desktop.
    window.setAttribute(Qt.WA_DontShowOnScreen, True)
    window.resize(WIN_W, WIN_H)
    window.show()
    settle(app, 1200)

    page_widgets = window._page_widgets

    def nav_page(i):
        window._switch_page(i)
        settle(app, 800)
        return window

    def nav_hw_sub(i):
        window._switch_page(0)
        settle(app, 300)
        hw = page_widgets[0]
        try:
            hw.switch_to(i)
        except Exception as e:
            print("   hw.switch_to error:", e)
        settle(app, 800)
        return window

    def nav_cal_tab(i):
        window._switch_page(1)
        settle(app, 300)
        cal = page_widgets[1]
        try:
            cal._workflow_tabs.setCurrentIndex(i)
        except Exception as e:
            print("   cal tab error:", e)
        settle(app, 900)
        return window

    def nav_workflows_picker():
        window._switch_page(5)
        settle(app, 300)
        wf = page_widgets[5]
        try:
            wf._show_picker()
        except Exception as e:
            print("   picker error:", e)
        settle(app, 700)
        return window

    def nav_workflow(wid):
        window._switch_page(5)
        settle(app, 300)
        wf = page_widgets[5]
        try:
            wf._on_workflow_selected(wid)
        except Exception as e:
            print("   workflow error:", e)
        settle(app, 1000)
        return window

    targets = [
        ("00_main_window",        lambda: nav_page(0)),
        ("01_hw_device",          lambda: nav_hw_sub(0)),
        ("02_hw_plate",           lambda: nav_hw_sub(2)),
        ("03_hw_ink",             lambda: nav_hw_sub(4)),
        ("04_hw_needle",          lambda: nav_hw_sub(5)),
        ("05_hw_pump",            lambda: nav_hw_sub(6)),
        ("06_hw_cameras",         lambda: nav_hw_sub(7)),
        ("10_cal_needle_loc",     lambda: nav_cal_tab(0)),
        ("11_cal_needle_offset",  lambda: nav_cal_tab(1)),
        ("12_cal_plate_loc",      lambda: nav_cal_tab(2)),
        ("13_cal_plate_z_autocal",lambda: nav_cal_tab(3)),
        ("20_workflow_picker",    lambda: nav_workflows_picker()),
        ("21_quick_print",        lambda: nav_workflow("quick_print")),
        ("22_cell_targeting",     lambda: nav_workflow("cell_targeting")),
        ("23_spheroid_pickup",    lambda: nav_workflow("spheroid_pickup")),
        ("24_cell_labeling",      lambda: nav_workflow("cell_labeling")),
    ]

    print(f"Capturing {len(targets)} screenshots @ {WIN_W}x{WIN_H} ...")
    for name, fn in targets:
        try:
            w = fn()
            grab(w, name)
        except Exception as e:
            print(f"  ERROR {name}: {e}")

    print("Done. Output:", OUT)
    try:
        controller.shutdown()
    except Exception:
        pass
    # Threads (poller/camera) keep the process alive; force a clean exit.
    os._exit(0)


if __name__ == "__main__":
    main()
