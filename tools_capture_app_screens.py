# -*- coding: utf-8 -*-
"""tools_capture_app_screens.py - screenshot the real MEBP GUI, page by page,
for use in tutorial material (see docs/videos/).

Usage:  python tools_capture_app_screens.py [-o OUTDIR] [--scale 1.0] [--list]
        (no argument = docs/videos/app-tour/screens)

Boots a full MainWindow with SIMULATED hardware, walks the navigation, and
writes one PNG per screen with QWidget.grab() - the same capture primitive
gui/pages/workflows/quick_print_report.py already uses for its HTML export.

Safety / robustness properties (each learned the hard way):

  * Leaves the operator's real config exactly as it found it. The app
    auto-saves (the calibration page has a ~500 ms debounced write), so this
    runs against a throwaway COPY of settings.json and redirects every store
    with a $MEBP_* override into the temp dir. Several stores have NO
    override - CalibrationSnapshotStore and CameraCalibrationStore were both
    observed rewriting real files during a capture - so config/hardware JSON
    is copied aside first and any file the run changed is restored afterwards.
    These files hold a taught plate calibration; taking screenshots must not
    be able to alter them.

  * No QT_QPA_PLATFORM=offscreen. That plugin has no font database on
    Windows, so every glyph renders as tofu - layout and icons look right and
    the text is silently unreadable. The real platform plugin is used and the
    window is parked off-screen instead.

  * Cannot deadlock on a startup dialog. On a fresh machine (no settings.json,
    no calibration_status.json) the app opens the onboarding wizard at t=0 and
    a calibration-status QMessageBox at t=900ms - both exec() their own event
    loop, which processEvents() never returns from, with the dialog invisible
    at (-4000,-4000). Onboarding is suppressed up front AND a modal-reaper
    timer closes any modal that still appears.

  * Never mislabels a capture. Every SCREENS entry declares the page title it
    expects; after navigating, the title is read back and the screen FAILS on
    mismatch rather than silently saving the wrong page under the right name.
    Exit code reflects failures.

Resolution: the window is 1920x1080 in CLIENT pixels; the saved PNG is that
times the display's devicePixelRatio (2880x1620 on a 150% Windows display,
1920x1080 at 100%). Both are 16:9, so compositions must scale rather than
assume pixel dimensions. --scale sets MEBP_UI_SCALE (widget sizing) only."""
import argparse
import glob
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

# Per-machine stores that honour a $MEBP_* directory override.
_STORE_ENV_DIRS = [
    "MEBP_CALIBRATION_STATUS_DIR",
    "MEBP_CONTEXT_PANEL_DIR",
    "MEBP_MICROSCOPE_CONFIG_DIR",
    "MEBP_WORKFLOW_SETTINGS_DIR",
    "MEBP_TARGET_TYPE_DIR",
    "MEBP_NEEDLE_TEMPLATE_DIR",
    "MEBP_SPHEROID_TRAINING_DIR",
    "MEBP_CHALLENGE_LOG_DIR",
]
# Overrides naming a specific file/dir path rather than a container dir.
_STORE_ENV_PATHS = [
    ("MEBP_FLUOR_MOSAIC_PATH", "fluor_mosaics"),
    ("MEBP_NEEDLE_BORE_CAL_PATH", "needle_bore_cal.json"),
    ("MEBP_SPHEROID_SINK_CAL_PATH", "spheroid_sink_cal.json"),
]

# (slug, page index, expected page-title substring, description, navigator)
# NOTE on `expect`: mode pages delegate get_page_title() to their active
# sub-page, so Print Builder → "Sketch"/"Prints" and an opened workflow →
# "Quick Print". Pages without sub-pages report their own name.
SCREENS = [
    ("hardware-device", 0, "Hardware", "Hardware Setup / Device",
     lambda w: _sub(w, 0, 0)),
    ("hardware-plate", 0, "Hardware", "Hardware Setup / Plate",
     lambda w: _sub(w, 0, 2)),
    ("hardware-needle", 0, "Hardware", "Hardware Setup / Needle",
     lambda w: _sub(w, 0, 5)),
    ("calibration-needle-location", 1, "Calibration",
     "Calibration / Needle Location", lambda w: _cal_tab(w, 0)),
    ("calibration-plate-location", 1, "Calibration",
     "Calibration / Plate Location", lambda w: _cal_tab(w, 2)),
    ("jog", 2, "Jog", "Jog Control", lambda w: None),
    ("print-builder-sketch", 3, "Sketch", "Print Builder / Sketch",
     lambda w: _sub(w, 3, 0)),
    ("print-builder-prints", 3, "Prints", "Print Builder / Prints",
     lambda w: _sub(w, 3, 2)),
    ("workflows-picker", 4, "Workflows", "Workflows / picker",
     lambda w: None),
    ("workflows-quick-print", 4, "Quick Print", "Workflows / Quick Print",
     lambda w: _open_workflow(w, "quick_print")),
]


# ── navigation (raises on drift instead of silently no-op'ing) ──────

def _page(win, index):
    stack = win._page_stack
    if not (0 <= index < stack.count()):
        raise RuntimeError("page index %d out of range (%d pages)"
                           % (index, stack.count()))
    return stack.widget(index)


def _sub(win, page_index, sub_index):
    page = _page(win, page_index)
    if not hasattr(page, "switch_to"):
        raise RuntimeError("%s has no switch_to() - mode-page API changed"
                           % page.__class__.__name__)
    page.switch_to(sub_index)


def _cal_tab(win, tab_index):
    page = _page(win, 1)
    tabs = getattr(page, "_workflow_tabs", None)
    if tabs is None:
        raise RuntimeError("calibration page has no _workflow_tabs")
    if not (0 <= tab_index < tabs.count()):
        raise RuntimeError("calibration tab %d out of range (%d tabs)"
                           % (tab_index, tabs.count()))
    tabs.setCurrentIndex(tab_index)


def _open_workflow(win, workflow_id):
    mode = getattr(win, "_workflows_mode", None)
    if mode is None or not hasattr(mode, "open_workflow"):
        raise RuntimeError("workflows mode has no open_workflow()")
    mode.open_workflow(workflow_id)


def _settle(app, cycles=25, pause=0.02):
    """Let queued paints, timers and lazy page builds finish."""
    for _ in range(cycles):
        app.processEvents()
        time.sleep(pause)


def _page_title(win):
    lbl = getattr(win, "_page_title", None)
    try:
        return lbl.text() if lbl is not None else ""
    except Exception:
        return ""


# ── isolation helpers ───────────────────────────────────────────────

def _redirect_stores(work):
    """Point every store that supports an override at the temp workdir."""
    for var in _STORE_ENV_DIRS:
        d = os.path.join(work, var.lower())
        os.makedirs(d, exist_ok=True)
        os.environ[var] = d
    for var, leaf in _STORE_ENV_PATHS:
        os.environ[var] = os.path.join(work, leaf)


def _config_files():
    paths = set()
    for pattern in ("config/hardware/*.json", "config/hardware/**/*.json"):
        paths.update(glob.glob(os.path.join(HERE, pattern), recursive=True))
    return sorted(paths)


def _backup_config(work):
    """Copy every per-machine config JSON aside before the run.

    Several stores (CalibrationSnapshotStore, CameraCalibrationStore, ...)
    have no $MEBP_* override, so booting the app WILL rewrite them — observed
    rewriting last_calibration.json and camera_calibrations.json. Content came
    back identical, but "identical this time" is not a safety property. These
    files hold a taught plate calibration; capturing screenshots must not be
    able to touch them at all."""
    backup_dir = os.path.join(work, "_config_backup")
    os.makedirs(backup_dir, exist_ok=True)
    saved = {}
    for path in _config_files():
        # Key by the RELATIVE path, flattened - config/hardware has
        # subdirectories (devices/, mosaics/, plate_types/) and two files can
        # share a basename. Colliding backups would restore the wrong bytes,
        # which is worse than not backing up at all.
        rel = os.path.relpath(path, HERE).replace(os.sep, "__")
        dest = os.path.join(backup_dir, rel)
        try:
            shutil.copy2(path, dest)
            saved[path] = dest
        except OSError:
            pass
    return saved


def _restore_config(saved):
    """Put back any config file the run changed. Returns what was reverted."""
    reverted = []
    for path, backup in saved.items():
        try:
            with open(path, "rb") as a, open(backup, "rb") as b:
                if a.read() == b.read():
                    continue
        except OSError:
            continue
        try:
            shutil.copy2(backup, path)
            reverted.append(path)
        except OSError:
            print("  !! could not restore %s" % path)
    if reverted:
        print("\n  Reverted %d config file(s) the app rewrote during capture:"
              % len(reverted))
        for path in sorted(reverted):
            print("     %s" % os.path.relpath(path, HERE))
    return reverted


def _install_modal_reaper(app):
    """Close any modal dialog that appears. processEvents() cannot return
    from a dialog's own exec() loop, so an unattended run would otherwise
    hang forever on an invisible off-screen dialog."""
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QDialog, QMessageBox

    def reap():
        w = app.activeModalWidget()
        if w is None:
            return
        print("  (closed startup dialog: %s)" % w.__class__.__name__)
        try:
            if isinstance(w, (QDialog, QMessageBox)):
                w.reject()
            else:
                w.close()
        except Exception:
            try:
                w.close()
            except Exception:
                pass

    timer = QTimer()
    timer.timeout.connect(reap)
    timer.start(400)
    return timer                       # keep a reference alive


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default=DEFAULT_OUT)
    ap.add_argument("--scale", default="1.0",
                    help="MEBP_UI_SCALE for the capture (default 1.0)")
    ap.add_argument("--list", action="store_true",
                    help="list the screens this would capture and exit")
    ap.add_argument("--keep-workdir", action="store_true",
                    help="do not delete the temp workdir (debugging)")
    args = ap.parse_args(argv)

    if args.list:
        for slug, idx, _expect, desc, _nav in SCREENS:
            print("%-32s page %d  %s" % (slug, idx, desc))
        return 0

    # Real platform plugin (offscreen has no fonts).
    os.environ.pop("QT_QPA_PLATFORM", None)
    os.environ["MEBP_UI_SCALE"] = str(args.scale)

    sys.path.insert(0, HERE)
    os.chdir(HERE)

    work = tempfile.mkdtemp(prefix="mebp_capture_")
    _redirect_stores(work)
    settings_copy = os.path.join(work, "settings.json")
    real = os.path.join(HERE, "settings.json")
    if os.path.exists(real):
        shutil.copy2(real, settings_copy)
    config_backup = _backup_config(work)

    from PySide6.QtWidgets import QApplication, QStyleFactory
    from SupportClasses.StageController import StageController
    from SupportClasses.Settings import Settings
    from SupportClasses.PrintRecorder import PrintRecorder
    import main as mebp_main

    # Suppress the first-run onboarding wizard BEFORE MainWindow is built —
    # it exec()s a modal at t=0 and would deadlock an unattended run.
    try:
        import gui.onboarding.wizard as _wiz
        _wiz.should_show_onboarding = lambda *a, **k: False
    except Exception as exc:
        print("  (note: could not suppress onboarding: %s)" % exc)

    app = QApplication(sys.argv[:1])
    fusion = QStyleFactory.create("Fusion")
    if fusion is not None:
        app.setStyle(fusion)
    mebp_main._apply_dark_palette(app)
    _reaper = _install_modal_reaper(app)          # noqa: F841 (keep alive)

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

    for slug, page_index, expect, desc, navigate in SCREENS:
        try:
            win._navigate_to(page_index)
            _settle(app, cycles=10)
            navigate(win)
            _settle(app, cycles=20)

            # Verify we are actually where we think we are — a silently
            # no-op'd navigation would otherwise save the PREVIOUS page under
            # this screen's filename, and the error would only surface as a
            # confusing tutorial months later.
            title = _page_title(win)
            if expect and expect.lower() not in title.lower():
                raise RuntimeError(
                    "expected a page titled like %r, got %r" % (expect, title))

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
        print("%d FAILED: %s" % (len(failed), ", ".join(s for s, _ in failed)))

    try:
        controller.shutdown()
    except Exception:
        pass

    _restore_config(config_backup)

    if args.keep_workdir:
        print("  workdir kept: %s" % work)
    else:
        # The workdir holds a copy of the operator's taught calibration —
        # don't leave it in %TEMP% forever.
        shutil.rmtree(work, ignore_errors=True)

    sys.stdout.flush()
    # Hard exit: background poller/camera threads otherwise keep us alive.
    os._exit(0 if (written and not failed) else 1)


if __name__ == "__main__":
    sys.exit(main())
