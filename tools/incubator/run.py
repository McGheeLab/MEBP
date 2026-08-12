"""
run.py — launch the standalone incubator heater tool.

Run from the repo root::

    python tools/incubator/run.py

Options (all optional)::

    --simulate            start with the simulated board preselected
    --port COM5           preselect a serial port
    --baud 38400          preselect a baud rate
    --sim-scale 300       simulated-time acceleration factor

This is a STANDALONE tool. It does not import or start the MEBP application,
and the app does not know it exists. It does reuse two hardware-agnostic pieces
read-only: ``SupportClasses.SerialUtils`` and the theme/scaling modules.

Only one process can own a COM port, so disconnect the ZP board in the main app
before connecting here.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path


def _find_repo_root() -> Path:
    """
    Locate the repo root by looking for the ``SupportClasses`` directory.

    Same probe-upwards idiom as ``tests/find_xy_stage.py``, so the tool works
    whether it is launched from the repo root, from ``tools/``, or by absolute
    path from anywhere.
    """
    here = Path(__file__).resolve().parent
    for cand in (here, here.parent, here.parent.parent, Path.cwd()):
        if (cand / "SupportClasses").is_dir():
            return cand
    return here.parent.parent


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="incubator",
        description="Standalone two-zone incubator heater bring-up tool "
                    "(SKR Mini E3 V3 / Marlin).",
    )
    ap.add_argument("--simulate", action="store_true",
                    help="preselect the simulated board (no hardware needed)")
    ap.add_argument("--port", default="", help="preselect a serial port, e.g. COM5")
    ap.add_argument("--baud", type=int, default=0, help="preselect a baud rate")
    ap.add_argument("--sim-scale", type=float, default=0.0,
                    help="simulated-time acceleration factor (default 300)")
    ap.add_argument("--verbose", action="store_true", help="debug logging")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )

    root = _find_repo_root()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

    # HiDPI hint must be set before Qt is imported (same as main.py).
    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")

    try:
        from PySide6.QtWidgets import QApplication
    except ImportError:
        print("ERROR: PySide6 is not installed.  pip install PySide6",
              file=sys.stderr)
        return 1

    # QApplication FIRST: gui.scaling.scale_factor() memoises on its first call,
    # so touching s() before the app exists would lock the UI scale to 1.0.
    app = QApplication(sys.argv[:1])
    app.setApplicationName("MEBP Incubator")

    # Fusion style + dark palette + the app QSS, in that order. This is NOT
    # optional: the native Windows/macOS styles ignore large parts of a
    # stylesheet, and the app's QSS sets a foreground colour on QWidget with no
    # background, so without this you get dark text on system-white panels.
    from tools.incubator.theme import install_theme
    install_theme(app)

    from tools.incubator.controller import IncubatorController
    from tools.incubator.gui import IncubatorWindow

    ctrl = IncubatorController()
    win = IncubatorWindow(ctrl)

    if args.port:
        win._port.setCurrentText(args.port)
    if args.baud:
        win._baud.setCurrentText(str(args.baud))
    if args.sim_scale:
        win._sim_scale.setValue(int(args.sim_scale))
    if args.simulate:
        win._sim.setChecked(True)

    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
