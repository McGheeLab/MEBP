"""
tools_microscope_hw_check.py — bench verification for the microscope body.

Exercises every microscope feature through the REAL GUI widget and controller,
exactly as an operator would drive them, and prints a pass/fail report. Run it
at the bench once the Nikon Ti is driver-reachable to complete the hardware
verification listed in
``coding plans/Update plans/MEBP_v75x_NIKON_TI_MICROSCOPE_CONTROL.md``.

Usage::

    python tools_microscope_hw_check.py                    # simulated backend
    python tools_microscope_hw_check.py nikon_ti           # turrets only
    python tools_microscope_hw_check.py nikon_ti --focus   # + focus moves
    python tools_microscope_hw_check.py micromanager --focus

⚠ **FOCUS MOVES ARE OPT-IN (`--focus`).** The focus drive moves the objective
toward the specimen; a blind move with a dish loaded can drive the objective
into the glass. Without ``--focus`` the focus axis is only READ. With it, moves
are limited to ``--step`` µm (default 5) and the starting position is always
restored. **Lower the stage / remove the sample before using ``--focus``.**

On a real backend this uses your saved configuration (driver, ProgID,
``z_units_per_um``, Micro-Manager config, slot names) and does **not** modify
your slot assignments. On ``simulated`` it uses a throwaway store.

Also runs the **optics write-support check** (always; READ-ONLY, writes
nothing): it answers whether the body's own optics database — the thing that
drives its physical LCD readout — declares its name fields as settable at all,
or whether that display is purely hardware-sensed (a coded objective/cube).
See ``MicroscopeBackend.probe_optic_write_support`` for why it must NOT settle
that by writing a value back to itself.
"""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# Run from anywhere: an elevated shell starts in system32, so don't rely on the
# working directory to find the package.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from PySide6.QtWidgets import QApplication

from SupportClasses.MicroscopeConfigStore import MicroscopeConfigStore, get_store
from SupportClasses.MicroscopeControl import MicroscopeController


class Report:
    def __init__(self):
        self.passed: list[str] = []
        self.failed: list[str] = []

    def check(self, label, got, want=None, *, ok=None):
        good = ok if ok is not None else (got == want)
        (self.passed if good else self.failed).append(label)
        detail = f"{got!r}" if want is None else f"{got!r} (expected {want!r})"
        print(f"  [{'PASS' if good else 'FAIL'}] {label}: {detail}")
        return good

    def note(self, text):
        print(f"  ---- {text}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("backend", nargs="?", default="simulated",
                    choices=("simulated", "nikon_ti", "micromanager"))
    ap.add_argument("--focus", action="store_true",
                    help="also MOVE the focus drive (see the safety warning)")
    ap.add_argument("--step", type=float, default=5.0,
                    help="focus excursion in µm (default 5)")
    args = ap.parse_args()

    simulated = args.backend == "simulated"
    tmp = None
    if simulated:
        tmp = tempfile.TemporaryDirectory()
        store = MicroscopeConfigStore(Path(tmp.name) / "microscope.json")
        store.set_backend("simulated")
        store.set_filter_labels({1: "DAPI", 2: "FITC", 3: "mCherry",
                                 4: "Cy5", 5: "Bright Field"})
        store.set_objective_labels({1: "4x", 2: "10x", 3: "20x", 4: "40x"})
    else:
        store = get_store()          # the operator's real configuration
        store.set_backend(args.backend)

    app = QApplication.instance() or QApplication([])
    r = Report()
    print(f"=== Microscope hardware check — backend: {args.backend} ===")
    if not simulated and not args.focus:
        print("    (focus is READ-ONLY; pass --focus to move it)")
    print()

    ctrl = MicroscopeController(store=store, threaded=True)
    from gui.widgets.microscope_panel import MicroscopePanel
    panel = MicroscopePanel(controller=ctrl, store=store)

    def settle():
        ctrl.wait_idle(timeout=30.0)
        app.processEvents()
        panel._render(force=True)

    try:
        # ── Connect ────────────────────────────────────────────────
        print("CONNECT")
        panel._toggle_connect()
        settle()
        st = ctrl.state()
        if not r.check("connected", st.connected, True):
            print(f"\n*** Cannot reach the body: {st.error}")
            print("*** Nothing further can be tested. Check that the driver / "
                  "SDK is installed and the body is powered on.")
            return 1
        r.check("no error on connect", st.error, None)
        r.note(f"body reports {st.filter_count} cube slots, "
               f"{st.objective_count} objective positions, "
               f"focus {st.focus_um} µm")
        if st.native_filter_names:
            r.note(f"hardware cube labels: {st.native_filter_names}")
        if st.native_objective_names:
            r.note(f"hardware objective labels: {st.native_objective_names}")

        # ── Diagnostics first: on an unverified body this is what names
        #    the devices, so print it before commanding anything. ───────
        print("\nDIAGNOSTICS")
        for line in ctrl.diagnostics().splitlines():
            print(f"  {line}")

        # ── Optics write-support check — READ-ONLY (inspects the driver's own
        #    type information; writes nothing — see
        #    MicroscopeBackend.probe_optic_write_support for why it must not
        #    settle this by writing a value back to itself). Answers whether an
        #    operator-typed cube/objective name could ever be pushed to the
        #    body's own display, or whether that display is driven entirely by
        #    a hardware-sensed Code (a coded optic's ring/chip). ────────────
        print("\nOPTICS WRITE-SUPPORT CHECK (read-only)")
        op = ctrl.probe_optic_write_support()
        ctrl.wait_idle(timeout=15.0)
        if op.error:
            r.note(f"check failed: {op.error}")
        else:
            support = ctrl.state().optic_write_support
            if not support:
                r.note("no result — no optics enumerable, or this driver has "
                       "no notion of an optics database")
            else:
                verdicts = []
                for logical, fields in support.items():
                    for name, verdict in sorted(fields.items()):
                        verdicts.append(verdict)
                        r.note(f"{logical}.{name}: "
                               + {True: "WRITABLE (declares a setter)",
                                  False: "read-only (no setter)"}.get(
                                      verdict, "undeterminable"))
                if True in verdicts:
                    r.note("=> a rename MAY be pushable — confirm one real "
                           "rename changes the body's display AND survives a "
                           "reconnect before relying on it")
                elif False in verdicts:
                    r.note("=> this body's optics names cannot be set from "
                           "software; the display is driven by a "
                           "hardware-sensed Code")
                else:
                    r.note("=> undeterminable from type info; ask before "
                           "attempting a real write")

        # ── Filter cubes ───────────────────────────────────────────
        print("\nFILTER CUBES — track and switch")
        start_cube = ctrl.state().filter_position
        r.check("current cube is tracked", start_cube is not None, True)
        labels = store.filter_labels()
        r.note(f"assigned names: {labels or '(none assigned yet)'}")
        for target in range(1, min(ctrl.state().filter_count, 6) + 1):
            idx = panel._filter_combo.findData(target)
            if idx < 0:
                continue
            panel._filter_combo.setCurrentIndex(idx)
            settle()
            r.check(f"switch to slot {target} "
                    f"({labels.get(target, 'unnamed')})",
                    ctrl.state().filter_position, target)
        if start_cube is not None:
            panel._filter_combo.setCurrentIndex(
                panel._filter_combo.findData(start_cube))
            settle()
            r.check("restored the starting cube",
                    ctrl.state().filter_position, start_cube)

        # ── Objectives ─────────────────────────────────────────────
        print("\nOBJECTIVES — track and switch")
        start_obj = ctrl.state().objective_position
        r.check("current objective is tracked", start_obj is not None, True)
        obj_labels = store.objective_labels()
        r.note(f"assigned names: {obj_labels or '(none assigned yet)'}")
        for target in range(1, min(ctrl.state().objective_count, 6) + 1):
            idx = panel._objective_combo.findData(target)
            if idx < 0:
                continue
            panel._objective_combo.setCurrentIndex(idx)
            settle()
            r.check(f"switch to position {target} "
                    f"({obj_labels.get(target, 'unnamed')})",
                    ctrl.state().objective_position, target)
        if start_obj is not None:
            panel._objective_combo.setCurrentIndex(
                panel._objective_combo.findData(start_obj))
            settle()
            r.check("restored the starting objective",
                    ctrl.state().objective_position, start_obj)

        # ── Focus ──────────────────────────────────────────────────
        print("\nFOCUS")
        start_focus = ctrl.state().focus_um
        r.check("focus position is readable", start_focus is not None, True)
        if start_focus is None:
            pass
        elif not args.focus:
            r.note("focus MOVES SKIPPED (pass --focus to enable)")
        else:
            step = abs(args.step)
            r.note(f"moving ±{step} µm from {start_focus:.2f} µm — "
                   f"the start position is restored afterwards")
            panel._step_spin.setValue(step)
            panel._btn_up.click()
            settle()
            up_delta = ctrl.state().focus_um - start_focus
            r.check(f"'up' moved {step} µm", round(up_delta, 3), round(step, 3))
            r.note("CHECK THE EYEPIECE/CAMERA: did the focal plane move the "
                   "way 'up' should? If not, clear the focus-direction box in "
                   "Microscope Setup.")
            panel._btn_down.click()
            settle()
            r.check("'down' returned to the start",
                    round(ctrl.state().focus_um - start_focus, 3), 0.0)

            target = start_focus + step * 2
            panel._goto_spin.setValue(target)
            panel._btn_goto.click()
            settle()
            r.check("absolute go-to", round(ctrl.state().focus_um, 2),
                    round(target, 2))
            r.note(f"MEASURE THIS MOVE ({step * 2} µm commanded). If the real "
                   f"travel differs by a constant factor, that factor is "
                   f"'Z units per µm' (currently "
                   f"{store.get('z_units_per_um')}).")

            panel._goto_spin.setValue(start_focus)
            panel._btn_goto.click()
            settle()
            r.check("restored the starting focus",
                    round(ctrl.state().focus_um, 2), round(start_focus, 2))

        # ── Robustness ─────────────────────────────────────────────
        print("\nROBUSTNESS")
        op = ctrl.set_filter(999)
        ctrl.wait_idle(timeout=30.0)
        r.check("an impossible slot is reported, not crashed",
                op.error is not None, True)
        settle()
        if start_cube is None:
            # Don't claim a pass off an unknown baseline — that would have read
            # as "verified" when the turret was never readable to begin with.
            r.note("turret-unmoved check SKIPPED: position was never readable")
        else:
            r.check("the turret did not move",
                    ctrl.state().filter_position, start_cube)
            # Belt and braces: never leave the body somewhere the check put it.
            if ctrl.state().filter_position != start_cube:
                ctrl.set_filter(start_cube)
                settle()
                r.note(f"restored the cube to slot {start_cube}")

        print("\nDISCONNECT")
        panel._toggle_connect()
        settle()
        r.check("disconnected", ctrl.state().connected, False)
    finally:
        ctrl.shutdown()
        if tmp is not None:
            tmp.cleanup()

    print(f"\n=== {len(r.passed)} passed, {len(r.failed)} failed ===")
    for f in r.failed:
        print(f"  FAILED: {f}")
    return 1 if r.failed else 0


if __name__ == "__main__":
    sys.exit(main())
