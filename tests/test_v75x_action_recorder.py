# -*- coding: utf-8 -*-
"""Tutorial walkthrough recorder (gui/action_recorder.py), the tape format
(SupportClasses/WalkthroughTape.py) and the tape → tour builder
(tools_build_tour_from_tape.py).

The behaviours pinned here are the ones an adversarial review found broken or
unproven. Each test name states the invariant; the docstrings say why it
matters, because several look arbitrary until you know the failure.
"""
import io
import json
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QDialog, QLineEdit,
    QFrame,
)
from PySide6.QtCore import Qt, QEvent, QPointF
from PySide6.QtGui import QMouseEvent, QKeyEvent

from gui.action_recorder import ActionRecorder, MARK_KEY
from SupportClasses.WalkthroughTape import read_tape

_app = QApplication.instance() or QApplication([])


class _Host(QWidget):
    """Stand-in for MainWindow: the recorder only needs a top-level widget."""

    def __init__(self):
        super().__init__()
        self.resize(1000, 500)
        lay = QVBoxLayout(self)
        self.btn = QPushButton("Connect")
        self.btn.setObjectName("btn_connect")
        self.icon_btn = QPushButton("\U0001F4D0")   # emoji-only, like the nav rail
        self.icon_btn.setObjectName("btn_calibrate")
        self.field = QLineEdit()
        self.field.setObjectName("sample_id")
        # A named container far bigger than the 25% area guard, holding an
        # unnamed child — the "click a canvas, get a half-window card" shape.
        self.card = QWidget(self)
        self.card.setObjectName("componentCard")
        self.card.setFixedSize(900, 400)
        self.canvas = QWidget(self.card)          # unnamed, no text
        self.canvas.setFixedSize(880, 380)
        lay.addWidget(self.btn)
        lay.addWidget(self.icon_btn)
        lay.addWidget(self.field)
        lay.addWidget(self.card)
        self.show()
        _app.processEvents()


def _click(widget, button=Qt.LeftButton):
    pos = QPointF(widget.rect().center())
    glob = QPointF(widget.mapToGlobal(widget.rect().center()))
    for etype in (QEvent.MouseButtonPress, QEvent.MouseButtonRelease):
        _app.sendEvent(widget, QMouseEvent(etype, pos, glob, button, button,
                                           Qt.NoModifier))
    _app.processEvents()


def _press(target, key):
    _app.sendEvent(target, QKeyEvent(QEvent.KeyPress, key, Qt.NoModifier))
    _app.processEvents()


def _raw_lines(session_dir):
    with io.open(os.path.join(session_dir, "tape.jsonl"), encoding="utf-8") as f:
        return [json.loads(ln) for ln in f if ln.strip()]


class _RecorderCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="rec_test_")
        self.host = _Host()
        self.rec = ActionRecorder(self.host, root_dir=self.tmp)

    def tearDown(self):
        if self.rec.armed:
            self.rec.stop()
        self.host.close()
        shutil.rmtree(self.tmp, ignore_errors=True)


class TestLifecycle(_RecorderCase):

    def test_inert_until_armed_and_filter_removed_on_stop(self):
        _click(self.host.btn)
        self.assertEqual(self.rec.step_count, 0)
        self.rec.start(capture_screens=False)
        _click(self.host.btn)
        self.assertEqual(self.rec.step_count, 1)
        self.rec.stop()
        _click(self.host.btn)
        self.assertEqual(self.rec.step_count, 1,
                         "filter must be removed on stop")

    def test_filter_never_consumes_the_event(self):
        """Recording must be unobservable in app behaviour."""
        self.rec.start(capture_screens=False)
        fired = []
        self.host.btn.clicked.connect(lambda: fired.append(1))
        _click(self.host.btn)
        self.assertEqual(fired, [1], "recording must not swallow the click")

    def test_capture_failure_does_not_propagate(self):
        self.rec.start(capture_screens=False)
        self.rec._record_click = lambda *a, **k: (_ for _ in ()).throw(
            RuntimeError("boom"))
        _click(self.host.btn)               # must not raise

    def test_same_second_restart_gets_a_distinct_directory(self):
        """Two takes inside one second must not interleave into one dir —
        the second stop() would otherwise truncate the first tape."""
        self.rec.start(capture_screens=False)
        first = self.rec.session_dir
        self.rec.stop()
        self.rec.start(capture_screens=False)
        self.assertNotEqual(self.rec.session_dir, first)
        self.rec.stop()

    def test_stop_before_start_returns_empty_string(self):
        """stop() always returns a tape path; it used to return a directory
        when unarmed, which the app prints into a copy-pasteable command."""
        self.assertEqual(self.rec.stop(), "")


class TestGeometry(_RecorderCase):

    def test_click_captures_rect_and_point_as_percent(self):
        self.rec.start(capture_screens=False)
        _click(self.host.btn)
        self.rec.stop()
        _h, steps = read_tape(os.path.join(self.rec.session_dir, "tape.jsonl"))
        self.assertEqual(len(steps), 1)
        rect = steps[0]["rect_pct"]
        for key in ("left", "top", "width", "height"):
            self.assertGreaterEqual(rect[key], 0.0)
            self.assertLessEqual(rect[key], 100.0)
        self.assertGreater(rect["width"], 50.0)     # wide button in a 1000px host
        self.assertEqual(steps[0]["object_name"], "btn_connect")
        click = steps[0]["click_pct"]
        self.assertTrue(0.0 <= click["x"] <= 100.0)
        self.assertTrue(0.0 <= click["y"] <= 100.0)

    def test_oversized_container_is_flagged_not_used_blindly(self):
        """Clicking an unnamed canvas inside a named half-window Card must
        not silently yield a half-window spotlight."""
        self.rec.start(capture_screens=False)
        _click(self.host.canvas)
        self.rec.stop()
        _h, steps = read_tape(os.path.join(self.rec.session_dir, "tape.jsonl"))
        step = steps[0]
        self.assertIsNotNone(step["click_pct"],
                             "click point is the fallback — must exist")
        if step["object_name"] == "componentCard":
            self.assertTrue(step["rect_oversized"],
                            "a container-sized target must be flagged")

    def test_degenerate_rect_still_records_the_step(self):
        """A zero-size target used to drop the click entirely — the operator
        saw the counter simply not advance, with no explanation."""
        tiny = QPushButton("x", self.host)
        tiny.setFixedSize(0, 0)
        tiny.show()
        _app.processEvents()
        self.rec.start(capture_screens=False)
        _click(tiny)
        self.rec.stop()
        _h, steps = read_tape(os.path.join(self.rec.session_dir, "tape.jsonl"))
        self.assertEqual(len(steps), 1)
        self.assertIsNone(steps[0]["rect_pct"])
        self.assertIsNotNone(steps[0]["click_pct"])

    def test_propagated_press_records_only_one_step(self):
        """Qt propagates an unhandled press up the parent chain and an
        app-level filter sees every hop — one physical click on a plain
        widget must still be exactly one step (and one screenshot)."""
        plain = QWidget(self.host)
        plain.setFixedSize(120, 40)
        plain.show()
        _app.processEvents()
        self.rec.start(capture_screens=False)
        _click(plain)
        self.rec.stop()
        _h, steps = read_tape(os.path.join(self.rec.session_dir, "tape.jsonl"))
        self.assertEqual(len(steps), 1)

    def test_button_kind_is_recorded(self):
        """A right-click taught as a left-click teaches the wrong gesture."""
        self.rec.start(capture_screens=False)
        _click(self.host.btn, button=Qt.RightButton)
        self.rec.stop()
        _h, steps = read_tape(os.path.join(self.rec.session_dir, "tape.jsonl"))
        self.assertEqual(steps[0]["button"], "right")


class TestWindowAcceptance(_RecorderCase):

    def test_dialog_click_is_recorded(self):
        """MEBP's workflows configure themselves in ⚙ Settings QDialogs. A
        main-window-only recorder misses the entire use case."""
        dlg = QDialog(self.host)
        dlg.setObjectName("settings_dialog")
        dlg_btn = QPushButton("Apply", dlg)
        dlg_btn.setObjectName("btn_apply")
        dlg.resize(300, 200)
        dlg.show()
        _app.processEvents()

        self.rec.start(capture_screens=False)
        _click(dlg_btn)
        self.rec.stop()
        _h, steps = read_tape(os.path.join(self.rec.session_dir, "tape.jsonl"))
        dlg.close()
        self.assertEqual(len(steps), 1, "dialog click must be recorded")
        self.assertEqual(steps[0]["object_name"], "btn_apply")
        self.assertEqual(steps[0]["window_kind"], "dialog")

    def test_popup_click_is_recorded(self):
        popup = QFrame(self.host, Qt.Popup)
        popup.setObjectName("combo_popup")
        item = QPushButton("24-well", popup)
        item.setObjectName("item_24well")
        popup.resize(200, 120)
        popup.show()
        _app.processEvents()

        self.rec.start(capture_screens=False)
        _click(item)
        self.rec.stop()
        _h, steps = read_tape(os.path.join(self.rec.session_dir, "tape.jsonl"))
        popup.close()
        self.assertEqual(len(steps), 1, "popup click must be recorded")
        self.assertEqual(steps[0]["window_kind"], "popup")

    def test_unrelated_window_is_ignored(self):
        other = QWidget()
        other.resize(200, 100)
        btn = QPushButton("elsewhere", other)
        other.show()
        _app.processEvents()

        self.rec.start(capture_screens=False)
        _click(btn)
        self.rec.stop()
        _h, steps = read_tape(os.path.join(self.rec.session_dir, "tape.jsonl"))
        other.close()
        self.assertEqual(len(steps), 0)

    def test_ignored_widget_is_never_recorded(self):
        """The REC button's own stop-click arrives while still armed."""
        self.rec.ignore_widget(self.host.btn)
        self.rec.start(capture_screens=False)
        _click(self.host.btn)
        _click(self.host.icon_btn)
        self.rec.stop()
        _h, steps = read_tape(os.path.join(self.rec.session_dir, "tape.jsonl"))
        self.assertEqual(len(steps), 1)
        self.assertEqual(steps[0]["object_name"], "btn_calibrate")


class TestMarking(_RecorderCase):

    def test_mark_flags_last_step_and_adds_none(self):
        self.rec.start(capture_screens=False)
        _click(self.host.btn)
        _click(self.host.icon_btn)
        _press(self.host, MARK_KEY)
        self.assertEqual(self.rec.step_count, 2, "F9 must not add a step")
        self.assertEqual(self.rec.mark_count, 1)
        self.rec.stop()
        _h, steps = read_tape(os.path.join(self.rec.session_dir, "tape.jsonl"))
        self.assertFalse(steps[0]["marked"])
        self.assertTrue(steps[1]["marked"])

    def test_mark_is_its_own_event_line(self):
        """The step's line is already flushed to disk when F9 arrives, so a
        mark cannot mutate it — it must be appended as its own event."""
        self.rec.start(capture_screens=False)
        _click(self.host.btn)
        _press(self.host, MARK_KEY)
        rows = _raw_lines(self.rec.session_dir)
        kinds = [r.get("kind") for r in rows]
        self.assertEqual(kinds, ["session", "click", "mark"])
        self.assertEqual(rows[2]["i"], 1)
        self.rec.stop()

    def test_mark_with_no_steps_is_safe(self):
        self.rec.start(capture_screens=False)
        _press(self.host, MARK_KEY)
        self.assertEqual(self.rec.step_count, 0)
        self.assertEqual(self.rec.mark_count, 0)

    def test_double_mark_counts_once(self):
        self.rec.start(capture_screens=False)
        _click(self.host.btn)
        _press(self.host, MARK_KEY)
        _press(self.host, MARK_KEY)
        self.assertEqual(self.rec.mark_count, 1)


class TestTapeDurability(_RecorderCase):

    def test_tape_is_readable_without_stop(self):
        """The whole point of incremental writing: a crash mid-recording must
        not lose the take. Previously everything buffered in RAM until stop."""
        self.rec.start(capture_screens=False)
        _click(self.host.btn)
        _click(self.host.icon_btn)
        _press(self.host, MARK_KEY)
        # No stop() — simulate the process dying here.
        header, steps = read_tape(
            os.path.join(self.rec.session_dir, "tape.jsonl"))
        self.assertEqual(len(steps), 2)
        self.assertTrue(steps[1]["marked"])
        self.assertEqual(header["steps"], 2, "counts computed without end line")
        self.assertEqual(header["marked"], 1)

    def test_end_line_written_on_stop(self):
        self.rec.start(capture_screens=False)
        _click(self.host.btn)
        self.rec.stop()
        rows = _raw_lines(self.rec.session_dir)
        self.assertEqual(rows[-1]["kind"], "end")
        self.assertEqual(rows[-1]["steps"], 1)

    def test_privacy_typed_text_is_not_captured(self):
        """QLineEdit.text() is whatever the operator typed — sample IDs,
        paths. It must not reach the tape (which the builder echoes into
        STORYBOARD.md and filenames)."""
        self.rec.start(capture_screens=False)
        self.host.field.setText("PATIENT-42-secret")
        _click(self.host.field)
        self.rec.stop()
        raw = io.open(os.path.join(self.rec.session_dir, "tape.jsonl"),
                      encoding="utf-8").read()
        self.assertNotIn("PATIENT-42-secret", raw)


class TestTapeFormat(unittest.TestCase):

    def test_read_tape_tolerates_truncated_tail(self):
        tmp = tempfile.mkdtemp(prefix="tape_fmt_")
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        path = os.path.join(tmp, "tape.jsonl")
        with io.open(path, "w", encoding="utf-8") as fh:
            fh.write(json.dumps({"kind": "session", "v": 2}) + "\n")
            fh.write(json.dumps({"kind": "click", "i": 1}) + "\n")
            fh.write('{"kind": "click", "i": 2, "lab')      # crashed mid-write
        header, steps = read_tape(path)
        self.assertEqual(len(steps), 1)
        self.assertEqual(header["steps"], 1)

    def test_v1_inline_marked_still_honoured(self):
        """Old tapes carried `marked` on the step itself."""
        tmp = tempfile.mkdtemp(prefix="tape_v1_")
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        path = os.path.join(tmp, "tape.jsonl")
        with io.open(path, "w", encoding="utf-8") as fh:
            fh.write(json.dumps({"kind": "session", "steps": 2,
                                 "marked": 1}) + "\n")
            fh.write(json.dumps({"kind": "click", "i": 1,
                                 "marked": False}) + "\n")
            fh.write(json.dumps({"kind": "click", "i": 2,
                                 "marked": True}) + "\n")
        _header, steps = read_tape(path)
        self.assertFalse(steps[0]["marked"])
        self.assertTrue(steps[1]["marked"])

    def test_unknown_kinds_are_skipped(self):
        tmp = tempfile.mkdtemp(prefix="tape_fwd_")
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        path = os.path.join(tmp, "tape.jsonl")
        with io.open(path, "w", encoding="utf-8") as fh:
            fh.write(json.dumps({"kind": "session"}) + "\n")
            fh.write(json.dumps({"kind": "future_thing", "x": 1}) + "\n")
            fh.write(json.dumps({"kind": "click", "i": 1}) + "\n")
        _header, steps = read_tape(path)
        self.assertEqual(len(steps), 1)


class TestScreenshots(_RecorderCase):

    def test_composite_grab_includes_an_overlapping_dialog(self):
        """window.grab() renders only the window's own tree, so a dialog
        covering it vanished from every frame. The composite must show it."""
        from PySide6.QtGui import QImage
        dlg = QDialog(self.host)
        dlg.setStyleSheet("background: #ff0000;")
        dlg.resize(400, 300)
        dlg.move(self.host.mapToGlobal(self.host.rect().topLeft()))
        dlg.show()
        _app.processEvents()

        self.rec.start(capture_screens=True)
        _click(self.host.btn)
        self.rec.stop()                     # drains the save queue
        dlg.close()

        shot = os.path.join(self.rec.session_dir, "step-0001.png")
        self.assertTrue(os.path.exists(shot), "PNG must be saved by stop()")
        img = QImage(shot)
        self.assertFalse(img.isNull())
        # Sample inside where the dialog was painted.
        px = img.pixelColor(int(40 * img.devicePixelRatio()),
                            int(40 * img.devicePixelRatio()))
        self.assertGreater(px.red(), 150, "dialog should be composited in")
        self.assertLess(px.green(), 100)

    def test_stop_drains_pending_saves(self):
        self.rec.start(capture_screens=True)
        _click(self.host.btn)
        _click(self.host.icon_btn)
        self.rec.stop()
        for i in (1, 2):
            self.assertTrue(os.path.exists(
                os.path.join(self.rec.session_dir, "step-%04d.png" % i)))


class TestRecButtonWiring(unittest.TestCase):
    """The REC button's live feedback, exercised by calling MainWindow's own
    handlers unbound against a fake self — the repo's established pattern,
    since a full MainWindow boot is slow and drags in camera enumeration."""

    def _fake_window(self):
        from gui.app import MainWindow

        class _Console:
            def __init__(self):
                self.lines = []

            def log(self, msg, level="info"):
                self.lines.append((level, msg))

        class _Fake:
            pass

        fake = _Fake()
        fake._rec_btn = QPushButton("● REC")
        fake.console = _Console()
        fake._action_recorder = None
        fake._on_recorder_step = MainWindow._on_recorder_step.__get__(fake)
        fake._on_recorder_mark = MainWindow._on_recorder_mark.__get__(fake)
        return fake

    def test_step_counter_and_mark_star_render(self):
        fake = self._fake_window()
        fake._on_recorder_step(3, 0)
        self.assertEqual(fake._rec_btn.text(), "● REC 3")
        fake._on_recorder_step(7, 2)
        self.assertEqual(fake._rec_btn.text(), "● REC 7 ★2",
                         "marks must be visible — F9 had no feedback at all")

    def test_mark_logs_to_the_console(self):
        """F9 used to give zero on-screen confirmation; the operator could not
        tell whether a mark landed until the tape was built, hours later."""
        fake = self._fake_window()

        class _Rec:
            step_count, mark_count = 4, 1
        fake._action_recorder = _Rec()
        fake._on_recorder_mark(4)
        self.assertEqual(fake._rec_btn.text(), "● REC 4 ★1")
        self.assertTrue(any("marked step 4" in m
                            for _lvl, m in fake.console.lines),
                        "a mark must be announced where the operator looks")

    def test_handler_wires_signals_and_ignores_the_button(self):
        """Guards the wiring that makes the two behaviours above reachable."""
        import ast
        import inspect
        from gui.app import MainWindow
        src = inspect.getsource(MainWindow._on_toggle_recording)
        tree = ast.parse(textwrap_dedent(src))
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
        attrs = {n.func.attr for n in calls if isinstance(n.func, ast.Attribute)}
        self.assertIn("ignore_widget", attrs)
        self.assertIn("connect", attrs)
        self.assertIn("setFixedWidth", attrs)
        self.assertIn("_on_recorder_mark", src)


def textwrap_dedent(src):
    import textwrap
    return textwrap.dedent(src)


class TestTourBuilder(unittest.TestCase):

    def _tape(self, tmp, marked_flags, **overrides):
        session = os.path.join(tmp, "20260101_000000")
        os.makedirs(session, exist_ok=True)
        path = os.path.join(session, "tape.jsonl")
        rect = {"left": 1.0, "top": 2.0, "width": 3.0, "height": 4.0}
        with io.open(path, "w", encoding="utf-8") as fh:
            fh.write(json.dumps({"kind": "session", "v": 2, "created": "x",
                                 "window": {"w": 1920, "h": 1080}}) + "\n")
            for i, marked in enumerate(marked_flags, start=1):
                row = {
                    "kind": "click", "i": i, "t": float(i),
                    "widget": "QPushButton", "object_name": "btn_calibrate",
                    "label": "\U0001F4D0",          # emoji-only, like the nav rail
                    "button": "left", "rect_pct": rect,
                    "click_pct": {"x": 50.0, "y": 50.0},
                    "out_of_frame": False, "rect_oversized": False,
                    "window_kind": "main", "page": "Jog Control",
                    "sub_page": "", "screen": "",
                }
                row.update(overrides)
                fh.write(json.dumps(row) + "\n")
                if marked:
                    fh.write(json.dumps({"kind": "mark", "i": i,
                                         "t": float(i)}) + "\n")
        return path

    def test_default_keeps_only_marked_steps(self):
        tmp = tempfile.mkdtemp(prefix="tour_test_")
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        tape = self._tape(tmp, [False, True, False, True])
        out = os.path.join(tmp, "tour")
        import tools_build_tour_from_tape as builder
        builder.main([tape, "-o", out])
        spots = json.load(io.open(os.path.join(out, "spotlights.json"),
                                  encoding="utf-8"))
        self.assertEqual(len(spots["beats"]), 2)
        sb = io.open(os.path.join(out, "STORYBOARD.md"),
                     encoding="utf-8").read()
        self.assertIn("4 step(s) captured, 2 kept", sb)

    def test_all_steps_keeps_everything(self):
        tmp = tempfile.mkdtemp(prefix="tour_test_")
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        tape = self._tape(tmp, [False, True, False])
        out = os.path.join(tmp, "tour")
        import tools_build_tour_from_tape as builder
        builder.main([tape, "-o", out, "--all-steps"])
        spots = json.load(io.open(os.path.join(out, "spotlights.json"),
                                  encoding="utf-8"))
        self.assertEqual(len(spots["beats"]), 3)

    def test_refuses_to_overwrite_director_edits(self):
        """A filled-in NARRATION.md is ~40 minutes of the director's work."""
        tmp = tempfile.mkdtemp(prefix="tour_test_")
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        tape = self._tape(tmp, [True])
        out = os.path.join(tmp, "tour")
        import tools_build_tour_from_tape as builder
        builder.main([tape, "-o", out])
        io.open(os.path.join(out, "NARRATION.md"), "w",
                encoding="utf-8").write("MY PRECIOUS NARRATION")
        with self.assertRaises(SystemExit):
            builder.main([tape, "-o", out])
        self.assertIn("MY PRECIOUS NARRATION",
                      io.open(os.path.join(out, "NARRATION.md"),
                              encoding="utf-8").read())
        builder.main([tape, "-o", out, "--force"])      # explicit opt-in works

    def test_oversized_rect_falls_back_to_click_point(self):
        tmp = tempfile.mkdtemp(prefix="tour_test_")
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        tape = self._tape(tmp, [True], rect_oversized=True,
                          rect_pct={"left": 0, "top": 0,
                                    "width": 90, "height": 80},
                          click_pct={"x": 25.0, "y": 60.0})
        out = os.path.join(tmp, "tour")
        import tools_build_tour_from_tape as builder
        builder.main([tape, "-o", out])
        spots = json.load(io.open(os.path.join(out, "spotlights.json"),
                                  encoding="utf-8"))
        beat = spots["beats"][0]
        self.assertEqual(beat["spot_source"], "click")
        self.assertLess(beat["spot"]["width"], 20.0)
        self.assertAlmostEqual(
            beat["spot"]["left"] + beat["spot"]["width"] / 2.0, 25.0, places=1)

    def test_icon_only_control_falls_back_to_object_name(self):
        import tools_build_tour_from_tape as builder
        step = {"label": "\U0001F4D0", "object_name": "btn_calibrate",
                "widget": "QPushButton"}
        self.assertEqual(builder._display(step), "btn_calibrate")
        self.assertEqual(builder._slug(step), "btn-calibrate")
        self.assertEqual(
            builder._display({"label": "Connect", "object_name": "b"}),
            "Connect")

    def test_numeric_label_is_kept(self):
        """'10x' / '96' / '1' are real control labels — they were being
        discarded in favour of the objectName."""
        import tools_build_tour_from_tape as builder
        self.assertEqual(
            builder._display({"label": "10", "object_name": "componentCard"}),
            "10")

    def test_builder_does_not_need_pyside(self):
        """It is a text transform; it must run where Qt is not installed."""
        import ast
        src = io.open(os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "tools_build_tour_from_tape.py"), encoding="utf-8").read()
        tree = ast.parse(src)
        names = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names += [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names.append(node.module or "")
        self.assertFalse([n for n in names if "PySide6" in n or
                          n.startswith("gui")],
                         "builder must not import Qt or gui.*: %s" % names)


if __name__ == "__main__":
    unittest.main(verbosity=2)
