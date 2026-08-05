"""Action recorder — capture what an operator DOES in the GUI, as material for
tutorial videos (see docs/videos/DIRECTING.md).

Armed from the ● REC button in the top bar. While armed, a QApplication event
filter appends one event per action to ``logs/sessions/<stamp>/tape.jsonl``
(flushed per event — a crash loses at most the final partial line) and grabs a
composite screenshot at each step.

The rows that matter most:

* ``rect_pct`` — the clicked widget's rectangle as a percentage of the main
  window. Tutorial spotlights are placed FROM this number instead of being
  eyeballed against a screenshot, which is where placement errors come from.
* ``click_pct`` — the raw press point, always recorded, so a click on a large
  canvas (whose "meaningful" ancestor would be uselessly big) can still be
  spotlighted at the exact spot.

Deliberate choices:

* Clicks inside dialogs, combo popups and menus ARE recorded (an acceptance
  ladder walks parent/transient chains back to the main window) — MEBP's
  workflows live in ⚙ Settings dialogs, so a main-window-only recorder would
  miss the main use case. Each step notes which kind of window it came from.
* Screenshots are a COMPOSITE: the main window plus every accepted visible
  top-level painted at its true position (modal, then popup, last). A plain
  ``window.grab()`` renders only the window's own tree, so a dialog covering
  it would silently vanish from every frame.
* The screen is captured BEFORE the click is delivered, so a step reads
  "here is the screen, here is the thing to click" — and each step's shot is
  also the previous step's result. The grab itself is synchronous (the
  pre-click state is the point); only the PNG encode/save runs on a worker
  thread so the click's delivery isn't delayed by disk I/O.
* ``Mark`` (F9) does not record an action — it flags the *most recent* step
  as a teaching beat, written as its own event line (the step line is already
  on disk).

The filter is installed only while armed and removed on stop, so a normal
session carries no overhead. It never consumes an event and never lets a
capture failure escape — recording must not alter app behaviour.
"""
from __future__ import annotations

import logging
import os
import queue
import threading
import time
from datetime import datetime

from PySide6.QtCore import QEvent, QObject, QPoint, Qt, Signal
from PySide6.QtGui import QPainter, QPixmap
from PySide6.QtWidgets import (
    QApplication, QAbstractButton, QLineEdit, QPlainTextEdit, QTextEdit,
    QWidget,
)

from SupportClasses.WalkthroughTape import TapeWriter, read_tape  # noqa: F401
# read_tape re-exported for back-compat: tests and tools may import it from
# here, but the canonical home is SupportClasses.WalkthroughTape (GUI-free).

logger = logging.getLogger(__name__)

# Flags the most recent step as a teaching beat.
MARK_KEY = Qt.Key_F9

# A "meaningful" ancestor bigger than this fraction of the window is a
# container, not a control — MEBP names its cards/stacks ("componentCard",
# "pagesContainer"), and a half-window spotlight is useless.
_MAX_TARGET_AREA_FRAC = 0.25

# Widget classes that never name a useful teaching target on their own.
_ANONYMOUS = {"QWidget", "QFrame", "QScrollArea", "QStackedWidget", "QSplitter"}

# Typed content (paths, sample IDs) must not leak into the tape via text().
_TEXT_ENTRY = (QLineEdit, QTextEdit, QPlainTextEdit)

_BUTTON_NAMES = {
    Qt.LeftButton: "left",
    Qt.RightButton: "right",
    Qt.MiddleButton: "middle",
}


def _widget_label(w: QWidget) -> str:
    """Best human-readable name for a widget. Skips ``text()`` on text-entry
    widgets — that would echo whatever the operator typed into the tape."""
    attrs = ("title", "toolTip") if isinstance(w, _TEXT_ENTRY) \
        else ("text", "title", "toolTip")
    for attr in attrs:
        try:
            val = getattr(w, attr, None)
            if callable(val):
                got = val()
                if got and isinstance(got, str):
                    return " ".join(got.split())[:120]
        except Exception:
            pass
    return ""


class ActionRecorder(QObject):
    """Records GUI actions to a session directory. Inert until ``start()``."""

    state_changed = Signal(bool)        # armed / not
    step_recorded = Signal(int, int)    # (steps, marks)
    step_marked = Signal(int)           # step index that was flagged

    def __init__(self, window, root_dir: str | None = None, parent=None):
        super().__init__(parent)
        self._window = window
        self._root = root_dir or os.path.join("logs", "sessions")
        self._armed = False
        self._steps: list[dict] = []
        self._marks = 0
        self._session_dir = ""
        self._last_tape_path = ""
        self._t0 = 0.0
        self._capture_screens = True
        self._tape: TapeWriter | None = None
        self._ignored: list[QWidget] = []
        self._last_press_key = None
        self._save_queue: "queue.Queue[tuple]" = queue.Queue()
        self._save_worker: threading.Thread | None = None

    # ── state ────────────────────────────────────────────────────

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def session_dir(self) -> str:
        return self._session_dir

    @property
    def step_count(self) -> int:
        return len(self._steps)

    @property
    def mark_count(self) -> int:
        return self._marks

    def ignore_widget(self, w: QWidget) -> None:
        """Never record clicks on ``w`` or its descendants. The app registers
        the ● REC button itself here — otherwise every tape would end with a
        junk 'clicked the stop button' step (the press arrives while still
        armed; stop fires on release)."""
        if w is not None and w not in self._ignored:
            self._ignored.append(w)

    # ── lifecycle ────────────────────────────────────────────────

    def start(self, capture_screens: bool = True) -> str:
        if self._armed:
            return self._session_dir
        self._session_dir = self._new_session_dir()
        os.makedirs(self._session_dir, exist_ok=True)
        self._steps = []
        self._marks = 0
        self._t0 = time.time()
        self._capture_screens = capture_screens

        try:
            dpr = float(self._window.devicePixelRatioF())
        except Exception:
            dpr = 1.0
        self._tape = TapeWriter(
            os.path.join(self._session_dir, "tape.jsonl"),
            window_w=self._window.width(), window_h=self._window.height(),
            dpr=dpr, capture="composite" if capture_screens else "none",
            created=datetime.now().isoformat(timespec="seconds"))

        if capture_screens:
            self._save_worker = threading.Thread(
                target=self._save_loop, daemon=True, name="rec-png-save")
            self._save_worker.start()

        self._armed = True
        app = QApplication.instance()
        if app is not None:
            app.installEventFilter(self)
        logger.info("Action recorder armed -> %s", self._session_dir)
        self.state_changed.emit(True)
        return self._session_dir

    def stop(self) -> str:
        """Disarm and finalize. Always returns the tape path ('' if this
        recorder never produced one)."""
        if not self._armed:
            return self._last_tape_path
        app = QApplication.instance()
        if app is not None:
            app.removeEventFilter(self)
        self._armed = False

        self._drain_saves(timeout_s=2.0)
        path = ""
        if self._tape is not None:
            try:
                path = self._tape.close(t=time.time() - self._t0,
                                        steps=len(self._steps),
                                        marked=self._marks)
            except Exception:
                logger.debug("tape close failed", exc_info=True)
                path = self._tape.path
            self._tape = None
        self._last_tape_path = path
        logger.info("Action recorder stopped — %d step(s), %d mark(s) -> %s",
                    len(self._steps), self._marks, path)
        self.state_changed.emit(False)
        return path

    def toggle(self) -> bool:
        self.stop() if self._armed else self.start()
        return self._armed

    def _new_session_dir(self) -> str:
        """Timestamped dir, uniquified — a stop + immediate re-arm within the
        same second must not interleave two takes in one directory."""
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self._root, stamp)
        path, n = base, 1
        while os.path.exists(path):
            n += 1
            path = "%s-%d" % (base, n)
        return path

    # ── capture ──────────────────────────────────────────────────

    def eventFilter(self, obj, event):        # noqa: N802  (Qt naming)
        """Never consumes the event — recording must not alter behaviour."""
        if not self._armed:
            return False
        try:
            etype = event.type()
            if etype == QEvent.MouseButtonPress:
                self._record_click(obj, event)
            elif etype == QEvent.KeyPress and event.key() == MARK_KEY:
                self._mark_last()
        except Exception:                      # a recorder must never break the app
            logger.debug("action recorder: capture failed", exc_info=True)
        return False

    def _record_click(self, obj, event) -> None:
        if not isinstance(obj, QWidget) or self._window is None:
            return

        # One physical press can be delivered several times: Qt propagates an
        # unhandled mouse press up the parent chain, and an application-level
        # filter sees every hop. Without this, clicking any non-interactive
        # widget (a label, a card background) records duplicate steps and
        # duplicate screenshots. Key on the event's own timestamp + global
        # position, which are identical across the propagated deliveries.
        try:
            gp = event.globalPosition().toPoint()
            key = (int(event.timestamp()), gp.x(), gp.y())
            if key == self._last_press_key:
                return
            self._last_press_key = key
        except Exception:
            pass

        for ignored in self._ignored:
            try:
                if obj is ignored or ignored.isAncestorOf(obj):
                    return
            except Exception:
                pass

        top = obj.window()
        window_kind = self._window_kind(top)
        if window_kind is None:
            # Unrelated top-level (stray tool window) — visible loss, not silent.
            logger.debug("recorder: click outside app windows ignored (%s)",
                         top.__class__.__name__ if top else "?")
            return

        target = self._meaningful(obj)
        rect_pct, out_of_frame, oversized = self._rect_pct(target)
        click_pct = self._click_pct(obj, event)

        step = {
            "kind": "click",
            "i": len(self._steps) + 1,
            "t": round(time.time() - self._t0, 3),
            "widget": target.metaObject().className(),
            "object_name": target.objectName(),
            "label": _widget_label(target),
            "button": _BUTTON_NAMES.get(event.button(), "other"),
            "rect_pct": rect_pct,
            "click_pct": click_pct,
            "out_of_frame": out_of_frame,
            "rect_oversized": oversized,
            "window_kind": window_kind,
            "top_level": {
                "class": top.metaObject().className(),
                "name": top.objectName(),
                "title": (top.windowTitle() or "")[:120],
            },
            "win": {"w": self._window.width(), "h": self._window.height()},
            "page": self._page_title(),
            "sub_page": self._sub_page_title(),
            "marked": False,
        }
        if self._capture_screens:
            step["screen"] = self._grab(step["i"])
        else:
            step["screen"] = ""

        self._steps.append(step)
        if self._tape is not None:
            try:
                self._tape.write_step(step)
            except Exception:
                logger.debug("tape write failed", exc_info=True)
        self.step_recorded.emit(len(self._steps), self._marks)

    def _mark_last(self) -> None:
        """F9 flags the most recent step as a teaching beat — written as its
        own event line, since the step's line is already on disk."""
        if not self._steps:
            logger.info("F9: nothing to mark yet — click something first")
            return
        step = self._steps[-1]
        if not step["marked"]:
            step["marked"] = True
            self._marks += 1
            if self._tape is not None:
                try:
                    self._tape.write_mark(step["i"],
                                          time.time() - self._t0)
                except Exception:
                    logger.debug("mark write failed", exc_info=True)
        logger.info("marked step %d (%s)", step["i"],
                    step.get("label") or step.get("object_name") or "?")
        self.step_marked.emit(step["i"])

    # ── window acceptance ladder ─────────────────────────────────

    def _window_kind(self, top: QWidget | None) -> str | None:
        """Which of our windows does this top-level belong to?

        main window → "main"; a QDialog / combo popup / menu whose parent or
        transient chain reaches the main window → "dialog"/"popup"; anything
        else → None (rejected). MEBP's workflows configure themselves in
        ⚙ Settings QDialogs, so recording only the main window would miss the
        recorder's primary use case entirely.
        """
        if top is None:
            return None
        if top is self._window:
            return "main"

        # Mask before comparing: Qt.Dialog (0x3) and Qt.Popup (0x9) share the
        # Qt.Window bit, so a bare `flags & Qt.Popup` is truthy for EVERY
        # dialog and would label them all "popup".
        is_popup = (top.windowFlags() & Qt.WindowType_Mask) == Qt.Popup

        # Parent chain: QDialog(parent=page) and QComboBox popup containers
        # both have parentWidget() chains that reach the main window even
        # though they are top-level windows.
        node, hops = top.parentWidget(), 0
        while node is not None and hops < 8:
            if node.window() is self._window:
                return "popup" if is_popup else "dialog"
            node = node.parentWidget()
            hops += 1

        # Transient chain: parentless dialogs shown modally get a transient
        # parent from Qt.
        try:
            wh = top.windowHandle()
            main_wh = self._window.windowHandle()
            hops = 0
            while wh is not None and hops < 8:
                tp = wh.transientParent()
                if tp is not None and tp is main_wh:
                    return "popup" if is_popup else "dialog"
                wh = tp
                hops += 1
        except Exception:
            pass

        app = QApplication.instance()
        if app is not None:
            if top is app.activePopupWidget():
                return "popup"
            if top is app.activeModalWidget():
                return "dialog"
        return None

    # ── geometry ─────────────────────────────────────────────────

    def _meaningful(self, w: QWidget) -> QWidget:
        """Walk up to the nearest widget that names itself — but reject
        container-sized ancestors. MEBP names its cards ("componentCard") and
        stacks ("pagesContainer"); resolving a canvas click to a half-window
        card produces a useless spotlight. A too-big ancestor is skipped and
        the walk continues; if nothing sane names itself, the originally
        clicked widget is the honest answer (click_pct still pinpoints the
        spot)."""
        win_area = max(1, self._window.width() * self._window.height())
        node, hops = w, 0
        while node is not None and hops < 6:
            named = bool(node.objectName())
            titled = bool(_widget_label(node))
            clickable = isinstance(node, QAbstractButton)
            if clickable or named or (
                    titled and
                    node.metaObject().className() not in _ANONYMOUS):
                area_frac = (node.width() * node.height()) / win_area
                if node is w or area_frac <= _MAX_TARGET_AREA_FRAC:
                    return node
                # Container-sized ancestor: keep walking; a higher-up control
                # is impossible (they only get bigger), so effectively this
                # falls through to the clicked widget.
            node = node.parentWidget()
            hops += 1
        return w

    def _rect_pct(self, w: QWidget):
        """Target rect as % of the MAIN window (via global coords, so dialog
        widgets land in the same coordinate space as the video frame).
        Returns (rect|None, out_of_frame, oversized). Never raises."""
        try:
            win = self._window
            ww, wh = win.width(), win.height()
            if ww <= 0 or wh <= 0 or w.width() <= 0 or w.height() <= 0:
                return None, False, False
            origin = win.mapToGlobal(QPoint(0, 0))
            tl = w.mapToGlobal(QPoint(0, 0))
            left = 100.0 * (tl.x() - origin.x()) / ww
            top = 100.0 * (tl.y() - origin.y()) / wh
            width = 100.0 * w.width() / ww
            height = 100.0 * w.height() / wh

            raw_right, raw_bottom = left + width, top + height
            out = (left < 0 or top < 0 or raw_right > 100 or raw_bottom > 100)
            # Clamp into frame space; the flag records that we did.
            c_left = min(100.0, max(0.0, left))
            c_top = min(100.0, max(0.0, top))
            c_right = min(100.0, max(0.0, raw_right))
            c_bottom = min(100.0, max(0.0, raw_bottom))
            rect = {
                "left": round(c_left, 3),
                "top": round(c_top, 3),
                "width": round(max(0.0, c_right - c_left), 3),
                "height": round(max(0.0, c_bottom - c_top), 3),
            }
            oversized = (width * height) > (100.0 * 100.0 *
                                            _MAX_TARGET_AREA_FRAC)
            return rect, out, oversized
        except Exception:
            return None, False, False

    def _click_pct(self, obj: QWidget, event):
        """The raw press point as % of the main window — always recorded, so
        a click inside a large canvas is recoverable even when no small named
        ancestor exists."""
        try:
            gp = event.globalPosition().toPoint()
            local = self._window.mapFromGlobal(gp)
            ww = max(1, self._window.width())
            wh = max(1, self._window.height())
            return {
                "x": round(min(100.0, max(0.0, 100.0 * local.x() / ww)), 3),
                "y": round(min(100.0, max(0.0, 100.0 * local.y() / wh)), 3),
            }
        except Exception:
            return None

    # ── screenshots ──────────────────────────────────────────────

    def _grab(self, index: int) -> str:
        """Composite screenshot: main window + every accepted visible
        top-level painted at its true position (modal, then popup, last —
        approximating z-order). QWidget.grab() renders only its own tree, so
        without the composite a dialog covering the window would silently
        vanish from every frame. The grab is synchronous (pre-click state is
        the point); PNG encode/save happens on the worker thread."""
        name = "step-%04d.png" % index
        try:
            win = self._window
            dpr = float(win.devicePixelRatioF())
            canvas = QPixmap(int(win.width() * dpr), int(win.height() * dpr))
            canvas.setDevicePixelRatio(dpr)
            canvas.fill(Qt.black)

            painter = QPainter(canvas)
            try:
                painter.drawPixmap(0, 0, win.grab())
                origin = win.mapToGlobal(QPoint(0, 0))
                app = QApplication.instance()
                modal = app.activeModalWidget() if app else None
                popup = app.activePopupWidget() if app else None
                others, tail = [], []
                for tl in (QApplication.topLevelWidgets() if app else []):
                    if tl is win or not tl.isVisible():
                        continue
                    if self._window_kind(tl) is None:
                        continue
                    (tail if tl in (modal, popup) else others).append(tl)
                # Modal above ordinary dialogs; popup above everything.
                tail.sort(key=lambda t: 1 if t is popup else 0)
                for tl in others + tail:
                    pos = tl.mapToGlobal(QPoint(0, 0))
                    painter.drawPixmap(pos.x() - origin.x(),
                                       pos.y() - origin.y(), tl.grab())
            finally:
                painter.end()

            # Encode/save off-thread: QPixmap isn't thread-safe, QImage is.
            self._save_queue.put((canvas.toImage(),
                                  os.path.join(self._session_dir, name)))
            return name
        except Exception:
            logger.debug("action recorder: grab failed", exc_info=True)
            return ""

    def _save_loop(self) -> None:
        while True:
            item = self._save_queue.get()
            if item is None:
                return
            image, path = item
            try:
                image.save(path)
            except Exception:
                logger.debug("png save failed: %s", path, exc_info=True)

    def _drain_saves(self, timeout_s: float) -> None:
        worker = self._save_worker
        if worker is None:
            return
        self._save_queue.put(None)
        worker.join(timeout=timeout_s)
        if worker.is_alive():
            logger.warning("recorder: PNG saves still draining at stop")
        self._save_worker = None

    # ── app context ──────────────────────────────────────────────

    def _page_title(self) -> str:
        lbl = getattr(self._window, "_page_title", None)
        try:
            return lbl.text() if lbl is not None else ""
        except Exception:
            return ""

    def _sub_page_title(self) -> str:
        try:
            stack = getattr(self._window, "_page_stack", None)
            if stack is None:
                return ""
            page = stack.currentWidget()
            getter = getattr(page, "get_sub_page_title", None)
            return getter() if callable(getter) else ""
        except Exception:
            return ""
