"""Action recorder — capture what an operator DOES in the GUI, as material for
tutorial videos (see docs/videos/DIRECTING.md).

Armed from the ● REC button in the top bar. While armed, a QApplication event
filter writes one row per action to ``logs/sessions/<stamp>/tape.jsonl`` and
grabs the window at each step.

The row that matters is ``rect_pct`` — the clicked widget's rectangle as a
percentage of the window. Tutorial spotlights are placed FROM that number
instead of being eyeballed against a screenshot, which is where placement
errors come from.

Two deliberate choices:

* The screen is grabbed BEFORE the click is delivered, so a step reads
  "here is the screen, here is the thing to click". Each step's shot is
  therefore also the previous step's result.
* ``Mark`` (F9) does not record an action — it flags the *most recent* step as
  a teaching beat. The operator demonstrates naturally and editorialises with
  one key, rather than directing while working.

The filter is installed only while armed and removed on stop, so a normal
session carries no overhead.
"""
from __future__ import annotations

import io
import json
import logging
import os
import time
from datetime import datetime

from PySide6.QtCore import QEvent, QObject, Qt, Signal
from PySide6.QtWidgets import QApplication, QAbstractButton, QWidget

logger = logging.getLogger(__name__)

# Flags the most recent step as a teaching beat.
MARK_KEY = Qt.Key_F9

# Widget classes that are never a useful teaching target on their own — we walk
# up to the nearest ancestor that names itself.
_ANONYMOUS = {"QWidget", "QFrame", "QScrollArea", "QStackedWidget", "QSplitter"}


def _widget_label(w: QWidget) -> str:
    """Best human-readable name for a widget: its text, else its tooltip."""
    for attr in ("text", "title", "toolTip"):
        try:
            val = getattr(w, attr, None)
            if callable(val):
                got = val()
                if got and isinstance(got, str):
                    return " ".join(got.split())[:120]
        except Exception:
            pass
    return ""


def _meaningful(w: QWidget) -> QWidget:
    """Walk up until the widget names itself (objectName, text, or a button)."""
    node, hops = w, 0
    while node is not None and hops < 6:
        named = bool(node.objectName())
        titled = bool(_widget_label(node))
        clickable = isinstance(node, QAbstractButton)
        if clickable or named or (titled and
                                  node.metaObject().className() not in _ANONYMOUS):
            return node
        node = node.parentWidget()
        hops += 1
    return w


class ActionRecorder(QObject):
    """Records GUI actions to a session directory. Inert until ``start()``."""

    state_changed = Signal(bool)   # armed / not
    step_recorded = Signal(int)    # running step count

    def __init__(self, window, root_dir: str | None = None, parent=None):
        super().__init__(parent)
        self._window = window
        self._root = root_dir or os.path.join("logs", "sessions")
        self._armed = False
        self._steps: list[dict] = []
        self._session_dir = ""
        self._t0 = 0.0
        self._capture_screens = True

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

    # ── lifecycle ────────────────────────────────────────────────

    def start(self, capture_screens: bool = True) -> str:
        if self._armed:
            return self._session_dir
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._session_dir = os.path.join(self._root, stamp)
        os.makedirs(self._session_dir, exist_ok=True)
        self._steps = []
        self._t0 = time.time()
        self._capture_screens = capture_screens
        self._armed = True

        app = QApplication.instance()
        if app is not None:
            app.installEventFilter(self)
        logger.info("Action recorder armed -> %s", self._session_dir)
        self.state_changed.emit(True)
        return self._session_dir

    def stop(self) -> str:
        if not self._armed:
            return self._session_dir
        app = QApplication.instance()
        if app is not None:
            app.removeEventFilter(self)
        self._armed = False
        path = self._write_tape()
        logger.info("Action recorder stopped — %d step(s) -> %s",
                    len(self._steps), path)
        self.state_changed.emit(False)
        return path

    def toggle(self) -> bool:
        self.stop() if self._armed else self.start()
        return self._armed

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
        # Only the top-level window we were given — ignore other windows.
        if obj.window() is not self._window:
            return

        target = _meaningful(obj)
        rect_pct = self._rect_pct(target)
        if rect_pct is None:
            return

        step = {
            "i": len(self._steps) + 1,
            "t": round(time.time() - self._t0, 3),
            "kind": "click",
            "widget": target.metaObject().className(),
            "object_name": target.objectName(),
            "label": _widget_label(target),
            "rect_pct": rect_pct,
            "page": self._page_title(),
            "sub_page": self._sub_page_title(),
            "marked": False,
        }
        if self._capture_screens:
            step["screen"] = self._grab(step["i"])
        self._steps.append(step)
        self.step_recorded.emit(len(self._steps))

    def _mark_last(self) -> None:
        """F9 flags the most recent step as a teaching beat."""
        if self._steps:
            self._steps[-1]["marked"] = True
            logger.info("marked step %d (%s)", self._steps[-1]["i"],
                        self._steps[-1].get("label") or
                        self._steps[-1].get("object_name") or "?")

    # ── helpers ──────────────────────────────────────────────────

    def _rect_pct(self, w: QWidget):
        """Widget rect as % of the window — resolution-independent, which is
        what a tutorial spotlight needs."""
        try:
            win = self._window
            ww, wh = win.width(), win.height()
            if ww <= 0 or wh <= 0 or w.width() <= 0 or w.height() <= 0:
                return None
            top_left = w.mapTo(win, w.rect().topLeft())
            return {
                "left": round(100.0 * top_left.x() / ww, 3),
                "top": round(100.0 * top_left.y() / wh, 3),
                "width": round(100.0 * w.width() / ww, 3),
                "height": round(100.0 * w.height() / wh, 3),
            }
        except Exception:
            return None

    def _grab(self, index: int) -> str:
        name = "step-%04d.png" % index
        try:
            self._window.grab().save(os.path.join(self._session_dir, name))
            return name
        except Exception:
            logger.debug("action recorder: grab failed", exc_info=True)
            return ""

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

    def _write_tape(self) -> str:
        path = os.path.join(self._session_dir, "tape.jsonl")
        header = {
            "kind": "session",
            "created": datetime.now().isoformat(timespec="seconds"),
            "window": {"w": self._window.width(), "h": self._window.height()},
            "steps": len(self._steps),
            "marked": sum(1 for s in self._steps if s.get("marked")),
        }
        with io.open(path, "w", encoding="utf-8") as fh:
            fh.write(json.dumps(header) + "\n")
            for step in self._steps:
                fh.write(json.dumps(step) + "\n")
        return path


def read_tape(path: str):
    """Return (header, steps) from a tape.jsonl. Tolerates a truncated final
    line so a tape from a crashed session is still usable."""
    header, steps = {}, []
    with io.open(path, encoding="utf-8") as fh:
        for n, line in enumerate(fh):
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except ValueError:
                continue                      # truncated tail — keep what we have
            if n == 0 and row.get("kind") == "session":
                header = row
            else:
                steps.append(row)
    return header, steps
