"""WalkthroughTape — the tape.jsonl format shared by the tutorial action
recorder (gui/action_recorder.py, the writer) and the tape → storyboard
builder (tools_build_tour_from_tape.py, the reader).

GUI-free on purpose: the builder is a text transform and must not drag
PySide6 in just to parse JSONL (it may run on a machine with no Qt at all).

Schema v2 — an EVENT LOG, one JSON object per line, flushed as it happens so
a crash mid-recording loses at most the final partial line:

  header: {"kind":"session","v":2,"created":ISO8601,
           "window":{"w","h"},"dpr":float,"capture":"composite"|"none"}
  step:   {"kind":"click","i":int,"t":float,"widget":cls,"object_name":str,
           "label":str,"button":"left"|"right"|"middle"|"other",
           "rect_pct":{left,top,width,height}|null,
           "click_pct":{"x","y"},"out_of_frame":bool,"rect_oversized":bool,
           "window_kind":"main"|"dialog"|"popup",
           "top_level":{"class","name","title"},
           "win":{"w","h"},"page":str,"sub_page":str,
           "marked":bool,"screen":str}
  mark:   {"kind":"mark","i":int,"t":float}     ← F9; the step line is already
                                                   on disk, so a mark is its
                                                   own event, folded on read
  end:    {"kind":"end","t":float,"steps":int,"marked":int}

v1 tapes (steps carried inline "marked", counts lived in the header) still
read correctly: an inline marked=True is honoured, and missing end-line
counts are computed.
"""
from __future__ import annotations

import io
import json
import logging

logger = logging.getLogger(__name__)

TAPE_VERSION = 2


def read_tape(path: str):
    """Return ``(header, steps)`` from a tape.jsonl.

    Tolerates: a truncated final line (crash mid-write), a missing ``end``
    line (session never stopped cleanly), unknown ``kind`` rows (forward
    compat), and v1 tapes. ``marked`` is always present on every returned
    step; header ``steps``/``marked`` counts are computed when absent.
    """
    header, steps = {}, []
    marks = set()
    with io.open(path, encoding="utf-8") as fh:
        for n, line in enumerate(fh):
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except ValueError:
                continue                    # truncated tail — keep what we have
            kind = row.get("kind")
            if n == 0 and kind == "session":
                header = row
            elif kind == "click":
                row.setdefault("marked", False)
                steps.append(row)
            elif kind == "mark":
                marks.add(row.get("i"))
            elif kind == "end":
                header.setdefault("steps", row.get("steps"))
                header.setdefault("marked", row.get("marked"))
            # unknown kinds: skip (forward compat)

    for step in steps:
        if step.get("i") in marks:
            step["marked"] = True

    header.setdefault("steps", len(steps))
    header.setdefault("marked", sum(1 for s in steps if s.get("marked")))
    return header, steps


class TapeWriter:
    """Incremental tape writer. One handle open start→close; every event is
    flushed immediately, so the on-disk tape is always current up to the last
    completed line — the crash-resilience ``read_tape`` promises is real, not
    aspirational."""

    def __init__(self, path: str, *, window_w: int, window_h: int,
                 dpr: float, capture: str, created: str):
        self._path = path
        self._fh = io.open(path, "w", encoding="utf-8", newline="\n")
        self._write({
            "kind": "session", "v": TAPE_VERSION, "created": created,
            "window": {"w": window_w, "h": window_h},
            "dpr": round(float(dpr), 3), "capture": capture,
        })

    @property
    def path(self) -> str:
        return self._path

    def _write(self, row: dict) -> None:
        self._fh.write(json.dumps(row) + "\n")
        self._fh.flush()

    def write_step(self, step: dict) -> None:
        self._write(step)

    def write_mark(self, step_index: int, t: float) -> None:
        self._write({"kind": "mark", "i": step_index, "t": round(t, 3)})

    def close(self, *, t: float, steps: int, marked: int) -> str:
        try:
            self._write({"kind": "end", "t": round(t, 3),
                         "steps": steps, "marked": marked})
        finally:
            try:
                self._fh.close()
            except Exception:
                logger.debug("tape close failed", exc_info=True)
        return self._path
