"""ContextPanelLayoutStore.py — the user's custom left-context panel layout.

v7.5.x: The left "context" panel (the box that shows the jog panel on some
pages) gains a pill-based view picker — a built-in **Jog** view and a **Custom**
view the operator composes from *sections* (cards): a live camera viewer, a
syringe overview, X/Y/Z/P1/P2/P3 location read-outs, jog controls, hardware
info, … The Custom layout is a **single, shared, machine-level** thing — the
same cards show everywhere the left box appears, and they persist across
restarts.

This store owns that persistence. One file:

    config/context_panel_layout.json

Format — an ordered list of sections, each a small dict:

    {
      "_format": "mebp-context-panel-layout",
      "version": 1,
      "sections": [
        {"type": "camera",    "id": "<uuid4>", "options": {}, "collapsed": false},
        {"type": "syringe",   "id": "<uuid4>", "options": {}, "collapsed": false},
        {"type": "positions", "id": "<uuid4>", "options": {}, "collapsed": false}
      ]
    }

All writes are atomic (tmp + ``os.replace``) so a crash can't truncate the file —
the same convention ``WorkflowSettingsStore`` / the calibration stores use. The
store is deliberately **Qt-free** so it can be unit-tested headless; the set of
valid section types is injected (``known_types``) rather than imported from the
Qt section registry, which keeps this module import-light and forward-compatible
(a newly registered section type is accepted as long as the caller lists it).
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
import uuid
from pathlib import Path
from typing import Callable, Iterable, Optional

logger = logging.getLogger(__name__)

_FORMAT = "mebp-context-panel-layout"
_VERSION = 1

# Resolved relative to the repo root (this file lives in SupportClasses/, so one
# parent up is the root).
_DEFAULT_PATH = Path(__file__).resolve().parent.parent / "config" / \
    "context_panel_layout.json"


class ContextPanelLayoutStore:
    """Load/save/mutate the single shared custom-context-panel layout.

    Every mutating method persists atomically and then notifies listeners, so a
    UI bound to the store rebuilds itself on any change (add / remove / move /
    collapse), and multiple bound views stay in lock-step.
    """

    def __init__(
        self,
        path: Path | str | None = None,
        *,
        known_types: Iterable[str] | None = None,
    ):
        if path is not None:
            self._path = Path(path)
        else:
            # An env override lets tests / CI isolate the layout file so
            # automated runs never read or write the repo's config/.
            env = os.environ.get("MEBP_CONTEXT_PANEL_DIR")
            self._path = (Path(env) / "context_panel_layout.json") if env \
                else _DEFAULT_PATH
        self._known: Optional[set[str]] = (
            set(known_types) if known_types is not None else None)
        self._sections: list[dict] = []
        self._listeners: list[Callable[["ContextPanelLayoutStore"], None]] = []
        self._load()

    # ── Load / save ───────────────────────────────────────────────

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            data = json.loads(self._path.read_text(encoding="utf-8"))
        except Exception as exc:  # malformed / partial file — start empty
            logger.warning("ContextPanelLayoutStore: failed to read %s: %s",
                           self._path, exc)
            return
        raw = data.get("sections") if isinstance(data, dict) else None
        if not isinstance(raw, list):
            return
        self._sections = [
            s for s in (self._coerce_section(entry) for entry in raw)
            if s is not None
        ]

    def _coerce_section(self, entry: object) -> Optional[dict]:
        """Normalise one persisted entry; drop it if invalid / unknown type."""
        if not isinstance(entry, dict):
            return None
        stype = entry.get("type")
        if not isinstance(stype, str) or not stype:
            return None
        if self._known is not None and stype not in self._known:
            logger.debug("ContextPanelLayoutStore: dropping unknown section "
                         "type %r", stype)
            return None
        sid = entry.get("id")
        if not isinstance(sid, str) or not sid:
            sid = uuid.uuid4().hex
        opts = entry.get("options")
        collapsed = bool(entry.get("collapsed", False))
        return {
            "type": stype,
            "id": sid,
            "options": dict(opts) if isinstance(opts, dict) else {},
            "collapsed": collapsed,
        }

    def _save(self) -> None:
        """Atomic write. Best-effort — a persistence failure logs but never
        raises (a layout change must not crash the UI)."""
        payload = {
            "_format": _FORMAT,
            "version": _VERSION,
            "sections": [dict(s) for s in self._sections],
        }
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=str(self._path.parent), suffix=".tmp")
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as f:
                    json.dump(payload, f, indent=2)
                os.replace(tmp, self._path)
            finally:
                if os.path.exists(tmp):
                    try:
                        os.remove(tmp)
                    except OSError:
                        pass
        except Exception as exc:
            logger.error("ContextPanelLayoutStore: failed to save %s: %s",
                         self._path, exc)

    # ── Read ──────────────────────────────────────────────────────

    def sections(self) -> list[dict]:
        """A copy of the ordered section list (safe to iterate/mutate)."""
        return [dict(s) for s in self._sections]

    def _index_of(self, section_id: str) -> int:
        for i, s in enumerate(self._sections):
            if s["id"] == section_id:
                return i
        return -1

    # ── Mutate (persist + notify) ─────────────────────────────────

    def set_sections(self, sections: list[dict]) -> None:
        self._sections = [
            s for s in (self._coerce_section(e) for e in (sections or []))
            if s is not None
        ]
        self._save()
        self._notify()

    def add_section(self, section_type: str, options: dict | None = None) -> str:
        """Append a section; returns its new id (empty string if rejected)."""
        entry = self._coerce_section(
            {"type": section_type, "options": options or {}})
        if entry is None:
            logger.debug("ContextPanelLayoutStore: refused to add section of "
                         "type %r", section_type)
            return ""
        self._sections.append(entry)
        self._save()
        self._notify()
        return entry["id"]

    def remove_section(self, section_id: str) -> bool:
        idx = self._index_of(section_id)
        if idx < 0:
            return False
        del self._sections[idx]
        self._save()
        self._notify()
        return True

    def move_section(self, section_id: str, delta: int) -> bool:
        """Move a section by ``delta`` positions (clamped to the ends)."""
        idx = self._index_of(section_id)
        if idx < 0:
            return False
        new_idx = max(0, min(len(self._sections) - 1, idx + delta))
        if new_idx == idx:
            return False
        s = self._sections.pop(idx)
        self._sections.insert(new_idx, s)
        self._save()
        self._notify()
        return True

    def set_section_options(self, section_id: str, options: dict) -> bool:
        idx = self._index_of(section_id)
        if idx < 0:
            return False
        self._sections[idx]["options"] = dict(options or {})
        self._save()
        self._notify()
        return True

    def set_collapsed(self, section_id: str, collapsed: bool,
                      *, notify: bool = False) -> bool:
        """Persist a card's collapsed state.

        ``notify`` defaults False: collapsing a card is a cosmetic per-card
        change that should not trigger a full panel rebuild (which would rip the
        card down and lose focus). Pass ``notify=True`` only if a bound view
        genuinely needs to re-read the whole layout.
        """
        idx = self._index_of(section_id)
        if idx < 0:
            return False
        self._sections[idx]["collapsed"] = bool(collapsed)
        self._save()
        if notify:
            self._notify()
        return True

    # ── Listeners ─────────────────────────────────────────────────

    def add_listener(
            self, cb: Callable[["ContextPanelLayoutStore"], None]) -> None:
        if cb not in self._listeners:
            self._listeners.append(cb)

    def remove_listener(
            self, cb: Callable[["ContextPanelLayoutStore"], None]) -> None:
        if cb in self._listeners:
            self._listeners.remove(cb)

    def _notify(self) -> None:
        for cb in list(self._listeners):
            try:
                cb(self)
            except Exception as exc:  # a bad listener must not break a mutation
                logger.debug("ContextPanelLayoutStore listener failed: %s", exc)
