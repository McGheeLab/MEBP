"""WorkflowLayoutStore.py — where each workflow page's card arrangement lives.

v7.21. Two facts per workflow:

* **promoted** — which settings-popout sections the operator moved onto the main
  workflow page;
* **order** — the top-to-bottom order of the cards in that page's column, both
  the page's own built-in cards and the promoted ones.

Kept as a MACHINE PREFERENCE (a section of ``settings.json``), deliberately NOT
in the workflow's saved profile: an arrangement is how this operator likes to
work, so loading a colleague's saved print profile must change the VALUES without
rearranging the page under them. That is also why it is not in
``set_extra_state`` beside the v7.19 plate queue — the queue *is* the work to be
done and travels with the profile; the layout is not.

Qt-free and duck-typed on ``Settings`` (``get_section`` / ``set_section`` /
``save``), so it is testable with a dict and cannot be the reason a test needs a
``QApplication`` or a real settings file.

⚠ Always load-modify-save through the injected settings object. Constructing a
fresh ``Settings()`` and saving it wipes the file (a documented incident in
CLAUDE.md), so this class never constructs one.

The order list is a HINT, never an authority: :func:`merge_order` reconciles a
stored order against the ids that actually exist right now, so a stored id for a
card that no longer exists is dropped and a NEW card (added by a later version)
appears at its natural position instead of vanishing. A stored order can
therefore never hide a card.
"""

from __future__ import annotations

import logging
from typing import Iterable

logger = logging.getLogger(__name__)

#: settings.json top-level section.
SECTION = "workflow_layout"

SCHEMA = 1


def merge_order(stored: Iterable[str] | None,
                present: Iterable[str]) -> list[str]:
    """Reconcile a stored order against the ids that exist now.

    * ids in ``stored`` that are still present keep their stored relative order;
    * ids not in ``stored`` (new cards, or ones just promoted) are appended in
      their ``present`` order;
    * ids in ``stored`` that no longer exist are dropped.

    The result is always a permutation of ``present`` — which is the property
    that makes a stale or hand-edited order incapable of hiding a card.
    """
    present_list = [str(p) for p in present]
    seen = set(present_list)
    out: list[str] = []
    for sid in (stored or ()):
        sid = str(sid)
        if sid in seen and sid not in out:
            out.append(sid)
    for sid in present_list:
        if sid not in out:
            out.append(sid)
    return out


def move_in_order(order: list[str], section_id: str, delta: int) -> list[str]:
    """Return ``order`` with ``section_id`` shifted by ``delta`` places.

    Clamped at both ends (a no-op at the edge rather than a wrap, because a card
    that jumped from the top to the bottom on one click would read as a bug).
    """
    out = [str(o) for o in order]
    sid = str(section_id)
    if sid not in out:
        return out
    i = out.index(sid)
    j = max(0, min(len(out) - 1, i + int(delta)))
    if i == j:
        return out
    out.insert(j, out.pop(i))
    return out


class WorkflowLayoutStore:
    """Per-workflow card arrangement, persisted in ``settings.json``."""

    def __init__(self, settings=None):
        self._settings = settings

    # ── raw section access ────────────────────────────────────────

    def _all(self) -> dict:
        if self._settings is None:
            return {}
        try:
            data = self._settings.get_section(SECTION)
        except Exception as exc:
            logger.debug("workflow layout read failed: %s", exc)
            return {}
        return data if isinstance(data, dict) else {}

    def _entry(self, workflow_id: str) -> dict:
        got = self._all().get(str(workflow_id))
        return got if isinstance(got, dict) else {}

    def _write(self, workflow_id: str, entry: dict, *, save: bool) -> None:
        if self._settings is None:
            return
        data = self._all()
        data["schema"] = SCHEMA
        data[str(workflow_id)] = entry
        try:
            self._settings.set_section(SECTION, data)
            if save:
                self._settings.save()
        except Exception as exc:
            logger.debug("workflow layout write failed: %s", exc)

    # ── promoted sections ─────────────────────────────────────────

    def promoted(self, workflow_id: str) -> list[str]:
        """Section keys the operator moved onto the main page, in stored order."""
        got = self._entry(workflow_id).get("promoted")
        if not isinstance(got, list):
            return []
        out: list[str] = []
        for sid in got:
            sid = str(sid)
            if sid and sid not in out:
                out.append(sid)
        return out

    def is_promoted(self, workflow_id: str, section_id: str) -> bool:
        return str(section_id) in self.promoted(workflow_id)

    def set_promoted(self, workflow_id: str, section_id: str, on: bool, *,
                     save: bool = True) -> None:
        cur = self.promoted(workflow_id)
        sid = str(section_id)
        if on and sid not in cur:
            cur.append(sid)
        elif not on and sid in cur:
            cur.remove(sid)
        else:
            return
        entry = self._entry(workflow_id)
        entry["promoted"] = cur
        self._write(workflow_id, entry, save=save)

    # ── order ─────────────────────────────────────────────────────

    def order(self, workflow_id: str) -> list[str]:
        got = self._entry(workflow_id).get("order")
        return [str(o) for o in got] if isinstance(got, list) else []

    def set_order(self, workflow_id: str, ids: Iterable[str], *,
                  save: bool = True) -> None:
        entry = self._entry(workflow_id)
        entry["order"] = [str(i) for i in ids]
        self._write(workflow_id, entry, save=save)

    def resolved_order(self, workflow_id: str,
                       present: Iterable[str]) -> list[str]:
        """The stored order reconciled against what exists — see merge_order."""
        return merge_order(self.order(workflow_id), present)

    # ── collapsed ─────────────────────────────────────────────────

    def collapsed(self, workflow_id: str) -> dict:
        got = self._entry(workflow_id).get("collapsed")
        if not isinstance(got, dict):
            return {}
        return {str(k): bool(v) for k, v in got.items()}

    def is_collapsed(self, workflow_id: str, section_id: str) -> bool:
        return bool(self.collapsed(workflow_id).get(str(section_id), False))

    def set_collapsed(self, workflow_id: str, section_id: str, on: bool, *,
                      save: bool = True) -> None:
        """A collapse is cosmetic, so ``save=False`` is a reasonable caller
        choice — the shutdown flush will pick it up either way."""
        cur = self.collapsed(workflow_id)
        cur[str(section_id)] = bool(on)
        entry = self._entry(workflow_id)
        entry["collapsed"] = cur
        self._write(workflow_id, entry, save=save)
