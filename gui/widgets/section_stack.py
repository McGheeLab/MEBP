"""section_stack.py — an ordered, reorderable column of workflow cards.

v7.21. The Setup column of a workflow page becomes a stack of *sections* the
operator can order, mixing the page's own built-in cards (object, queue,
parameters, readiness…) with sections moved out of the ⚙ settings popout.

**Entries are EXTERNALLY OWNED.** The stack places widgets and nothing else — it
never builds them and never deletes them. That is the difference from
``CustomContextPanel``, which rebuilds its bodies from a registry on every change:
a settings section CANNOT be rebuilt, because the settings dialog persists its
fields by *widget identity* (``_fields[key] = (widget, default)``). Moving the
card is therefore the only safe operation, and it is also a complete one — save,
load, Reset and the Common-Print-Settings links all keep working because none of
them cares where the widget is laid out.

Two affordance paths, because a card may or may not have a header:

* a titled :class:`~gui.widgets.components.Card` gets ▲▼ (and ↩ when it came from
  the popout) injected into its OWN header via ``add_header_widget`` — no wrapper,
  no double border;
* anything else (a bare ``QFrame`` like Quick Print's object row or status strip)
  is given a thin control strip above it.

The stack owns the *order*; persistence is the caller's business (it emits
``order_changed`` and lets the page write it wherever that page's layout lives).
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget,
)

from gui.scaling import s, sf
from gui.styles import COLORS
from gui.widgets.components import Card

logger = logging.getLogger(__name__)


class _Entry:
    """One section in the stack: its id, the caller's widget, and whatever
    wrapper/controls the stack had to add to give it ▲▼."""

    __slots__ = ("section_id", "widget", "host", "label", "returnable",
                 "buttons")

    def __init__(self, section_id, widget, host, label, returnable):
        self.section_id = section_id
        self.widget = widget
        #: What actually sits in the stack's layout — the widget itself when it
        #: could carry its own controls, else a wrapper.
        self.host = host
        self.label = label
        self.returnable = returnable
        self.buttons: list[QWidget] = []


class SectionStack(QWidget):
    """A vertical column of externally-owned section widgets, reorderable ▲▼."""

    #: Emitted with the new id order whenever the operator moves a card.
    order_changed = Signal(list)
    #: Emitted with a section id when its ↩ ("return to settings") is pressed.
    return_requested = Signal(str)
    #: Emitted whenever a section joins or leaves (NOT on a reorder), so a host
    #: can hide itself while empty.
    contents_changed = Signal()

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._entries: list[_Entry] = []
        self._lay = QVBoxLayout(self)
        self._lay.setContentsMargins(0, 0, 0, 0)
        self._lay.setSpacing(s(8))
        self._lay.addStretch(1)

    # ── queries ───────────────────────────────────────────────────

    def ids(self) -> list[str]:
        return [e.section_id for e in self._entries]

    def has(self, section_id: str) -> bool:
        return any(e.section_id == section_id for e in self._entries)

    def count(self) -> int:
        return len(self._entries)

    def widget_for(self, section_id: str):
        for e in self._entries:
            if e.section_id == section_id:
                return e.widget
        return None

    # ── mutation ──────────────────────────────────────────────────

    def add(self, section_id: str, widget: QWidget, *, label: str = "",
            returnable: bool = False, index: int | None = None) -> None:
        """Place ``widget`` in the stack under ``section_id``.

        ``returnable`` adds a ↩ button that emits :attr:`return_requested` — used
        for a section that came out of the settings popout and can go back.
        Re-adding an existing id is ignored (the caller's own bookkeeping decides
        membership; silently stacking two copies of one card would leave one of
        them orphaned in the layout).
        """
        section_id = str(section_id)
        if widget is None or self.has(section_id):
            return
        host, buttons = self._decorate(section_id, widget, label, returnable)
        entry = _Entry(section_id, widget, host, label, returnable)
        entry.buttons = buttons
        at = len(self._entries) if index is None else max(0, min(int(index),
                                                                 len(self._entries)))
        self._entries.insert(at, entry)
        self._lay.insertWidget(at, host)
        self._refresh_buttons()
        self.contents_changed.emit()

    def take(self, section_id: str):
        """Remove a section and hand its widget BACK to the caller, unparented.

        Returns the widget, or ``None`` if it was not here. Any control strip the
        stack added is destroyed, and the ▲▼ buttons it injected into a card's own
        header are removed — so the returned widget is exactly what was handed in,
        which is what lets a promoted section go back into the popout looking as
        it always did.
        """
        section_id = str(section_id)
        for i, e in enumerate(self._entries):
            if e.section_id != section_id:
                continue
            self._entries.pop(i)
            for b in e.buttons:
                b.setParent(None)
                b.deleteLater()
            e.buttons = []
            self._lay.removeWidget(e.host)
            if e.host is not e.widget:
                # A wrapper: free the real widget FIRST, or deleting the wrapper
                # takes the caller's widget with it.
                e.widget.setParent(None)
                e.host.setParent(None)
                e.host.deleteLater()
            else:
                e.widget.setParent(None)
            self._refresh_buttons()
            self.contents_changed.emit()
            return e.widget
        return None

    def set_order(self, ids) -> None:
        """Re-lay the stack in ``ids`` order. Ids not present are ignored; present
        ids missing from ``ids`` keep their relative order at the end, so a stale
        stored order can never drop a card off the page."""
        wanted = [str(i) for i in ids]
        by_id = {e.section_id: e for e in self._entries}
        new: list[_Entry] = []
        for sid in wanted:
            e = by_id.pop(sid, None)
            if e is not None:
                new.append(e)
        for e in self._entries:
            if e.section_id in by_id:
                new.append(e)
        if [e.section_id for e in new] == self.ids():
            return
        self._entries = new
        for i, e in enumerate(new):
            self._lay.removeWidget(e.host)
            self._lay.insertWidget(i, e.host)
        self._refresh_buttons()

    def move(self, section_id: str, delta: int) -> None:
        ids = self.ids()
        sid = str(section_id)
        if sid not in ids:
            return
        i = ids.index(sid)
        j = max(0, min(len(ids) - 1, i + int(delta)))
        if i == j:
            return
        ids.insert(j, ids.pop(i))
        self.set_order(ids)
        self.order_changed.emit(list(ids))

    # ── affordances ───────────────────────────────────────────────

    def _mk_btn(self, text: str, tip: str, cb) -> QPushButton:
        b = QPushButton(text)
        b.setObjectName("flatBtn")
        b.setToolTip(tip)
        b.setCursor(Qt.PointingHandCursor)
        b.setFixedSize(s(20), s(20))
        b.clicked.connect(cb)
        return b

    def _controls(self, section_id: str, returnable: bool) -> list[QPushButton]:
        out = [
            self._mk_btn("▲", "Move this section up",
                         lambda _c=False, k=section_id: self.move(k, -1)),
            self._mk_btn("▼", "Move this section down",
                         lambda _c=False, k=section_id: self.move(k, +1)),
        ]
        if returnable:
            out.append(self._mk_btn(
                "↩", "Move this section back into the ⚙ Settings popout",
                lambda _c=False, k=section_id: self.return_requested.emit(k)))
        return out

    def _decorate(self, section_id: str, widget: QWidget, label: str,
                  returnable: bool):
        """Give ``widget`` its ▲▼ controls; return ``(host, buttons)``."""
        buttons = self._controls(section_id, returnable)
        if isinstance(widget, Card) and widget.has_header():
            for b in buttons:
                widget.add_header_widget(b)
            return widget, buttons
        # No header to put them in — wrap in a thin captioned strip. Deliberately
        # unstyled (no second border): the widget below usually carries its own
        # frame, and nesting two would read as a card inside a card.
        host = QWidget()
        vb = QVBoxLayout(host)
        vb.setContentsMargins(0, 0, 0, 0)
        vb.setSpacing(s(2))
        bar = QHBoxLayout()
        bar.setContentsMargins(0, 0, 0, 0)
        bar.setSpacing(s(4))
        cap = QLabel(label or "")
        cap.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
        bar.addWidget(cap, 1)
        for b in buttons:
            bar.addWidget(b)
        vb.addLayout(bar)
        vb.addWidget(widget)
        return host, buttons

    def _refresh_buttons(self) -> None:
        """Grey ▲ on the first card and ▼ on the last — at the edge the move is a
        no-op, and a live button that does nothing reads as broken."""
        last = len(self._entries) - 1
        for i, e in enumerate(self._entries):
            if len(e.buttons) >= 2:
                e.buttons[0].setEnabled(i > 0)
                e.buttons[1].setEnabled(i < last)


class PromotedSectionsPanel(QWidget):
    """A :class:`SectionStack` in a bounded, self-hiding scroll area.

    For a workflow page whose main column is NOT itself scrollable — which is
    most of them. Promoting three tall sections onto such a page would otherwise
    grow it past the window and push the run row (and therefore **Abort**) out of
    reach, with no scrollbar to get back. So the drawer:

    * is **hidden while empty**, giving a page with nothing promoted exactly its
      pre-v7.21 geometry;
    * **bounds its own height** and scrolls internally beyond that, so no number
      of promoted sections can displace the page's own controls.

    Quick Print does NOT use this: its Setup column is already a ``QScrollArea``,
    so a second nested scroll there would be a scrollbar inside a scrollbar.
    """

    def __init__(self, *, max_height_px: int | None = None,
                 title: str = "Moved here from ⚙ Settings",
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._stack = SectionStack()
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(s(4))

        self._title = QLabel(title)
        self._title.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
        outer.addWidget(self._title)

        from PySide6.QtWidgets import QScrollArea
        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        self._scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._scroll.setWidget(self._stack)
        self._scroll.setMaximumHeight(
            int(max_height_px) if max_height_px else s(300))
        outer.addWidget(self._scroll)

        self._stack.contents_changed.connect(self._sync_visible)
        self._sync_visible()

    @property
    def stack(self) -> SectionStack:
        return self._stack

    def _sync_visible(self) -> None:
        self.setVisible(self._stack.count() > 0)


def wire_section_promotion(page, dialog, stack, *, settings, workflow_id,
                           store=None):
    """Connect a page's section stack to its settings popout. ONE implementation.

    Registers ``stack`` as the popout's promotion host, restores the stored
    promotions and card order, and keeps the order persisted as the operator
    moves cards. Returns the :class:`~SupportClasses.WorkflowLayoutStore.\
WorkflowLayoutStore` (or ``None`` if wiring failed), which the caller should
    keep — the store is stateless over ``settings``, but holding it documents
    ownership.

    Best-effort by design: a page built via ``__new__`` in a partial-page test,
    or one with no settings object, must not raise from here.

    ORDER MATTERS and is the reason this is shared rather than copied per page:
    promotions are restored FIRST (they add cards to the stack) and the stored
    order applied SECOND, because an order restored before the promoted cards
    exist could not place them.
    """
    from SupportClasses.WorkflowLayoutStore import WorkflowLayoutStore

    if stack is None or dialog is None:
        return None
    st = store if store is not None else WorkflowLayoutStore(settings)
    wid = str(workflow_id)

    def _persist_order():
        try:
            st.set_order(wid, stack.ids())
        except Exception as exc:
            logger.debug("section order persist failed: %s", exc)

    def _on_return(sid):
        try:
            dialog.set_section_promoted(str(sid), False)
        except Exception as exc:
            logger.debug("section return failed: %s", exc)

    try:
        stack.order_changed.connect(lambda _ids: _persist_order())
        stack.return_requested.connect(_on_return)
        dialog.set_promotion_host(stack, layout_store=st,
                                 on_change=_persist_order)
        stack.set_order(st.resolved_order(wid, stack.ids()))
    except Exception as exc:
        logger.debug("section promotion wiring failed: %s", exc)
        return None
    return st
