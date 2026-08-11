"""
lablink_workflow.py — the LabLink imaging integration control surface (v7.17).

Operator: *"we want to be able integrate lablink to any imaging workflows … if
we make a flourescence scan we should push the scan results to lablink for a
nd2studios recipe to do things like deconvolution or other recipes. lets make a
workflow page that sets this up. when we turn it on with this workflow page, it
automatically sends outputs of various types to lablink for processing."*

This page **configures and observes**; it does not own the work. The queue, the
worker thread and every network call live in the Qt-free
:class:`~SupportClasses.LabLinkService.LabLinkService` singleton, so sending
continues while the operator navigates away — which is the normal case, since a
scan finishes and they move on.

Four things it is careful about:

**Discover, do not hardcode.** Every recipe name, knob bound, size limit and
timeout is read from the hub (``GET /hello`` + ``GET /workflows``) when the
operator presses Discover, and nothing is cached to disk. The integration doc
devotes its longest section to this: a hardcoded recipe name becomes a silent
"not found" for users the first time an operator renames one.

**Knobs have three states, not two.** Omitted = the recipe's own default ·
``null`` = derive it from the file · a value = pinned. They are three different
instructions and the mode combo on each knob row is what keeps them distinct all
the way to the wire.

**Nothing is hidden.** A successful upload proves only that bytes arrived. Until
``.nd3`` is added to the hub's recipe ``match`` patterns every job will be
refused, so the page says that in as many words instead of showing a queue that
quietly never completes.

**The token is a secret.** No TLS anywhere in the protocol, and one shared token
grants read/write/delete on every channel. The field is masked, the page names
where the secret came from, and it is never logged.
"""

from __future__ import annotations

import logging
import threading
import time

from PySide6.QtCore import Qt, QObject, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QPushButton, QScrollArea, QSizePolicy,
    QSpinBox, QVBoxLayout, QWidget,
)

from gui.scaling import s, sf
from gui.styles import COLORS

logger = logging.getLogger(__name__)

#: Knob modes. The wire meaning of each is in the module docstring.
MODE_DEFAULT = "default"
MODE_DERIVE = "derive"
MODE_PINNED = "pinned"

MODE_LABELS = (
    (MODE_DEFAULT, "Recipe default"),
    (MODE_DERIVE, "Derive from file"),
    (MODE_PINNED, "Set to"),
)

STATE_COLORS = {
    "done": "green",
    "failed": "red",
    "dropped": "peach",
    "queued": "subtext0",
}


class _HubProbe(QObject):
    """Runs ``hello`` + ``workflows`` off the GUI thread.

    Both are blocking HTTP with a socket timeout above the hub's long-poll
    window (tens of seconds against an unreachable host), and this repo has
    fixed the "the UI freezes while hardware is talked to" class of bug enough
    times to have three update plans about it.
    """

    finished = Signal(bool, str, object, object)   # ok, message, hello, workflows

    def probe(self, client_factory) -> None:
        threading.Thread(target=self._run, args=(client_factory,),
                         daemon=True, name="LabLinkProbe").start()

    def _run(self, client_factory) -> None:
        try:
            hub = client_factory()
        except Exception as exc:
            self.finished.emit(False, f"Not configured: {exc}", None, None)
            return
        try:
            info = hub.hello()
        except Exception as exc:
            self.finished.emit(
                False, f"{type(exc).__name__}: {exc}", None, None)
            return
        # Ask before assuming: a plain file exchange answers 404 on /s, and the
        # confusing version of that failure is discovering it one upload later.
        caps = (info.get("capabilities") or {}) if isinstance(info, dict) else {}
        if not caps.get("sessions"):
            self.finished.emit(
                False,
                "This hub answered, but it does not run analyses "
                "(capabilities.sessions is not set) — it is a file exchange "
                "only. Nothing can be processed here.",
                info, None)
            return
        try:
            flows = hub.workflows()
        except Exception as exc:
            self.finished.emit(
                False, f"Connected, but /workflows failed: {exc}", info, None)
            return
        self.finished.emit(True, "", info, flows)


class _KnobRow:
    """One knob's controls, and the tri-state rule that gives them meaning."""

    def __init__(self, schema: dict, on_change):
        self.schema = dict(schema or {})
        self.name = str(self.schema.get("name") or "")
        self._on_change = on_change

        label_text = str(self.schema.get("label") or self.name)
        unit = str(self.schema.get("unit") or "")
        if unit:
            label_text += f" ({unit})"
        self.label = QLabel(label_text)
        self.label.setToolTip(self._tooltip())
        self.label.setWordWrap(True)

        self.mode = QComboBox()
        for value, text in MODE_LABELS:
            self.mode.addItem(text, value)
        self.mode.setToolTip(self._tooltip())
        self.mode.currentIndexChanged.connect(self._changed)

        self.value = self._build_value_widget()
        self.value.setToolTip(self._tooltip())

        self.note = QLabel("")
        self.note.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
        self.note.setWordWrap(True)

    # ── Construction helpers ──────────────────────────────────────

    def _tooltip(self) -> str:
        bits = []
        if self.schema.get("help"):
            bits.append(str(self.schema["help"]))
        spec = [f"type {self.schema.get('type')}"]
        lo, hi = self.schema.get("min"), self.schema.get("max")
        if lo is not None or hi is not None:
            spec.append(f"range {lo}…{hi}")
        if self.schema.get("enum"):
            spec.append("one of " + ", ".join(
                str(e) for e in self.schema["enum"]))
        if self.schema.get("default") is not None:
            spec.append(f"default {self.schema['default']!r}")
        if self.schema.get("unset_means") == "derive":
            spec.append("left unset, this is derived from the file")
        bits.append(" · ".join(spec))
        cond = self.schema.get("applies_when") or {}
        if cond:
            bits.append(f"Only read {_condition_text(cond)}. Pinning it when "
                        f"that does not hold is refused, not ignored.")
        return "\n\n".join(bits)

    def _build_value_widget(self) -> QWidget:
        kind = str(self.schema.get("type") or "")
        enum = self.schema.get("enum") or []
        if enum:
            w = QComboBox()
            for item in enum:
                w.addItem(str(item), item)
            w.currentIndexChanged.connect(self._changed)
            return w
        if kind == "bool":
            w = QComboBox()
            w.addItem("true", True)
            w.addItem("false", False)
            w.currentIndexChanged.connect(self._changed)
            return w
        if kind in ("int", "channel_list"):
            w = QSpinBox()
            lo = self.schema.get("min")
            hi = self.schema.get("max")
            # channel_list is a zero-based index into the sidecar's channels,
            # so 0 is a legal value and must not become the "unset" sentinel —
            # that is what the mode combo is for.
            w.setRange(int(lo) if lo is not None else 0,
                       int(hi) if hi is not None else 9999)
            w.valueChanged.connect(self._changed)
            return w
        if kind == "float":
            w = QDoubleSpinBox()
            w.setDecimals(3)
            lo = self.schema.get("min")
            hi = self.schema.get("max")
            w.setRange(float(lo) if lo is not None else -1e9,
                       float(hi) if hi is not None else 1e9)
            w.setSingleStep(0.1)
            w.valueChanged.connect(self._changed)
            return w
        w = QLineEdit()
        if self.schema.get("max_len"):
            w.setMaxLength(int(self.schema["max_len"]))
        w.editingFinished.connect(self._changed)
        return w

    def _changed(self, *_a) -> None:
        self.value.setEnabled(self.mode.currentData() == MODE_PINNED)
        self._on_change()

    # ── State ─────────────────────────────────────────────────────

    def widgets(self) -> tuple:
        return (self.label, self.mode, self.value, self.note)

    def set_from_stored(self, knobs: dict) -> None:
        """Apply a stored knob map. ⚠ ``in`` not ``.get()``.

        A stored ``None`` means "derive" and a stored ``0`` means a pinned zero.
        Both are falsy, so membership is the only test that can tell the three
        states apart — reading this with ``knobs.get(name)`` would collapse a
        pinned zero into "recipe default" and silently change what runs.
        """
        if self.name in (knobs or {}):
            raw = knobs[self.name]
            if raw is None:
                self._select_mode(MODE_DERIVE)
            else:
                self._select_mode(MODE_PINNED)
                self._set_value(raw)
        else:
            self._select_mode(MODE_DEFAULT)
        self.value.setEnabled(self.mode.currentData() == MODE_PINNED)

    def _select_mode(self, mode: str) -> None:
        idx = self.mode.findData(mode)
        if idx >= 0:
            self.mode.setCurrentIndex(idx)

    def _set_value(self, raw) -> None:
        w = self.value
        if isinstance(raw, list) and raw:
            raw = raw[0]              # channel_list rendered as one index
        if isinstance(w, QComboBox):
            idx = w.findData(raw)
            if idx < 0:
                idx = w.findText(str(raw))
            if idx >= 0:
                w.setCurrentIndex(idx)
        elif isinstance(w, (QSpinBox, QDoubleSpinBox)):
            try:
                w.setValue(type(w.value())(raw))
            except (TypeError, ValueError):
                pass
        else:
            w.setText(str(raw))

    def contribution(self) -> tuple:
        """``(include, value)`` for this knob's entry in the knob map."""
        mode = self.mode.currentData()
        if mode == MODE_DEFAULT:
            return (False, None)      # omit the key entirely
        if mode == MODE_DERIVE:
            return (True, None)       # an explicit null
        w = self.value
        if isinstance(w, QComboBox):
            value = w.currentData()
            if value is None:
                value = w.currentText()
        elif isinstance(w, (QSpinBox, QDoubleSpinBox)):
            value = w.value()
        else:
            value = w.text().strip()
        if str(self.schema.get("type")) == "channel_list":
            value = [int(value)]
        return (True, value)

    def effective_value(self):
        """What this knob will resolve to, for evaluating a sibling's
        ``applies_when`` — the pinned value if pinned, else the declared
        default. A derived knob has no value we can know here."""
        include, value = self.contribution()
        if include and value is not None:
            return value
        return self.schema.get("default")

    def apply_condition(self, resolved: dict) -> None:
        """Disable the row when its ``applies_when`` does not hold.

        The doc is explicit: *"Read applies_when and disable the control rather
        than letting someone set it"* — a knob nothing will read is refused,
        not ignored, so leaving it editable manufactures a 400.
        """
        cond = self.schema.get("applies_when") or {}
        if not cond:
            return
        holds = _condition_holds(cond, resolved)
        self.mode.setEnabled(holds)
        self.value.setEnabled(
            holds and self.mode.currentData() == MODE_PINNED)
        self.note.setText(
            "" if holds else f"not used — {_condition_text(cond)}")


def _condition_text(cond: dict) -> str:
    if "equals" in cond:
        return f"when {cond.get('knob')} is {cond['equals']}"
    allowed = ", ".join(str(v) for v in (cond.get("in") or ()))
    return f"when {cond.get('knob')} is one of {allowed}"


def _condition_holds(cond: dict, resolved: dict) -> bool:
    value = resolved.get(cond.get("knob"))
    if "equals" in cond:
        return value == cond["equals"]
    return value in (cond.get("in") or ())


class _SourcePanel(QGroupBox):
    """One output kind: whether to send it, through which recipe, and how."""

    def __init__(self, name: str, title: str, page: "LabLinkWorkflowPage"):
        super().__init__(title)
        self._name = name
        self._page = page
        self._knob_rows: list = []
        self._loading = False

        lay = QVBoxLayout(self)
        lay.setSpacing(s(6))

        self.enable = QCheckBox("Send these outputs for processing")
        self.enable.toggled.connect(self._commit)
        lay.addWidget(self.enable)

        grid = QGridLayout()
        grid.setHorizontalSpacing(s(8))
        grid.setVerticalSpacing(s(4))
        grid.setColumnStretch(1, 1)

        self.workflow = QComboBox()
        self.workflow.setToolTip(
            "Which software on the hub runs this. Read from the hub — press "
            "Discover above.")
        self.workflow.currentIndexChanged.connect(self._on_workflow_changed)
        grid.addWidget(QLabel("Workflow"), 0, 0)
        grid.addWidget(self.workflow, 0, 1)

        self.recipe = QComboBox()
        self.recipe.setToolTip(
            "The analysis to run. Recipes are installed on the hub by its "
            "operator; MEBP can only choose one that is already there.")
        self.recipe.currentIndexChanged.connect(self._on_recipe_changed)
        grid.addWidget(QLabel("Recipe"), 1, 0)
        grid.addWidget(self.recipe, 1, 1)

        self.ceiling = QSpinBox()
        self.ceiling.setRange(0, 65536)
        self.ceiling.setSuffix(" MB")
        self.ceiling.setSpecialValueText("no local limit")
        self.ceiling.setToolTip(
            "Refuse to upload anything larger than this, before spending the "
            "bytes. This is IN ADDITION to the hub's own max_file_bytes, which "
            "is read at run time and is authoritative. Worth setting on a "
            "relayed Tailscale link (measured here: about 1.5 MB/s).")
        self.ceiling.valueChanged.connect(self._commit)
        grid.addWidget(QLabel("Refuse above"), 2, 0)
        grid.addWidget(self.ceiling, 2, 1)
        lay.addLayout(grid)

        self.gate = QLabel("")
        self.gate.setWordWrap(True)
        self.gate.setVisible(False)
        self.gate.setStyleSheet(
            f"color: {COLORS['peach']}; font-size: {sf(8)}pt;")
        lay.addWidget(self.gate)

        self.requires = QLabel("")
        self.requires.setWordWrap(True)
        self.requires.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
        lay.addWidget(self.requires)

        self.knob_box = QWidget()
        self._knob_grid = QGridLayout(self.knob_box)
        self._knob_grid.setContentsMargins(0, s(4), 0, 0)
        self._knob_grid.setHorizontalSpacing(s(8))
        self._knob_grid.setVerticalSpacing(s(3))
        self._knob_grid.setColumnStretch(0, 3)
        self._knob_grid.setColumnStretch(2, 2)
        lay.addWidget(self.knob_box)

    # ── Load / commit ─────────────────────────────────────────────

    def load(self) -> None:
        cfg = self._page.config.source(self._name)
        self._loading = True
        try:
            self.enable.setChecked(bool(cfg.get("enabled")))
            self.ceiling.setValue(int(cfg.get("max_upload_mb") or 0))
            self._populate_workflows(cfg.get("workflow", ""))
            self._populate_recipes(cfg.get("recipe", ""))
            self._rebuild_knobs(cfg.get("knobs") or {})
        finally:
            self._loading = False
        self._refresh_gate()

    def _fill_combo(self, combo: QComboBox, names: list, want: str) -> None:
        """Populate a chooser so that what it SHOWS is what is stored.

        ⚠ The empty choice is deliberate and load-bearing. Auto-selecting the
        first discovered recipe would leave the combo displaying an analysis the
        operator never chose while the store held "" — the source would read as
        armed with a recipe named and would silently send nothing, which is the
        "green tick beside something that is actually refused" failure this
        codebase has paid for before. It also decides *which analysis runs on
        the data* on their behalf, and a deconvolution is not a segmentation.
        """
        combo.blockSignals(True)
        combo.clear()
        combo.addItem("— choose —", "")
        for name in names:
            combo.addItem(name, name)
        if want and combo.findData(want) < 0:
            # A stored choice the hub has not been asked about (or no longer
            # offers) stays visible: dropping it silently would read as "you
            # never configured this".
            combo.addItem(f"{want} (not offered by this hub)", want)
        combo.setCurrentIndex(max(0, combo.findData(want)))
        combo.blockSignals(False)

    def _populate_workflows(self, want: str) -> None:
        self._fill_combo(
            self.workflow,
            [str(w.get("name") or "") for w in self._page.discovered], want)

    def _populate_recipes(self, want: str) -> None:
        self._fill_combo(
            self.recipe,
            [str(r.get("name") or "") for r in self._recipes()], want)

    def _recipes(self) -> list:
        want = self.workflow.currentData() or ""
        for flow in self._page.discovered:
            if str(flow.get("name")) == want:
                return list(flow.get("recipes") or [])
        return []

    def _recipe_schema(self) -> dict:
        want = self.recipe.currentData() or ""
        for r in self._recipes():
            if str(r.get("name")) == want:
                return dict(r)
        return {}

    def _on_workflow_changed(self, *_a) -> None:
        if self._loading:
            return
        self._populate_recipes("")
        self._rebuild_knobs({})
        self._commit()

    def _on_recipe_changed(self, *_a) -> None:
        if self._loading:
            return
        self._rebuild_knobs({})
        self._commit()

    def _refresh_gate(self) -> None:
        """Say why an armed source is not actually sending.

        A ticked box beside an unset recipe would otherwise look like a working
        configuration, and the operator would watch nothing happen.
        """
        if not self.enable.isChecked():
            self.gate.setVisible(False)
            return
        if not self.recipe.currentData():
            self.gate.setText(
                "⚠ Armed, but no recipe is chosen — nothing will be sent. "
                "Press Discover above, then pick the analysis to run.")
            self.gate.setVisible(True)
        elif not self._page.config.enabled():
            self.gate.setText(
                "⚠ Automatic sending is switched off above, so this source is "
                "idle.")
            self.gate.setVisible(True)
        else:
            self.gate.setVisible(False)

    def _rebuild_knobs(self, stored: dict) -> None:
        while self._knob_grid.count():
            item = self._knob_grid.takeAt(self._knob_grid.count() - 1)
            w = item.widget()
            if w is not None:
                w.setParent(None)
        self._knob_rows = []

        recipe = self._recipe_schema()
        needs = list(recipe.get("requires_metadata") or [])
        if recipe:
            text = recipe.get("title") or recipe.get("name") or ""
            if needs:
                text += ("   ⚠ needs in the sidecar: " + ", ".join(needs)
                         + " — a missing one is refused by name, which is "
                           "recoverable; a guessed one is not.")
            self.requires.setText(str(text))
        elif self.recipe.currentData():
            self.requires.setText(
                "Knob list unknown — press Discover to read this recipe's "
                "knobs and their bounds from the hub.")
        else:
            self.requires.setText("")

        knobs = list(recipe.get("knobs") or [])
        if not knobs:
            self.knob_box.setVisible(False)
            return
        self.knob_box.setVisible(True)
        head = QLabel("Knobs — omitted uses the recipe default, "
                      "\"Derive from file\" sends an explicit null")
        head.setWordWrap(True)
        head.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt; "
            f"font-weight: 600;")
        self._knob_grid.addWidget(head, 0, 0, 1, 3)
        for schema in knobs:
            row = _KnobRow(schema, self._on_knob_changed)
            row.set_from_stored(stored)
            r = self._knob_grid.rowCount()
            label, mode, value, note = row.widgets()
            self._knob_grid.addWidget(label, r, 0)
            self._knob_grid.addWidget(mode, r, 1)
            self._knob_grid.addWidget(value, r, 2)
            self._knob_grid.addWidget(note, r + 1, 0, 1, 3)
            self._knob_rows.append(row)
        self._apply_conditions()

    def _apply_conditions(self) -> None:
        resolved = {r.name: r.effective_value() for r in self._knob_rows}
        for row in self._knob_rows:
            row.apply_condition(resolved)

    def _on_knob_changed(self) -> None:
        self._apply_conditions()
        self._commit()

    def knobs(self) -> dict:
        """The knob map to store. ⚠ Built by membership, so a pinned zero and
        an explicit null both survive as themselves."""
        out: dict = {}
        for row in self._knob_rows:
            if not row.mode.isEnabled():
                continue          # applies_when does not hold: send nothing
            include, value = row.contribution()
            if include:
                out[row.name] = value
        return out

    def _commit(self, *_a) -> None:
        if self._loading:
            return
        self._page.config.set_source(self._name, {
            "enabled": bool(self.enable.isChecked()),
            "workflow": str(self.workflow.currentData() or ""),
            "recipe": str(self.recipe.currentData() or ""),
            "knobs": self.knobs(),
            "max_upload_mb": int(self.ceiling.value()),
        })
        self._refresh_gate()
        self._page.refresh_status()


class LabLinkWorkflowPage(QWidget):
    """Configure and watch the LabLink imaging integration."""

    back_requested = Signal()

    def __init__(self, controller=None, settings=None, camera_manager=None,
                 parent: QWidget | None = None, *, config=None, service=None):
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        self._camera_manager = camera_manager

        from SupportClasses import LabLinkConfigStore as cfg_mod
        self.config = config if config is not None else cfg_mod.get_store()
        self._sources = cfg_mod.SOURCES
        self._source_labels = cfg_mod.SOURCE_LABELS
        self._service = service
        self._bridge = None

        #: The hub's own answer to "what can you run". Never persisted:
        #: *"Cache them for the length of a user's session if you like; do not
        #: ship them."*
        self.discovered: list = []
        self.hello: dict = {}

        self._probe = _HubProbe(self)
        self._probe.finished.connect(self._on_probe_finished)

        self._build_ui()
        self.load()

    # ── UI ────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(16), s(12), s(16), s(16))
        outer.setSpacing(s(10))

        header = QHBoxLayout()
        header.setSpacing(s(8))
        back = QPushButton("← Back to Workflows")
        back.setCursor(Qt.PointingHandCursor)
        back.clicked.connect(self.back_requested.emit)
        header.addWidget(back)
        title = QLabel("LabLink Processing")
        title.setStyleSheet(
            f"color: {COLORS['blue']}; font-size: {sf(14)}pt; font-weight: 600;")
        header.addWidget(title)
        header.addStretch(1)
        self._status = QLabel("")
        self._status.setStyleSheet(f"color: {COLORS['subtext0']};")
        header.addWidget(self._status)
        outer.addLayout(header)

        intro = QLabel(
            "Send imaging outputs to a LabLink hub for processing — an "
            "ND2Studios deconvolution or segmentation recipe, say. Turn a "
            "source on and its outputs are offered automatically as they are "
            "produced; nothing is sent while this is off.")
        intro.setWordWrap(True)
        intro.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        outer.addWidget(intro)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        body = QWidget()
        lay = QVBoxLayout(body)
        lay.setContentsMargins(0, 0, s(6), 0)
        lay.setSpacing(s(10))
        scroll.setWidget(body)
        outer.addWidget(scroll, stretch=1)

        lay.addWidget(self._build_format_banner())
        lay.addWidget(self._build_connection_group())
        lay.addWidget(self._build_master_group())
        self._panels: dict = {}
        for name in self._sources:
            panel = _SourcePanel(name, self._source_labels.get(name, name), self)
            self._panels[name] = panel
            lay.addWidget(panel)
        lay.addWidget(self._build_queue_group())
        lay.addStretch(1)

    def _build_format_banner(self) -> QWidget:
        """What actually goes on the wire, and the one thing that can refuse it.

        Not a warning by default. ``.nd3`` became readable by the hub on
        2026-08-08 — every `nd2studios` recipe lists ``*.nd3`` in its
        ``inputs[].match`` and the engine ingests the container natively — so
        crying wolf here would be worse than saying nothing. The banner
        escalates only if a build is pointed at a hub too old for it.
        """
        from SupportClasses.LabLinkJob import ND3_NOT_YET_ACCEPTED
        stale = bool(ND3_NOT_YET_ACCEPTED)
        frame = QFrame()
        frame.setStyleSheet(
            f"background: {COLORS['surface0']}; "
            f"border-left: {s(3)}px solid "
            f"{COLORS['peach'] if stale else COLORS['blue']}; "
            f"border-radius: {s(4)}px;")
        row = QVBoxLayout(frame)
        row.setContentsMargins(s(10), s(8), s(10), s(8))
        if stale:
            message = (
                "⚠ This hub may not be able to read <b>.nd3</b>. MEBP sends a "
                "<b>.nd3</b> container plus its <code>.job.json</code> sidecar; "
                "a hub whose recipes still match <code>*.nd2</code> / "
                "<code>*.tif</code> only will refuse the job. The refusal is "
                "not silent — it appears in the queue below in the hub's own "
                "words.")
        else:
            message = (
                "Each output is sent as a <b>.nd3</b> container plus a matching "
                "<code>.job.json</code> sidecar — the hub pairs them by stem. "
                "The sidecar is <b>authoritative</b>: it overrides what the "
                "image file claims, because a container invents a bit depth and "
                "a placeholder channel name. Anything MEBP does not actually "
                "know is left out rather than guessed, and LabLink names a "
                "missing field. That matters more than it sounds — on real data, "
                "supplying the objective NA and the channel's emission "
                "wavelength moved a segmented object count from 2855 to 2660, "
                "with no warning from any layer.")
        text = QLabel(message)
        text.setWordWrap(True)
        text.setStyleSheet(f"color: {COLORS['text']}; font-size: {sf(9)}pt;")
        row.addWidget(text)
        return frame

    def _build_connection_group(self) -> QGroupBox:
        box = QGroupBox("Hub connection")
        lay = QVBoxLayout(box)
        lay.setSpacing(s(6))

        grid = QGridLayout()
        grid.setHorizontalSpacing(s(8))
        grid.setColumnStretch(1, 1)

        self._url_edit = QLineEdit()
        self._url_edit.setPlaceholderText("http://100.x.y.z:8765")
        self._url_edit.setToolTip(
            "The hub's address. Prefer its Tailscale address: the protocol has "
            "no TLS, and plain campus Wi-Fi blocks client-to-client traffic "
            "outright (measured on this rig).")
        self._url_edit.editingFinished.connect(self._commit_connection)
        grid.addWidget(QLabel("Address"), 0, 0)
        grid.addWidget(self._url_edit, 0, 1)

        self._token_edit = QLineEdit()
        self._token_edit.setEchoMode(QLineEdit.Password)
        self._token_edit.setPlaceholderText("site token")
        self._token_edit.setToolTip(
            "The hub's shared site token. One token grants read, overwrite and "
            "delete on every channel, and it travels unencrypted — set "
            "LABLINK_TOKEN in the environment instead to keep it out of any "
            "file.")
        self._token_edit.editingFinished.connect(self._commit_connection)
        grid.addWidget(QLabel("Token"), 1, 0)
        grid.addWidget(self._token_edit, 1, 1)
        lay.addLayout(grid)

        self._token_note = QLabel("")
        self._token_note.setWordWrap(True)
        self._token_note.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
        lay.addWidget(self._token_note)

        row = QHBoxLayout()
        row.setSpacing(s(8))
        self._btn_discover = QPushButton("↓ Test connection & discover recipes")
        self._btn_discover.setToolTip(
            "Ask the hub what it is and what it can run (GET /hello, "
            "GET /workflows). Nothing is uploaded and nothing is cached to "
            "disk — recipe names and knob bounds are read fresh each time so a "
            "rename on the hub cannot leave MEBP pointing at a recipe that is "
            "gone.")
        self._btn_discover.clicked.connect(self.discover)
        row.addWidget(self._btn_discover)
        row.addStretch(1)
        lay.addLayout(row)

        self._hub_note = QLabel("Not contacted yet.")
        self._hub_note.setWordWrap(True)
        self._hub_note.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        lay.addWidget(self._hub_note)
        return box

    def _build_master_group(self) -> QGroupBox:
        box = QGroupBox("Automatic sending")
        lay = QVBoxLayout(box)
        lay.setSpacing(s(6))
        self._master = QCheckBox(
            "Send outputs to LabLink as they are produced")
        self._master.setToolTip(
            "The master switch. Off, every publish point in the app is a "
            "no-op — pushing lab images off the machine is not something to "
            "start doing without being asked.")
        self._master.toggled.connect(self._commit_master)
        lay.addWidget(self._master)
        note = QLabel(
            "Uploads are held while a print or a scan is running, then resume — "
            "sustained socket writes alongside the serial link are how a "
            "\"the printer randomly disconnects\" report starts. Retries happen "
            "only within this session: if MEBP closes with work queued, that "
            "work is reported as unsent rather than silently resumed later.")
        note.setWordWrap(True)
        note.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
        lay.addWidget(note)
        return box

    def _build_queue_group(self) -> QGroupBox:
        box = QGroupBox("Recent jobs")
        lay = QVBoxLayout(box)
        lay.setSpacing(s(4))
        self._queue_summary = QLabel("Nothing sent yet.")
        self._queue_summary.setStyleSheet(f"color: {COLORS['subtext0']};")
        lay.addWidget(self._queue_summary)
        self._queue_body = QWidget()
        self._queue_lay = QVBoxLayout(self._queue_body)
        self._queue_lay.setContentsMargins(0, 0, 0, 0)
        self._queue_lay.setSpacing(s(2))
        lay.addWidget(self._queue_body)
        return box

    # ── Load / commit ─────────────────────────────────────────────

    def load(self) -> None:
        self._url_edit.setText(self.config.base_url())
        source = self.config.token_source()
        if source == "environment":
            self._token_edit.setText("")
            self._token_edit.setEnabled(False)
            self._token_note.setText(
                "The token is coming from the LABLINK_TOKEN environment "
                "variable, which wins over anything stored here. That is the "
                "safer arrangement — nothing secret is written to disk.")
        else:
            self._token_edit.setEnabled(True)
            self._token_edit.setText(self.config.token())
            self._token_note.setText(
                "Stored in config/hardware/lablink.json, which is excluded from "
                "git. It is still a plaintext secret on this machine — set "
                "LABLINK_TOKEN in the environment to avoid the file entirely."
                if source == "file" else
                "No token set. Ask the hub's operator for the site token, or "
                "set LABLINK_TOKEN in the environment.")
        self._master.setChecked(self.config.enabled())
        for panel in self._panels.values():
            panel.load()
        self.refresh_status()
        self.refresh_queue()

    def _commit_connection(self) -> None:
        self.config.set_base_url(self._url_edit.text())
        if self._token_edit.isEnabled():
            self.config.set_token(self._token_edit.text())
        self._token_note_refresh()
        self.refresh_status()

    def _token_note_refresh(self) -> None:
        if self.config.token_source() == "file" and self.config.token():
            self._token_note.setText(
                "Stored in config/hardware/lablink.json, which is excluded "
                "from git. It is still a plaintext secret on this machine.")

    def _commit_master(self, on: bool) -> None:
        self.config.set_enabled(bool(on))
        if on:
            self._ensure_service()
        self.refresh_status()

    # ── Discovery ─────────────────────────────────────────────────

    def discover(self) -> None:
        if not self.config.is_configured():
            self._hub_note.setText(
                "Set an address and a token first — both are needed to ask the "
                "hub anything.")
            return
        self._btn_discover.setEnabled(False)
        self._hub_note.setText("Contacting the hub…")
        self._probe.probe(self._client_factory)

    def _client_factory(self):
        service = self._ensure_service()
        return service._client_factory()      # noqa: SLF001 - one owner

    def _on_probe_finished(self, ok: bool, message: str, info, flows) -> None:
        self._btn_discover.setEnabled(True)
        self.hello = dict(info or {})
        if not ok:
            self.discovered = []
            self._hub_note.setText(f"⚠ {message}")
            return
        self.discovered = list(flows or [])
        self._hub_note.setText(self._describe_hub())
        # Re-load the panels so the freshly discovered recipes and knob schemas
        # populate, without disturbing what the operator already chose.
        for panel in self._panels.values():
            panel.load()
        self.refresh_status()

    def _describe_hub(self) -> str:
        caps = self.hello.get("capabilities") or {}
        bits = [f"protocol {self.hello.get('protocol', '?')}"]
        max_file = self.hello.get("max_file_bytes")
        if max_file:
            bits.append(f"max upload {float(max_file) / (1024 * 1024):.0f} MB")
        if caps.get("longpoll_max_s"):
            bits.append(f"long poll {caps['longpoll_max_s']}s")
        n_recipes = sum(len(f.get("recipes") or []) for f in self.discovered)
        bits.append(f"{len(self.discovered)} workflow(s), "
                    f"{n_recipes} recipe(s)")
        text = "Connected — " + " · ".join(bits) + "."
        if not caps.get("recipe_metadata_requirements"):
            # Fail closed: absent means "cannot tell you", not "requires
            # nothing". Reading it the other way is failing OPEN against an
            # older hub, which finishes with different numbers and no warning.
            text += (" This hub does not publish which metadata each recipe "
                     "needs, so an empty list above means unknown, not none.")
        unusable = [f"{u.get('name')} ({u.get('problem')})"
                    for f in self.discovered
                    for u in (f.get("unusable_recipes") or [])]
        if unusable:
            text += "  Unusable on the hub: " + "; ".join(unusable) + "."
        return text

    # ── Service ───────────────────────────────────────────────────

    def _ensure_service(self):
        if self._service is None:
            from SupportClasses.LabLinkService import get_service
            self._service = get_service(store=self.config,
                                        busy_check=self._machine_busy)
        if self._bridge is None:
            from gui.widgets.lablink_bridge import LabLinkBridge
            self._bridge = LabLinkBridge(self._service, self)
            self._bridge.changed.connect(self.refresh_queue)
        return self._service

    def _machine_busy(self) -> bool:
        """True while the machine must not be disturbed.

        Hashing a few hundred MB and writing a socket alongside the serial link
        is enough to starve a reader and trip a disconnect watchdog, and the
        report would read as "the printer randomly disconnects since v7.17".
        """
        ctrl = self._controller
        if ctrl is None:
            return False
        for attr in ("is_printing", "is_print_running"):
            try:
                probe = getattr(ctrl, attr, None)
                if probe is None:
                    continue
                return bool(probe() if callable(probe) else probe)
            except Exception:
                continue
        return False

    # ── Status / queue ────────────────────────────────────────────

    def refresh_status(self) -> None:
        if not self.config.is_configured():
            self._status.setText("Not configured")
            self._status.setStyleSheet(f"color: {COLORS['subtext0']};")
            return
        if not self.config.enabled():
            self._status.setText("Off")
            self._status.setStyleSheet(f"color: {COLORS['subtext0']};")
            return
        armed = [self._source_labels.get(n, n) for n in self._sources
                 if self.config.source_enabled(n)]
        if not armed:
            self._status.setText("On, but no source is armed")
            self._status.setStyleSheet(f"color: {COLORS['peach']};")
            return
        self._status.setText(f"On — {len(armed)} source(s)")
        self._status.setStyleSheet(f"color: {COLORS['green']};")

    def refresh_queue(self) -> None:
        from SupportClasses.LabLinkService import peek_service
        service = self._service or peek_service()
        while self._queue_lay.count():
            item = self._queue_lay.takeAt(self._queue_lay.count() - 1)
            w = item.widget()
            if w is not None:
                w.setParent(None)
        if service is None:
            self._queue_summary.setText("Nothing sent yet.")
            return
        snap = service.snapshot(limit=20)
        jobs = list(snap.get("jobs") or [])
        counts = snap.get("counts") or {}
        summary = ", ".join(f"{v} {k}" for k, v in sorted(counts.items()))
        pending = snap.get("pending") or 0
        if snap.get("dropped_full"):
            summary += (f" · {snap['dropped_full']} dropped because the queue "
                        f"was full — those outputs were NOT sent")
        self._queue_summary.setText(
            (summary or "Nothing sent yet.")
            + (f" · {pending} waiting" if pending else ""))
        for job in reversed(jobs):
            self._queue_lay.addWidget(self._job_row(job))

    def _job_row(self, job: dict) -> QWidget:
        state = str(job.get("state") or "")
        spec = job.get("spec") or {}
        what = self._source_labels.get(spec.get("source"), spec.get("source"))
        target = spec.get("well") or spec.get("plate_key") or spec.get("path") or ""
        recipe = job.get("recipe") or ""
        line = f"{state:<10} {what}"
        if target:
            line += f" · {target}"
        if recipe:
            line += f" · {recipe}"
        if job.get("upload_stem"):
            line += f" · {job['upload_stem']}"
        detail = str(job.get("detail") or "")
        missing = job.get("missing_metadata") or ()
        if missing:
            detail = (detail + "  " if detail else "") + (
                "sidecar could not state: " + ", ".join(str(m) for m in missing))
        lbl = QLabel(line + (f"\n    {detail}" if detail else ""))
        lbl.setWordWrap(True)
        lbl.setTextInteractionFlags(Qt.TextSelectableByMouse)
        colour = COLORS.get(STATE_COLORS.get(state, "text"), COLORS["text"])
        lbl.setStyleSheet(f"color: {colour}; font-size: {sf(8)}pt;")
        lbl.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)
        return lbl

    # ── Workflow-page contract ────────────────────────────────────

    def showEvent(self, event):  # noqa: N802 (Qt override)
        super().showEvent(event)
        self.load()
        if self.config.enabled():
            self._ensure_service()

    def get_page_title(self) -> str:
        return "LabLink Processing"

    def get_context_widget(self):
        """No jog panel: this page commands no motion."""
        return None

    def set_hardware_config(self, config) -> None:
        self._hw_config = config

    def set_calibration_data(self, *_a, **_k) -> None:
        return

    def on_status_update(self, *_a, **_k) -> None:
        return
