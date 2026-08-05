"""target_type_dialog.py — author a target type and its signature rule.

v7.9 (post-audit). Target types could ONLY be created by hand-editing JSON:
``TargetTypeStore.save_user`` / ``delete_user`` existed and were called from
nothing in ``gui/``, the user directory did not exist, and the panel's entire
authoring instruction was "add JSON files under
config/hardware/target_types/user/". A biologist wanting "bright in FITC but dim
in mCherry" had to invent an id and hand-write

    {"combinator": "all", "clauses": [{"imaging_channel": "FITC", …}, …]}

with the exact vocabulary from the store module — no colour picker, no channel
picker, no validation, and a trailing comma made the type vanish with no message.

Modelled on the preset-library editors this codebase already has (well types,
needle types): built-ins are never modified, a user entry SHADOWS a built-in of
the same id, Delete is disabled for a built-in, and editing a built-in becomes
"save as a user override" — which ``save_user``'s shadow-by-id contract already
provides for free.

⚠ The banner is gated on ``RULE_EVALUATION_IMPLEMENTED``, not hard-coded: rule
EVALUATION is a deliberate deferral (decision D3), so the dialog must say the
rule is recorded intent — and must stop saying it by itself the day an evaluator
lands. There is deliberately **no "test this rule" button**: an inert one reads as
"coming soon" and would get clicked, the same argument the trypsin card makes for
not shipping a disabled overlap checkbox.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QColorDialog, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QGridLayout, QHBoxLayout, QLabel, QLineEdit, QMessageBox, QPlainTextEdit,
    QPushButton, QRadioButton, QScrollArea, QVBoxLayout, QWidget,
)

from gui.scaling import s, sf
from gui.styles import COLORS
from gui.widgets.components import Card

from SupportClasses.TargetTypeStore import (
    COMBINATOR_ALL, COMBINATOR_ANY, METRIC_AUTO, METRIC_BACKGROUND_SIGMA,
    METRIC_INTENSITY, METRIC_PERCENTILE, RULE_EVALUATION_IMPLEMENTED,
    STATE_ANY, STATE_NEGATIVE, STATE_POSITIVE, SignatureClause, SignatureRule,
    TargetType, safe_id,
)

logger = logging.getLogger(__name__)


#: The operator's own words for each state — matching the store's reasoning that
#: the STATE is what only the operator can know, while the numeric level is what
#: the deferred evaluator gets to choose.
STATE_LABELS: tuple[tuple[str, str], ...] = (
    (STATE_POSITIVE, "is bright in"),
    (STATE_NEGATIVE, "is dim / absent in"),
    (STATE_ANY, "don't care about"),
)

#: Metric → the operator's words. AUTO is not offered here: it is expressed by
#: leaving the level at its special "let the software choose" value, so the
#: widget cannot describe a (threshold, metric) pair the dataclass would rewrite.
METRIC_LABELS: tuple[tuple[str, str], ...] = (
    (METRIC_BACKGROUND_SIGMA, "σ above background"),
    (METRIC_INTENSITY, "absolute brightness (0-255)"),
    (METRIC_PERCENTILE, "percentile of this channel"),
)

_AUTO_LEVEL_TEXT = "let the software choose"


def _small(text: str = "", color_key: str = "subtext0") -> QLabel:
    lbl = QLabel(text)
    lbl.setWordWrap(True)
    lbl.setStyleSheet(f"color: {COLORS[color_key]}; font-size: {sf(9)}pt;")
    return lbl


class _ClauseRow:
    """One clause: imaging channel · state · level · metric · remove."""

    def __init__(self, channels, on_change, on_remove):
        self._on_change = on_change

        self.channel = QComboBox()
        self.channel.setEditable(False)
        for name in channels:
            self.channel.addItem(name, name)
        self.channel.setMinimumWidth(s(140))
        self.channel.currentIndexChanged.connect(lambda *_: self._on_change())

        self.state = QComboBox()
        for value, label in STATE_LABELS:
            self.state.addItem(label, value)
        self.state.setMinimumWidth(s(150))
        self.state.currentIndexChanged.connect(lambda *_: self._changed())

        self.level = QDoubleSpinBox()
        self.level.setRange(0.0, 100000.0)
        self.level.setDecimals(2)
        self.level.setSingleStep(0.5)
        self.level.setValue(0.0)
        # 0 IS "auto": the store's own model is that a clause may assert only a
        # STATE and leave the level to the evaluator (threshold=None).
        self.level.setSpecialValueText(_AUTO_LEVEL_TEXT)
        self.level.setMinimumWidth(s(150))
        self.level.setToolTip(
            "How bright counts as bright. Leave it at “let the software choose” "
            "to assert only the state and let the (future) detector pick a level.")
        self.level.valueChanged.connect(lambda *_: self._changed())

        self.metric = QComboBox()
        for value, label in METRIC_LABELS:
            self.metric.addItem(label, value)
        self.metric.setMinimumWidth(s(170))
        self.metric.currentIndexChanged.connect(lambda *_: self._on_change())

        self.remove = QPushButton("✕")
        self.remove.setFixedWidth(s(28))
        self.remove.setCursor(Qt.CursorShape.PointingHandCursor)
        self.remove.setToolTip("Remove this channel from the rule.")
        self.remove.clicked.connect(lambda: on_remove(self))

        self._changed()

    def _changed(self):
        """Keep the widgets from expressing a state the dataclass would rewrite."""
        auto = self.level.value() <= 0.0
        dont_care = self.state.currentData() == STATE_ANY
        # A metric means nothing without a number, and nothing at all for a
        # "don't care" clause — `SignatureClause.__post_init__` would collapse
        # both, so grey them rather than let the UI imply otherwise.
        self.metric.setEnabled(not auto and not dont_care)
        self.level.setEnabled(not dont_care)
        self._on_change()

    def widgets(self):
        return (self.channel, self.state, self.level, self.metric, self.remove)

    def to_clause(self) -> SignatureClause:
        auto = self.level.value() <= 0.0
        return SignatureClause(
            imaging_channel=self.channel.currentData() or "",
            state=self.state.currentData() or STATE_POSITIVE,
            threshold=None if auto else float(self.level.value()),
            metric=(METRIC_AUTO if auto
                    else (self.metric.currentData() or METRIC_INTENSITY)),
        )

    def load(self, clause: SignatureClause) -> None:
        name = clause.imaging_channel
        idx = self.channel.findData(name)
        if idx < 0 and name:
            # A rule authored on another rig may name a channel this one does not
            # have. The store's contract is REPORT, NEVER REPAIR — so keep it.
            self.channel.addItem(f"{name} (not on this rig)", name)
            idx = self.channel.findData(name)
        if idx >= 0:
            self.channel.setCurrentIndex(idx)
        idx = self.state.findData(clause.state)
        if idx >= 0:
            self.state.setCurrentIndex(idx)
        self.level.setValue(0.0 if clause.threshold is None
                            else float(clause.threshold))
        idx = self.metric.findData(clause.metric)
        if idx >= 0:
            self.metric.setCurrentIndex(idx)
        self._changed()


class TargetTypeDialog(QDialog):
    """Create or edit one target type.

    ``target_type=None`` creates; passing one edits (a built-in becomes a user
    override on save). ``result_type`` holds the saved type after ``accept()``.
    """

    def __init__(self, parent=None, *, target_type: TargetType | None = None,
                 store=None, existing_ids=(), prefill: TargetType | None = None):
        """``prefill`` populates the form from an existing type while still
        treating this as a NEW type — i.e. Duplicate. A fresh id is derived from
        the new name, so the original's id (which bore rows point at) is never
        reused."""
        super().__init__(parent)
        self._store = store
        self._existing_ids = {str(i) for i in existing_ids}
        self._editing = target_type
        self._is_new = target_type is None
        self._prefill = prefill
        self._rows: list[_ClauseRow] = []
        self.result_type: TargetType | None = None

        self.setWindowTitle("New target type" if self._is_new
                            else f"Edit target type — {target_type.label}")
        self.setMinimumWidth(s(560))

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(10), s(10), s(10), s(10))
        outer.setSpacing(s(8))

        # The deferral, stated where it cannot be missed — and self-retiring.
        if not RULE_EVALUATION_IMPLEMENTED:
            banner = _small(
                "⚠ Recorded intent only. This rule is saved and shown, but "
                "nothing yet selects objects from a fluorescence image using it "
                "— you still choose the targets by clicking them. Authoring it "
                "now means the run records WHAT you were after.", "peach")
            outer.addWidget(banner)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        body = QWidget()
        body_lay = QVBoxLayout(body)
        body_lay.setContentsMargins(0, 0, 0, 0)
        body_lay.setSpacing(s(8))

        body_lay.addWidget(self._build_identity_card())
        body_lay.addWidget(self._build_rule_card())
        body_lay.addStretch(1)
        scroll.setWidget(body)
        outer.addWidget(scroll, stretch=1)

        self._preview = _small("")
        outer.addWidget(self._preview)

        self._buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save
            | QDialogButtonBox.StandardButton.Cancel)
        self._buttons.accepted.connect(self._on_save)
        self._buttons.rejected.connect(self.reject)
        outer.addWidget(self._buttons)

        if target_type is not None:
            self._load(target_type)
        elif prefill is not None:
            self._load(prefill)
            self._name.setText(f"{prefill.label} (copy)")
        else:
            self._add_clause()
        self._refresh_preview()

    # ── build ─────────────────────────────────────────────────────

    def _build_identity_card(self) -> Card:
        card = Card("What is it?")
        grid = QGridLayout()
        grid.setHorizontalSpacing(s(8))
        grid.setVerticalSpacing(s(6))

        self._name = QLineEdit()
        self._name.setPlaceholderText("e.g. GFP-only cells")
        self._name.textChanged.connect(lambda *_: self._refresh_preview())
        grid.addWidget(QLabel("Name"), 0, 0)
        grid.addWidget(self._name, 0, 1)

        self._color = "#cba6f7"
        self._color_btn = QPushButton("")
        self._color_btn.setFixedWidth(s(56))
        self._color_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._color_btn.setToolTip("Colour used for this type's markers.")
        self._color_btn.clicked.connect(self._pick_color)
        self._color_lbl = _small(self._color)
        crow = QHBoxLayout()
        crow.setContentsMargins(0, 0, 0, 0)
        crow.addWidget(self._color_btn)
        crow.addWidget(self._color_lbl, stretch=1)
        cwrap = QWidget()
        cwrap.setLayout(crow)
        grid.addWidget(QLabel("Colour"), 1, 0)
        grid.addWidget(cwrap, 1, 1)

        self._notes = QPlainTextEdit()
        self._notes.setPlaceholderText("Optional — what this is, how you stain it…")
        self._notes.setFixedHeight(s(56))
        grid.addWidget(QLabel("Notes"), 2, 0, Qt.AlignmentFlag.AlignTop)
        grid.addWidget(self._notes, 2, 1)

        holder = QWidget()
        holder.setLayout(grid)
        card.add_widget(holder)
        self._apply_color_button()
        if not self._is_new:
            card.add_widget(_small(
                f"Id: {self._editing.id} — kept as-is. Renaming changes only the "
                f"display name, never the id, so bores already assigned to this "
                f"type keep their assignment."))
        return card

    def _build_rule_card(self) -> Card:
        card = Card("Fluorescent signature")
        # The store's own reasoning: "bright in BOTH" is ALL over two positive
        # clauses; "bright in ONE" is ALL over a positive plus a negative — which
        # is what makes "only that channel" unambiguous.
        self._all_radio = QRadioButton(
            "Match ALL of these — an object must satisfy every row")
        self._any_radio = QRadioButton(
            "Match ANY of these — satisfying one row qualifies")
        self._all_radio.setChecked(True)
        for r in (self._all_radio, self._any_radio):
            r.toggled.connect(lambda *_: self._refresh_preview())
            card.add_widget(r)

        holder = QWidget()
        self._grid = QGridLayout(holder)
        self._grid.setContentsMargins(0, s(4), 0, 0)
        self._grid.setHorizontalSpacing(s(8))
        self._grid.setVerticalSpacing(s(6))
        for col, text in enumerate(
                ("Imaging channel", "State", "Level", "Measured in", "")):
            head = QLabel(text)
            head.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt; "
                f"font-weight: 600;")
            self._grid.addWidget(head, 0, col)
        card.add_widget(holder)

        add = QPushButton("＋ Add channel")
        add.setCursor(Qt.CursorShape.PointingHandCursor)
        add.clicked.connect(lambda: (self._add_clause(), self._refresh_preview()))
        card.add_widget(add)

        self._rule_note = _small("")
        card.add_widget(self._rule_note)
        return card

    # ── channels ──────────────────────────────────────────────────

    @staticmethod
    def _channels() -> list[str]:
        try:
            from SupportClasses.FluorescenceMosaicStore import CHANNELS
            return list(CHANNELS)
        except Exception:
            logger.debug("imaging channels unavailable", exc_info=True)
            return []

    def _add_clause(self, clause: SignatureClause | None = None) -> _ClauseRow:
        row = _ClauseRow(self._channels(), self._refresh_preview,
                         self._remove_clause)
        self._rows.append(row)
        r = len(self._rows)
        for col, w in enumerate(row.widgets()):
            self._grid.addWidget(w, r, col)
        if clause is not None:
            row.load(clause)
        return row

    def _remove_clause(self, row: _ClauseRow) -> None:
        if row not in self._rows:
            return
        for w in row.widgets():
            self._grid.removeWidget(w)
            w.setParent(None)
            w.deleteLater()
        self._rows.remove(row)
        # Re-place the survivors so there is no hole in the grid.
        for i, r in enumerate(self._rows, start=1):
            for col, w in enumerate(r.widgets()):
                self._grid.addWidget(w, i, col)
        self._refresh_preview()

    # ── state ─────────────────────────────────────────────────────

    def _pick_color(self) -> None:
        chosen = QColorDialog.getColor(QColor(self._color), self,
                                       "Target type colour")
        if chosen.isValid():
            self._color = chosen.name()
            self._apply_color_button()
            self._refresh_preview()

    def _apply_color_button(self) -> None:
        self._color_btn.setStyleSheet(
            f"background-color: {self._color}; border: 1px solid "
            f"{COLORS['surface2']}; border-radius: {s(3)}px;")
        self._color_lbl.setText(self._color)

    def _load(self, tt: TargetType) -> None:
        self._name.setText(tt.name or tt.id)
        self._color = tt.color
        self._apply_color_button()
        self._notes.setPlainText(tt.notes or "")
        rule = tt.signature_rule
        (self._any_radio if rule.combinator == COMBINATOR_ANY
         else self._all_radio).setChecked(True)
        for clause in rule.clauses:
            self._add_clause(clause)
        if not rule.clauses:
            self._add_clause()

    def _build_rule(self) -> SignatureRule:
        return SignatureRule(
            combinator=(COMBINATOR_ANY if self._any_radio.isChecked()
                        else COMBINATOR_ALL),
            clauses=[r.to_clause() for r in self._rows],
        )

    def _refresh_preview(self, *_):
        rule = self._build_rule()
        # Reuse the store's own describe() so the preview and the card list can
        # never word the same rule differently.
        self._preview.setText(f"Preview:  {rule.describe()}")
        notes = []
        # ⚠ `is_empty` is a METHOD here, while its siblings `is_gating` and
        # `gating_clauses` are @property — so `if rule.is_empty:` is a bound
        # method, always truthy, and the "asserts nothing" warning fired on every
        # rule including complete ones. Caught by the round-trip smoke test.
        if rule.is_empty():
            # The store explains exactly why the fail-safe runs this way; this
            # dialog is the only place the operator will ever read it.
            notes.append(
                "⚠ This rule asserts nothing, so it matches NOTHING — not "
                "everything. Add at least one “is bright in” or “is dim / "
                "absent in” row.")
        seen: dict[str, list[str]] = {}
        for c in rule.clauses:
            seen.setdefault(c.imaging_channel, []).append(c.state)
        for name, states in seen.items():
            if name and STATE_POSITIVE in states and STATE_NEGATIVE in states:
                notes.append(
                    f"⚠ {name} is asked to be both bright and dim — one of those "
                    f"rows will never be satisfiable.")
        self._rule_note.setText("  ".join(notes))
        self._rule_note.setStyleSheet(
            f"color: {COLORS['peach'] if notes else COLORS['subtext0']}; "
            f"font-size: {sf(9)}pt;")

    # ── save ──────────────────────────────────────────────────────

    def _on_save(self) -> None:
        name = self._name.text().strip()
        if not name:
            QMessageBox.warning(
                self, "Name needed",
                "Give the target type a name — it is what the bore rows and the "
                "run record show.")
            return
        if self._is_new:
            new_id = safe_id(name.lower().replace(" ", "-"))
            if new_id in self._existing_ids:
                btn = QMessageBox.question(
                    self, "That id already exists",
                    f"A target type with the id “{new_id}” already exists.\n\n"
                    f"Overwrite it (a built-in stays intact — your version "
                    f"shadows it), or Cancel and pick another name?",
                    QMessageBox.StandardButton.Save
                    | QMessageBox.StandardButton.Cancel,
                    QMessageBox.StandardButton.Cancel)
                if btn != QMessageBox.StandardButton.Save:
                    return
        else:
            # NEVER re-derive the id on rename: it is what every bore assignment
            # and every stamped run record points at.
            new_id = self._editing.id

        tt = TargetType(
            id=new_id, name=name, color=self._color,
            notes=self._notes.toPlainText().strip(),
            signature_rule=self._build_rule(),
            builtin=False,          # an edit of a built-in becomes an override
        )
        if self._store is not None:
            try:
                ok = self._store.save_user(tt)
            except Exception as exc:
                logger.exception("save_user failed")
                QMessageBox.critical(self, "Could not save",
                                     f"Saving “{name}” failed:\n{exc}")
                return
            if not ok:
                QMessageBox.critical(
                    self, "Could not save",
                    f"Saving “{name}” failed — check that "
                    f"config/hardware/target_types/user/ is writable.")
                return
        self.result_type = tt
        self.accept()
