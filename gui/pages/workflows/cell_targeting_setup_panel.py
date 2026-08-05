"""cell_targeting_setup_panel.py — the Cell Targeting & Removal SETUP tab.

v7.9. The operator's framing (2026-08-01):

    "then in another tab in the cell targeting and removal workflow, we use it to
    setup the workflow. **the setup page is where we will do most of the options
    and establishment of what and how to do everything**, then the other page is
    used as a viewer for the work being done. … for now we are focusing on the
    user defines something for each pump channel to go and do something"

So this panel is the primary establishment surface, and its headline is the
**per-bore program table**: ONE ROW PER BORE of the configured needle assembly,
each row naming that bore's pump, its ROLE, and the class of object (target type)
it services. Per decision D6 there is exactly one role per bore and the run ORDER
is DERIVED from the roles (``BORE_ROLE_ORDER``) rather than authored as a step
list — a table the operator cannot misorder.

Three things this panel deliberately does NOT do:

* **It does not offer an "overlap" control.** Decision D5 puts the trypsin dose
  on a DIFFERENT bore from the aspirate, and fused bores are laterally offset, so
  the tool must physically travel between the push and the pull. Aspirating before
  the push finishes is not deferred — it is impossible. A disabled control would
  imply "coming later"; a tooltip that explains the geometry is honest.
* **It does not claim the signature rules work.** Per decision D3 target types and
  their fluorescent-signature rules are AUTHORED AND PERSISTED only; nothing
  evaluates them against an image yet (see ``SupportClasses/TargetTypeStore``).
  The card says so in the UI, because a picker that silently implies
  auto-detection would be read as a working detector.
* **It does not own the numbers that already have a home.** The assembly-wide
  values (removal/place Z, push/pull volumes and flows, incubation, prep) are
  created by the host page and merely LAID OUT here through :meth:`add_group`,
  then registered with the settings dialog via ``register_external`` — the v7.7
  pattern for a control that must live in front of the operator yet still ride
  along with the saved profile. A widget has exactly one parent, so the alternative
  would be two editing surfaces for one setting: the
  ``UNIFIED_MOSAIC_CALIBRATION`` failure.

The panel is GUI-only: it reads the needle assembly and the target-type store and
emits :attr:`programs_changed`; it commands no motion and writes no store.
"""

from __future__ import annotations

import logging
import math

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractSpinBox, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QComboBox, QDoubleSpinBox, QMessageBox, QPushButton, QScrollArea,
)

from gui.scaling import s, sf
from gui.styles import COLORS
from gui.widgets.components import Card, FormRow, ReadinessList

from SupportClasses.PhysicalModels import (
    needle_bore_at, needle_bore_count, needle_bore_offset_um,
    needle_bore_z_offset_mm,
)
from SupportClasses.PickAndPlaceManager import (
    BORE_ROLE_ORDER, BoreProgram, BoreRole,
)

logger = logging.getLogger(__name__)


#: Role → the words the operator sees. IDLE first, then the derived run order, so
#: the combo reads in the sequence the roles actually act in.
ROLE_LABELS: dict[BoreRole, str] = {
    BoreRole.IDLE: "Idle (unused this run)",
    BoreRole.PUSH_REAGENT: "Dose the target with reagent",
    BoreRole.ASPIRATE_TARGET: "Aspirate the target",
    BoreRole.DISPENSE_PLACE: "Dispense at the placement",
}

ROLE_ORDER_FOR_COMBO: tuple[BoreRole, ...] = (BoreRole.IDLE,) + BORE_ROLE_ORDER

#: Placeholder for "no target type chosen". An untyped bore is legal — a
#: single-bore run needs no classification (``stamp_target_type(None) == {}``).
NO_TARGET_TYPE = "(any / untyped)"

#: Sentinel for a bore whose pump the operator has not named.
NO_PUMP = "(unassigned)"

#: "No declaration has ever been seen for this bore" — distinct from a bore that
#: declares None, which is a legitimate value. See `_apply_remembered`.
_UNKNOWN = object()

#: Smallest dose the operator can enter, in nL. Below a picolitre nothing on this
#: class of hardware moves, and the field's own minimum is a better guard than a
#: magic zero: the old field accepted 0 to mean "derive from the depth instead",
#: which is exactly the ambiguity volume-primary entry removes.
_MIN_DOSE_NL = 0.001

#: nL per µL. The model layer is µL everywhere; this factor appears at the two
#: boundaries where the operator's number crosses into it, and nowhere else.
NL_PER_UL = 1000.0


def _small(text: str = "", color_key: str = "subtext0") -> QLabel:
    lbl = QLabel(text)
    lbl.setWordWrap(True)
    lbl.setStyleSheet(f"color: {COLORS[color_key]}; font-size: {sf(9)}pt;")
    return lbl


def _dspin(lo, hi, val, suffix="", decimals=2, step=None, tip="") -> QDoubleSpinBox:
    sb = QDoubleSpinBox()
    sb.setRange(lo, hi)
    sb.setDecimals(decimals)
    if suffix:
        sb.setSuffix(suffix)
    sb.setValue(val)
    if step is not None:
        sb.setSingleStep(step)
    if tip:
        sb.setToolTip(tip)
    sb.setMinimumWidth(s(78))
    return sb


class SetupGroup:
    """A titled card in the panel whose rows the HOST page fills.

    Mirrors ``WorkflowSettingsDialog.SettingsSection``'s surface (``add`` /
    ``add_widget`` / ``add_note``) on purpose: the page's layout code then reads
    the same whether a field is laid out in the popout or promoted onto Setup, so
    promoting one is a one-line change and cannot silently drop its label.
    """

    def __init__(self, card: Card, help_rows: list | None = None):
        self._card = card
        # The FormRows this group builds, so the panel can register them with the
        # MainWindow's Help toggle. Without that registration every `help_text`
        # here is DEAD WEIGHT: FormRow hides its help label at construction and
        # only `MainWindow.register_form_row` ever reveals it, so turning the
        # top-bar Help toggle on changed nothing at all on this tab.
        self._help_rows = help_rows if help_rows is not None else []

    def add(self, label: str, widget: QWidget,
            help_text: str | None = None) -> QWidget:
        text = help_text
        if not text:
            try:
                text = widget.toolTip() or None
            except Exception:
                text = None
        row = FormRow(label, widget, help_text=text)
        self._card.add_widget(row)
        self._help_rows.append(row)
        return widget

    def add_widget(self, widget: QWidget) -> QWidget:
        self._card.add_widget(widget)
        return widget

    def add_note(self, text: str) -> QLabel:
        lbl = _small(text)
        self._card.add_widget(lbl)
        return lbl

    def card(self) -> Card:
        return self._card


class _BoreRow:
    """The widgets of ONE row of the per-bore program table.

    Not a QWidget: the row's cells are placed directly into the table's shared
    ``QGridLayout`` so every column lines up across bores (the aligned-grid
    lesson from ``jog_button_array``; a per-row HBox gives a ragged table).
    """

    def __init__(self, bore_index: int, on_change):
        self.bore_index = int(bore_index)
        self._on_change = on_change

        self.bore_label = QLabel()
        self.bore_label.setTextFormat(Qt.TextFormat.RichText)
        self.bore_label.setMinimumWidth(s(120))

        self.pump = QComboBox()
        self.pump.setToolTip(
            "Which syringe pump feeds this bore. Taken from the needle assembly "
            "when Hardware Setup → Needle assigns one; set it here otherwise.")
        self.pump.setMinimumWidth(s(88))

        self.role = QComboBox()
        for r in ROLE_ORDER_FOR_COMBO:
            self.role.addItem(ROLE_LABELS[r], r.value)
        self.role.setToolTip(
            "What this bore does during a run. The order the roles act in is "
            "derived (push → aspirate → dispense), so there is nothing to "
            "misorder here.")
        self.role.setMinimumWidth(s(150))

        self.target_type = QComboBox()
        self._type_tip = (
            "The user-defined class of object this bore services (Target types "
            "below). Recorded intent — the fluorescent-signature rules are not "
            "evaluated against an image yet.")
        self.target_type.setToolTip(self._type_tip)
        self.target_type.setMinimumWidth(s(130))

        # ⚠ VOLUME-PRIMARY, in nL. A dose is what the operator is choosing; the
        # column depth is an implementation detail of how it gets metered, and
        # "0.100 mm through this bore" is not a quantity anyone can reason about.
        # µL was also the wrong unit: a real dose through a 200 µm bore is
        # ~0.003 µL, so a 4-decimal µL field printed "0.0031" and a 30 µm bore
        # printed "0.0001" — indistinguishable from zero.
        self.volume = _dspin(
            _MIN_DOSE_NL, 2_000_000.0, 3.142, " nL", 3, 0.5,
            "Reagent volume this bore doses onto the target. Capped at what the "
            "bore itself holds; the column depth beside it is derived from this "
            "bore's own orifice area.")
        # Derived, read-only: the number the executor meters with, shown so the
        # geometry stays visible, but not a second place to type the same dose.
        self.depth = _dspin(
            0.0, 500.0, 0.10, " mm", 4, 0.0,
            "Column depth this dose corresponds to through this bore's orifice "
            "— derived from the volume, not typed.")
        self.depth.setReadOnly(True)
        self.depth.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.rate = _dspin(
            0.01, 50.0, 0.5, " µL/s", 2, 0.1,
            "Flow used for this bore's dose (⇒ dose duration = volume / rate). "
            "Clamped to this bore's own Hagen-Poiseuille ceiling; the readout "
            "shows the rate that will actually run.")
        self.lead = _dspin(
            0.0, 3600.0, 0.0, " s", 1, 0.5,
            "EXTRA time after the shift, ADDED to the incubation below — the "
            "full dose→aspirate interval is lead + incubation.")

        self.offset = _small()
        self.offset.setMinimumWidth(s(140))

        #: This bore's orifice area (mm²), pushed in by ``_refresh_row_labels``.
        #: 0 = unknown, in which case the derived depth is simply not shown —
        #: never guessed, because a guessed area is a wrong metered volume.
        self._area_mm2: float = 0.0

        #: A legacy profile's ``depth_mm``, held until the bore geometry arrives.
        #: The migration CANNOT complete at load time: the profile is restored
        #: before ``set_hardware_config``, so there is no orifice area yet to turn
        #: a depth into a volume. None = nothing pending.
        self._legacy_depth_mm: float | None = None

        #: The push cells' own tooltips, so re-gating can put them back instead
        #: of leaving the "not used by this role" note on a row that IS pushing.
        self._push_tips = {w: w.toolTip()
                           for w in (self.volume, self.depth,
                                     self.rate, self.lead)}

        # A role change re-gates this row's own parameter cells, so it must
        # refresh before the host is told.
        self.role.currentIndexChanged.connect(self._on_role_changed)
        for w in (self.pump, self.target_type):
            w.currentIndexChanged.connect(lambda *_: self._on_change())
        self.volume.valueChanged.connect(self._on_volume_changed)
        for w in (self.rate, self.lead):
            w.valueChanged.connect(lambda *_: self._on_change())
        # `depth` is derived and read-only, so it has no change handler: it is
        # written by `_sync_depth` and read back by `_collect_programs`.

    # ── volume ⇄ derived depth ────────────────────────────────────

    def _on_volume_changed(self, *_):
        self._sync_depth()
        self._on_change()

    def set_orifice_area_mm2(self, area) -> None:
        """Tell the row this bore's orifice area, then re-derive the depth."""
        try:
            self._area_mm2 = max(0.0, float(area or 0.0))
        except (TypeError, ValueError):
            self._area_mm2 = 0.0
        self._sync_depth()

    def _sync_depth(self) -> None:
        """depth = volume / orifice area, in the executor's own units."""
        if self._area_mm2 <= 0:
            self.depth.setSpecialValueText("— (bore Ø unknown)")
            self.depth.blockSignals(True)
            self.depth.setValue(0.0)
            self.depth.blockSignals(False)
            return
        self.depth.setSpecialValueText("")
        mm = (self.volume.value() / NL_PER_UL) / self._area_mm2
        self.depth.blockSignals(True)
        self.depth.setValue(min(mm, self.depth.maximum()))
        self.depth.blockSignals(False)

    def resolve_legacy_depth(self) -> bool:
        """Turn a pending legacy ``depth_mm`` into a dose, once, when it can be.

        Returns True if a migration actually happened. Called from
        ``_refresh_row_labels``, i.e. the first moment the bore's orifice area is
        known. Deliberately one-shot: after this the VOLUME is authoritative, so
        changing bores holds the dose and re-derives the depth — the whole point
        of entering a volume.
        """
        if self._legacy_depth_mm is None or self._area_mm2 <= 0:
            return False
        mm, self._legacy_depth_mm = self._legacy_depth_mm, None
        if mm <= 0:
            return False
        self.set_volume_uL(self._area_mm2 * mm)
        logger.info("Bore %d: migrated a saved dose depth of %.4f mm to %.3f nL "
                    "through its own orifice.", self.bore_index + 1, mm,
                    self.volume.value())
        return True

    def volume_uL(self) -> float:
        """The dose in the model's own unit."""
        return float(self.volume.value()) / NL_PER_UL

    def set_volume_uL(self, uL) -> None:
        """Write a µL dose into the nL field without re-entering the host."""
        try:
            nL = float(uL) * NL_PER_UL
        except (TypeError, ValueError):
            return
        self.volume.blockSignals(True)
        self.volume.setValue(max(self.volume.minimum(),
                                 min(nL, self.volume.maximum())))
        self.volume.blockSignals(False)
        self._sync_depth()

    # ── read ──────────────────────────────────────────────────────

    def role_value(self) -> BoreRole:
        raw = self.role.currentData()
        try:
            return BoreRole(str(raw))
        except (ValueError, TypeError):
            return BoreRole.IDLE

    def pump_id(self) -> str:
        return self.pump.currentData() or ""

    def target_type_id(self) -> str:
        return self.target_type.currentData() or ""

    # ── write ─────────────────────────────────────────────────────

    def _on_role_changed(self, *_):
        self.refresh_enabled()
        self._on_change()

    def refresh_enabled(self) -> None:
        """Only the pushing bore uses the push volume / depth / rate / lead.

        The other roles' flows are assembly-wide (one slow push, one fast pull —
        see the "Reagent push / pull" group), so showing per-row copies of them
        would be a second editing surface for one number.
        """
        pushing = self.role_value() is BoreRole.PUSH_REAGENT
        active = self.role_value().is_active
        self.target_type.setEnabled(active)
        # A disabled control with no explanation reads as broken. The four dose
        # cells already swapped in a reason; the target-type combo did not.
        self.target_type.setToolTip(
            self._type_tip if active else
            "Only a bore with a role services a class of object — set this "
            "bore's role first.")
        idle_tip = (
            "Only a “Dose the target” bore uses these — the aspirate and "
            "dispense flows are set once for the whole assembly under the "
            "aspirating bore's column group.")
        for w in (self.volume, self.depth, self.rate, self.lead):
            w.setEnabled(pushing)
            w.setToolTip(self._push_tips[w] if pushing else idle_tip)

    def set_holdup_uL(self, holdup_uL) -> None:
        """Cap the dose at what this bore physically holds. Takes µL, caps in nL.

        A field whose ceiling is orders of magnitude above the bore's holdup makes
        a decimal-place slip ask for far more than the bore can contain. The
        plunger then soft-clamps, which silently SHORTENS the move and breaks the
        load/dose/pull/dispense volume balance with no report. Skipped when the
        holdup is unknown — a missing getter must never forbid work.
        """
        try:
            cap_uL = float(holdup_uL)
        except (TypeError, ValueError):
            return
        if cap_uL <= 0:
            return
        # ⚠ FLOOR to the field's own precision. `setMaximum` on a 3-decimal spin
        # box rounds, and a cap that rounds UP is not a cap — it would admit a
        # dose fractionally larger than the bore holds, which is the very thing
        # this guards against.
        q = 10.0 ** self.volume.decimals()
        cap_nL = math.floor(cap_uL * NL_PER_UL * q) / q
        current = self.volume.value()
        self.volume.setMaximum(max(cap_nL, _MIN_DOSE_NL))
        if current > cap_nL:
            logger.info(
                "Bore %d's dose %.3f nL exceeds what the bore holds "
                "(%.3f nL) — capped.", self.bore_index + 1, current, cap_nL)
            self._sync_depth()


class CellTargetingSetupPanel(QWidget):
    """Setup tab: per-bore program table + target types + trypsin timing.

    Signals:
        programs_changed: A role / pump / target type / push parameter changed.
    """

    programs_changed = Signal()
    #: The operator asked to go pick targets on the other tab.
    goto_targets_requested = Signal()

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._needle = None
        self._pump_ids: list[str] = []
        self._rows: list[_BoreRow] = []
        #: bore_index → the last program dict seen for it. Rebuilding the table
        #: (a needle change, a restored profile) must not silently wipe the
        #: operator's role assignments — the preserve-and-restore pattern the
        #: channel-count spin was missing (v7.9 hard-parts (f)/4).
        self._remembered: dict[int, dict] = {}
        #: bore_index → the pump the ASSEMBLY declared for it last time the table
        #: was built. Used to tell "the operator hand-wired this bore" (keep it)
        #: from "the assembly was re-wired / a different needle was mounted"
        #: (hardware truth wins) — see :meth:`_apply_remembered`.
        self._declared_pumps: dict[int, str] = {}
        #: The target type selected in the library list ("" = none).
        self._selected_tt_id: str = ""
        #: Every FormRow this panel builds, for the Help toggle.
        self._help_rows: list = []
        #: pump id → its own flow ceiling (µL/s), pushed in by the host.
        #: The panel never reaches for the controller itself.
        self._flow_ceilings: dict[str, float] = {}
        self._target_types: list = []
        self._rebuilding = False

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        body = QWidget()
        cols = QHBoxLayout(body)
        cols.setContentsMargins(s(2), s(2), s(2), s(2))
        cols.setSpacing(s(10))

        self._left = QVBoxLayout()
        self._left.setSpacing(s(10))
        self._right = QVBoxLayout()
        self._right.setSpacing(s(10))
        cols.addLayout(self._left, stretch=3)
        cols.addLayout(self._right, stretch=2)

        # FIRST, because on a half-configured machine it is the only thing that
        # matters: what to do next, in one place, instead of a disabled button
        # with no tooltip and a status line that says "Idle."
        self._left.addWidget(self._build_readiness_card())
        self._left.addWidget(self._build_bore_table_card())
        self._left.addWidget(self._build_target_types_card())
        self._left.addWidget(self._build_trypsin_card())
        self._left.addStretch(1)
        # The host's promoted groups land in the right column, under this
        # heading, so the "what/how much/how fast" establishment reads as one
        # block beside the per-bore assignment.
        # (No heading paragraph: the groups below are self-titled and "saved
        # with the profile" is what ⚙ Settings in the header already says.)
        self._right_stretch_added = False

        scroll.setWidget(body)
        outer.addWidget(scroll)

        self.reload_target_types()
        self._rebuild_rows()

    # ── host-filled groups ────────────────────────────────────────

    def add_group(self, title: str, *, collapsible: bool = False,
                  collapsed: bool = False) -> SetupGroup:
        """A card in the right-hand column for the host's promoted fields.

        ``collapsible``/``collapsed``: for the settings an operator configures
        once and then stops reading. Note the widgets inside a collapsed card are
        STILL PARENTED and still read by ``_current_config`` — collapsing changes
        what claims vertical space, never what governs a run.
        """
        card = Card(title, collapsible=collapsible)
        if collapsed:
            card.set_collapsed(True)
        if self._right_stretch_added:
            # Keep the trailing stretch last no matter when a group is added.
            self._right.insertWidget(self._right.count() - 1, card)
        else:
            self._right.addWidget(card)
        return SetupGroup(card, self._help_rows)

    def finalize(self) -> None:
        """Call once the host has added every group (adds the trailing stretch)."""
        if not self._right_stretch_added:
            self._right.addStretch(1)
            self._right_stretch_added = True

    # ── per-bore program table ────────────────────────────────────

    # ── readiness ─────────────────────────────────────────────────

    def _build_readiness_card(self) -> Card:
        """"What to do next" — the checklist plus the jump to the other tab.

        The single most-missing element in this feature was any statement that the
        workflow's entire input (which cells to remove) lives on a DIFFERENT tab.
        A first-time operator could complete this whole page and find Start
        greyed out with no tooltip and no explanation anywhere.
        """
        card = Card("Readiness — what to do next")
        self.readiness_list = ReadinessList()
        card.add_widget(self.readiness_list)

        self.goto_targets_btn = QPushButton("Pick the cells  →")
        # The 42-word ordering essay that used to sit under this button is now
        # its tooltip: the operator asks for it by hovering the thing they are
        # about to press, instead of reading it every visit forever. The
        # readiness checklist above already names the NEXT missing step, which is
        # the part that actually changes.
        self.goto_targets_btn.setToolTip(
            "Bring the well into view and click each cell to remove, then a "
            "placement for each one.\n\n"
            "Order: needle form + bore geometry (Hardware Setup → Needle) → "
            "bore offsets (Calibration → Needle Location) → a role per bore "
            "below → pick the cells.")
        self.goto_targets_btn.clicked.connect(
            lambda: self.goto_targets_requested.emit())
        card.add_widget(self.goto_targets_btn)
        return card

    def _build_bore_table_card(self) -> Card:
        card = Card("Needle assembly — one program per bore")
        card.setToolTip(
            "One row per bore of the configured assembly (Hardware Setup → "
            "Needle). Give each bore a role; the run order is derived "
            "(push → aspirate → dispense), so there is nothing to misorder.")

        holder = QWidget()
        self._grid = QGridLayout(holder)
        self._grid.setContentsMargins(0, 0, 0, 0)
        self._grid.setHorizontalSpacing(s(8))
        self._grid.setVerticalSpacing(s(6))
        # ⚠ "Dose", not "Push". The right-hand column has its OWN "Push depth"
        # (the ASPIRATING bore's reagent column), and two live controls with the
        # same label, the same units and different meanings, 15 cm apart, is not a
        # naming nit — it is a wrong-number-entered-in-the-wrong-field waiting to
        # happen. "Dose" = what the dosing bore ejects onto the cell;
        # "push/pull" = the aspirating bore's column and extraction.
        headers = ("Bore", "Pump", "Role", "Target type",
                   "Dose", "≙ depth", "Dose flow", "Lead time",
                   "Mount offset")
        self._headers: list[QLabel] = []
        for col, text in enumerate(headers):
            head = QLabel(text)
            head.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt; "
                f"font-weight: 600;")
            self._grid.addWidget(head, 0, col)
            self._headers.append(head)
        self._header_row_count = 1
        card.add_widget(holder)

        self._bore_empty = _small(
            "No needle configured yet — set the needle form and its bores on "
            "Hardware Setup → Needle.", "peach")
        card.add_widget(self._bore_empty)

        # Arming a dosing bore is now deliberate (see `_default_role`), so make
        # the intended workflow ONE click rather than zero — otherwise "safe by
        # default" just becomes "hard to use".
        self._arm_dose_row = QWidget()
        arm = QHBoxLayout(self._arm_dose_row)
        arm.setContentsMargins(0, 0, 0, 0)
        arm.setSpacing(s(8))
        self._arm_dose_btn = QPushButton("")
        self._arm_dose_btn.setToolTip(
            "Give this bore the dosing role: it loads reagent from its own well, "
            "doses the cell, then the tool shifts so the aspirating bore is over "
            "the same cell. The aspirating bore then loads nothing.")
        self._arm_dose_btn.clicked.connect(self._arm_dosing_bore)
        arm.addWidget(self._arm_dose_btn)
        arm.addStretch(1)
        self._arm_dose_row.setVisible(False)
        card.add_widget(self._arm_dose_row)

        self._bore_notes = _small("")
        card.add_widget(self._bore_notes)

        # Help-mode substitute for per-cell FormRows (see set_help_mode).
        self._help_legend = _small(
            "Bore — one lumen of the assembly, with its measured geometry. "
            "Pump — the syringe that feeds it. "
            "Role — what it does this run; the order is derived "
            "(dose → aspirate → dispense), so there is nothing to misorder. "
            "Target type — the class of object it services (recorded intent; "
            "nothing detects it yet). "
            "Dose, ≙ depth, Dose flow and Lead time — used only by a DOSING "
            "bore. Type the dose as a volume in nL; the depth beside it is the "
            "column that corresponds to through this bore's own orifice, and is "
            "derived rather than typed. The lead time is ADDED to the "
            "incubation. Mount offset — this bore's measured lateral offset from "
            "bore 1, without which it is driven to bore 1's position.")
        self._help_legend.setVisible(False)
        card.add_widget(self._help_legend)
        return card

    def _arm_dosing_bore(self) -> None:
        """Give the first idle non-datum bore the dosing role."""
        for row in self._rows:
            if row.bore_index == 0:
                continue
            if row.role.currentData() == BoreRole.IDLE.value:
                idx = row.role.findData(BoreRole.PUSH_REAGENT.value)
                if idx >= 0:
                    row.role.setCurrentIndex(idx)   # fires _on_row_changed
                return

    def _refresh_arm_dose_row(self) -> None:
        """Offer the one-click arm only when it is actually available."""
        if not hasattr(self, "_arm_dose_row"):
            return
        candidate = None
        has_doser = False
        for row in self._rows:
            data = row.role.currentData()
            if data == BoreRole.PUSH_REAGENT.value:
                has_doser = True
            elif (row.bore_index != 0 and data == BoreRole.IDLE.value
                  and candidate is None):
                candidate = row.bore_index
        show = (len(self._rows) >= 2) and not has_doser and candidate is not None
        self._arm_dose_row.setVisible(show)
        if show:
            self._arm_dose_btn.setText(
                f"Use bore {candidate + 1} to dose the reagent")

    def set_pump_ids(self, pump_ids) -> None:
        """The enabled+configured pump ids, for every row's Pump combo."""
        self._pump_ids = [str(p) for p in (pump_ids or [])]
        self._rebuild_rows()

    def set_needle(self, needle) -> None:
        """Point the table at a needle assembly (None clears it)."""
        self._needle = needle
        self._rebuild_rows()

    def set_flow_ceilings(self, ceilings) -> None:
        """Per-pump flow ceilings (µL/s) so a clamped dose flow can be SHOWN.

        ``SafetyLimits.clamp_flow_rate`` hard-clamps to each bore's own
        Hagen-Poiseuille ceiling and emits only a ``logger.warning``. On a pulled
        30 µm tip a typed 5 µL/s therefore runs ~100× slower while the readout
        still says 5.00 — and the operator's lead-time arithmetic is wrong by two
        orders of magnitude with no sign of it. Quick Print already surfaces this;
        this panel did not.
        """
        clean = {}
        for pid, v in (ceilings or {}).items():
            # Strict: a bare MagicMock's __float__ is 1.0 and would fabricate a
            # 1 µL/s limit for every pump.
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                continue
            if float(v) > 0:
                clean[str(pid).strip().upper()] = float(v)
        self._flow_ceilings = clean
        self._refresh_notes()

    def flow_ceiling_for(self, pump_id):
        """This pump's ceiling, or None when unknown."""
        if not pump_id:
            return None
        return self._flow_ceilings.get(str(pump_id).strip().upper())

    def effective_dose_rate(self, pump_id, requested):
        """The rate that will ACTUALLY run, after the per-bore clamp."""
        ceiling = self.flow_ceiling_for(pump_id)
        try:
            req = float(requested)
        except (TypeError, ValueError):
            return None
        if ceiling is None:
            return req
        return min(req, ceiling)

    def bore_count(self) -> int:
        return len(self._rows)

    def refresh_offsets(self) -> None:
        """Re-read the mount offsets off the live needle (no rebuild).

        ``NeedleBoreCalibrationStore.apply_to_needle`` writes the measured
        offsets onto the SAME ``NeedleBore`` objects this panel holds, and the
        Calibration page does that on each hardware-config push. Depending on the
        app's fan-out order that can land after this table was built, so the
        readouts are refreshed when the page is shown — a re-READ, deliberately
        not a second place that APPLIES a calibration.
        """
        for row in self._rows:
            self._refresh_row_labels(row)
        self._refresh_notes()

    def _default_role(self, bore_index: int, n_bores: int) -> BoreRole:
        """Bore 0 aspirates. **Everything else idles.**

        Bore 0 is the ``needle_origin_um`` datum, so making it the aspirating
        bore means a single-bore assembly (and every pre-v7.9 needle) reproduces
        the legacy sequence with no offsets applied at all.

        ⚠ A second bore used to default to PUSH_REAGENT, which silently ARMED a
        two-bore dosing sequence — a lateral shift of 100-500 µm performed with
        the needle ~0.1 mm above the glass, plus a reagent dose onto live cells —
        the instant the operator switched the needle form, possibly for an
        unrelated reason, with default numbers they had never seen. Nothing asked
        them to confirm it.

        Arming it is now one explicit click ("Use bore N to dose the reagent"),
        which is the same trade the trypsin card already argues for elsewhere:
        make the safe thing the default and the consequential thing deliberate.
        """
        if bore_index == 0:
            return BoreRole.ASPIRATE_TARGET
        return BoreRole.IDLE

    def _rebuild_rows(self, *, remember_first: bool = True) -> None:
        """(Re)build one row per bore, restoring remembered selections.

        ``remember_first=False`` is used by :meth:`apply_state`, which has just
        REPLACED the remembered programs from a saved profile — harvesting the
        live rows first would immediately overwrite them with what is on screen.
        """
        if self._rebuilding:
            return
        self._rebuilding = True
        try:
            if remember_first:
                self._remember_current()
            for row in self._rows:
                for w in (row.bore_label, row.pump, row.role, row.target_type,
                          row.volume, row.depth, row.rate, row.lead, row.offset):
                    self._grid.removeWidget(w)
                    w.setParent(None)
                    w.deleteLater()
            self._rows = []

            n = 0 if self._needle is None else max(
                1, int(needle_bore_count(self._needle)))
            self._bore_empty.setVisible(n == 0)

            for k in range(n):
                row = _BoreRow(k, self._on_row_changed)
                declared = self._populate_pump_combo(row)
                self._populate_type_combo(row)
                self._apply_remembered(row, n, declared)
                self._declared_pumps[k] = declared
                self._refresh_row_labels(row)
                grid_row = k + self._header_row_count
                for col, w in enumerate((
                        row.bore_label, row.pump, row.role, row.target_type,
                        row.volume, row.depth, row.rate, row.lead, row.offset)):
                    self._grid.addWidget(w, grid_row, col)
                row.refresh_enabled()
                self._rows.append(row)
        finally:
            self._rebuilding = False
        self._refresh_column_visibility()
        self._refresh_notes()
        self.programs_changed.emit()

    #: Grid column indices, so "hide the dose columns" is not four magic numbers.
    _COL_TARGET_TYPE = 3
    _COL_DOSE = (4, 5, 6, 7)          # dose · ≙ depth · dose flow · lead time
    _COL_OFFSET = 8

    def _refresh_column_visibility(self) -> None:
        """Show only the columns that can apply to this configuration.

        ⚠ HIDDEN ≠ UNREAD. ``_collect_programs`` still reads every cell, so each
        hidden column is justified by the value being unreachable downstream:

        * **Dose columns** — consumed by ``_current_config`` only from the row
          with the dosing role. With no such row ``trypsin_enabled`` stays False
          and they never reach the config at all.
        * **Target type** — an id that is not in the (empty) library resolves to
          "" exactly as an unset one does.
        * **Mount offset** — a read-only label on a single-bore assembly, where
          bore 1 IS the datum and its offset is zero by definition.
        """
        header_visible = bool(self._rows)
        has_types = bool(getattr(self, "_target_types", None))
        has_dosing = any(r.role_value() is BoreRole.PUSH_REAGENT
                         for r in self._rows)
        multi_bore = len(self._rows) > 1

        def show(col: int, visible: bool, widgets) -> None:
            if col < len(self._headers):
                self._headers[col].setVisible(visible and header_visible)
            for w in widgets:
                w.setVisible(visible)

        show(self._COL_TARGET_TYPE, has_types,
             [r.target_type for r in self._rows])
        for col, attr in zip(self._COL_DOSE,
                             ("volume", "depth", "rate", "lead")):
            show(col, has_dosing, [getattr(r, attr) for r in self._rows])
        show(self._COL_OFFSET, multi_bore, [r.offset for r in self._rows])

    def _populate_pump_combo(self, row: _BoreRow) -> str:
        """Fill one row's Pump combo; returns the pump the ASSEMBLY declares."""
        declared = self._declared_pump(row.bore_index)
        row.pump.blockSignals(True)
        try:
            row.pump.clear()
            row.pump.addItem(NO_PUMP, "")
            for pid in self._pump_ids:
                row.pump.addItem(pid, pid)
            # The assembly's own wiring wins as the default: Hardware Setup →
            # Needle is where a bore's pump is declared.
            if declared:
                if row.pump.findData(declared) < 0:
                    # The assembly names a pump that is not enabled/configured.
                    # Say so, exactly as Hardware Setup's own bore-pump combo
                    # does — an unlabelled entry looks like a working choice, and
                    # driving it fails at prep time with the needle in a well.
                    row.pump.addItem(f"{declared} (not enabled)", declared)
                row.pump.setCurrentIndex(row.pump.findData(declared))
        finally:
            row.pump.blockSignals(False)
        return declared

    def _declared_pump(self, bore_index: int) -> str:
        """The pump the needle assembly itself names for a bore ("" if none)."""
        bore = needle_bore_at(self._needle, bore_index)
        try:
            return str(getattr(bore, "pump_id", "") or "")
        except Exception:
            return ""

    def _populate_type_combo(self, row: _BoreRow) -> None:
        row.target_type.blockSignals(True)
        try:
            row.target_type.clear()
            row.target_type.addItem(NO_TARGET_TYPE, "")
            for tt in self._target_types:
                row.target_type.addItem(tt.label, tt.id)
        finally:
            row.target_type.blockSignals(False)

    def _apply_remembered(self, row: _BoreRow, n_bores: int,
                          declared: str = "") -> None:
        saved = self._remembered.get(row.bore_index)
        prog = (BoreProgram.from_dict(saved) if saved
                else BoreProgram(bore_index=row.bore_index,
                                 role=self._default_role(row.bore_index, n_bores)))
        row.role.blockSignals(True)
        try:
            idx = row.role.findData(prog.role.value)
            row.role.setCurrentIndex(idx if idx >= 0 else 0)
        finally:
            row.role.blockSignals(False)
        # HARDWARE TRUTH WINS WHEN IT CHANGES. The Pump combo exists so a bore
        # whose assembly declares no pump can be hand-wired, and a hand-set value
        # must survive a rebuild (ANY hardware-config push rebuilds this table —
        # the preserve-and-restore lesson of hard-parts (f)/4). But when the
        # assembly's declaration for THIS bore has changed — a different needle
        # mounted, or Hardware Setup → Needle re-wired — a pump remembered from
        # the previous assembly would keep driving the WRONG SYRINGE, because
        # ``_aspirate_pump_id()`` and ``trypsin_bore`` are read straight off this
        # table. Measured before the fix: swapping a P1/P2 backpack for a P2/P3
        # one left the table (and the executed config) on P1/P2.
        #
        # ⚠ THE SENTINEL IS LOAD-BEARING. `.get(k, declared)` defaulted the
        # "previously declared" value to the CURRENT one, so "I have never seen a
        # declaration for this bore" was indistinguishable from "it has not
        # changed" — and that is exactly the state at STARTUP: the panel is built
        # with needle=None, `load_last()` restores the saved profile while there
        # are still zero rows, and the first REAL build therefore finds
        # `_declared_pumps` empty. Result: across a restart the saved profile's
        # pump beat the mounted assembly's declaration, so a P2/P3 backpack ran on
        # P1 — a syringe not plumbed to this needle at all. The in-session swap was
        # tested; the restart was not. Now an unknown history means HARDWARE WINS,
        # which is the fail-safe direction. (`_UNKNOWN` rather than None because
        # None is a legitimate "this bore declares no pump".)
        known = self._declared_pumps.get(row.bore_index, _UNKNOWN)
        rewired = bool(declared) and known != declared
        if prog.pump_id and not rewired:
            idx = row.pump.findData(prog.pump_id)
            if idx >= 0:
                row.pump.blockSignals(True)
                row.pump.setCurrentIndex(idx)
                row.pump.blockSignals(False)
        elif rewired and prog.pump_id and prog.pump_id != declared:
            logger.info(
                "Bore %d's assembly now declares pump %s (was %s) — using the "
                "declared pump instead of the remembered %s.",
                row.bore_index + 1, declared,
                self._declared_pumps.get(row.bore_index), prog.pump_id)
        if prog.target_type_id:
            idx = row.target_type.findData(prog.target_type_id)
            if idx < 0:
                # ⚠ PERMANENT-LOSS FIX. An unresolvable id used to be silently
                # dropped, the row fell back to "(any / untyped)", and the very
                # next row edit called `_remember_current()` → the profile was
                # rewritten with `target_type_id: ""`. Renaming one JSON file
                # therefore ERASED every assignment, with no message. Re-add it as
                # a placeholder — exactly what `_populate_pump_combo` and the
                # trypsin reagent already do — so it round-trips instead.
                label = prog.target_type_name or prog.target_type_id
                row.target_type.blockSignals(True)
                row.target_type.addItem(f"{label} (missing)",
                                        prog.target_type_id)
                row.target_type.blockSignals(False)
                idx = row.target_type.findData(prog.target_type_id)
            if idx >= 0:
                row.target_type.blockSignals(True)
                row.target_type.setCurrentIndex(idx)
                row.target_type.blockSignals(False)
        for widget, value in ((row.rate, prog.rate_uL_s),
                              (row.lead, prog.lead_time_s)):
            widget.blockSignals(True)
            try:
                widget.setValue(float(value))
            except (TypeError, ValueError):
                pass
            finally:
                widget.blockSignals(False)
        # ⚠ The dose is stored in µL and entered in nL, and a LEGACY profile may
        # carry the retired magic zero ("derive it from the depth instead").
        # Resolve that here, once, rather than leaving a 0 that the volume field's
        # own minimum would silently promote to 0.001 nL.
        try:
            vol_uL = float(prog.volume_uL or 0.0)
        except (TypeError, ValueError):
            vol_uL = 0.0
        if vol_uL <= 0:
            row._legacy_depth_mm = float(prog.depth_mm or 0.0)
        else:
            row._legacy_depth_mm = None
            row.set_volume_uL(vol_uL)

    def _refresh_row_labels(self, row: _BoreRow) -> None:
        """Bore identity + its geometry, and the measured mount offset."""
        bore = needle_bore_at(self._needle, row.bore_index)
        summary = ""
        try:
            summary = bore.summary_line()
        except Exception:
            summary = ""
        # This bore's own orifice area drives the derived column depth, and its
        # holdup caps the dose (see set_holdup_uL). Area FIRST, so the cap's
        # re-derive uses the right geometry.
        try:
            from SupportClasses.PhysicalModels import needle_orifice_area_mm2
            row.set_orifice_area_mm2(needle_orifice_area_mm2(bore))
        except Exception:
            logger.debug("could not resolve bore %d's orifice area",
                         row.bore_index + 1, exc_info=True)
        try:
            row.set_holdup_uL(getattr(bore, "internal_volume_uL", None))
        except Exception:
            logger.debug("could not cap bore %d's dose volume",
                         row.bore_index + 1, exc_info=True)
        # A legacy profile's magic-zero volume becomes a real dose only once the
        # bore geometry is known — which is HERE, not at load time.
        row.resolve_legacy_depth()
        row.bore_label.setText(
            f"<b>Bore {row.bore_index + 1}</b>"
            f"<br><span style=\"color:{COLORS['subtext0']};"
            f"font-size:{sf(8)}pt\">{summary or '—'}</span>")
        row.bore_label.setToolTip(summary or "")

        if row.bore_index == 0:
            row.offset.setText("datum (0, 0)")
            row.offset.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
            row.offset.setToolTip(
                "Bore 1 IS the calibrated needle origin, so its offset is zero "
                "by definition.")
            return
        ox, oy = needle_bore_offset_um(self._needle, row.bore_index)
        dz = needle_bore_z_offset_mm(self._needle, row.bore_index)
        if abs(ox) < 1e-9 and abs(oy) < 1e-9:
            # An unmeasured offset does not fail loudly at run time — the bore
            # simply lands on bore 0's position, i.e. 100-500 µm off the cell.
            # That is larger than the cell, so it has to be visible here.
            row.offset.setText("⚠ not measured — Calibration → Needle Location")
            row.offset.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
            # NAME THE PAGE. Every other pointer on this panel sends the operator
            # to Hardware Setup, so "measure the bore offsets" without a
            # destination sent them hunting in the wrong place.
            row.offset.setToolTip(
                "This bore has no measured lateral offset from bore 1, so it "
                "would be driven to bore 1's position — typically 100-500 µm "
                "off the target.\n\n"
                "Measure them on Calibration → Needle Location. They must be "
                "re-measured after every needle change or re-seat, because a "
                "fused assembly's rotation in the holder is arbitrary.")
            return
        text = f"Δ {ox:+.0f}, {oy:+.0f} µm"
        if dz:
            text += f" · Z {dz:+.3f} mm"
        row.offset.setText(text)
        row.offset.setStyleSheet(
            f"color: {COLORS['green']}; font-size: {sf(9)}pt;")
        row.offset.setToolTip(
            "Measured lateral offset from bore 1. To put this bore on a target "
            "the stage goes to target − offset.")

    def _on_row_changed(self) -> None:
        if self._rebuilding:
            return
        self._remember_current()
        # Arming or disarming a dosing bore is what makes the four dose columns
        # apply or not, so the table has to re-narrow on a role change.
        self._refresh_column_visibility()
        self._refresh_notes()
        self.programs_changed.emit()

    def _remember_current(self) -> None:
        for prog in self._collect_programs():
            self._remembered[prog.bore_index] = prog.to_dict()

    # ── programs ──────────────────────────────────────────────────

    def _collect_programs(self) -> list[BoreProgram]:
        # ``BoreProgram.z_offset_mm`` is deliberately left at its default: the
        # working heights are assembly-wide (the Heights group's Removal Z /
        # Place Z, which are the executor's only two), and a per-row copy would
        # be a second editing surface for the same number. Each bore's MECHANICAL
        # z offset lives on the NeedleBore and is applied by the executor.
        out: list[BoreProgram] = []
        for row in self._rows:
            tt = self._target_type_by_id(row.target_type_id())
            out.append(BoreProgram(
                bore_index=row.bore_index,
                pump_id=row.pump_id(),
                role=row.role_value(),
                target_type_id=row.target_type_id(),
                # STAMP, DON'T REFERENCE: carry the resolved name + colour so a
                # later rename / delete cannot relabel a recorded run.
                target_type_name=(tt.label if tt is not None else ""),
                target_type_color=(tt.color if tt is not None else ""),
                # THE ONE nL→µL boundary. Every model field downstream is µL.
                volume_uL=row.volume_uL(),
                # Derived from that same volume, so the two can never disagree
                # about the dose — which is what mattered, since the executor
                # prefers `trypsin_volume_uL` while `release_depth_mm` wins on
                # the aspirating side.
                depth_mm=float(row.depth.value()),
                rate_uL_s=float(row.rate.value()),
                lead_time_s=float(row.lead.value()),
            ))
        return out

    def programs(self) -> list[BoreProgram]:
        """Every bore's program, in bore order."""
        return self._collect_programs()

    def program_for_role(self, role: BoreRole) -> BoreProgram | None:
        """The FIRST program with ``role``, or None.

        First, not "the only one": a second row with the same role is reported by
        :meth:`validation_notes` rather than silently changing which bore runs.
        """
        for prog in self._collect_programs():
            if prog.role is role:
                return prog
        return None

    def active_bore_indices(self) -> list[int]:
        return [p.bore_index for p in self._collect_programs() if p.role.is_active]

    # ── target types ──────────────────────────────────────────────

    def _build_target_types_card(self) -> Card:
        card = Card("Target types")
        row = QHBoxLayout()
        row.setSpacing(s(8))
        row.addWidget(_small(
            "A target type is a user-defined class of object plus its "
            "fluorescent-signature rule."), stretch=1)
        # v7.9 (post-audit): target types were authorable ONLY by hand-editing
        # JSON. `save_user`/`delete_user` existed and were called from nowhere in
        # the GUI. Same button set and the same built-in protections as the
        # well-type / needle-type preset editors.
        self._tt_new_btn = QPushButton("＋ New…")
        self._tt_new_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._tt_new_btn.setToolTip("Author a new target type.")
        self._tt_new_btn.clicked.connect(self._new_target_type)
        row.addWidget(self._tt_new_btn)

        self._tt_edit_btn = QPushButton("✎ Edit…")
        self._tt_edit_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._tt_edit_btn.setToolTip(
            "Edit the selected type. Editing a built-in saves your version as an "
            "override — the built-in itself stays intact.")
        self._tt_edit_btn.clicked.connect(self._edit_selected_target_type)
        row.addWidget(self._tt_edit_btn)

        self._tt_dup_btn = QPushButton("⧉ Duplicate…")
        self._tt_dup_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._tt_dup_btn.setToolTip("Start a new type from the selected one.")
        self._tt_dup_btn.clicked.connect(self._duplicate_selected_target_type)
        row.addWidget(self._tt_dup_btn)

        self._tt_del_btn = QPushButton("🗑 Delete")
        self._tt_del_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._tt_del_btn.setToolTip(
            "Delete one of YOUR types. Built-ins cannot be deleted.")
        self._tt_del_btn.clicked.connect(self._delete_selected_target_type)
        row.addWidget(self._tt_del_btn)

        reload_btn = QPushButton("⟳ Reload")
        reload_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        reload_btn.setToolTip(
            "Re-read the target types from disk (config/hardware/target_types).")
        reload_btn.clicked.connect(self.reload_target_types)
        row.addWidget(reload_btn)
        holder = QWidget()
        holder.setLayout(row)
        card.add_widget(holder)

        self._types_body = QVBoxLayout()
        self._types_body.setContentsMargins(0, 0, 0, 0)
        self._types_body.setSpacing(s(4))
        body = QWidget()
        body.setLayout(self._types_body)
        card.add_widget(body)

        # The single most important thing this card has to say. Stated in the UI,
        # not just in a docstring, because a type picker that looks like a
        # detector would be trusted as one.
        note = _small(
            "⚠ Recorded intent only — nothing detects these yet. Pick the "
            "cells by clicking them.", "peach")
        note.setToolTip(
            "The signature rules are saved and shown, but nothing selects "
            "objects from a fluorescence image using them. Choose the removal "
            "targets by clicking each cell.")
        card.add_widget(note)
        return card

    def reload_target_types(self) -> None:
        """Re-read the store and refresh the list + every row's combo.

        ⚠ A transient failure must NOT empty the list. It used to set
        ``self._target_types = []`` on any exception, so one unreadable config dir
        made the card read "No target types defined yet" with three built-ins
        sitting on disk — an actively false statement, whose real reason went to a
        debug log the operator will never see.
        """
        self._load_errors = []
        try:
            from SupportClasses.TargetTypeStore import get_store
            store = get_store()
            store.reload()
            self._target_types = list(store.all())
            self._load_errors = list(getattr(store, "load_errors", []) or [])
        except Exception as exc:
            logger.warning("target type store unavailable: %s", exc, exc_info=True)
            # KEEP whatever we had. Report the failure instead of pretending the
            # library is empty.
            self._load_errors = [("(target-type library)", str(exc))]
        self._refresh_types_list()
        for row in self._rows:
            keep = row.target_type_id()
            keep_label = row.target_type.currentText()
            self._populate_type_combo(row)
            idx = row.target_type.findData(keep)
            if idx < 0 and keep:
                # Same reasoning as `_apply_remembered`: keep the assignment
                # visible as "(missing)" rather than silently discarding it.
                label = keep_label.replace(" (missing)", "")
                row.target_type.blockSignals(True)
                row.target_type.addItem(f"{label} (missing)", keep)
                row.target_type.blockSignals(False)
                idx = row.target_type.findData(keep)
            if idx >= 0:
                row.target_type.blockSignals(True)
                row.target_type.setCurrentIndex(idx)
                row.target_type.blockSignals(False)
        # The first authored type makes the column apply; deleting the last one
        # makes it stop applying.
        self._refresh_column_visibility()

    def _target_type_by_id(self, type_id: str):
        if not type_id:
            return None
        for tt in self._target_types:
            if tt.id == type_id:
                return tt
        return None

    def _refresh_types_list(self) -> None:
        while self._types_body.count():
            item = self._types_body.takeAt(0)
            w = item.widget()
            if w is not None:
                w.setParent(None)
                w.deleteLater()
        for name, reason in (getattr(self, "_load_errors", None) or []):
            self._types_body.addWidget(_small(
                f"⚠ {name} could not be read: {reason}", "peach"))
        if not self._target_types:
            if getattr(self, "_load_errors", None):
                self._types_body.addWidget(_small(
                    "No target types loaded — fix the file(s) above, then Reload.",
                    "peach"))
            else:
                self._types_body.addWidget(_small(
                    "No target types defined yet — use ＋ New… to create one."))
            return
        if getattr(self, "_load_errors", None):
            self._types_body.addWidget(_small(
                f"Showing the {len(self._target_types)} type(s) that did load."))
        try:
            from SupportClasses.FluorescenceMosaicStore import CHANNELS
        except Exception:
            CHANNELS = ()
        for tt in self._target_types:
            row = QWidget()
            h = QHBoxLayout(row)
            h.setContentsMargins(0, 0, 0, 0)
            h.setSpacing(s(6))
            swatch = QLabel()
            swatch.setFixedSize(s(12), s(12))
            swatch.setStyleSheet(
                f"background: {tt.color}; border-radius: {s(3)}px;")
            h.addWidget(swatch)
            unknown = []
            try:
                unknown = tt.signature_rule.unknown_imaging_channels(CHANNELS)
            except Exception:
                unknown = []
            text = tt.describe()
            if unknown:
                # Reported, never repaired — the cube may simply not be mounted
                # on this rig today, and the authored rule stays intact.
                text += f"  ⚠ unknown imaging channel(s): {', '.join(unknown)}"
            h.addWidget(_small(text, "peach" if unknown else "text"), stretch=1)
            if tt.builtin:
                h.addWidget(_small("built-in", "overlay0"))
            # Click to select, double-click to edit — the same affordance the
            # other preset libraries in this app use.
            row.setCursor(Qt.CursorShape.PointingHandCursor)
            row.mousePressEvent = (
                lambda _e, tid=tt.id: self._select_target_type(tid))
            row.mouseDoubleClickEvent = (
                lambda _e, tid=tt.id: (self._select_target_type(tid),
                                       self._edit_selected_target_type()))
            if tt.id == getattr(self, "_selected_tt_id", ""):
                row.setStyleSheet(
                    f"background: {COLORS['surface1']}; border-radius: {s(3)}px;")
            self._types_body.addWidget(row)
        self._refresh_tt_buttons()

    def _select_target_type(self, type_id: str) -> None:
        self._selected_tt_id = str(type_id or "")
        self._refresh_types_list()

    def _selected_target_type(self):
        return self._target_type_by_id(getattr(self, "_selected_tt_id", ""))

    def _refresh_tt_buttons(self) -> None:
        tt = self._selected_target_type()
        for btn in (getattr(self, "_tt_edit_btn", None),
                    getattr(self, "_tt_dup_btn", None)):
            if btn is not None:
                btn.setEnabled(tt is not None)
        if getattr(self, "_tt_del_btn", None) is not None:
            # A built-in is never deletable — your version SHADOWS it instead.
            deletable = tt is not None and not tt.builtin
            self._tt_del_btn.setEnabled(deletable)
            self._tt_del_btn.setToolTip(
                "Built-ins cannot be deleted — edit one to save your own "
                "override instead." if (tt is not None and tt.builtin)
                else "Delete the selected target type.")

    def _tt_store(self):
        try:
            from SupportClasses.TargetTypeStore import get_store
            return get_store()
        except Exception:
            logger.warning("target type store unavailable", exc_info=True)
            return None

    def _open_target_type_dialog(self, tt=None, prefill=None) -> None:
        store = self._tt_store()
        if store is None:
            return
        try:
            from gui.dialogs.target_type_dialog import TargetTypeDialog
        except Exception:
            logger.exception("could not open the target type editor")
            return
        dlg = TargetTypeDialog(
            self, target_type=tt, store=store, prefill=prefill,
            existing_ids=[t.id for t in self._target_types])
        if dlg.exec() and dlg.result_type is not None:
            self._selected_tt_id = dlg.result_type.id
            self.reload_target_types()
            self.programs_changed.emit()

    def _new_target_type(self) -> None:
        self._open_target_type_dialog(None)

    def _edit_selected_target_type(self) -> None:
        tt = self._selected_target_type()
        if tt is not None:
            self._open_target_type_dialog(tt)

    def _duplicate_selected_target_type(self) -> None:
        tt = self._selected_target_type()
        if tt is None:
            return
        # Opened as NEW with the source's rule pre-filled, so it derives a FRESH
        # id from the new name. Reusing the original's id would silently overwrite
        # the type every already-assigned bore points at.
        self._open_target_type_dialog(None, prefill=tt)

    def _delete_selected_target_type(self) -> None:
        tt = self._selected_target_type()
        if tt is None or tt.builtin:
            return
        store = self._tt_store()
        if store is None:
            return
        # Name the consequence for bores already using it. This is only TRUE
        # because a missing id is now kept as "(missing)" rather than erased.
        users = [f"Bore {p.bore_index + 1}" for p in self._collect_programs()
                 if p.target_type_id == tt.id]
        extra = ""
        if users:
            who = ", ".join(users)
            verb = "uses" if len(users) == 1 else "use"
            extra = ("\n\n"
                     f"{who} currently {verb} it. Deleting it leaves that "
                     f"assignment recorded as “(missing)” rather than "
                     f"clearing it.")
        btn = QMessageBox.question(
            self, "Delete target type",
            f"Delete “{tt.label}”?{extra}",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel)
        if btn != QMessageBox.StandardButton.Yes:
            return
        try:
            store.delete_user(tt.id)
        except Exception as exc:
            logger.exception("delete_user failed")
            QMessageBox.critical(
                self, "Could not delete",
                f"Deleting “{tt.label}” failed:\n{exc}")
            return
        self._selected_tt_id = ""
        self.reload_target_types()
        self.programs_changed.emit()

    # ── trypsin bore ──────────────────────────────────────────────

    def _build_trypsin_card(self) -> Card:
        card = Card("Dosing bore — where it refills")
        card.setToolTip(
            "Give a bore the “Dose the target with reagent” role in the table "
            "above; its dose, flow and lead time are that row's own cells. This "
            "card only says WHERE that bore refills.")
        self.trypsin_reagent = QComboBox()
        self.trypsin_reagent.setMinimumWidth(s(180))
        self.trypsin_reagent.setToolTip(
            "The reagent (e.g. trypsin) the pushing bore loads, and therefore "
            "which reagent well it dips into. Listed reagents are library inks "
            "with a reagent location (Hardware Setup → Ink).")
        self.trypsin_reagent.currentIndexChanged.connect(
            lambda *_: self.programs_changed.emit())
        card.add_widget(FormRow("Reagent for the pushing bore",
                                self.trypsin_reagent))
        self.trypsin_status = _small("")
        card.add_widget(self.trypsin_status)

        # Still NOT a disabled control — a greyed "overlap" checkbox would read
        # as "coming soon" when the geometry forbids it outright. But the
        # explanation is a tooltip now rather than a permanent paragraph: it
        # answers a question nobody asks until they go looking for the control.
        self.trypsin_status.setToolTip(
            "There is no “start aspirating before the push ends” option: the "
            "dosing bore and the aspirating bore are laterally offset, so the "
            "tool has to travel between the push and the pull. They are never "
            "over the target at the same time.")
        return card

    def set_reagent_choices(self, names, *, preferred: str | None = None) -> None:
        """Populate the trypsin reagent combo, preserving the current choice."""
        combo = self.trypsin_reagent
        keep = combo.currentData() or preferred or ""
        combo.blockSignals(True)
        try:
            combo.clear()
            combo.addItem("(none — bore already loaded)", "")
            for name in names or []:
                combo.addItem(str(name), str(name))
            idx = combo.findData(keep)
            if idx >= 0:
                combo.setCurrentIndex(idx)
        finally:
            combo.blockSignals(False)

    def trypsin_reagent_name(self) -> str:
        return self.trypsin_reagent.currentData() or ""

    def register_help_rows(self) -> None:
        """Register this panel's FormRows with the MainWindow's Help toggle.

        Copied from ``WorkflowSettingsDialog._note_help_row``: walk up the parent
        chain for a callable ``register_form_row``, best-effort.

        ⚠ MUST be called from the host's ``showEvent``, not from ``__init__``: the
        panel is parented to the page before the page is in the window, so a
        construction-time walk finds nothing. ``register_form_row`` de-duplicates,
        so repeat calls are free.
        """
        w = self.parent()
        seen = 0
        while w is not None and seen < 12:
            reg = getattr(w, "register_form_row", None)
            if callable(reg):
                for row in self._help_rows:
                    try:
                        reg(row)
                    except Exception as exc:
                        logger.debug("register_form_row failed: %s", exc)
                return
            w = w.parent()
            seen += 1

    def set_help_mode(self, visible: bool) -> None:
        """Reveal/hide the bore-table column legend.

        The table's cells CANNOT be FormRows — they share one QGridLayout so the
        columns line up across bores, and per-row FormRows give a ragged table —
        so help mode gets an honest substitute: one paragraph explaining the nine
        columns.
        """
        if hasattr(self, "_help_legend"):
            self._help_legend.setVisible(bool(visible))

    def target_types(self) -> list:
        """The target types currently loaded (a copy — callers must not mutate)."""
        return list(self._target_types)

    def set_trypsin_status(self, text: str, color_key: str = "subtext0") -> None:
        self.trypsin_status.setText(text)
        self.trypsin_status.setStyleSheet(
            f"color: {COLORS[color_key]}; font-size: {sf(9)}pt;")

    # ── validation ────────────────────────────────────────────────

    def validation_notes(self) -> list[str]:
        """Advisories about the current program table (never a hard refusal).

        Following the house convention: advise, and hard-block only the
        physically impossible. Each note names the bore in the 1-based form the
        table and the turret both use.

        Returns ``[]`` when there is no assembly at all: with no needle there is
        nothing to validate, the empty-state label already says what to do, and
        printing "⚠ No bore is set to aspirate — bore 1 will be used" beside "No
        needle configured yet" is nonsense — the kind that makes an operator stop
        trusting the warnings that matter.
        """
        if self._needle is None or not self._rows:
            return []
        progs = self._collect_programs()
        notes: list[str] = []

        by_role: dict[BoreRole, list[BoreProgram]] = {}
        for p in progs:
            by_role.setdefault(p.role, []).append(p)

        asp = by_role.get(BoreRole.ASPIRATE_TARGET, [])
        push = by_role.get(BoreRole.PUSH_REAGENT, [])
        place = by_role.get(BoreRole.DISPENSE_PLACE, [])

        if not asp:
            notes.append(
                "No bore is set to aspirate the target — bore 1 will be used.")
        elif len(asp) > 1:
            notes.append(
                "More than one bore is set to aspirate: bore "
                f"{asp[0].bore_index + 1} runs, the rest are ignored.")
        if len(push) > 1:
            notes.append(
                "More than one bore is set to push reagent: bore "
                f"{push[0].bore_index + 1} runs, the rest are ignored.")
        if push and asp and push[0].bore_index == asp[0].bore_index:
            notes.append(
                f"Bore {push[0].bore_index + 1} both pushes and aspirates, so "
                "there is no shift between the dose and the pull. A separate "
                "pushing bore is what the two-bore sequence is for.")
        if push and not push[0].pump_id:
            notes.append(
                f"Bore {push[0].bore_index + 1} pushes reagent but names no "
                "pump — the push will be skipped.")
        if place and asp and place[0].bore_index != asp[0].bore_index:
            # Honest disclosure rather than a silent no-op: the executor
            # delivers through the bore that aspirated.
            notes.append(
                f"Bore {place[0].bore_index + 1} is marked to dispense, but the "
                "extracted volume is delivered through the aspirating bore "
                f"(bore {asp[0].bore_index + 1}). The assignment is recorded, "
                "not yet driven.")
        for p in progs:
            if not p.role.is_active:
                continue
            # A pump the operator (or a restored profile) named that disagrees
            # with the assembly's own wiring drives the WRONG SYRINGE. The rebuild
            # re-asserts a CHANGED declaration, but a profile restored before any
            # declaration was recorded can still land here, so say it out loud.
            declared = self._declared_pump(p.bore_index)
            if declared and p.pump_id and p.pump_id != declared:
                notes.append(
                    f"Bore {p.bore_index + 1} is programmed to pump "
                    f"{p.pump_id} but the assembly declares {declared} — check "
                    "Hardware Setup → Needle before running.")
        for p in progs:
            if not p.role.is_active or p.bore_index == 0:
                continue
            ox, oy = needle_bore_offset_um(self._needle, p.bore_index)
            if abs(ox) < 1e-9 and abs(oy) < 1e-9:
                notes.append(
                    f"Bore {p.bore_index + 1} has no measured offset from bore "
                    "1, so it will be driven to bore 1's position (typically "
                    "100-500 µm off the target).")
        # The dose flow is silently clamped to each bore's own Hagen-Poiseuille
        # ceiling. Naming the LEAD-TIME consequence is the point: the clamp alone
        # is not the harm, the operator's timing arithmetic being wrong by two
        # orders of magnitude is.
        for p in push:
            ceiling = self.flow_ceiling_for(p.pump_id)
            requested = float(p.rate_uL_s or 0.0)
            if ceiling and requested > ceiling:
                notes.append(
                    f"Bore {p.bore_index + 1}'s dose flow {requested:.2f} µL/s "
                    f"exceeds this bore's flow ceiling ({ceiling:.3f} µL/s) — it "
                    f"will be auto-limited to {ceiling:.3f} µL/s, so the dose "
                    f"takes {requested / ceiling:.0f}× longer than the lead time "
                    f"assumes.")
        # A dose bigger than the bore holds cannot be delivered: the plunger
        # clamps, which silently breaks the volume balance.
        for p in push:
            bore = needle_bore_at(self._needle, p.bore_index)
            holdup = getattr(bore, "internal_volume_uL", None)
            vol = float(p.volume_uL or 0.0)
            if isinstance(holdup, (int, float)) and holdup > 0 and vol > holdup:
                notes.append(
                    f"Bore {p.bore_index + 1}'s dose volume {vol:.4f} µL exceeds "
                    f"what the bore holds ({float(holdup):.4f} µL) — the plunger "
                    f"will clamp and the volume balance will break.")
        return notes

    def _refresh_notes(self) -> None:
        self._refresh_arm_dose_row()
        # With no needle there is nothing to validate, and the empty-state label
        # already says what to do. Printing "⚠ No bore is set to aspirate — bore 1
        # will be used" beside "No needle configured yet" is nonsense, and the kind
        # of thing that makes an operator stop trusting the warnings that matter.
        if self._needle is None or not self._rows:
            self._bore_notes.setText("")
            self._bore_notes.setVisible(False)
            return
        self._bore_notes.setVisible(True)
        notes = self.validation_notes()
        if not notes:
            active = self.active_bore_indices()
            self._bore_notes.setText(
                f"✓ {len(active)} active bore(s)." if active
                else "No active bores — every bore is idle.")
            self._bore_notes.setStyleSheet(
                f"color: {COLORS['green'] if active else COLORS['subtext0']}; "
                f"font-size: {sf(9)}pt;")
            return
        self._bore_notes.setText("⚠ " + "\n⚠ ".join(notes))
        self._bore_notes.setStyleSheet(
            f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")

    # ── persistence (rides along with the settings profile) ───────

    def to_state(self) -> dict:
        """The panel's non-widget state, for ``WorkflowSettingsDialog``.

        The bore table is rebuilt whenever the needle changes, so it cannot be a
        registered field: ``widget_value`` handles only the four fixed widget
        types, and a dynamic table would silently vanish on every restart. This
        is the sanctioned hook (``set_extra_state``) for exactly that case.
        """
        self._remember_current()
        return {
            # Remembered rather than only-current, so a profile saved with a
            # different assembly mounted still carries its assignments back.
            "bore_programs": [self._remembered[k]
                              for k in sorted(self._remembered)],
            "trypsin_reagent": self.trypsin_reagent_name(),
            # WHICH assembly those programs were authored against. Without this,
            # a restart cannot tell a re-wired assembly from an unchanged one and
            # the saved pumps win — see `_apply_remembered`'s sentinel note.
            "assembly_fingerprint": {
                "bore_count": len(self._rows),
                "declared_pumps": {str(k): v
                                   for k, v in self._declared_pumps.items()},
            },
        }

    def apply_state(self, state) -> None:
        if not isinstance(state, dict):
            return
        # Restore the fingerprint FIRST: `_rebuild_rows` below consults
        # `_declared_pumps` to decide whether the assembly was re-wired, so it has
        # to know what the profile was authored against before it runs.
        fp = state.get("assembly_fingerprint")
        if isinstance(fp, dict):
            declared = fp.get("declared_pumps")
            if isinstance(declared, dict):
                restored = {}
                for k, v in declared.items():
                    try:
                        restored[int(k)] = (str(v).strip().upper() if v else None)
                    except (TypeError, ValueError):
                        continue
                self._declared_pumps = restored
        progs = state.get("bore_programs")
        if isinstance(progs, list):
            self._remembered = {}
            for entry in progs:
                prog = BoreProgram.from_dict(entry)
                self._remembered[int(prog.bore_index)] = prog.to_dict()
            self._rebuild_rows(remember_first=False)
        reagent = state.get("trypsin_reagent")
        if isinstance(reagent, str):
            idx = self.trypsin_reagent.findData(reagent)
            if idx >= 0:
                self.trypsin_reagent.setCurrentIndex(idx)
            elif reagent:
                # The ink library may not be loaded yet (the combo is
                # hardware-populated); remember it so the next populate keeps it.
                self.trypsin_reagent.addItem(reagent, reagent)
                self.trypsin_reagent.setCurrentIndex(
                    self.trypsin_reagent.count() - 1)
