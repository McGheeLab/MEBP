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

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QComboBox,
    QDoubleSpinBox, QPushButton, QScrollArea,
)

from gui.scaling import s, sf
from gui.styles import COLORS
from gui.widgets.components import Card, FormRow

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
    BoreRole.PUSH_REAGENT: "Push reagent onto the target",
    BoreRole.ASPIRATE_TARGET: "Aspirate the target",
    BoreRole.DISPENSE_PLACE: "Dispense at the placement",
}

ROLE_ORDER_FOR_COMBO: tuple[BoreRole, ...] = (BoreRole.IDLE,) + BORE_ROLE_ORDER

#: Placeholder for "no target type chosen". An untyped bore is legal — a
#: single-bore run needs no classification (``stamp_target_type(None) == {}``).
NO_TARGET_TYPE = "(any / untyped)"

#: Sentinel for a bore whose pump the operator has not named.
NO_PUMP = "(unassigned)"


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

    def __init__(self, card: Card):
        self._card = card

    def add(self, label: str, widget: QWidget,
            help_text: str | None = None) -> QWidget:
        text = help_text
        if not text:
            try:
                text = widget.toolTip() or None
            except Exception:
                text = None
        self._card.add_widget(FormRow(label, widget, help_text=text))
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
        self.target_type.setToolTip(
            "The user-defined class of object this bore services (Target types "
            "below). Recorded intent — the fluorescent-signature rules are not "
            "evaluated against an image yet.")
        self.target_type.setMinimumWidth(s(130))

        self.volume = _dspin(
            0.0, 1000.0, 0.0, " µL", 4, 0.01,
            "Reagent volume pushed onto the target. 0 = derive it from this "
            "bore's own orifice area × the push depth.")
        self.depth = _dspin(
            0.001, 5.0, 0.10, " mm", 3, 0.05,
            "Push depth — the reagent column is this bore's orifice area × this "
            "depth. Ignored when an explicit volume is given.")
        self.rate = _dspin(
            0.01, 50.0, 0.5, " µL/s", 2, 0.1,
            "Flow used for this bore's push (⇒ push duration = volume / rate).")
        self.lead = _dspin(
            0.0, 3600.0, 0.0, " s", 1, 0.5,
            "Lead time: how long the reagent acts after the push finishes and "
            "the tool has shifted, before the aspirate starts.")

        self.offset = _small()
        self.offset.setMinimumWidth(s(140))

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
        for w in (self.volume, self.depth, self.rate, self.lead):
            w.valueChanged.connect(lambda *_: self._on_change())

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
        idle_tip = (
            "Only a “Push reagent” bore uses these — the aspirate and dispense "
            "flows are set once for the whole assembly under Reagent push / pull.")
        for w in (self.volume, self.depth, self.rate, self.lead):
            w.setEnabled(pushing)
            w.setToolTip(self._push_tips[w] if pushing else idle_tip)


class CellTargetingSetupPanel(QWidget):
    """Setup tab: per-bore program table + target types + trypsin timing.

    Signals:
        programs_changed: A role / pump / target type / push parameter changed.
    """

    programs_changed = Signal()

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

        self._left.addWidget(self._build_bore_table_card())
        self._left.addWidget(self._build_target_types_card())
        self._left.addWidget(self._build_trypsin_card())
        self._left.addStretch(1)
        # The host's promoted groups land in the right column, under this
        # heading, so the "what/how much/how fast" establishment reads as one
        # block beside the per-bore assignment.
        self._right.addWidget(_small(
            "Assembly-wide run parameters. Everything here is saved with the "
            "workflow profile (⚙ Settings → Save)."))
        self._right_stretch_added = False

        scroll.setWidget(body)
        outer.addWidget(scroll)

        self.reload_target_types()
        self._rebuild_rows()

    # ── host-filled groups ────────────────────────────────────────

    def add_group(self, title: str) -> SetupGroup:
        """A card in the right-hand column for the host's promoted fields."""
        card = Card(title)
        if self._right_stretch_added:
            # Keep the trailing stretch last no matter when a group is added.
            self._right.insertWidget(self._right.count() - 1, card)
        else:
            self._right.addWidget(card)
        return SetupGroup(card)

    def finalize(self) -> None:
        """Call once the host has added every group (adds the trailing stretch)."""
        if not self._right_stretch_added:
            self._right.addStretch(1)
            self._right_stretch_added = True

    # ── per-bore program table ────────────────────────────────────

    def _build_bore_table_card(self) -> Card:
        card = Card("Needle assembly — one program per bore")
        card.add_widget(_small(
            "One row per bore of the configured assembly (Hardware Setup → "
            "Needle). Give each bore a role; the run order is derived "
            "(push → aspirate → dispense)."))

        holder = QWidget()
        self._grid = QGridLayout(holder)
        self._grid.setContentsMargins(0, 0, 0, 0)
        self._grid.setHorizontalSpacing(s(8))
        self._grid.setVerticalSpacing(s(6))
        headers = ("Bore", "Pump", "Role", "Target type",
                   "Push volume", "Push depth", "Push flow", "Lead time",
                   "Mount offset")
        for col, text in enumerate(headers):
            head = QLabel(text)
            head.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt; "
                f"font-weight: 600;")
            self._grid.addWidget(head, 0, col)
        self._header_row_count = 1
        card.add_widget(holder)

        self._bore_empty = _small(
            "No needle configured yet — set the needle form and its bores on "
            "Hardware Setup → Needle.", "peach")
        card.add_widget(self._bore_empty)

        self._bore_notes = _small("")
        card.add_widget(self._bore_notes)
        return card

    def set_pump_ids(self, pump_ids) -> None:
        """The enabled+configured pump ids, for every row's Pump combo."""
        self._pump_ids = [str(p) for p in (pump_ids or [])]
        self._rebuild_rows()

    def set_needle(self, needle) -> None:
        """Point the table at a needle assembly (None clears it)."""
        self._needle = needle
        self._rebuild_rows()

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
        """Bore 0 aspirates; a second bore doses. Everything else idles.

        Bore 0 is the ``needle_origin_um`` datum, so making it the aspirating
        bore means a single-bore assembly (and every pre-v7.9 needle) reproduces
        the legacy sequence with no offsets applied at all.
        """
        if bore_index == 0:
            return BoreRole.ASPIRATE_TARGET
        if bore_index == 1 and n_bores >= 2:
            return BoreRole.PUSH_REAGENT
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
        self._refresh_notes()
        self.programs_changed.emit()

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
                    row.pump.addItem(declared, declared)
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
        rewired = bool(declared) and declared != self._declared_pumps.get(
            row.bore_index, declared)
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
            if idx >= 0:
                row.target_type.blockSignals(True)
                row.target_type.setCurrentIndex(idx)
                row.target_type.blockSignals(False)
        for widget, value in ((row.volume, prog.volume_uL),
                              (row.depth, prog.depth_mm),
                              (row.rate, prog.rate_uL_s),
                              (row.lead, prog.lead_time_s)):
            widget.blockSignals(True)
            try:
                widget.setValue(float(value))
            except (TypeError, ValueError):
                pass
            finally:
                widget.blockSignals(False)

    def _refresh_row_labels(self, row: _BoreRow) -> None:
        """Bore identity + its geometry, and the measured mount offset."""
        bore = needle_bore_at(self._needle, row.bore_index)
        summary = ""
        try:
            summary = bore.summary_line()
        except Exception:
            summary = ""
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
            row.offset.setText("⚠ offset not measured")
            row.offset.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
            row.offset.setToolTip(
                "This bore has no measured lateral offset from bore 1, so it "
                "would be driven to bore 1's position — typically 100-500 µm "
                "off the target. Measure the bore offsets after every needle "
                "change or re-seat (the assembly's rotation in the holder is "
                "arbitrary).")
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
                volume_uL=float(row.volume.value()),
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
            "⚠ Recorded intent only: these rules are saved and shown, but "
            "nothing selects objects from a fluorescence image from them yet. "
            "Choose the removal targets by clicking them.", "peach")
        card.add_widget(note)
        return card

    def reload_target_types(self) -> None:
        """Re-read the store and refresh the list + every row's combo."""
        try:
            from SupportClasses.TargetTypeStore import get_store
            store = get_store()
            store.reload()
            self._target_types = list(store.all())
        except Exception as exc:
            logger.debug("target type store unavailable: %s", exc)
            self._target_types = []
        self._refresh_types_list()
        for row in self._rows:
            keep = row.target_type_id()
            self._populate_type_combo(row)
            idx = row.target_type.findData(keep)
            if idx >= 0:
                row.target_type.blockSignals(True)
                row.target_type.setCurrentIndex(idx)
                row.target_type.blockSignals(False)

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
        if not self._target_types:
            self._types_body.addWidget(_small(
                "No target types defined yet — add JSON files under "
                "config/hardware/target_types/user/."))
            return
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
            self._types_body.addWidget(row)

    # ── trypsin bore ──────────────────────────────────────────────

    def _build_trypsin_card(self) -> Card:
        card = Card("Trypsin bore — reagent well")
        card.add_widget(_small(
            "Give a bore the “Push reagent onto the target” role above; its push "
            "volume, flow and lead time are that row's own cells. This card only "
            "says WHERE that bore refills."))
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

        # Not a disabled control: an "overlap" checkbox greyed out would read as
        # "coming soon". The geometry forbids it outright.
        card.add_widget(_small(
            "There is no “start aspirating before the push ends” option: the "
            "pushing bore and the aspirating bore are laterally offset, so the "
            "tool has to travel between the push and the pull. They are never "
            "over the target at the same time."))
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
        """
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
        return notes

    def _refresh_notes(self) -> None:
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
        }

    def apply_state(self, state) -> None:
        if not isinstance(state, dict):
            return
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
