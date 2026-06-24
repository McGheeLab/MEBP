"""workflow_settings_dialog.py — scrollable, saveable per-workflow settings.

v7.5.x: Every workflow gets a comprehensive "⚙ Settings" popout built on this
shell. It provides:

  * A **scrollable** body (QScrollArea, ``setWidgetResizable(True)``) so a
    workflow can expose a large, robust set of options without the dialog
    growing off-screen.
  * ``add_section(title)`` → a :class:`SettingsSection` whose ``add(...)`` lays
    out a labelled :class:`FormRow` AND registers the field for persistence /
    reset. Generic get/set handles QSpinBox / QDoubleSpinBox / QCheckBox /
    QComboBox.
  * A **profile bar** (Save / Save As… / Load / Delete / Import file… / Export
    file… / Reset to defaults) backed by :class:`WorkflowSettingsStore`, so the
    operator can keep the settings they like for a workflow and reload them.
    The last-used values auto-restore on the next visit (``__last__.json``).
  * A shared read-only **Locations & Hardware** panel (``build_locations_widget``)
    that shows where every ink / reagent / service well is, the needle + syringe
    geometry, pump↔ink assignments, and the calibrated Z reference heights — the
    "shows where all inks and stuff are" requirement.

The page creates its config widgets eagerly (so ``page._foo`` attributes and the
``_current_config`` / ``_build_settings`` builders keep working) and hands them to
this dialog to lay out. The dialog is a persistent, modeless child of the page —
shown on demand, never destroyed while the page lives, so the hosted widgets stay
valid.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Callable, Optional

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDoubleSpinBox, QFileDialog, QFrame,
    QHBoxLayout, QInputDialog, QLabel, QMessageBox, QPushButton, QScrollArea,
    QSizePolicy, QSpinBox, QVBoxLayout, QWidget,
)

from gui.styles import COLORS
from gui.scaling import s, sf, sp
from gui.widgets.components import Card, FormRow
from SupportClasses.WorkflowSettingsStore import WorkflowSettingsStore

logger = logging.getLogger(__name__)


# ════════════════════════════════════════════════════════════════════
#  Generic widget value get / set
# ════════════════════════════════════════════════════════════════════

def widget_value(w: QWidget):
    """Read a JSON-serialisable value out of a config widget."""
    if isinstance(w, QCheckBox):
        return bool(w.isChecked())
    if isinstance(w, QSpinBox):
        return int(w.value())
    if isinstance(w, QDoubleSpinBox):
        return float(w.value())
    if isinstance(w, QComboBox):
        data = w.currentData()
        token: dict = {"text": w.currentText()}
        if data is None or isinstance(data, (str, int, float, bool)):
            token["data"] = data
        return token
    return None


def set_widget_value(w: QWidget, val) -> bool:
    """Apply a stored value to a config widget. Returns False if a combo's
    target isn't present yet (caller should mark it pending and retry once the
    combo is populated)."""
    if isinstance(w, QCheckBox):
        w.setChecked(bool(val))
        return True
    if isinstance(w, QSpinBox):
        try:
            w.setValue(int(round(float(val))))
        except (TypeError, ValueError):
            return False
        return True
    if isinstance(w, QDoubleSpinBox):
        try:
            w.setValue(float(val))
        except (TypeError, ValueError):
            return False
        return True
    if isinstance(w, QComboBox):
        data = None
        text = None
        if isinstance(val, dict):
            data = val.get("data")
            text = val.get("text")
        else:
            text = val
        # An empty selection ("" / the "(none)" placeholder) is a deliberate
        # choice — select the item whose data is "" if present (the placeholder),
        # else index 0. This lets Reset / restore put a combo back to "(none)".
        if (data is None or data == "") and (text is None or text == ""):
            i = w.findData("")
            if i < 0 and w.count() > 0:
                i = 0
            if i >= 0:
                w.setCurrentIndex(i)
                return True
            return False
        if data is not None:
            i = w.findData(data)
            if i >= 0:
                w.setCurrentIndex(i)
                return True
        if text is not None and text != "":
            i = w.findText(text)
            if i >= 0:
                w.setCurrentIndex(i)
                return True
            if w.isEditable():
                w.setCurrentText(text)
                return True
        # Not-yet-present selection — not resolvable now (combo not populated).
        return False
    return False


# ════════════════════════════════════════════════════════════════════
#  Section helper
# ════════════════════════════════════════════════════════════════════

class SettingsSection:
    """A titled group of settings rows inside the dialog scroll body."""

    def __init__(self, dialog: "WorkflowSettingsDialog", card: Card):
        self._dialog = dialog
        self._card = card

    def add(self, key: str, label: str, widget: QWidget, default,
            help: str | None = None) -> QWidget:
        """Lay out ``widget`` as a labelled row and register it for persistence.

        ``key`` is the persistence key; ``default`` is what Reset restores.
        """
        self._dialog._register(key, widget, default)
        row = FormRow(label, widget, help_text=help)
        if help:
            row.set_help_visible(True)
        self._card.add_widget(row)
        return widget

    def add_check(self, key: str, checkbox: QCheckBox, default: bool,
                  help: str | None = None) -> QCheckBox:
        """Add a checkbox (carries its own label) and register it."""
        self._dialog._register(key, checkbox, default)
        self._card.add_widget(checkbox)
        if help:
            self.add_note(help)
        return checkbox

    def add_widget(self, w: QWidget) -> QWidget:
        """Add a display-only widget (status label, swatch, …) — not persisted."""
        self._card.add_widget(w)
        return w

    def add_note(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setWordWrap(True)
        lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        self._card.add_widget(lbl)
        return lbl


# ════════════════════════════════════════════════════════════════════
#  Dialog
# ════════════════════════════════════════════════════════════════════

class WorkflowSettingsDialog(QDialog):
    """Scrollable, saveable settings popout for one workflow."""

    def __init__(self, workflow_id: str, title: str, *,
                 parent: QWidget | None = None,
                 on_change: Optional[Callable[[], None]] = None):
        super().__init__(parent)
        self.setWindowTitle(f"{title} — Settings")
        self.setModal(False)
        self._workflow_id = workflow_id
        self._title = title
        self._store = WorkflowSettingsStore(workflow_id)
        self._on_change = on_change

        self._fields: dict[str, tuple[QWidget, object]] = {}
        self._order: list[str] = []
        # Combos whose saved value couldn't be applied yet because the combo was
        # empty at load time (hardware-populated). Resolved by resolve_pending()
        # once set_hardware_config repopulates them.
        self._pending: dict[str, object] = {}
        # One-shot: the last-used combo selections, re-applied AFTER the page's
        # own combo repopulate (which auto-defaults to e.g. the pump's assigned
        # ink). Without this a restored "(none)" choice would be clobbered by
        # that default on the first set_hardware_config.
        self._reapply_combos: dict[str, object] = {}

        self._info_card: Card | None = None
        self._info_refresher: Optional[Callable[[], QWidget]] = None

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(12), s(12), s(12))
        outer.setSpacing(s(8))

        # ── Title ──
        head = QLabel(f"⚙  {title} — Settings")
        head.setStyleSheet(
            f"color: {COLORS['blue']}; font-size: {sf(13)}pt; font-weight: 600;")
        outer.addWidget(head)

        # ── Profile bar ──
        outer.addWidget(self._build_profile_bar())

        # ── Scrollable body ──
        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        content = QWidget()
        self._content_layout = QVBoxLayout(content)
        self._content_layout.setContentsMargins(0, 0, s(6), 0)
        self._content_layout.setSpacing(s(10))
        self._scroll.setWidget(content)
        outer.addWidget(self._scroll, stretch=1)

        # ── Footer ──
        footer = QHBoxLayout()
        footer.addStretch(1)
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.hide)
        footer.addWidget(close_btn)
        outer.addLayout(footer)

        self.resize(s(540), s(660))

    # ── Profile bar ───────────────────────────────────────────────

    def _build_profile_bar(self) -> QFrame:
        frame = QFrame(self)
        frame.setObjectName("wfProfileBar")
        frame.setStyleSheet(
            f"QFrame#wfProfileBar {{"
            f"  background-color: {COLORS['surface0']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: {sp(6)};"
            f"}}"
        )
        row = QHBoxLayout(frame)
        row.setContentsMargins(s(8), s(6), s(8), s(6))
        row.setSpacing(s(6))

        row.addWidget(QLabel("Settings file:"))
        self._profile_combo = QComboBox()
        self._profile_combo.setMinimumWidth(s(150))
        self._profile_combo.setToolTip(
            "Saved settings for this workflow. Select one to load it.")
        self._profile_combo.activated.connect(self._on_profile_activated)
        row.addWidget(self._profile_combo, stretch=1)

        def _btn(text, slot, tip=""):
            b = QPushButton(text)
            b.setCursor(Qt.PointingHandCursor)
            if tip:
                b.setToolTip(tip)
            b.clicked.connect(slot)
            row.addWidget(b)
            return b

        _btn("Save", self._on_save, "Overwrite the selected settings file.")
        _btn("Save As…", self._on_save_as, "Save the current settings under a new name.")
        _btn("Delete", self._on_delete, "Delete the selected settings file.")
        _btn("Import…", self._on_import, "Load settings from a file anywhere on disk.")
        _btn("Export…", self._on_export, "Save the current settings to a file anywhere on disk.")
        _btn("Reset", self._on_reset, "Restore the factory default settings.")

        self._refresh_profile_combo()
        return frame

    def _refresh_profile_combo(self, select: str | None = None) -> None:
        combo = self._profile_combo
        combo.blockSignals(True)
        combo.clear()
        combo.addItem("(unsaved)", userData=None)
        for name in self._store.list_profiles():
            combo.addItem(name, userData=name)
        if select is not None:
            i = combo.findData(select)
            if i >= 0:
                combo.setCurrentIndex(i)
        combo.blockSignals(False)

    def _selected_profile(self) -> str | None:
        return self._profile_combo.currentData()

    # ── Profile actions ───────────────────────────────────────────

    def _on_profile_activated(self, _idx: int) -> None:
        name = self._selected_profile()
        if not name:
            return
        values = self._store.load_profile(name)
        if values is None:
            QMessageBox.warning(self, "Load failed",
                                f"Could not read settings file '{name}'.")
            return
        self.apply(values)
        logger.info("Loaded workflow settings '%s' for %s", name,
                    self._workflow_id)

    def _save_profile_or_warn(self, name: str) -> bool:
        """Save, surfacing any write failure to the operator (not silent)."""
        try:
            self._store.save_profile(name, self.collect())
        except Exception as exc:
            QMessageBox.warning(
                self, "Save failed",
                f"Could not save settings '{name}':\n{exc}")
            return False
        self._refresh_profile_combo(select=name)
        return True

    def _on_save(self) -> None:
        name = self._selected_profile()
        if not name:
            # Nothing selected yet — behave like Save As.
            self._on_save_as()
            return
        self._save_profile_or_warn(name)

    def _on_save_as(self) -> None:
        name, ok = QInputDialog.getText(
            self, "Save settings as", "Name for this settings file:")
        if not ok or not str(name).strip():
            return
        self._save_profile_or_warn(str(name).strip())

    def _on_delete(self) -> None:
        name = self._selected_profile()
        if not name:
            return
        resp = QMessageBox.question(
            self, "Delete settings",
            f"Delete the saved settings file '{name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No)
        if resp != QMessageBox.StandardButton.Yes:
            return
        self._store.delete_profile(name)
        self._refresh_profile_combo()

    def _on_import(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Import workflow settings", "",
            "Settings files (*.json);;All files (*)")
        if not path:
            return
        values = self._store.read_file(path)
        if values is None:
            QMessageBox.warning(self, "Import failed",
                                "That file is not a readable settings file.")
            return
        self.apply(values)

    def _on_export(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "Export workflow settings",
            f"{self._workflow_id}_settings.json",
            "Settings files (*.json);;All files (*)")
        if not path:
            return
        if not str(path).lower().endswith(".json"):
            path = f"{path}.json"
        try:
            self._store.write_file(path, self.collect(), name=Path(path).stem)
        except Exception as exc:
            QMessageBox.warning(self, "Export failed",
                                f"Could not write '{path}':\n{exc}")

    def _on_reset(self) -> None:
        self.reset_defaults()
        self._profile_combo.setCurrentIndex(0)  # "(unsaved)"

    # ── Sections ──────────────────────────────────────────────────

    def add_section(self, title: str, collapsible: bool = False) -> SettingsSection:
        card = Card(title, collapsible=collapsible)
        self._content_layout.addWidget(card)
        return SettingsSection(self, card)

    def add_info_section(self, title: str = "Locations & Hardware") -> Card:
        """Add the read-only info card. Its body is rebuilt by the registered
        refresher each time the dialog is shown."""
        card = Card(title)
        self._content_layout.addWidget(card)
        self._info_card = card
        return card

    def set_info_refresher(self, fn: Callable[[], QWidget]) -> None:
        """``fn()`` must return a fresh QWidget to mount in the info card."""
        self._info_refresher = fn

    def finalize(self) -> None:
        """Call after all sections are added — adds the trailing stretch."""
        self._content_layout.addStretch(1)

    # ── Field registry ────────────────────────────────────────────

    def _register(self, key: str, widget: QWidget, default) -> None:
        if key in self._fields:
            logger.warning("Duplicate settings key '%s' for %s",
                           key, self._workflow_id)
        self._fields[key] = (widget, default)
        self._order.append(key)

    def collect(self) -> dict:
        return {k: widget_value(w) for k, (w, _d) in self._fields.items()}

    def apply(self, values: dict, *, reapply_combos: bool = False) -> None:
        if not isinstance(values, dict):
            return
        for key, (widget, _default) in self._fields.items():
            if key not in values:
                continue
            ok = set_widget_value(widget, values[key])
            if isinstance(widget, QComboBox):
                if not ok:
                    self._pending[key] = values[key]
                else:
                    self._pending.pop(key, None)
                # A restored combo can still be clobbered by the page's own
                # combo repopulate (which auto-defaults). Re-assert it once after
                # that repopulate when this is the initial last-used restore.
                if reapply_combos:
                    self._reapply_combos[key] = values[key]
            else:
                self._pending.pop(key, None)
        self._emit_change()

    def resolve_pending(self) -> None:
        """Re-assert hardware-dependent combos after the page repopulates them:
        apply combos that weren't resolvable at load time (``_pending``) AND
        re-assert the initial last-used combo selections once (``_reapply_combos``,
        so a restored "(none)" isn't overwritten by the page's auto-default)."""
        for key in list(self._pending.keys()):
            widget, _default = self._fields.get(key, (None, None))
            if widget is None or set_widget_value(widget, self._pending[key]):
                self._pending.pop(key, None)
        for key, value in list(self._reapply_combos.items()):
            widget, _default = self._fields.get(key, (None, None))
            if widget is not None:
                set_widget_value(widget, value)
            self._reapply_combos.pop(key, None)

    def reset_defaults(self) -> None:
        for _key, (widget, default) in self._fields.items():
            set_widget_value(widget, default)
        self._pending.clear()
        self._reapply_combos.clear()
        self._emit_change()

    def _emit_change(self) -> None:
        if self._on_change is not None:
            try:
                self._on_change()
            except Exception as exc:
                logger.debug("settings on_change failed: %s", exc)

    # ── Persistence convenience ───────────────────────────────────

    def load_last(self) -> None:
        values = self._store.load_last()
        if values:
            # reapply_combos=True: re-assert the restored combo selections once
            # after the page's set_hardware_config repopulates them.
            self.apply(values, reapply_combos=True)
        self._refresh_profile_combo()

    def save_last(self) -> None:
        try:
            self._store.save_last(self.collect())
        except Exception as exc:
            logger.debug("save_last failed: %s", exc)

    # ── Info panel refresh on show ────────────────────────────────

    def _refresh_info(self) -> None:
        if self._info_card is None or self._info_refresher is None:
            return
        # Clear the card body.
        lay = self._info_card.body_layout()
        while lay.count():
            item = lay.takeAt(0)
            w = item.widget()
            if w is not None:
                w.setParent(None)
                w.deleteLater()
        try:
            widget = self._info_refresher()
        except Exception as exc:
            logger.debug("info refresher failed: %s", exc)
            widget = QLabel("(hardware info unavailable)")
        if widget is not None:
            self._info_card.add_widget(widget)

    # ── Qt lifecycle ──────────────────────────────────────────────

    def showEvent(self, event):
        self.resolve_pending()
        self._refresh_info()
        super().showEvent(event)

    def hideEvent(self, event):
        self.save_last()
        super().hideEvent(event)

    def closeEvent(self, event):
        self.save_last()
        super().closeEvent(event)


# ════════════════════════════════════════════════════════════════════
#  Shared "Locations & Hardware" read-only panel
# ════════════════════════════════════════════════════════════════════

def _kv_row(label: str, value: str, value_color: str | None = None) -> QWidget:
    w = QWidget()
    row = QHBoxLayout(w)
    row.setContentsMargins(0, 0, 0, 0)
    row.setSpacing(s(8))
    l = QLabel(label)
    l.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
    row.addWidget(l)
    row.addStretch(1)
    v = QLabel(value)
    v.setTextInteractionFlags(Qt.TextSelectableByMouse)
    v.setStyleSheet(
        f"color: {value_color or COLORS['text']}; font-size: {sf(9)}pt;"
        f"font-weight: 600;")
    v.setAlignment(Qt.AlignRight)
    row.addWidget(v)
    return w


def _subheader(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f"color: {COLORS['mauve']}; font-size: {sf(9)}pt; font-weight: 700;"
        f"margin-top: {sp(4)};")
    return lbl


def _swatch_row(name: str, color_hex: str, detail: str) -> QWidget:
    w = QWidget()
    row = QHBoxLayout(w)
    row.setContentsMargins(0, 0, 0, 0)
    row.setSpacing(s(6))
    sw = QLabel()
    sw.setFixedSize(s(12), s(12))
    safe = color_hex if isinstance(color_hex, str) and color_hex else COLORS["surface2"]
    sw.setStyleSheet(
        f"background-color: {safe}; border: 1px solid {COLORS['surface2']};"
        f"border-radius: {sp(3)};")
    row.addWidget(sw)
    nm = QLabel(name)
    nm.setStyleSheet(f"color: {COLORS['text']}; font-size: {sf(9)}pt; font-weight: 600;")
    row.addWidget(nm)
    row.addStretch(1)
    dt = QLabel(detail)
    dt.setTextInteractionFlags(Qt.TextSelectableByMouse)
    dt.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
    dt.setAlignment(Qt.AlignRight)
    row.addWidget(dt)
    return w


def _fmt(value, suffix: str = "", nd: int = 3) -> str:
    if value is None:
        return "—"
    try:
        return f"{float(value):.{nd}f}{suffix}"
    except (TypeError, ValueError):
        return f"{value}{suffix}"


def build_locations_widget(controller, hw_config, well_positions, *,
                           z_references: dict | None = None,
                           safe_z: float | None = None,
                           extras: list[tuple[str, str]] | None = None) -> QWidget:
    """Build the read-only "where everything is" panel.

    Defensive (``getattr`` throughout) so it works with partial / fake configs.
    """
    # Late import to avoid any import-order coupling; this module is tiny + pure.
    try:
        from gui.pages.workflows._reagent_prep import service_well_names
    except Exception:
        service_well_names = lambda _hw: {}  # noqa: E731

    well_positions = well_positions or {}
    z_references = z_references or {}

    root = QWidget()
    lay = QVBoxLayout(root)
    lay.setContentsMargins(0, 0, 0, 0)
    lay.setSpacing(s(3))

    # ── Needle ──
    lay.addWidget(_subheader("Needle"))
    needle = getattr(hw_config, "needle", None) if hw_config else None
    if needle is None:
        lay.addWidget(_kv_row("Needle", "not configured", COLORS["peach"]))
    else:
        gauge = getattr(needle, "gauge", None)
        idv = getattr(needle, "id_um", None)
        odv = getattr(needle, "od_um", None)
        length_mm = getattr(needle, "length_mm", None)
        ivol = getattr(needle, "internal_volume_uL", None)
        area = getattr(needle, "cross_section_area_mm2", None)
        lay.addWidget(_kv_row("Gauge", f"{gauge}G" if gauge else "—"))
        lay.addWidget(_kv_row("Inner / outer Ø",
                              f"{_fmt(idv, ' µm', 0)} / {_fmt(odv, ' µm', 0)}"))
        lay.addWidget(_kv_row("Length", _fmt(length_mm, " mm", 2)))
        lay.addWidget(_kv_row("1 needle (bore volume)", _fmt(ivol, " µL", 4)))
        lay.addWidget(_kv_row("Bore cross-section", _fmt(area, " mm²", 5)))

    # ── Pumps & inks ──
    lay.addWidget(_subheader("Pumps"))
    pumps = getattr(hw_config, "pumps", None) if hw_config else None
    if not pumps:
        lay.addWidget(_kv_row("Pumps", "none configured", COLORS["peach"]))
    else:
        for pid, pcfg in pumps.items():
            syringe = getattr(pcfg, "syringe", None)
            svol = getattr(syringe, "volume_uL", None) if syringe else None
            try:
                ink_names = list(getattr(pcfg, "ink_names", []) or [])
            except Exception:
                ink_names = []
            inks = ", ".join(ink_names) if ink_names else "(no ink)"
            syr = f"{int(svol)} µL syringe" if svol else "no syringe"
            lay.addWidget(_kv_row(str(pid), f"{syr} · {inks}"))

    # ── Ink / reagent locations ──
    lay.addWidget(_subheader("Ink / reagent locations"))
    ink_locations = getattr(hw_config, "ink_locations", None) if hw_config else None
    ink_library = getattr(hw_config, "ink_library", {}) if hw_config else {}
    if not ink_locations:
        lay.addWidget(_kv_row("Reagent locations", "none assigned",
                              COLORS["peach"]))
    else:
        for ink_name, wells in ink_locations.items():
            spec = (ink_library or {}).get(ink_name)
            itype = getattr(spec, "ink_type", "") if spec else ""
            isub = (getattr(spec, "ink_subtype", "") or "").strip() if spec else ""
            color = getattr(spec, "color", "") if spec else ""
            wells_l = list(wells or [])
            calibrated = sum(1 for w in wells_l if w in well_positions)
            wells_txt = ", ".join(wells_l) if wells_l else "(none)"
            detail = wells_txt
            if wells_l:
                detail += f"  ·  {calibrated}/{len(wells_l)} cal"
            type_label = (f"{itype} · {isub}" if itype == "ink" and isub else itype)
            name = ink_name + (f" [{type_label}]" if type_label else "")
            lay.addWidget(_swatch_row(name, color, detail))

    # ── Service-well roles ──
    lay.addWidget(_subheader("Service wells (prep / clean)"))
    roles = service_well_names(hw_config)
    for role in ("waste", "oil", "wash", "buffer"):
        wn = roles.get(role)
        if not wn:
            lay.addWidget(_kv_row(role, "unassigned", COLORS["peach"]))
        elif wn in well_positions:
            lay.addWidget(_kv_row(role, f"{wn}  ✓", COLORS["green"]))
        else:
            lay.addWidget(_kv_row(role, f"{wn}  (not calibrated)",
                                  COLORS["yellow"]))

    # ── Calibration heights ──
    lay.addWidget(_subheader("Calibration heights"))
    lay.addWidget(_kv_row("Calibrated wells", str(len(well_positions))))
    if safe_z is not None:
        lay.addWidget(_kv_row("Safe / move Z", _fmt(safe_z, " mm", 3)))
    else:
        lay.addWidget(_kv_row("Safe / move Z", "not set", COLORS["peach"]))
    label_map = [
        ("plate_bottom_z", "Plate bottom Z"),
        ("plate_top_z", "Plate top Z"),
        ("fast_move_z", "Fast-move Z"),
        ("max_z", "Max Z"),
        ("replace_z", "Replace Z"),
    ]
    for key, label in label_map:
        if key in z_references:
            lay.addWidget(_kv_row(label, _fmt(z_references.get(key), " mm", 3)))

    # ── Caller-supplied extras (e.g. selected ink/reagent source well) ──
    if extras:
        lay.addWidget(_subheader("This workflow"))
        for label, value in extras:
            lay.addWidget(_kv_row(label, value))

    lay.addStretch(1)
    return root
