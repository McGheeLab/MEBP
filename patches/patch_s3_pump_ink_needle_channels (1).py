#!/usr/bin/env python3
"""
patch_s3_pump_ink_needle_channels.py — MEBP v7.2.4 Session 3 Patch

Applies Session 3 changes via targeted str_replace on existing files:
  A) SupportClasses/HardwareConfig.py — pump_ink_map, channel map, validate
  B) gui/pages/hardware_setup.py — reorder UI, channel map section, exclusions

Run from the MEBP project root:
    python patch_s3_pump_ink_needle_channels.py
"""

import os
import sys
import shutil

GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

applied = 0
skipped = 0
failed  = 0


def find_project_root():
    for candidate in [os.getcwd(), os.path.dirname(os.path.abspath(__file__))]:
        if os.path.isdir(os.path.join(candidate, "SupportClasses")) and \
           os.path.isdir(os.path.join(candidate, "gui")):
            return candidate
    return None


def read_file(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def write_file(path, content):
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


def backup(path):
    bak = path + ".v723.bak"
    if not os.path.exists(bak):
        shutil.copy2(path, bak)


def replace_text(content, old, new, tag):
    global applied, skipped, failed
    if old not in content:
        if new.strip() and new.strip()[:60] in content:
            print(f"  {YELLOW}SKIP{RESET}: {tag} — already applied")
            skipped += 1
        else:
            print(f"  {RED}MISS{RESET}: {tag} — old text not found")
            failed += 1
        return content
    result = content.replace(old, new, 1)
    print(f"  {GREEN}OK{RESET}:   {tag}")
    applied += 1
    return result


def insert_after(content, anchor, insertion, tag):
    global applied, skipped, failed
    if anchor not in content:
        print(f"  {RED}MISS{RESET}: {tag} — anchor not found")
        failed += 1
        return content
    if insertion.strip()[:60] in content:
        print(f"  {YELLOW}SKIP{RESET}: {tag} — already applied")
        skipped += 1
        return content
    idx = content.index(anchor) + len(anchor)
    result = content[:idx] + "\n" + insertion + content[idx:]
    print(f"  {GREEN}OK{RESET}:   {tag}")
    applied += 1
    return result


def insert_before(content, anchor, insertion, tag):
    global applied, skipped, failed
    if anchor not in content:
        print(f"  {RED}MISS{RESET}: {tag} — anchor not found")
        failed += 1
        return content
    if insertion.strip()[:60] in content:
        print(f"  {YELLOW}SKIP{RESET}: {tag} — already applied")
        skipped += 1
        return content
    idx = content.index(anchor)
    result = content[:idx] + insertion + "\n" + content[idx:]
    print(f"  {GREEN}OK{RESET}:   {tag}")
    applied += 1
    return result


# ═══════════════════════════════════════════════════════════════════
# PATCH A: SupportClasses/HardwareConfig.py
# ═══════════════════════════════════════════════════════════════════

def patch_hardware_config(root):
    filepath = os.path.join(root, "SupportClasses", "HardwareConfig.py")
    print(f"\n{'='*60}")
    print(f"PATCH A: {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return
    backup(filepath)
    content = read_file(filepath)

    # ── A1: Add needle_channel_pump_map field ─────────────────────
    content = insert_after(content,
        '    buffer_ink_name: str | None = None',
        '''
    # ── v7.2.4: Needle channel → pump mapping ────────────────────
    needle_channel_pump_map: dict[int, str] = field(default_factory=dict)
    # Maps channel index (0-based) → pump_id
    # Single-channel needle: {0: "P1"}
    # Multi-channel: {0: "P1", 1: "P2", 2: "P3"}
''',
        "A1: Add needle_channel_pump_map field")

    # ── A2: Add pump_ink_map / ink_pump_map properties ────────────
    content = insert_after(content,
        '    notes: str = ""',
        '''
    # ══════════════════════════════════════════════════════════════
    #  v7.2.4: PUMP-INK MAPPING PROPERTIES (S3.1)
    # ══════════════════════════════════════════════════════════════

    @property
    def pump_ink_map(self) -> dict[str, str | None]:
        """
        Get pump → ink_name mapping for quick lookup.
        Returns dict like {"P1": "Hydrogel A", "P2": "MSC Cells", "P3": None}
        """
        result = {}
        for pid, pcfg in self.pumps.items():
            if pcfg.enabled:
                result[pid] = pcfg.ink.name if pcfg.ink else None
            else:
                result[pid] = None
        return result

    @property
    def ink_pump_map(self) -> dict[str, str]:
        """
        Get ink_name → pump_id reverse lookup.
        Returns dict like {"Hydrogel A": "P1", "MSC Cells": "P2"}
        """
        result = {}
        for pid, pcfg in self.pumps.items():
            if pcfg.enabled and pcfg.ink:
                result[pcfg.ink.name] = pid
        return result

    @property
    def unassigned_inks(self) -> list[str]:
        """Get list of ink names in the library not assigned to any pump."""
        assigned = set(self.ink_pump_map.keys())
        return [name for name in self.ink_library if name not in assigned]

    @property
    def enabled_pump_ids(self) -> list[str]:
        """List of pump IDs that are enabled (have syringes)."""
        return [pid for pid, p in self.pumps.items() if p.enabled and p.is_configured]
''',
        "A2: Add pump_ink_map / ink_pump_map / unassigned_inks / enabled_pump_ids")

    # ── A3: Enhance validate() — add ink uniqueness + channel checks ─
    # Find the end of existing validation, before the return
    content = replace_text(content,
        '''        return (len(issues) == 0, issues)

    @property
    def is_valid(self) -> bool:''',
        '''        # v7.2.4: Ink uniqueness — each ink assigned to at most one pump
        ink_assignments: dict[str, list[str]] = {}
        for pid, pcfg in self.pumps.items():
            if pcfg.enabled and pcfg.ink:
                ink_name = pcfg.ink.name
                ink_assignments.setdefault(ink_name, []).append(pid)
        for ink_name, pump_ids in ink_assignments.items():
            if len(pump_ids) > 1:
                issues.append(
                    f"Ink '{ink_name}' assigned to multiple pumps: "
                    f"{', '.join(pump_ids)}")

        # v7.2.4: Needle channel-pump mapping
        if self.needle is not None:
            num_channels = self.needle.num_channels
            enabled_ids = self.enabled_pump_ids

            if num_channels > 0 and enabled_ids:
                if len(self.needle_channel_pump_map) != num_channels:
                    issues.append(
                        f"Needle has {num_channels} channel(s) but "
                        f"{len(self.needle_channel_pump_map)} mapped — "
                        f"all channels must be assigned to pumps")

                for ch_idx, pump_id in self.needle_channel_pump_map.items():
                    if pump_id not in enabled_ids:
                        issues.append(
                            f"Channel {ch_idx + 1} mapped to {pump_id} "
                            f"but {pump_id} is not enabled/configured")

                mapped_pumps: dict[str, list[int]] = {}
                for ch_idx, pump_id in self.needle_channel_pump_map.items():
                    mapped_pumps.setdefault(pump_id, []).append(ch_idx)
                for pump_id, channels in mapped_pumps.items():
                    if len(channels) > 1:
                        ch_strs = [str(c + 1) for c in channels]
                        issues.append(
                            f"Pump {pump_id} assigned to multiple channels: "
                            f"{', '.join(ch_strs)}")

        return (len(issues) == 0, issues)

    @property
    def is_valid(self) -> bool:''',
        "A3: Enhanced validate() with ink uniqueness + channel map checks")

    # ── A4: Add channel map helper methods ────────────────────────
    content = insert_before(content,
        '''    # ══════════════════════════════════════════════════════════════
    #  INK LIBRARY''',
        '''    # ══════════════════════════════════════════════════════════════
    #  v7.2.4: NEEDLE CHANNEL MAP HELPERS (S3.2)
    # ══════════════════════════════════════════════════════════════

    def set_channel_pump(self, channel_index: int, pump_id: str | None):
        """Assign a pump to a needle channel."""
        if pump_id is None:
            self.needle_channel_pump_map.pop(channel_index, None)
        else:
            if pump_id not in self.pumps:
                raise ValueError(f"Unknown pump: {pump_id}")
            self.needle_channel_pump_map[channel_index] = pump_id

    def get_channel_pump(self, channel_index: int) -> str | None:
        """Get pump ID assigned to a channel, or None."""
        return self.needle_channel_pump_map.get(channel_index)

    def clear_channel_map(self):
        """Clear all channel-pump assignments."""
        self.needle_channel_pump_map.clear()

    def auto_assign_channels(self):
        """Auto-assign channels to enabled pumps in order."""
        if self.needle is None:
            return
        self.needle_channel_pump_map.clear()
        enabled = self.enabled_pump_ids
        for ch_idx in range(self.needle.num_channels):
            if ch_idx < len(enabled):
                self.needle_channel_pump_map[ch_idx] = enabled[ch_idx]

    def _clear_invalid_channel_mappings(self):
        """Remove channel mappings that reference disabled pumps."""
        enabled = set(self.enabled_pump_ids)
        invalid = [ch for ch, pid in self.needle_channel_pump_map.items()
                    if pid not in enabled]
        for ch in invalid:
            del self.needle_channel_pump_map[ch]

    def get_pump_for_ink(self, ink_name: str) -> str | None:
        """Get the pump ID that has the given ink assigned, or None."""
        return self.ink_pump_map.get(ink_name)

    def get_channel_for_ink(self, ink_name: str) -> int | None:
        """Get the needle channel index that carries the given ink, or None."""
        pump_id = self.get_pump_for_ink(ink_name)
        if pump_id is None:
            return None
        for ch_idx, pid in self.needle_channel_pump_map.items():
            if pid == pump_id:
                return ch_idx
        return None

''',
        "A4: Add channel map helper methods")

    # ── A5: Update set_pump_syringe to clear invalid channel maps ─
    content = replace_text(content,
        '''        self.pumps[pump].syringe = syringe
        self.pumps[pump].enabled = syringe is not None''',
        '''        self.pumps[pump].syringe = syringe
        self.pumps[pump].enabled = syringe is not None
        # v7.2.4: Clear channel mappings if pump changes
        self._clear_invalid_channel_mappings()''',
        "A5: Clear invalid channel maps on syringe change")

    # ── A6: Update to_dict — add channel map + version ────────────
    content = replace_text(content,
        '''            "version": "7.2",''',
        '''            "version": "7.2.4",''',
        "A6a: Bump version to 7.2.4")

    content = replace_text(content,
        '''            "buffer_ink_name": self.buffer_ink_name,
        }''',
        '''            "buffer_ink_name": self.buffer_ink_name,
            # v7.2.4: Channel mapping (int keys → string for JSON)
            "needle_channel_pump_map": {
                str(ch): pid for ch, pid in self.needle_channel_pump_map.items()
            },
        }''',
        "A6b: Add needle_channel_pump_map to to_dict()")

    # ── A7: Update from_dict — restore channel map ────────────────
    content = replace_text(content,
        '''        config.buffer_ink_name = data.get("buffer_ink_name")
        return config''',
        '''        config.buffer_ink_name = data.get("buffer_ink_name")

        # v7.2.4: Needle channel-pump map (JSON string keys → int)
        raw_map = data.get("needle_channel_pump_map", {})
        config.needle_channel_pump_map = {
            int(ch): pid for ch, pid in raw_map.items()
        }

        return config''',
        "A7: Restore needle_channel_pump_map in from_dict()")

    # ── A8: Update __repr__ to show channel map ───────────────────
    content = replace_text(content,
        '''        return (f"HardwareConfig('{self.config_name}', needle={self.needle.gauge if self.needle else '?'}G, "
                f"plate={self.plate_format}, pumps=[{pumps}])")''',
        '''        ch_map = ""
        if self.needle_channel_pump_map:
            ch_map = f", channels={self.needle_channel_pump_map}"
        return (
            f"HardwareConfig('{self.config_name}', "
            f"needle={self.needle.gauge if self.needle else '?'}G, "
            f"plate={self.plate_format}, pumps=[{pumps}]{ch_map})")''',
        "A8: Updated __repr__ with channel map")

    write_file(filepath, content)
    print(f"  {GREEN}DONE{RESET}: HardwareConfig.py patched")


# ═══════════════════════════════════════════════════════════════════
# PATCH B: gui/pages/hardware_setup.py
# ═══════════════════════════════════════════════════════════════════

def patch_hardware_setup(root):
    filepath = os.path.join(root, "gui", "pages", "hardware_setup.py")
    print(f"\n{'='*60}")
    print(f"PATCH B: {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return
    backup(filepath)
    content = read_file(filepath)

    # ── B1: Add functools.partial import ──────────────────────────
    content = insert_after(content,
        'from pathlib import Path',
        'from functools import partial',
        "B1: Add functools.partial import")

    # ── B2: Add _channel_map_widgets init to __init__ ─────────────
    content = insert_after(content,
        '        self._last_valid = False',
        '''
        # v7.2.4: Channel mapping widgets (dynamic)
        self._channel_map_widgets: list[tuple[QLabel, QComboBox]] = []''',
        "B2: Add _channel_map_widgets to __init__")

    # ── B3: Wire pump changed handler instead of direct config_changed ─
    content = replace_text(content,
        '            pw.changed.connect(self._on_config_changed)\n            pump_lay.addWidget(pw)',
        '            pw.changed.connect(self._on_pump_changed)\n            pump_lay.addWidget(pw)',
        "B3: Wire pumps to _on_pump_changed instead of _on_config_changed")

    # ── B4: Add pump-ink summary label after pump widgets ─────────
    content = insert_after(content,
        "        self._content_layout.addWidget(pump_group)\n\n"
        "        # ── Section 5:" if "        self._content_layout.addWidget(pump_group)\n\n        # ── Section 5:" in content
        else "self._content_layout.addWidget(pump_group)",
        '''
        # v7.2.4: Pump-ink summary label (S3.6)
        self.pump_ink_summary = QLabel("")
        self.pump_ink_summary.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; "
            f"font-size: 9pt; padding: 4px 8px;")
        self.pump_ink_summary.setWordWrap(True)
        pump_lay.addWidget(self.pump_ink_summary)
''',
        "B4: Add pump-ink summary label")

    # ── B5: Rename Needle Section from 2 to 5 and reorder ─────────
    # Change the section comment for Needle
    content = replace_text(content,
        '# ── Section 2: Needle Configuration',
        '# ── Section 5: Needle Configuration  (v7.2.4: MOVED DOWN)',
        "B5a: Rename Needle section comment to Section 5")

    # Change Plate from 3 to 2
    content = replace_text(content,
        '# ── Section 3: Well Plate Format',
        '# ── Section 2: Well Plate Format  (v7.2.4: MOVED UP)',
        "B5b: Rename Plate section comment to Section 2")

    # Change Ink from 4 to 3
    content = replace_text(content,
        '# ── Section 4: Ink Library  (v7.2.3: MOVED UP)',
        '# ── Section 3: Ink Library',
        "B5c: Rename Ink section comment to Section 3")

    # Change Pump from 5 to 4
    content = replace_text(content,
        '# ── Section 5: Pump Channels  (v7.2.3: MOVED DOWN)',
        '# ── Section 4: Pump Channels  (v7.2.4: with exclusive inks)',
        "B5d: Rename Pump section comment to Section 4")

    # Change Rosette from 6 to 7
    content = replace_text(content,
        '# ── Section 6: Rosette Library  (v7.2.3: NEW)',
        '# ── Section 7: Rosette Library',
        "B5e: Rename Rosette section comment to Section 7")

    # ── B6: Wire channels_spin to _on_channels_changed ────────────
    content = replace_text(content,
        '        self.channels_spin.valueChanged.connect(self._on_config_changed)',
        '        self.channels_spin.valueChanged.connect(self._on_channels_changed)',
        "B6: Wire channels_spin to _on_channels_changed")

    # ── B7: Add Needle Channel Assignment section (Section 6) ─────
    # Insert AFTER the needle section (after needle_info_label and addWidget)
    content = insert_after(content,
        '        self._content_layout.addWidget(needle_group)\n\n'
        '        # ── Section 7: Rosette Library' if '        self._content_layout.addWidget(needle_group)\n\n        # ── Section 7: Rosette Library' in content
        else '        self._content_layout.addWidget(needle_group)',
        '''
        # ── Section 6: Needle Channel → Pump Mapping (v7.2.4: NEW) ─
        self.channel_map_group = QGroupBox("Needle Channel Assignment")
        self.channel_map_group.setStyleSheet(self._group_style())
        self._channel_map_layout = QVBoxLayout(self.channel_map_group)

        self.channel_map_info = QLabel(
            "Each needle channel must be assigned to a unique enabled pump.")
        self.channel_map_info.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; font-size: 9pt;")
        self.channel_map_info.setWordWrap(True)
        self._channel_map_layout.addWidget(self.channel_map_info)

        self._channel_rows_widget = QWidget()
        self._channel_rows_layout = QVBoxLayout(self._channel_rows_widget)
        self._channel_rows_layout.setContentsMargins(0, 0, 0, 0)
        self._channel_rows_layout.setSpacing(4)
        self._channel_map_layout.addWidget(self._channel_rows_widget)

        self.channel_map_status = QLabel("")
        self.channel_map_status.setStyleSheet(f"font-size: 9pt; padding: 2px 4px;")
        self._channel_map_layout.addWidget(self.channel_map_status)

        self._content_layout.addWidget(self.channel_map_group)
        self._rebuild_channel_map_rows()

''',
        "B7: Add Needle Channel Assignment section (Section 6)")

    # ── B8: Add _on_channels_changed method ───────────────────────
    content = insert_after(content,
        '        self._on_config_changed()',
        '''
    def _on_channels_changed(self, value: int):
        """v7.2.4 S3.9: Rebuild channel mapping rows on channel count change."""
        self._rebuild_channel_map_rows()
        self._on_config_changed()
''',
        "B8: Add _on_channels_changed method") if '    def _on_channels_changed' not in content else content

    # ── B9: Add pump-related handler methods ──────────────────────
    content = insert_before(content,
        '    def _on_config_changed(self):',
        '''    # ════════════════════════════════════════════════════════════════
    #  v7.2.4: PUMP / CHANNEL MAP HANDLERS
    # ════════════════════════════════════════════════════════════════

    def _on_pump_changed(self):
        """Called when any pump widget changes. Refreshes exclusions + channel map."""
        self._refresh_pump_ink_exclusions()
        self._update_pump_ink_summary()
        self._refresh_channel_map_pump_options()
        self._on_config_changed()

    def _refresh_pump_ink_exclusions(self):
        """v7.2.4 S3.5: Gray out inks assigned to other pumps."""
        ink_names = list(self._config.ink_library.keys())
        for pid, pw in self._pump_widgets.items():
            excluded = set()
            for other_pid, other_pw in self._pump_widgets.items():
                if other_pid != pid:
                    selected = other_pw.get_selected_ink_name()
                    if selected:
                        excluded.add(selected)
            pw.set_ink_names(ink_names, excluded)

    def _update_pump_ink_summary(self):
        """v7.2.4 S3.6: Update pump-ink summary label."""
        parts = []
        for pid in ["P1", "P2", "P3"]:
            pw = self._pump_widgets[pid]
            if pw.enable_check.isChecked():
                ink_name = pw.get_selected_ink_name()
                parts.append(f"{pid}→{ink_name or '(none)'}")
        self.pump_ink_summary.setText(
            ("Assignment: " + ", ".join(parts)) if parts else "No pumps enabled")

    def _rebuild_channel_map_rows(self):
        """v7.2.4 S3.7: Build N rows for channel→pump assignment."""
        self._channel_map_widgets.clear()
        while self._channel_rows_layout.count():
            item = self._channel_rows_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        num_channels = self.channels_spin.value()
        enabled_pumps = self._get_enabled_pump_ids()

        for ch_idx in range(num_channels):
            row_w = QWidget()
            row_l = QHBoxLayout(row_w)
            row_l.setContentsMargins(0, 0, 0, 0)
            row_l.setSpacing(8)

            label = QLabel("Bore →" if num_channels == 1 else f"Channel {ch_idx + 1} →")
            label.setMinimumWidth(80)
            row_l.addWidget(label)

            combo = QComboBox()
            combo.addItem("— Unassigned —", None)
            for pid in enabled_pumps:
                combo.addItem(pid, pid)
            combo.currentIndexChanged.connect(
                partial(self._on_channel_map_changed, ch_idx))
            row_l.addWidget(combo)
            row_l.addStretch()

            self._channel_rows_layout.addWidget(row_w)
            self._channel_map_widgets.append((label, combo))
        self._update_channel_map_status()

    def _refresh_channel_map_pump_options(self):
        """v7.2.4 S3.10: Refresh pump combos when pumps enable/disable."""
        enabled_pumps = self._get_enabled_pump_ids()
        for ch_idx, (label, combo) in enumerate(self._channel_map_widgets):
            current = combo.currentData()
            combo.blockSignals(True)
            combo.clear()
            combo.addItem("— Unassigned —", None)
            for pid in enabled_pumps:
                combo.addItem(pid, pid)
            if current:
                idx = combo.findData(current)
                if idx >= 0:
                    combo.setCurrentIndex(idx)
            combo.blockSignals(False)
        self._update_channel_map_status()

    def _on_channel_map_changed(self, ch_idx: int, _combo_idx: int = None):
        """Handle channel mapping combo change."""
        self._update_channel_map_status()
        self._on_config_changed()

    def _update_channel_map_status(self):
        """Update channel mapping validation indicator."""
        assigned_pumps = set()
        all_assigned = True
        has_duplicate = False
        for ch_idx, (label, combo) in enumerate(self._channel_map_widgets):
            pid = combo.currentData()
            if pid is None:
                all_assigned = False
            elif pid in assigned_pumps:
                has_duplicate = True
            else:
                assigned_pumps.add(pid)

        n = len(self._channel_map_widgets)
        if n == 0:
            self.channel_map_status.setText("")
        elif has_duplicate:
            self.channel_map_status.setText("⚠ Duplicate pump assignment")
            self.channel_map_status.setStyleSheet(
                f"color: {COLORS.get('red', '#f38ba8')}; font-size: 9pt;")
        elif not all_assigned:
            self.channel_map_status.setText(
                f"⚠ {n - len(assigned_pumps)} channel(s) unassigned")
            self.channel_map_status.setStyleSheet(
                f"color: {COLORS.get('yellow', '#f9e2af')}; font-size: 9pt;")
        else:
            self.channel_map_status.setText("✓ All channels assigned")
            self.channel_map_status.setStyleSheet(
                f"color: {COLORS.get('green', '#a6e3a1')}; font-size: 9pt;")

    def _get_enabled_pump_ids(self) -> list[str]:
        return [pid for pid, pw in self._pump_widgets.items()
                if pw.enable_check.isChecked()]

''',
        "B9: Add pump/channel map handler methods")

    # ── B10: Update _rebuild_config to capture channel map ────────
    content = replace_text(content,
        '''            self._config.pumps[pid] = pcfg

    # ════════════════════════════════════════════════════════════════
    #  INK LIBRARY CRUD''',
        '''            self._config.pumps[pid] = pcfg

        # v7.2.4 S3.12: Capture channel map state
        self._config.needle_channel_pump_map.clear()
        for ch_idx, (label, combo) in enumerate(self._channel_map_widgets):
            pid = combo.currentData()
            if pid:
                self._config.needle_channel_pump_map[ch_idx] = pid

    # ════════════════════════════════════════════════════════════════
    #  INK LIBRARY CRUD''',
        "B10: Capture channel map in _rebuild_config")

    # ── B11: Update _apply_config_to_ui to restore channel map ────
    content = replace_text(content,
        '''        # ── 7. Emit signals ──────────────────────────────────────
        self._on_config_changed()
        logger.info("Config restore complete")''',
        '''        # ── 7. Refresh pump exclusions + summary ─────────────────
        self._refresh_pump_ink_exclusions()
        self._update_pump_ink_summary()

        # ── 8. Needle Channel → Pump Map (v7.2.4 S3.11) ─────────
        self._rebuild_channel_map_rows()
        for ch_idx, (label, combo) in enumerate(self._channel_map_widgets):
            mapped_pump = self._config.needle_channel_pump_map.get(ch_idx)
            if mapped_pump:
                idx = combo.findData(mapped_pump)
                if idx >= 0:
                    combo.blockSignals(True)
                    combo.setCurrentIndex(idx)
                    combo.blockSignals(False)
        self._update_channel_map_status()
        logger.debug(f"  Channel map: {self._config.needle_channel_pump_map}")

        # ── 9. Emit signals ──────────────────────────────────────
        self._on_config_changed()
        logger.info("Config restore complete")''',
        "B11: Restore channel map in _apply_config_to_ui")

    # ── B12: Add get_selected_ink_name to PumpChannelWidget ───────
    content = insert_after(content,
        '''        return config

    def set_config(self, config: PumpChannelConfig, ink_names: list[str] | None = None):''',
        '''
    def get_selected_ink_name(self) -> str | None:
        """Get currently selected ink name (for exclusion tracking)."""
        return self.ink_combo.currentData()

    def set_ink_names(self, ink_names: list[str], excluded: set[str] | None = None):
        """
        Update ink combo options. v7.2.4: excluded inks shown grayed out.
        """
        current = self.ink_combo.currentData()
        self._ink_names = ink_names
        self._excluded_inks = excluded or set()
        self.ink_combo.blockSignals(True)
        self.ink_combo.clear()
        self.ink_combo.addItem("— None —", None)
        for name in ink_names:
            self.ink_combo.addItem(name, name)
            idx = self.ink_combo.count() - 1
            if name in self._excluded_inks:
                item = self.ink_combo.model().item(idx)
                if item:
                    item.setFlags(item.flags() & ~Qt.ItemIsEnabled)
        if current:
            idx = self.ink_combo.findData(current)
            if idx >= 0:
                self.ink_combo.setCurrentIndex(idx)
        self.ink_combo.blockSignals(False)
''',
        "B12: Add get_selected_ink_name + set_ink_names to PumpChannelWidget")

    # ── B13: Add _excluded_inks init to PumpChannelWidget ─────────
    content = replace_text(content,
        '        self._ink_names: list[str] = []\n        self._build_ui()',
        '        self._ink_names: list[str] = []\n        self._excluded_inks: set[str] = set()  # v7.2.4\n        self._build_ui()',
        "B13: Add _excluded_inks init to PumpChannelWidget")

    # ── B14: Use _refresh_pump_ink_exclusions instead of old combos refresh ─
    content = replace_text(content,
        '''    def _refresh_pump_ink_combos(self):
        """Refresh ink names in all pump combos."""
        ink_names = list(self._config.ink_library.keys())
        for pw in self._pump_widgets.values():
            pw.update_ink_list(ink_names)''',
        '''    def _refresh_pump_ink_combos(self):
        """Refresh ink names in all pump combos with exclusion support."""
        self._refresh_pump_ink_exclusions()''',
        "B14: Update _refresh_pump_ink_combos to use exclusions")

    write_file(filepath, content)
    print(f"  {GREEN}DONE{RESET}: hardware_setup.py patched")


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    root = find_project_root()
    if root is None:
        print(f"{RED}ERROR{RESET}: Cannot find MEBP project root.")
        print("Run from the MEBP directory (where SupportClasses/ and gui/ exist).")
        sys.exit(1)

    os.chdir(root)
    print(f"\n{'='*60}")
    print(f" MEBP v7.2.4 — Session 3 Patch")
    print(f" Pump-Ink Assignment + Needle-Channel Mapping")
    print(f"{'='*60}")
    print(f"Project root: {root}")

    patch_hardware_config(root)
    patch_hardware_setup(root)

    print(f"\n{'='*60}")
    print(f"Session 3 Patch Summary")
    print(f"{'='*60}")
    print(f"  Applied:  {applied}")
    print(f"  Skipped:  {skipped} (already applied)")
    print(f"  Failed:   {failed}")
    if failed > 0:
        print(f"{YELLOW}WARNING: {failed} patches could not be applied.{RESET}")
        print(f"Review the MISS messages above and apply manually.")
    print(f"Files modified:")
    print(f"  SupportClasses/HardwareConfig.py — pump maps + channel map + validation")
    print(f"  gui/pages/hardware_setup.py      — reorder + channel UI + exclusions")


if __name__ == "__main__":
    main()
