#!/usr/bin/env python3
"""
patch_s4_preview_overhaul.py — MEBP v7.2.4 Session 4 Patch

Issues Covered: 4, 5, 6
  - Issue 4: Remove XZ/ZY projections from print_well_setup.py
  - Issue 5: Print Objects preview overhaul (XY only, zoomable, pannable)
  - Issue 6: Out-of-bounds object flashing red

Tasks:
  S4.1  Remove projections from print_well_setup.py
  S4.2  Remove projections from print_objects.py
  S4.3  Restructure print_objects layout: preview LEFT, objects RIGHT
  S4.4  Add zoom (mouse wheel) via WellPreviewWidget
  S4.5  Add pan (middle-click / Ctrl+drag) via WellPreviewWidget
  S4.6  Add zoom-to-fit + zoom percentage via WellPreviewWidget
  S4.7  Fix axis alignment (X=right, Y=up) via WellPreviewWidget
  S4.8  Draw well boundary circle via WellPreviewWidget
  S4.9  Implement _check_bounds()
  S4.10 Add _oob_flash_timer for red flashing
  S4.11 OOB trajectory segments in red/dashed via WellPreviewWidget
  S4.12 Object designer → collapsible panel below preview
  S4.13 Test coverage (see test_s4_preview_overhaul.py)

Files Modified:
  gui/pages/print_well_setup.py  — remove projections, simplify layout
  gui/pages/print_objects.py     — full layout restructure + OOB features

Files Created:
  gui/widgets/well_preview.py    — new XY-only preview widget (deployed separately)

Usage:
    python patch_s4_preview_overhaul.py [project_root]
    
    If project_root not given, searches parent directories for main.py.
"""

import os
import sys
import re
from pathlib import Path


# ═══════════════════════════════════════════════════════════════════
# Utilities
# ═══════════════════════════════════════════════════════════════════

def find_project_root(start: str = ".") -> Path:
    """Walk upward to find MEBP project root (contains main.py)."""
    p = Path(start).resolve()
    for _ in range(10):
        if (p / "main.py").exists() and (p / "gui").is_dir():
            return p
        if p.parent == p:
            break
        p = p.parent
    raise FileNotFoundError(
        "Could not find MEBP project root. Pass it as an argument.")


def safe_replace(filepath: Path, old: str, new: str, label: str) -> bool:
    """Replace exactly one occurrence of `old` with `new` in file."""
    if not filepath.exists():
        print(f"  ⚠  SKIP {label}: file not found: {filepath}")
        return False
    text = filepath.read_text(encoding="utf-8")
    count = text.count(old)
    if count == 0:
        # Check if already applied
        if new in text:
            print(f"  ⏭  SKIP {label}: already applied")
            return True
        print(f"  ⚠  SKIP {label}: pattern not found in {filepath.name}")
        return False
    if count > 1:
        print(f"  ⚠  WARN {label}: pattern found {count}× (replacing first)")
    text = text.replace(old, new, 1)
    filepath.write_text(text, encoding="utf-8")
    print(f"  ✅  {label}")
    return True


def safe_insert_after(filepath: Path, anchor: str, insertion: str, label: str) -> bool:
    """Insert text after first occurrence of anchor."""
    if not filepath.exists():
        print(f"  ⚠  SKIP {label}: file not found")
        return False
    text = filepath.read_text(encoding="utf-8")
    if insertion.strip() in text:
        print(f"  ⏭  SKIP {label}: already applied")
        return True
    idx = text.find(anchor)
    if idx == -1:
        print(f"  ⚠  SKIP {label}: anchor not found")
        return False
    insert_pos = idx + len(anchor)
    text = text[:insert_pos] + insertion + text[insert_pos:]
    filepath.write_text(text, encoding="utf-8")
    print(f"  ✅  {label}")
    return True


def safe_delete_block(filepath: Path, start_marker: str, end_marker: str, label: str) -> bool:
    """Delete a block of text between start_marker and end_marker (inclusive)."""
    if not filepath.exists():
        print(f"  ⚠  SKIP {label}: file not found")
        return False
    text = filepath.read_text(encoding="utf-8")
    start_idx = text.find(start_marker)
    if start_idx == -1:
        print(f"  ⏭  SKIP {label}: start marker not found (may be already applied)")
        return True
    end_idx = text.find(end_marker, start_idx)
    if end_idx == -1:
        print(f"  ⚠  SKIP {label}: end marker not found")
        return False
    end_idx += len(end_marker)
    text = text[:start_idx] + text[end_idx:]
    filepath.write_text(text, encoding="utf-8")
    print(f"  ✅  {label}")
    return True


def regex_replace(filepath: Path, pattern: str, replacement: str, label: str) -> bool:
    """Regex-based replacement (for multi-line patterns)."""
    if not filepath.exists():
        print(f"  ⚠  SKIP {label}: file not found")
        return False
    text = filepath.read_text(encoding="utf-8")
    new_text, count = re.subn(pattern, replacement, text, count=1, flags=re.DOTALL)
    if count == 0:
        if replacement.strip()[:60] in text:
            print(f"  ⏭  SKIP {label}: already applied")
            return True
        print(f"  ⚠  SKIP {label}: regex pattern not matched")
        return False
    filepath.write_text(new_text, encoding="utf-8")
    print(f"  ✅  {label}")
    return True


# ═══════════════════════════════════════════════════════════════════
# Patch 1: print_well_setup.py — Remove projections (S4.1)
# ═══════════════════════════════════════════════════════════════════

def patch_well_setup_remove_projections(root: Path) -> int:
    """Remove ZY and XZ projection views from print_well_setup.py."""
    filepath = root / "gui" / "pages" / "print_well_setup.py"
    ok = 0

    print("\n── Patch 1: print_well_setup.py — Remove projections ──")

    # 1a. Remove MiniProjectionView import
    ok += safe_replace(
        filepath,
        "from gui.widgets.projection_canvas import MiniProjectionView",
        "# MiniProjectionView removed in v7.2.4 (XY-only layout)",
        "S4.1a: Remove MiniProjectionView import",
    )

    # 1b. Replace _build_ui top section: remove ZY splitter, XZ view
    #     Replace the entire top_splitter + projections block with simple plate view
    ok += safe_replace(
        filepath,
        # Old: top_splitter with ZY side projection
        """        # ── Top area: Plate view + ZY projection ──────────────────
        top_splitter = QSplitter(Qt.Orientation.Horizontal)

        # Plate view
        plate_container = QWidget()
        plate_layout = QVBoxLayout(plate_container)
        plate_layout.setContentsMargins(0, 0, 0, 0)
        plate_layout.setSpacing(2)

        self.plate_view = WellPlateView()
        plate_layout.addWidget(self.plate_view, stretch=1)

        # Legend + selection count
        info_row = QHBoxLayout()
        self.legend = WellRoleLegend()
        info_row.addWidget(self.legend)
        self.selection_label = QLabel("[0 wells selected]")
        self.selection_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 11px;")
        info_row.addWidget(self.selection_label)
        plate_layout.addLayout(info_row)

        top_splitter.addWidget(plate_container)

        # ZY side projection (uses unified projection_canvas widget)
        self.zy_view = MiniProjectionView("ZY")
        top_splitter.addWidget(self.zy_view)
        top_splitter.setStretchFactor(0, 5)
        top_splitter.setStretchFactor(1, 1)

        main_layout.addWidget(top_splitter, stretch=3)

        # ── XZ bottom projection ──────────────────────────────────
        self.xz_view = MiniProjectionView("XZ")
        main_layout.addWidget(self.xz_view)""",
        # New: plate view fills full width, no projections
        """        # ── Interactive Plate View (XY) — full width ─────────────
        # v7.2.4: Removed ZY and XZ projections (Issue #4)
        plate_container = QWidget()
        plate_layout = QVBoxLayout(plate_container)
        plate_layout.setContentsMargins(0, 0, 0, 0)
        plate_layout.setSpacing(2)

        self.plate_view = WellPlateView()
        plate_layout.addWidget(self.plate_view, stretch=1)

        # Legend + selection count
        info_row = QHBoxLayout()
        self.legend = WellRoleLegend()
        info_row.addWidget(self.legend)
        self.selection_label = QLabel("[0 wells selected]")
        self.selection_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 11px;")
        info_row.addWidget(self.selection_label)
        plate_layout.addLayout(info_row)

        main_layout.addWidget(plate_container, stretch=3)""",
        "S4.1b: Replace top area — plate view full width, remove projections",
    )

    # 1c. Replace _refresh_projections to be a no-op
    ok += safe_replace(
        filepath,
        """    def _refresh_projections(self) -> None:
        \"\"\"Update ZY and XZ projection views.\"\"\"
        z_offsets = {}
        for name, wa in self._model.assignments.items():
            z_offsets[name] = wa.get_effective_z()
        self.zy_view.set_plate_data(self._model.plate, z_offsets)
        self.xz_view.set_plate_data(self._model.plate, z_offsets)""",
        """    def _refresh_projections(self) -> None:
        \"\"\"No-op: ZY/XZ projections removed in v7.2.4 (Issue #4).\"\"\"
        pass""",
        "S4.1c: Replace _refresh_projections with no-op",
    )

    return ok


# ═══════════════════════════════════════════════════════════════════
# Patch 2: print_objects.py — Import + preview swap (S4.2)
# ═══════════════════════════════════════════════════════════════════

def patch_objects_preview_import(root: Path) -> int:
    """Replace ProjectionCanvas with WellPreviewWidget import."""
    filepath = root / "gui" / "pages" / "print_objects.py"
    ok = 0

    print("\n── Patch 2: print_objects.py — Preview import swap ──")

    # 2a. Add WellPreviewWidget import (alongside existing imports)
    ok += safe_insert_after(
        filepath,
        "from gui.styles import COLORS",
        """

# v7.2.4: XY-only well preview with zoom/pan/bounds (S4.2)
try:
    from gui.widgets.well_preview import WellPreviewWidget, ObjectPath as WPObjectPath
    HAS_WELL_PREVIEW = True
except ImportError:
    HAS_WELL_PREVIEW = False
""",
        "S4.2a: Add WellPreviewWidget import",
    )

    return ok


# ═══════════════════════════════════════════════════════════════════
# Patch 3: print_objects.py — Layout restructure (S4.3, S4.12)
# ═══════════════════════════════════════════════════════════════════

def patch_objects_layout(root: Path) -> int:
    """Restructure layout: preview LEFT, objects RIGHT, designer BELOW collapsible."""
    filepath = root / "gui" / "pages" / "print_objects.py"
    ok = 0

    print("\n── Patch 3: print_objects.py — Layout restructure ──")

    # 3a. Replace the main splitter structure in _build_ui
    ok += safe_replace(
        filepath,
        """        # ── Main splitter ─────────────────────────────────────
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left panel: Designer + Auto-Layout
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(4, 4, 4, 4)
        left_layout.setSpacing(6)

        self._build_designer_section(left_layout)
        self._build_auto_layout_section(left_layout)
        self._build_csv_import_section(left_layout)
        left_layout.addStretch()

        left_scroll.setWidget(left_widget)
        splitter.addWidget(left_scroll)

        # Right panel: Preview + Objects List + Summary
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(4, 4, 4, 4)
        right_layout.setSpacing(4)

        self._build_preview_section(right_layout)
        self._build_objects_list_section(right_layout)
        self._build_summary_section(right_layout)

        splitter.addWidget(right)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)

        outer.addWidget(splitter)""",
        """        # ── v7.2.4: Restructured layout (S4.3 + S4.12) ─────────
        # Vertical splitter: top (preview + objects) | bottom (designer)
        v_splitter = QSplitter(Qt.Orientation.Vertical)

        # ── Top: Preview (LEFT) + Objects List (RIGHT) ────────────
        top_splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left: Well Preview (XY only, zoomable)
        preview_container = QWidget()
        preview_layout = QVBoxLayout(preview_container)
        preview_layout.setContentsMargins(4, 4, 4, 4)
        preview_layout.setSpacing(4)
        self._build_preview_section(preview_layout)
        top_splitter.addWidget(preview_container)

        # Right: Objects List + Summary
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(4, 4, 4, 4)
        right_layout.setSpacing(4)
        self._build_objects_list_section(right_layout)
        self._build_summary_section(right_layout)
        top_splitter.addWidget(right)

        top_splitter.setStretchFactor(0, 3)  # Preview gets more space
        top_splitter.setStretchFactor(1, 2)

        v_splitter.addWidget(top_splitter)

        # ── Bottom: Collapsible Designer + Auto-Layout ────────────
        designer_container = QWidget()
        designer_scroll = QScrollArea()
        designer_scroll.setWidgetResizable(True)
        designer_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        designer_widget = QWidget()
        designer_layout = QVBoxLayout(designer_widget)
        designer_layout.setContentsMargins(4, 4, 4, 4)
        designer_layout.setSpacing(6)

        self._build_designer_section(designer_layout)
        self._build_auto_layout_section(designer_layout)
        self._build_csv_import_section(designer_layout)
        designer_layout.addStretch()

        designer_scroll.setWidget(designer_widget)
        designer_outer = QVBoxLayout(designer_container)
        designer_outer.setContentsMargins(0, 0, 0, 0)
        designer_outer.addWidget(designer_scroll)

        v_splitter.addWidget(designer_container)
        v_splitter.setStretchFactor(0, 3)  # Preview area dominant
        v_splitter.setStretchFactor(1, 2)  # Designer collapsible

        outer.addWidget(v_splitter)""",
        "S4.3: Restructure layout — preview LEFT, objects RIGHT, designer BELOW",
    )

    return ok


# ═══════════════════════════════════════════════════════════════════
# Patch 4: print_objects.py — Replace preview widget (S4.4-S4.8)
# ═══════════════════════════════════════════════════════════════════

def patch_objects_preview_widget(root: Path) -> int:
    """Replace ProjectionCanvas preview with WellPreviewWidget."""
    filepath = root / "gui" / "pages" / "print_objects.py"
    ok = 0

    print("\n── Patch 4: print_objects.py — Preview widget swap ──")

    # 4a. Replace _build_preview_section to use WellPreviewWidget
    ok += safe_replace(
        filepath,
        """    def _build_preview_section(self, parent_layout):
        \"\"\"Well preview canvas (uses ProjectionCanvas L-shaped views).\"\"\"
        try:
            from gui.widgets.projection_canvas import (
                ProjectionCanvas, create_well_preview, ObjectPath,
            )
            self._preview = create_well_preview()
            if hasattr(self._preview, 'object_placed'):
                self._preview.object_placed.connect(self._on_object_repositioned)
            if hasattr(self._preview, 'object_moved'):
                self._preview.object_moved.connect(self._on_object_repositioned)
        else:
            self._preview = QLabel(
                "Preview unavailable\\n(projection_canvas.py not found)")
            self._preview.setAlignment(Qt.AlignCenter)
            self._preview.setStyleSheet(
                f"color: {COLORS['subtext0']}; background: {COLORS['mantle']}; "
                f"border-radius: 6px; min-height: 200px;")

        parent_layout.addWidget(self._preview, stretch=1)""",
        """    def _build_preview_section(self, parent_layout):
        \"\"\"XY-only well preview with zoom/pan (v7.2.4 S4.4-S4.8).\"\"\"
        if HAS_WELL_PREVIEW:
            self._preview = WellPreviewWidget()
            self._preview.oob_detected.connect(self._on_oob_detected)
            self._update_well_diameter()
        else:
            # Fallback: try legacy ProjectionCanvas
            try:
                from gui.widgets.projection_canvas import (
                    ProjectionCanvas, create_well_preview, ObjectPath,
                )
                self._preview = create_well_preview()
            except ImportError:
                self._preview = QLabel(
                    "Preview unavailable\\n(well_preview.py not found)")
                self._preview.setAlignment(Qt.AlignCenter)
                self._preview.setStyleSheet(
                    f"color: {COLORS['subtext0']}; background: {COLORS['mantle']}; "
                    f"border-radius: 6px; min-height: 200px;")

        parent_layout.addWidget(self._preview, stretch=1)""",
        "S4.4-S4.8: Replace preview with WellPreviewWidget",
    )

    return ok


# ═══════════════════════════════════════════════════════════════════
# Patch 5: print_objects.py — OOB flash timer + bounds check (S4.9-S4.11)
# ═══════════════════════════════════════════════════════════════════

def patch_objects_oob_system(root: Path) -> int:
    """Add out-of-bounds flash timer and bounds checking."""
    filepath = root / "gui" / "pages" / "print_objects.py"
    ok = 0

    print("\n── Patch 5: print_objects.py — OOB flash + bounds ──")

    # 5a. Add OOB attributes to __init__ (after _build_ui() call)
    ok += safe_insert_after(
        filepath,
        "        self._build_ui()",
        """

        # v7.2.4: Out-of-bounds detection (S4.9-S4.10)
        self._oob_indices: set[int] = set()
        self._oob_flash_state: bool = False
        self._oob_flash_timer = QTimer(self)
        self._oob_flash_timer.setInterval(500)
        self._oob_flash_timer.timeout.connect(self._toggle_oob_flash)
""",
        "S4.9: Add OOB attributes + flash timer",
    )

    # 5b. Add OOB methods at end of class (before final helper functions)
    # Find a good insertion point - after _refresh_preview_all or _update_well_diameter
    ok += safe_insert_after(
        filepath,
        """    def _update_well_diameter(self):
        \"\"\"Set well boundary circle on preview from workspace plate format.\"\"\"""",
        """

    # ── Out-of-Bounds Detection (v7.2.4 S4.9-S4.11) ─────────────

    def _check_bounds(self, entry: dict) -> bool:
        \"\"\"Check if a print object is within the well boundary.

        Returns True if in-bounds, False if out-of-bounds.
        \"\"\"
        well_diam = self._get_well_diameter_mm()
        well_radius = well_diam / 2.0
        import math

        # Get trajectory points
        obj = self._build_print_object(
            name=entry.get("name", "check"),
            obj_type=entry.get("object_type", "point"),
            params=entry.get("params", {}),
            position=entry.get("position", (0, 0, 0)),
            color=entry.get("color", "#ffffff"),
            pump_id=entry.get("ink_pump", "P1"),
        )
        if obj and hasattr(obj, 'trajectory') and obj.trajectory is not None:
            traj = obj.trajectory
            for i in range(len(traj)):
                px, py = float(traj[i, 0]), float(traj[i, 1])
                if math.sqrt(px * px + py * py) > well_radius:
                    return False
        return True

    def _refresh_oob_state(self):
        \"\"\"Recheck all objects for OOB and start/stop flash timer.\"\"\"
        old_oob = set(self._oob_indices)
        new_oob = set()
        for i, entry in enumerate(self._objects):
            if not self._check_bounds(entry):
                new_oob.add(i)
        self._oob_indices = new_oob

        if new_oob:
            if not self._oob_flash_timer.isActive():
                self._oob_flash_timer.start()
        else:
            self._oob_flash_timer.stop()
            self._oob_flash_state = False

        # Update list colors if changed
        if old_oob != new_oob:
            self._refresh_objects_list_colors()

    def _on_oob_detected(self, indices: list):
        \"\"\"Handle OOB signal from WellPreviewWidget.\"\"\"
        self._oob_indices = set(indices)
        if indices:
            if not self._oob_flash_timer.isActive():
                self._oob_flash_timer.start()
        self._refresh_objects_list_colors()

    def _toggle_oob_flash(self):
        \"\"\"Toggle flash state for OOB items (called by QTimer every 500ms).\"\"\"
        self._oob_flash_state = not self._oob_flash_state
        self._refresh_objects_list_colors()

    def _refresh_objects_list_colors(self):
        \"\"\"Update list item colors — flash red for OOB items.\"\"\"
        for i in range(self._objects_list.count()):
            item = self._objects_list.item(i)
            if item is None:
                continue
            if i in self._oob_indices:
                if self._oob_flash_state:
                    item.setBackground(QColor("#f38ba8"))  # Red flash
                    item.setForeground(QColor("#1e1e2e"))  # Dark text
                else:
                    item.setBackground(QColor("#45475a"))  # Surface2
                    if i < len(self._objects):
                        color = QColor(self._objects[i].get("color", "#cdd6f4"))
                        item.setForeground(color)
                # Prepend warning icon to text if not already there
                text = item.text()
                if not text.startswith("⚠"):
                    item.setText(f"⚠ {text}")
            else:
                item.setBackground(QColor("transparent"))
                if i < len(self._objects):
                    color = QColor(self._objects[i].get("color", "#cdd6f4"))
                    item.setForeground(color)
                # Remove warning icon if present
                text = item.text()
                if text.startswith("⚠ "):
                    item.setText(text[2:])

    def _get_well_diameter_mm(self) -> float:
        \"\"\"Get well diameter from hardware config or workspace.\"\"\"
        if hasattr(self, '_hw_config') and self._hw_config:
            try:
                fmt = self._hw_config.plate_format
                from SupportClasses.WellPlate import PLATE_DEFINITIONS
                plate_def = PLATE_DEFINITIONS.get(fmt, {})
                return plate_def.get("well_diameter", 6.0)
            except (ImportError, AttributeError):
                pass
        if hasattr(self, '_workspace') and self._workspace:
            try:
                fmt = self._workspace.plate_format
                from SupportClasses.WellPlate import PLATE_DEFINITIONS
                plate_def = PLATE_DEFINITIONS.get(fmt, {})
                return plate_def.get("well_diameter", 6.0)
            except (ImportError, AttributeError):
                pass
        return 6.0  # Default 96-well plate
""",
        "S4.9-S4.11: Add OOB detection, flash timer, and list color methods",
    )

    return ok


# ═══════════════════════════════════════════════════════════════════
# Patch 6: print_objects.py — Wire preview refresh methods (S4.2)
# ═══════════════════════════════════════════════════════════════════

def patch_objects_preview_refresh(root: Path) -> int:
    """Update preview refresh methods to work with WellPreviewWidget."""
    filepath = root / "gui" / "pages" / "print_objects.py"
    ok = 0

    print("\n── Patch 6: print_objects.py — Preview refresh wiring ──")

    # 6a. Update _show_single_preview to use WellPreviewWidget API
    ok += safe_replace(
        filepath,
        """    def _show_single_preview(self, obj, ghost=True):
        \"\"\"Show a single object in the preview canvas.\"\"\"
        if not HAS_PROJECTION_CANVAS or not isinstance(self._preview, ProjectionCanvas):
            return
        if not HAS_NUMPY or not hasattr(obj, 'trajectory') or obj.trajectory is None:
            return

        traj = obj.trajectory
        pts = [(float(traj[i, 0]), float(traj[i, 1]), float(traj[i, 2]))
               for i in range(len(traj))]
        color = getattr(obj, 'color', "#a6e3a1")
        obj_path = ObjectPath(name=obj.name, color=color, points=pts)
        self._preview.set_object_paths([obj_path])
        self._update_well_diameter()
        self._preview.refresh()""",
        """    def _show_single_preview(self, obj, ghost=True):
        \"\"\"Show a single object in the preview canvas.\"\"\"
        if not hasattr(obj, 'trajectory') or obj.trajectory is None:
            return

        traj = obj.trajectory
        pts = [(float(traj[i, 0]), float(traj[i, 1]), float(traj[i, 2]))
               for i in range(len(traj))]
        color = getattr(obj, 'color', "#a6e3a1")

        # v7.2.4: Use WellPreviewWidget or legacy ProjectionCanvas
        if HAS_WELL_PREVIEW and isinstance(self._preview, WellPreviewWidget):
            obj_path = WPObjectPath(name=obj.name, color=color, points=pts)
            self._preview.set_object_paths([obj_path])
        elif HAS_PROJECTION_CANVAS and isinstance(self._preview, ProjectionCanvas):
            obj_path = ObjectPath(name=obj.name, color=color, points=pts)
            self._preview.set_object_paths([obj_path])
            self._preview.refresh()
        self._update_well_diameter()""",
        "S4.2a: Update _show_single_preview for WellPreviewWidget",
    )

    # 6b. Update _refresh_preview_all
    ok += safe_replace(
        filepath,
        """    def _refresh_preview_all(self, highlight_index=None):
        \"\"\"Regenerate preview showing all objects in current print.\"\"\"
        if not HAS_PROJECTION_CANVAS or not isinstance(self._preview, ProjectionCanvas):
            return

        all_paths = []
        for i, entry in enumerate(self._objects):
            obj = self._build_print_object(
                name=entry["name"],
                obj_type=entry["object_type"],
                params=entry.get("params", {}),
                position=entry.get("position", (0, 0, 0)),
                color=entry.get("color", DEFAULT_COLORS[0]),
                pump_id=entry.get("ink_pump", "P1"),
            )
            if obj and hasattr(obj, 'trajectory') and obj.trajectory is not None:
                traj = obj.trajectory
                pts = [(float(traj[j, 0]), float(traj[j, 1]), float(traj[j, 2]))
                       for j in range(len(traj))]
                color = entry.get("color", DEFAULT_COLORS[0])
                if highlight_index is not None and i == highlight_index:
                    color = "#ffffff"  # Highlight
                all_paths.append(ObjectPath(name=entry["name"], color=color, points=pts))

        if all_paths:
            self._preview.set_object_paths(all_paths)
        else:
            if hasattr(self._preview, 'clear_object_paths'):
                self._preview.clear_object_paths()
            elif hasattr(self._preview, 'clear_path'):
                self._preview.clear_path()

        self._update_well_diameter()
        self._preview.refresh()""",
        """    def _refresh_preview_all(self, highlight_index=None):
        \"\"\"Regenerate preview showing all objects in current print.\"\"\"

        all_paths = []
        for i, entry in enumerate(self._objects):
            obj = self._build_print_object(
                name=entry["name"],
                obj_type=entry["object_type"],
                params=entry.get("params", {}),
                position=entry.get("position", (0, 0, 0)),
                color=entry.get("color", DEFAULT_COLORS[0]),
                pump_id=entry.get("ink_pump", "P1"),
            )
            if obj and hasattr(obj, 'trajectory') and obj.trajectory is not None:
                traj = obj.trajectory
                pts = [(float(traj[j, 0]), float(traj[j, 1]), float(traj[j, 2]))
                       for j in range(len(traj))]
                color = entry.get("color", DEFAULT_COLORS[0])
                if highlight_index is not None and i == highlight_index:
                    color = "#ffffff"

                # v7.2.4: Use WPObjectPath for WellPreviewWidget
                if HAS_WELL_PREVIEW and isinstance(self._preview, WellPreviewWidget):
                    all_paths.append(WPObjectPath(
                        name=entry["name"], color=color, points=pts))
                else:
                    all_paths.append(ObjectPath(
                        name=entry["name"], color=color, points=pts))

        if all_paths:
            self._preview.set_object_paths(all_paths)
            if HAS_WELL_PREVIEW and isinstance(self._preview, WellPreviewWidget):
                self._preview.set_highlight(highlight_index)
        else:
            if hasattr(self._preview, 'clear_object_paths'):
                self._preview.clear_object_paths()
            elif hasattr(self._preview, 'clear_path'):
                self._preview.clear_path()

        self._update_well_diameter()
        if hasattr(self._preview, 'refresh'):
            self._preview.refresh()

        # v7.2.4: Refresh OOB state after preview update
        self._refresh_oob_state()""",
        "S4.2b: Update _refresh_preview_all for WellPreviewWidget + OOB",
    )

    # 6c. Update _update_well_diameter to work with new widget
    ok += safe_replace(
        filepath,
        """    def _update_well_diameter(self):
        \"\"\"Set well boundary circle on preview from workspace plate format.\"\"\"""",
        """    def _update_well_diameter(self):
        \"\"\"Set well boundary circle on preview from workspace/HW config.\"\"\"
        diam = self._get_well_diameter_mm()
        if HAS_WELL_PREVIEW and isinstance(self._preview, WellPreviewWidget):
            self._preview.set_well_diameter(diam)
            return""",
        "S4.8: Update _update_well_diameter for WellPreviewWidget",
    )

    return ok


# ═══════════════════════════════════════════════════════════════════
# Patch 7: print_objects.py — Wire OOB into refresh_objects_list
# ═══════════════════════════════════════════════════════════════════

def patch_objects_list_oob_wire(root: Path) -> int:
    """Wire OOB flash into the objects list refresh cycle."""
    filepath = root / "gui" / "pages" / "print_objects.py"
    ok = 0

    print("\n── Patch 7: print_objects.py — Wire OOB into list ──")

    # 7a. Add OOB color refresh call at end of _refresh_objects_list
    # Find the existing _refresh_objects_list method and add OOB color call
    ok += safe_insert_after(
        filepath,
        """            self._objects_list.addItem(item)""",
        """

        # v7.2.4: Apply OOB flash colors (S4.10)
        if hasattr(self, '_oob_indices'):
            self._refresh_objects_list_colors()""",
        "S4.10: Wire OOB flash into _refresh_objects_list",
    )

    return ok


# ═══════════════════════════════════════════════════════════════════
# Patch 8: print_objects.py — Add QColor import if missing
# ═══════════════════════════════════════════════════════════════════

def patch_objects_qcolor_import(root: Path) -> int:
    """Ensure QColor is imported for OOB coloring."""
    filepath = root / "gui" / "pages" / "print_objects.py"
    ok = 0

    print("\n── Patch 8: print_objects.py — Ensure QColor import ──")

    text = filepath.read_text(encoding="utf-8")
    if "from PySide6.QtGui import" in text and "QColor" not in text.split("from PySide6.QtGui import")[1].split("\n")[0]:
        # QColor not in the QtGui import line — add it
        ok += safe_replace(
            filepath,
            "from PySide6.QtGui import QColor, QFont, QIcon",
            "from PySide6.QtGui import QColor, QFont, QIcon",
            "S4.10b: QColor already imported",
        )
    else:
        print(f"  ⏭  SKIP S4.10b: QColor already imported")
        ok += 1

    return ok


# ═══════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════

def main():
    if len(sys.argv) > 1:
        root = Path(sys.argv[1]).resolve()
    else:
        try:
            root = find_project_root()
        except FileNotFoundError as e:
            print(f"ERROR: {e}")
            sys.exit(1)

    print(f"MEBP v7.2.4 Session 4 Patch — Preview Overhaul + Bounds Check")
    print(f"Project root: {root}")
    print(f"{'=' * 60}")

    # Verify prerequisite files exist
    required = [
        root / "gui" / "pages" / "print_well_setup.py",
        root / "gui" / "pages" / "print_objects.py",
    ]
    missing = [f for f in required if not f.exists()]
    if missing:
        print(f"\nERROR: Missing prerequisite files:")
        for f in missing:
            print(f"  - {f}")
        sys.exit(1)

    # Verify new widget exists
    new_widget = root / "gui" / "widgets" / "well_preview.py"
    if not new_widget.exists():
        print(f"\n⚠  WARNING: {new_widget} not found.")
        print(f"   Copy well_preview.py to gui/widgets/ before running this patch.")
        print(f"   The patch will still apply but preview will use legacy fallback.")

    total = 0
    total += patch_well_setup_remove_projections(root)
    total += patch_objects_preview_import(root)
    total += patch_objects_layout(root)
    total += patch_objects_preview_widget(root)
    total += patch_objects_oob_system(root)
    total += patch_objects_preview_refresh(root)
    total += patch_objects_list_oob_wire(root)
    total += patch_objects_qcolor_import(root)

    print(f"\n{'=' * 60}")
    print(f"Session 4 patch complete: {total} changes applied")
    print(f"\nNew file to deploy:")
    print(f"  gui/widgets/well_preview.py")
    print(f"\nFiles modified:")
    print(f"  gui/pages/print_well_setup.py")
    print(f"  gui/pages/print_objects.py")


if __name__ == "__main__":
    main()
