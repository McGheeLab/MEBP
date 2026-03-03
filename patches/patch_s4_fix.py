#!/usr/bin/env python3
"""
patch_s4_fix.py — MEBP v7.2.4 Session 4 Fix Patch

Fixes the 3 patterns that didn't match in the original patch:
  S4.3  — Layout restructure (whitespace / indentation mismatch)
  S4.4-S4.8 — Preview widget swap (actual uses create_interactive_well_preview)
  S4.2a — _show_single_preview (different signature ghost=False vs ghost=True)

Run AFTER patch_s4_preview_overhaul.py:
    python patch_s4_fix.py [project_root]
"""

import os
import sys
import re
from pathlib import Path


def find_project_root(start: str = ".") -> Path:
    p = Path(start).resolve()
    for _ in range(10):
        if (p / "main.py").exists() and (p / "gui").is_dir():
            return p
        if p.parent == p:
            break
        p = p.parent
    raise FileNotFoundError("Could not find MEBP project root.")


def main():
    if len(sys.argv) > 1:
        root = Path(sys.argv[1]).resolve()
    else:
        try:
            root = find_project_root()
        except FileNotFoundError as e:
            print(f"ERROR: {e}")
            sys.exit(1)

    print(f"MEBP v7.2.4 Session 4 Fix Patch")
    print(f"Project root: {root}")
    print(f"{'=' * 60}")

    po_path = root / "gui" / "pages" / "print_objects.py"
    if not po_path.exists():
        print(f"ERROR: {po_path} not found")
        sys.exit(1)

    text = po_path.read_text(encoding="utf-8")
    fixes_applied = 0

    # ──────────────────────────────────────────────────────────────
    # FIX 1: Layout restructure (S4.3)
    # The original patch looked for exact whitespace but the file
    # may have slight differences. Use regex to be robust.
    # ──────────────────────────────────────────────────────────────

    if "v7.2.4: Restructured layout" in text:
        print("  ⏭  SKIP FIX 1 (S4.3): Layout already restructured")
        fixes_applied += 1
    else:
        # Match the old layout pattern flexibly
        old_layout_re = (
            r'# ── Main splitter[^\n]*\n'
            r'\s+splitter = QSplitter\(Qt\.Orientation\.Horizontal\)\s*\n'
            r'\s*\n'
            r'\s+# Left panel: Designer \+ Auto-Layout\s*\n'
            r'\s+left_scroll = QScrollArea\(\)\s*\n'
            r'\s+left_scroll\.setWidgetResizable\(True\)\s*\n'
            r'\s+left_scroll\.setHorizontalScrollBarPolicy\(Qt\.ScrollBarPolicy\.ScrollBarAlwaysOff\)\s*\n'
            r'\s+left_widget = QWidget\(\)\s*\n'
            r'\s+left_layout = QVBoxLayout\(left_widget\)\s*\n'
            r'\s+left_layout\.setContentsMargins\(4, 4, 4, 4\)\s*\n'
            r'\s+left_layout\.setSpacing\(6\)\s*\n'
            r'\s*\n'
            r'\s+self\._build_designer_section\(left_layout\)\s*\n'
            r'\s+self\._build_auto_layout_section\(left_layout\)\s*\n'
            r'\s+self\._build_csv_import_section\(left_layout\)\s*\n'
            r'\s+left_layout\.addStretch\(\)\s*\n'
            r'\s*\n'
            r'\s+left_scroll\.setWidget\(left_widget\)\s*\n'
            r'\s+splitter\.addWidget\(left_scroll\)\s*\n'
            r'\s*\n'
            r'\s+# Right panel: Preview \+ Objects List \+ Summary\s*\n'
            r'\s+right = QWidget\(\)\s*\n'
            r'\s+right_layout = QVBoxLayout\(right\)\s*\n'
            r'\s+right_layout\.setContentsMargins\(4, 4, 4, 4\)\s*\n'
            r'\s+right_layout\.setSpacing\(4\)\s*\n'
            r'\s*\n'
            r'\s+self\._build_preview_section\(right_layout\)\s*\n'
            r'\s+self\._build_objects_list_section\(right_layout\)\s*\n'
            r'\s+self\._build_summary_section\(right_layout\)\s*\n'
            r'\s*\n'
            r'\s+splitter\.addWidget\(right\)\s*\n'
            r'\s+splitter\.setStretchFactor\(0, 2\)\s*\n'
            r'\s+splitter\.setStretchFactor\(1, 3\)\s*\n'
            r'\s*\n'
            r'\s+outer\.addWidget\(splitter\)'
        )

        new_layout = """# ── v7.2.4: Restructured layout (S4.3 + S4.12) ─────────
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

        outer.addWidget(v_splitter)"""

        new_text, count = re.subn(old_layout_re, new_layout, text, count=1)
        if count > 0:
            text = new_text
            print("  ✅  FIX 1 (S4.3): Layout restructured — preview LEFT, objects RIGHT, designer BELOW")
            fixes_applied += 1
        else:
            print("  ⚠  SKIP FIX 1 (S4.3): Could not match layout pattern (may need manual edit)")

    # ──────────────────────────────────────────────────────────────
    # FIX 2: Preview widget swap (S4.4-S4.8)
    # Actual code uses create_interactive_well_preview() not
    # create_well_preview()
    # ──────────────────────────────────────────────────────────────

    if "WellPreviewWidget()" in text and "_build_preview_section" in text and "v7.2.4 S4.4" in text:
        print("  ⏭  SKIP FIX 2 (S4.4-S4.8): Preview already replaced")
        fixes_applied += 1
    else:
        # Match the actual _build_preview_section
        old_preview_re = (
            r'def _build_preview_section\(self, parent_layout\):\s*\n'
            r'\s+if HAS_PROJECTION_CANVAS:\s*\n'
            r'\s+self\._preview = create_interactive_well_preview\(\)\s*\n'
            r'\s+self\._preview\.set_library_resolver\(self\._resolve_library_object\)\s*\n'
            r'(.*?)'  # Match everything until the addWidget line
            r'\s+parent_layout\.addWidget\(self\._preview, stretch=1\)'
        )

        new_preview = """def _build_preview_section(self, parent_layout):
        \"\"\"XY-only well preview with zoom/pan (v7.2.4 S4.4-S4.8).\"\"\"
        if HAS_WELL_PREVIEW:
            self._preview = WellPreviewWidget()
            self._preview.oob_detected.connect(self._on_oob_detected)
            self._update_well_diameter()
        elif HAS_PROJECTION_CANVAS:
            # Fallback to legacy ProjectionCanvas
            self._preview = create_interactive_well_preview()
            self._preview.set_library_resolver(self._resolve_library_object)
            if hasattr(self._preview, 'object_placed'):
                self._preview.object_placed.connect(self._on_object_repositioned)
            if hasattr(self._preview, 'object_moved'):
                self._preview.object_moved.connect(self._on_object_repositioned)
        else:
            self._preview = QLabel(
                "Preview unavailable\\n(well_preview.py not found)")
            self._preview.setAlignment(Qt.AlignCenter)
            self._preview.setStyleSheet(
                f"color: {COLORS['subtext0']}; background: {COLORS['mantle']}; "
                f"border-radius: 6px; min-height: 200px;")

        parent_layout.addWidget(self._preview, stretch=1)"""

        new_text, count = re.subn(old_preview_re, new_preview, text, count=1, flags=re.DOTALL)
        if count > 0:
            text = new_text
            print("  ✅  FIX 2 (S4.4-S4.8): Preview widget replaced with WellPreviewWidget")
            fixes_applied += 1
        else:
            print("  ⚠  SKIP FIX 2 (S4.4-S4.8): Could not match _build_preview_section")

    # ──────────────────────────────────────────────────────────────
    # FIX 3: _show_single_preview (S4.2a)
    # Actual signature is ghost=False and docstring differs
    # ──────────────────────────────────────────────────────────────

    if "HAS_WELL_PREVIEW and isinstance(self._preview, WellPreviewWidget)" in text and "_show_single_preview" in text:
        print("  ⏭  SKIP FIX 3 (S4.2a): _show_single_preview already updated")
        fixes_applied += 1
    else:
        old_show_re = (
            r'def _show_single_preview\(self, obj, ghost=(?:True|False)\):\s*\n'
            r'\s+"""[^"]*"""\s*\n'
            r'\s+if not HAS_PROJECTION_CANVAS or not isinstance\(self\._preview, ProjectionCanvas\):\s*\n'
            r'\s+return\s*\n'
            r'\s+if not HAS_NUMPY or not hasattr\(obj, \'trajectory\'\) or obj\.trajectory is None:\s*\n'
            r'\s+return\s*\n'
            r'\s*\n'
            r'\s+traj = obj\.trajectory\s*\n'
            r'\s+pts = \[\(float\(traj\[i, 0\]\), float\(traj\[i, 1\]\), float\(traj\[i, 2\]\)\)\s*\n'
            r'\s+for i in range\(len\(traj\)\)\]\s*\n'
            r'\s+color = getattr\(obj, \'color\', "#a6e3a1"\)\s*\n'
            r'\s+obj_path = ObjectPath\(name=obj\.name, color=color, points=pts\)\s*\n'
            r'\s+self\._preview\.set_object_paths\(\[obj_path\]\)\s*\n'
            r'\s+self\._update_well_diameter\(\)\s*\n'
            r'\s+self\._preview\.refresh\(\)'
        )

        new_show = """def _show_single_preview(self, obj, ghost=False):
        \"\"\"Show a single object trajectory in the preview.\"\"\"
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
        self._update_well_diameter()"""

        new_text, count = re.subn(old_show_re, new_show, text, count=1, flags=re.DOTALL)
        if count > 0:
            text = new_text
            print("  ✅  FIX 3 (S4.2a): _show_single_preview updated for WellPreviewWidget")
            fixes_applied += 1
        else:
            print("  ⚠  SKIP FIX 3 (S4.2a): Could not match _show_single_preview")

    # ──────────────────────────────────────────────────────────────
    # Write back
    # ──────────────────────────────────────────────────────────────

    po_path.write_text(text, encoding="utf-8")

    print(f"\n{'=' * 60}")
    print(f"Fix patch complete: {fixes_applied}/3 fixes applied")

    if fixes_applied < 3:
        print("\n⚠  Some fixes could not be applied automatically.")
        print("   This is likely due to prior patches modifying the same regions.")
        print("   You may need to make the following manual edits:")
        if "v7.2.4: Restructured layout" not in text:
            print("   - FIX 1: Restructure _build_ui() layout (see SESSION4_CHANGELOG.md)")
        if "WellPreviewWidget()" not in text:
            print("   - FIX 2: Replace _build_preview_section() (see SESSION4_CHANGELOG.md)")
        if "HAS_WELL_PREVIEW and isinstance(self._preview, WellPreviewWidget)" not in text:
            print("   - FIX 3: Update _show_single_preview() (see SESSION4_CHANGELOG.md)")


if __name__ == "__main__":
    main()
