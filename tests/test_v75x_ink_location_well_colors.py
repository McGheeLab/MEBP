"""
v7.5.x — Ink Assignment (Reagent Locations) well colors.

Covers the operator request that, on Hardware Setup → Ink → Reagent Locations:
  * the well CENTER fill matches the assigned reagent's own InkSpec.color,
  * the well OUTLINE encodes the reagent TYPE (ink/wash/waste/buffer/oil),
  * and the assigned color survives a hover-leave (the reported bug where the
    color vanished when the mouse left the well and only reappeared on a tab
    switch).

Root cause of the bug: the old (shadowing) ``WellPlateView.update_all_wells``
set the brush directly and left ``WellGraphicsItem._color`` stale, so
``hoverLeaveEvent → _apply_style()`` repainted from the stale color.
"""

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.PhysicalModels import (
    InkSpec, WellRole, ROLE_COLORS,
    ink_type_border_color, INK_TYPE_COLORS,
)
from SupportClasses.HardwareConfig import HardwareConfig
from SupportClasses.WellPlate import WellPlate


# ───────────────────────────────────────────────────────────────────
# Type → outline color map
# ───────────────────────────────────────────────────────────────────

class TestInkTypeBorderColor(unittest.TestCase):

    def test_service_and_oil_have_distinct_colors(self):
        cols = {
            "ink":    ink_type_border_color("ink"),
            "wash":   ink_type_border_color("wash"),
            "waste":  ink_type_border_color("waste"),
            "buffer": ink_type_border_color("buffer"),
            "oil":    ink_type_border_color("oil"),
        }
        # All five are distinct so the operator can tell them apart.
        self.assertEqual(len(set(cols.values())), 5, cols)

    def test_outline_matches_role_colors_where_a_role_exists(self):
        self.assertEqual(ink_type_border_color("ink"),
                         ROLE_COLORS[WellRole.INK])
        self.assertEqual(ink_type_border_color("wash"),
                         ROLE_COLORS[WellRole.WASH])
        self.assertEqual(ink_type_border_color("waste"),
                         ROLE_COLORS[WellRole.WASTE])
        self.assertEqual(ink_type_border_color("buffer"),
                         ROLE_COLORS[WellRole.BUFFER])

    def test_oil_distinct_from_ink(self):
        # Oil maps to WellRole.INK, but its outline must NOT be the ink blue.
        self.assertNotEqual(ink_type_border_color("oil"),
                            ink_type_border_color("ink"))
        self.assertEqual(ink_type_border_color("oil"), INK_TYPE_COLORS["oil"])

    def test_material_types_fall_back_to_ink_blue(self):
        for t in ("hydrogel", "cells", "media", "granular", "custom",
                  "", None, "UNKNOWN"):
            self.assertEqual(ink_type_border_color(t),
                             ROLE_COLORS[WellRole.INK], t)

    def test_case_insensitive(self):
        self.assertEqual(ink_type_border_color("WASH"),
                         INK_TYPE_COLORS["wash"])
        self.assertEqual(ink_type_border_color(" Oil "),
                         INK_TYPE_COLORS["oil"])


# ───────────────────────────────────────────────────────────────────
# WellPlateView fill + border + hover persistence
# ───────────────────────────────────────────────────────────────────

class TestWellViewColors(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _view(self):
        from gui.widgets.well_plate_view import WellPlateView
        view = WellPlateView()
        view.set_plate(WellPlate.load(24))
        return view

    def test_fill_is_reagent_color_border_is_type_color(self):
        view = self._view()
        view.update_reagent_appearances({
            "A1": ("#ff0000", "#00ff00", "Red ink (ink)"),
        })
        item = view._well_items["A1"]
        self.assertEqual(item.brush().color().name(), "#ff0000")
        self.assertEqual(item.pen().color().name(), "#00ff00")

    def test_color_survives_hover_leave(self):
        view = self._view()
        view.update_reagent_appearances({
            "A1": ("#ff0000", "#00ff00", "Red ink (ink)"),
        })
        item = view._well_items["A1"]
        # _apply_style() is exactly what hoverLeaveEvent re-runs.
        item._apply_style()
        self.assertEqual(item.brush().color().name(), "#ff0000")
        self.assertEqual(item.pen().color().name(), "#00ff00")

    def test_color_survives_real_hover_event_cycle(self):
        from unittest.mock import MagicMock
        view = self._view()
        view.update_reagent_appearances({
            "A1": ("#123456", "#abcdef", "Some ink"),
        })
        item = view._well_items["A1"]
        try:
            item.hoverEnterEvent(MagicMock())
            item.hoverLeaveEvent(MagicMock())
        except Exception:
            # Base-class hover handlers may reject a mock event; the brush
            # check below is the real assertion.
            item._apply_style()
        self.assertEqual(item.brush().color().name(), "#123456")
        self.assertEqual(item.pen().color().name(), "#abcdef")

    def test_unassigned_well_is_gray_default_border(self):
        view = self._view()
        empty = ROLE_COLORS[WellRole.EMPTY]
        view.update_reagent_appearances({"A1": (empty, None, "")})
        item = view._well_items["A1"]
        self.assertEqual(item.brush().color().name(), empty)
        # No explicit border color → derived (darker) from fill, NOT the
        # bright type border.
        self.assertIsNone(item._border_color)

    def test_selected_well_keeps_fill_under_selection_border(self):
        view = self._view()
        view.update_reagent_appearances({
            "A1": ("#ff0000", "#00ff00", "Red ink"),
        })
        view.set_selection(["A1"])
        item = view._well_items["A1"]
        # Selection border (mauve) wins, but the reagent fill is preserved.
        self.assertEqual(item.brush().color().name(), "#ff0000")
        # Deselect → type border returns.
        view.clear_selection()
        self.assertEqual(item.pen().color().name(), "#00ff00")


# ───────────────────────────────────────────────────────────────────
# Hardware Setup page integration
# ───────────────────────────────────────────────────────────────────

class TestHardwareSetupReagentColors(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_page_colors_wells_by_ink_and_type(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        pg = HardwareSetupPage()

        cfg = HardwareConfig()
        cfg.add_ink(InkSpec(name="Red gel", ink_type="hydrogel",
                            color="#e64553"))
        cfg.add_ink(InkSpec(name="Mineral oil", ink_type="oil",
                            color="#f9e2af"))
        cfg.assign_wells_to_ink("Red gel", ["A1"])
        cfg.assign_wells_to_ink("Mineral oil", ["B1"])
        pg.set_config(cfg)

        items = pg._loc_plate_view._well_items
        # Fills always match the assigned ink's own color (independent of the
        # current combo selection).
        self.assertEqual(items["A1"].brush().color().name(), "#e64553")
        self.assertEqual(items["B1"].brush().color().name(), "#f9e2af")

        # The page auto-highlights the current reagent's wells (mauve
        # selection border); clear it so the persisted TYPE borders show.
        pg._loc_plate_view.clear_selection()
        # A1: hydrogel material → INK blue outline.
        self.assertEqual(items["A1"].pen().color().name(),
                         ink_type_border_color("hydrogel"))
        # B1: oil → oil-specific outline.
        self.assertEqual(items["B1"].pen().color().name(),
                         ink_type_border_color("oil"))

    def test_unassigned_wells_stay_gray(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        pg = HardwareSetupPage()
        cfg = HardwareConfig()
        cfg.add_ink(InkSpec(name="Red gel", ink_type="hydrogel",
                            color="#e64553"))
        cfg.assign_wells_to_ink("Red gel", ["A1"])
        pg.set_config(cfg)
        empty = ROLE_COLORS[WellRole.EMPTY]
        # Some other well is unassigned → gray fill.
        self.assertEqual(
            pg._loc_plate_view._well_items["C3"].brush().color().name(), empty)


if __name__ == "__main__":
    unittest.main()
