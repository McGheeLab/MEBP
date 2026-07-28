"""
test_v75x_responsive_context_panel.py

Responsive / "sized to fit" left-panel content:

  * ``container_scale`` / ``quantize`` — the CSS clamp()-style width→scale
    primitive.
  * ``JogButtonArray`` — its step buttons, direction pad and pump columns now
    rescale from the widget's width (``_apply_scale`` shrinks on a narrow panel,
    grows on a wide one) and share aligned grids that fill the width, while the
    jog signal contract (directions/step values) is unchanged.
"""

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSize
from PySide6.QtGui import QResizeEvent
from PySide6.QtWidgets import QApplication, QPushButton, QWidget

_app = QApplication.instance() or QApplication(sys.argv)

from gui.scaling import s as _sc
from gui.widgets.responsive import (
    container_scale, quantize, DEFAULT_MIN_SCALE, DEFAULT_MAX_SCALE,
)
from gui.widgets.jog_button_array import JogButtonArray


class TestContainerScale(unittest.TestCase):
    def test_unity_at_design_width(self):
        self.assertAlmostEqual(container_scale(500, 500), 1.0)

    def test_clamped_range(self):
        self.assertEqual(container_scale(50, 500), DEFAULT_MIN_SCALE)   # tiny
        self.assertEqual(container_scale(5000, 500), DEFAULT_MAX_SCALE)  # huge

    def test_monotonic_between_bounds(self):
        narrow = container_scale(300, 500)
        wide = container_scale(700, 500)
        self.assertLess(narrow, 1.0)
        self.assertGreater(wide, 1.0)
        self.assertLess(narrow, wide)

    def test_nonpositive_is_unity(self):
        self.assertEqual(container_scale(0, 500), 1.0)
        self.assertEqual(container_scale(500, 0), 1.0)

    def test_quantize(self):
        self.assertEqual(quantize(0.833, 0.02), round(0.833 / 0.02) * 0.02)
        self.assertEqual(quantize(1.0, 0.0), 1.0)


class TestJogButtonArrayResponsive(unittest.TestCase):
    def test_structure_registered_for_scaling(self):
        arr = JogButtonArray(compact=True, show_pumps=True,
                             pump_action_labels=True)
        # 3 step rows (XY/Z/P) × 5 presets = 15; 3 custom edits.
        self.assertEqual(len(arr._scale_presets), 15)
        self.assertEqual(len(arr._scale_customs), 3)
        # 5 XY/Z pad + home + 2 Z = 7 direction buttons.
        self.assertEqual(len(arr._scale_dir), 7)
        # 3 pumps × (aspirate + dispense) = 6 word buttons; no arrows.
        self.assertEqual(len(arr._scale_pump_words), 6)
        self.assertEqual(len(arr._scale_pump_arrows), 0)

    def test_arrow_mode_registers_arrows(self):
        arr = JogButtonArray(compact=False, show_pumps=True,
                             pump_action_labels=False)
        self.assertEqual(len(arr._scale_pump_arrows), 6)   # 3 × ▲/▼
        self.assertEqual(len(arr._scale_pump_words), 0)

    def test_scale_shrinks_and_grows_buttons(self):
        arr = JogButtonArray(compact=True, show_pumps=True,
                             pump_action_labels=True)
        arr._apply_scale(DEFAULT_MIN_SCALE)
        narrow_dir = arr._scale_dir[0].width()
        narrow_preset = arr._scale_presets[0].minimumWidth()
        narrow_pt = arr._scale_dir[0].font().pointSizeF()

        arr._apply_scale(DEFAULT_MAX_SCALE)
        wide_dir = arr._scale_dir[0].width()
        wide_preset = arr._scale_presets[0].minimumWidth()
        wide_pt = arr._scale_dir[0].font().pointSizeF()

        self.assertLess(narrow_dir, wide_dir)
        self.assertLess(narrow_preset, wide_preset)
        self.assertLess(narrow_pt, wide_pt)

    def test_resize_event_drives_scale(self):
        arr = JogButtonArray(compact=True, show_pumps=True,
                             pump_action_labels=True)
        arr.resize(700, 500)
        arr.resizeEvent(QResizeEvent(QSize(700, 500), QSize(0, 0)))
        wide_scale = arr._rscale
        arr.resize(280, 500)
        arr.resizeEvent(QResizeEvent(QSize(280, 500), QSize(0, 0)))
        narrow_scale = arr._rscale
        self.assertGreater(wide_scale, narrow_scale)

    def test_signal_contract_unchanged(self):
        from PySide6.QtWidgets import QPushButton as _QPB
        arr = JogButtonArray(compact=True, show_pumps=True,
                             pump_action_labels=True)
        # Z direction contract (height frame: ▲ up = +, ▼ down = −).
        z = []
        arr.jog_z_requested.connect(z.append)
        btns = {b.toolTip(): b for b in arr.findChildren(_QPB)}
        btns["Move Z up"].click()
        btns["Move Z down"].click()
        self.assertGreater(z[0], 0)
        self.assertLess(z[1], 0)

        # XY contract.
        xy = []
        arr.jog_xy_requested.connect(lambda dx, dy: xy.append((dx, dy)))
        # find the left arrow by text
        left = next(b for b in arr.findChildren(_QPB) if b.text() == "◀")
        left.click()
        self.assertLess(xy[-1][0], 0)   # ◀ → negative dx
        self.assertEqual(xy[-1][1], 0)

        # Pump aspirate emits negative, dispense positive (unchanged sign).
        pump = []
        arr.jog_pump_requested.connect(lambda pid, d: pump.append((pid, d)))
        asp, disp = arr._pump_buttons["P1"]
        asp.click()
        disp.click()
        self.assertLess(pump[0][1], 0)    # aspirate
        self.assertGreater(pump[1][1], 0)  # dispense


class TestDescendantFontScaling(unittest.TestCase):
    """scale_descendant_fonts shrinks/grows every descendant's font by the
    factor (the 'shrink text by a %% of width' pass), and honours skip_subtrees."""

    def test_scales_and_skips(self):
        from PySide6.QtWidgets import QVBoxLayout, QLabel
        from gui.widgets.responsive import scale_descendant_fonts
        root = QWidget()
        lay = QVBoxLayout(root)
        lbl = QLabel("hi")
        lay.addWidget(lbl)
        skipped = QWidget()
        skl = QVBoxLayout(skipped)
        inner = QLabel("x")
        skl.addWidget(inner)
        lay.addWidget(skipped)

        scale_descendant_fonts(root, 0.5, skip_subtrees=[skipped])
        small = lbl.font().pointSizeF()
        inner_after = inner.font().pointSizeF()
        scale_descendant_fonts(root, 1.4, skip_subtrees=[skipped])
        big = lbl.font().pointSizeF()

        self.assertLess(small, big)                # scaled with factor
        self.assertEqual(inner.font().pointSizeF(), inner_after)  # skipped

    def test_control_panel_font_scales_with_width(self):
        from PySide6.QtGui import QResizeEvent
        from gui.pages.hardware.control_panel import HardwareControlPanel
        p = HardwareControlPanel(show_connect=False, bypass_safety=False,
                                 embedded=True, pump_action_labels=True)
        p.resize(520, 700)
        p.resizeEvent(QResizeEvent(QSize(520, 700), QSize(0, 0)))
        wide = p._font_factor
        p.resize(150, 700)
        p.resizeEvent(QResizeEvent(QSize(150, 700), QSize(0, 0)))
        narrow = p._font_factor
        self.assertLess(narrow, wide)


class TestPanelMinimumsAllowNarrow(unittest.TestCase):
    """The jog panel + its host panels report a small minimum WIDTH so the
    context box can be dragged to its 100 px minimum — content is proportional
    and scrunches to fit rather than flooring the width."""

    def test_jog_array_reports_small_min(self):
        arr = JogButtonArray(compact=True, show_pumps=True,
                             pump_action_labels=True)
        # ~60 px design floor (DPI-scaled). Well under the old ~150 px.
        self.assertLessEqual(arr.minimumSizeHint().width(), _sc(110))

    def test_control_panel_reports_small_min(self):
        from gui.pages.hardware.control_panel import HardwareControlPanel
        p = HardwareControlPanel(show_connect=False, bypass_safety=False,
                                 embedded=True, pump_action_labels=True)
        self.assertLessEqual(p.minimumSizeHint().width(), _sc(130))

    def test_context_panel_reports_small_min(self):
        from gui.widgets.standard_jog_context import StandardJogContextPanel
        p = StandardJogContextPanel(controller=None, settings=None)
        self.assertLessEqual(p.minimumSizeHint().width(), _sc(130))


if __name__ == "__main__":
    unittest.main()
