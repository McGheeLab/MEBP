"""v7.21.1 — the XY max speed set in the hardware config is authoritative.

Background (the bench report this fixes): a fluorescence mosaic and a print both
ran with the log line

    set_speed_mm_s: 50.0 mm/s = 50000 µm/s = SMS 100% (max=50000 µm/s)

on a stage whose true top speed is 5.5 mm/s, no matter what Hardware Setup said.
Two independent defects:

  1. ``StageController.safe_travel_to`` hardcoded ``fast_xy_speed_mm_s=50.0``
     and NOT ONE of its callers overrode it, so every travel commanded 50 mm/s.
     Prior SMS is modal, so every later un-speeded move (e.g. the mosaic's 35
     remaining raster tiles) inherited it.
  2. The mm/s↔SMS denominator came from the protocol's nominal ``max_speed``
     (50000) because nothing pushed a per-machine value — and there was no UI
     to declare one at all (``spin_max_xy_speed`` existed but was never added
     to a layout).

These tests drive the PRODUCTION controller, not a stand-in, because a fake that
agrees with the code proves nothing about it.
"""

import unittest
from unittest.mock import MagicMock

from SupportClasses.StageController import StageController
from SupportClasses.SafetyLimits import SafetyLimits


def _bare_controller() -> StageController:
    """A controller with the speed machinery live but no hardware.

    ``__new__`` + explicit field seeding (the pattern the rest of this suite
    uses) so no serial port is opened and no thread is started.
    """
    ctrl = StageController.__new__(StageController)
    ctrl.safety_limits = SafetyLimits()
    ctrl._declared_xy_top_speed_um_s = None
    ctrl.xy_stage = None
    ctrl.zp_stage = None
    ctrl.xy_jog = None
    ctrl.zp_jog = None
    ctrl._jog_speed_pct = {}
    ctrl.on_speed_limits_changed = None
    return ctrl


class TestDeclarationIsDistinguishableFromTheDefault(unittest.TestCase):
    """The load-bearing distinction: "never declared" ≠ "declared 10000".

    ``SafetyLimits.max_xy_speed`` carries a 10000 µm/s DEFAULT. Feeding a
    default into the SMS denominator would make every commanded speed run
    FASTER than requested on a stage whose real top speed is higher, because
    ``SMS% = requested ÷ denominator``. So an undeclared machine must keep the
    protocol's own max_speed.
    """

    def test_undeclared_reports_none_even_though_safety_has_a_default(self):
        ctrl = _bare_controller()
        self.assertEqual(ctrl.safety_limits.max_xy_speed, 10_000.0,
                         "fixture assumption: SafetyLimits carries a default")
        self.assertIsNone(ctrl.declared_xy_top_speed_um_s())

    def test_undeclared_never_touches_the_stage_denominator(self):
        ctrl = _bare_controller()
        xy = MagicMock()
        ctrl.xy_stage = xy
        ctrl._push_xy_top_speed_to_stage()
        xy.set_max_speed_um_s.assert_not_called()

    def test_declaring_reports_it(self):
        ctrl = _bare_controller()
        ctrl.set_xy_top_speed_um_s(5500.0)
        self.assertEqual(ctrl.declared_xy_top_speed_um_s(), 5500.0)


class TestOneSetterMovesEveryConsumer(unittest.TestCase):
    def test_setter_updates_denominator_anchor_and_jog_together(self):
        ctrl = _bare_controller()
        xy = MagicMock()
        ctrl.xy_stage = xy
        jog = MagicMock()
        ctrl.xy_jog = jog

        ctrl.set_xy_top_speed_um_s(5500.0)

        # 1. the mm/s↔SMS denominator on the live stage
        xy.set_max_speed_um_s.assert_called_once_with(5500.0)
        # 2. the anchor every jog / print speed-% surface reads
        self.assertEqual(ctrl.get_max_xy_speed_um_s(), 5500.0)
        self.assertEqual(ctrl.safety_limits.max_xy_speed, 5500.0)
        # 3. the jog handler's 100% anchor
        jog.set_speed_max.assert_called_with(5500.0)

    def test_rejects_nonsense_rather_than_declaring_it(self):
        ctrl = _bare_controller()
        for bad in (0, -5, None, "fast"):
            ctrl.set_xy_top_speed_um_s(bad)
            self.assertIsNone(ctrl.declared_xy_top_speed_um_s(),
                              f"{bad!r} must not become a declaration")

    def test_connect_seeding_uses_the_declaration(self):
        """The seeding hook `connect_stages` calls must honour the declaration
        (it used to read the timing store and nothing else)."""
        ctrl = _bare_controller()
        ctrl._declared_xy_top_speed_um_s = 5500.0
        xy = MagicMock()
        ctrl.xy_stage = xy
        ctrl._push_xy_top_speed_to_stage()
        xy.set_max_speed_um_s.assert_called_once_with(5500.0)


class TestSafeTravelResolvesTheConfiguredSpeed(unittest.TestCase):
    """The original bug: a hardcoded 50 mm/s no caller overrode."""

    def _travel_ctrl(self, declared_um_s):
        # is_xy_connected / is_zp_connected are properties on StageController,
        # so they are overridden in a subclass rather than assigned per-instance.
        class _Connected(StageController):
            is_xy_connected = True
            is_zp_connected = False

        ctrl = _bare_controller()
        ctrl.__class__ = _Connected
        if declared_um_s:
            ctrl.set_xy_top_speed_um_s(declared_um_s)
        ctrl.xy_stage = MagicMock()
        # No needle present ⇒ the ZP-absent branch falls through to the XY move
        # instead of aborting, so the speed resolution is reachable.
        ctrl._needle_present = lambda: False
        ctrl.zero_position = {"x": 0.0, "y": 0.0}
        ctrl.move_xy_absolute = MagicMock()
        ctrl.wait_for_xy_arrival = MagicMock(return_value=True)
        ctrl._pos_poller = MagicMock()
        ctrl._motion = None
        return ctrl

    def test_default_is_the_configured_max_not_fifty(self):
        ctrl = self._travel_ctrl(5500.0)
        ctrl.safe_travel_to(1000.0, 2000.0, safe_z_mm=-46.85, target_z_mm=None)
        ctrl.xy_stage.set_speed_mm_s.assert_called_once_with(5.5)

    def test_the_old_hardcoded_value_would_have_been_visibly_wrong(self):
        """Guard the guard: assert the pre-fix answer differs by ~9x, so the
        test above cannot pass merely because nothing sets a speed at all."""
        ctrl = self._travel_ctrl(5500.0)
        ctrl.safe_travel_to(1000.0, 2000.0, safe_z_mm=-46.85, target_z_mm=None)
        (commanded,), _ = ctrl.xy_stage.set_speed_mm_s.call_args
        self.assertLess(commanded, 50.0 / 5.0,
                        "the resolved travel speed must not be the old 50 mm/s")

    def test_an_explicit_speed_still_wins(self):
        ctrl = self._travel_ctrl(5500.0)
        ctrl.safe_travel_to(1000.0, 2000.0, safe_z_mm=-46.85, target_z_mm=None,
                            fast_xy_speed_mm_s=2.0)
        ctrl.xy_stage.set_speed_mm_s.assert_called_once_with(2.0)

    def test_undeclared_machine_still_gets_a_sane_travel_speed(self):
        """No declaration ⇒ the safety anchor (or the conservative fallback),
        never 0 and never None reaching the stage."""
        ctrl = self._travel_ctrl(None)
        ctrl.safe_travel_to(1000.0, 2000.0, safe_z_mm=-46.85, target_z_mm=None)
        (commanded,), _ = ctrl.xy_stage.set_speed_mm_s.call_args
        self.assertGreater(commanded, 0.0)
        self.assertEqual(commanded, 10.0)  # 10000 µm/s safety default


class TestApplyDeviceSettingsCarriesIt(unittest.TestCase):
    def test_xy_max_speed_um_s_is_applied(self):
        ctrl = _bare_controller()
        ctrl._pending_axis_map = {}
        ctrl._pending_steps_per_mm = {}
        ctrl._pending_per_axis_max_feedrate = {}
        ctrl._refresh_zp_move_feedrates = MagicMock()
        ctrl.apply_device_settings(xy_max_speed_um_s=5500.0)
        self.assertEqual(ctrl.declared_xy_top_speed_um_s(), 5500.0)

    def test_none_leaves_an_existing_declaration_alone(self):
        ctrl = _bare_controller()
        ctrl._pending_axis_map = {}
        ctrl._pending_steps_per_mm = {}
        ctrl._pending_per_axis_max_feedrate = {}
        ctrl._refresh_zp_move_feedrates = MagicMock()
        ctrl.set_xy_top_speed_um_s(5500.0)
        ctrl.apply_device_settings(axis_map={"Z": "Z"})
        self.assertEqual(ctrl.declared_xy_top_speed_um_s(), 5500.0)


class TestNoCallerHardcodesTravelSpeed(unittest.TestCase):
    """A NEW caller passing its own literal would silently reintroduce the bug,
    so pin that the production call sites leave the speed to the resolver."""

    def test_production_callers_do_not_pass_fast_xy_speed_mm_s(self):
        import ast
        import pathlib

        root = pathlib.Path(__file__).resolve().parents[1]
        offenders = []
        checked = 0
        for path in list((root / "SupportClasses").rglob("*.py")) + \
                list((root / "gui").rglob("*.py")):
            try:
                tree = ast.parse(path.read_text(encoding="utf-8"))
            except (SyntaxError, UnicodeDecodeError):
                continue
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                fn = node.func
                name = getattr(fn, "attr", None) or getattr(fn, "id", None)
                if name != "safe_travel_to":
                    continue
                checked += 1
                for kw in node.keywords:
                    if kw.arg == "fast_xy_speed_mm_s":
                        offenders.append(f"{path.name}:{node.lineno}")
        # Guard the guard: a matcher that finds nothing passes vacuously.
        self.assertGreater(checked, 5,
                           "expected to find the known safe_travel_to callers")
        self.assertEqual(
            offenders, [],
            "these callers pin their own XY travel speed instead of "
            "inheriting the configured max: " + ", ".join(offenders))


class TestTheFieldIsActuallyOnScreen(unittest.TestCase):
    """The reported defect was a widget that EXISTED but was never parented.

    ``spin_max_xy_speed`` was created as a headless shadow in v7.4.2 with a
    comment saying it "is reparented into the XY Calibration section below" — it
    never was, so from v7.4.2 to v7.21.0 there was no way to set the XY max
    speed at all. Asserting the attribute exists would have passed throughout.
    These tests drive the REAL panel and probe the rendered widget tree.
    """

    @classmethod
    def setUpClass(cls):
        import os
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication([])

    def _panel(self):
        from gui.pages.hardware.stage_panel import StageHardwarePanel
        return StageHardwarePanel()

    def test_max_speed_spin_is_parented_and_visible(self):
        panel = self._panel()
        spin = panel.spin_max_xy_speed
        self.assertIsNotNone(spin.parentWidget(),
                             "spin_max_xy_speed is not in the widget tree")
        # isHidden(), not isVisible(): isVisible() is False for any widget whose
        # ancestor is unshown, which an offscreen panel always is.
        self.assertFalse(spin.isHidden(),
                         "spin_max_xy_speed is present but hidden")

    def test_it_lives_in_the_xy_stage_calibration_section(self):
        """Walk up from the spin box to the reorderable section that owns it.

        (The QGroupBox title is blanked by ReorderableSection, which renders the
        heading itself — so identity against the registered section, not the
        title text, is what actually pins placement.)
        """
        panel = self._panel()
        target = panel._section_list._sections["xy_cal"]
        w = panel.spin_max_xy_speed.parentWidget()
        chain = []
        while w is not None:
            chain.append(w)
            if w is target:
                return
            w = w.parentWidget()
        self.fail("spin_max_xy_speed is not inside the 'xy_cal' section; "
                  f"ancestors were {[type(x).__name__ for x in chain]}")

    def test_the_mm_per_s_hint_tracks_the_value(self):
        panel = self._panel()
        panel.spin_max_xy_speed.setValue(5500.0)
        self.assertIn("5.50 mm/s", panel.lbl_xy_max_speed_hint.text())


if __name__ == "__main__":
    unittest.main()
