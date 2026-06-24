"""
test_v75x_cal_z_envelope_no_clobber.py

Regression for the "Z clamped: 48.x → 48.4 mm on every jog" bug.

Symptom: with Left/Right triggers mapped to ``move_z_at_velocity`` and a
device-setup Z range of 0..60, every jog was clamped to a single point
(~48.4 mm). Root cause: the calibration page pushed its captured
``Max Z`` / ``Plate Bottom Z`` heights into ``SafetyLimits.z_max`` /
``z_min``:

    set_z_from_current(max_z,          as_max=True)   # z_max = 16.4
    set_z_from_current(plate_bottom_z, as_max=False)  # z_min = 48.4

On a machine where the needle *descends as Z increases*, ``Max Z`` is
numerically *below* ``Plate Bottom Z`` (16.4 < 48.4), so the assignment
inverted the envelope (z_min 48.4 > z_max 16.4) and ``clamp_z`` collapsed
to a single point. It also silently clobbered the user's device-setup
range on every startup.

Fix (v7.5.x): the Z soft-limit envelope is owned exclusively by Hardware
Setup → Device. The calibration setters and ``_load_calibration`` no longer
touch ``safety_limits``; the captured heights remain only as print/Z
references (``_max_z`` / ``_plate_bottom_z``).
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

from SupportClasses.SafetyLimits import SafetyLimits
import gui.pages.calibration as cal_mod
from gui.pages.calibration import CalibrationPage


def _fake_page(sl: SafetyLimits) -> SimpleNamespace:
    """Minimal stand-in exposing exactly what the setters touch — no
    QApplication / real CalibrationPage instantiation required."""
    return SimpleNamespace(
        controller=SimpleNamespace(safety_limits=sl),
        _max_z=None,
        _plate_bottom_z=None,
        _zoff_lbl_max_z=MagicMock(),
        _zoff_lbl_plate_bottom_z=MagicMock(),
        _emit_calibration_data_changed=lambda: None,
        # v7.5.x: the label now renders the value in the unified user frame;
        # the controller stub has no converter, so identity is fine here.
        _zoff_user_z=lambda z: z,
    )


class TestCalZEnvelopeNoClobber(unittest.TestCase):
    def test_set_max_z_does_not_touch_safety_limits(self):
        sl = SafetyLimits(z_min=0.0, z_max=60.0, enabled=True)
        page = _fake_page(sl)
        page._zoff_capture_current_z = lambda: 16.4

        CalibrationPage._zoff_set_max_z(page)

        # Captured as a reference height …
        self.assertEqual(page._max_z, 16.4)
        # … but the envelope is untouched.
        self.assertEqual(sl.z_min, 0.0)
        self.assertEqual(sl.z_max, 60.0)

    def test_set_plate_bottom_z_does_not_touch_safety_limits(self):
        sl = SafetyLimits(z_min=0.0, z_max=60.0, enabled=True)
        page = _fake_page(sl)
        page._zoff_capture_current_z = lambda: 48.4

        CalibrationPage._zoff_set_plate_bottom_z(page)

        self.assertEqual(page._plate_bottom_z, 48.4)
        self.assertEqual(sl.z_min, 0.0)
        self.assertEqual(sl.z_max, 60.0)

    def test_inverting_capture_order_no_longer_collapses_envelope(self):
        """The exact field values from the bug report (max_z=16.4 <
        plate_bottom_z=48.4) must leave the device-setup envelope valid and
        non-inverted, so clamp_z stays a pass-through inside the range."""
        sl = SafetyLimits(z_min=10.0, z_max=60.0, enabled=True)
        page = _fake_page(sl)

        page._zoff_capture_current_z = lambda: 16.4
        CalibrationPage._zoff_set_max_z(page)
        page._zoff_capture_current_z = lambda: 48.4
        CalibrationPage._zoff_set_plate_bottom_z(page)

        # Envelope unchanged + still valid (min < max).
        self.assertEqual((sl.z_min, sl.z_max), (10.0, 60.0))
        self.assertLess(sl.z_min, sl.z_max)
        # Jogs across the working range pass through unclamped (the bug
        # collapsed every one of these to 48.4).
        for z in (12.0, 48.28, 48.509, 55.0):
            self.assertEqual(sl.clamp_z(z), z)

    def test_calibration_module_no_longer_pushes_into_envelope(self):
        """Guard the _load_calibration path (too heavyweight to instantiate
        headless): the calibration source must not call set_z_from_current,
        which is how it used to clobber/invert the envelope on startup."""
        src = Path(cal_mod.__file__).read_text(encoding="utf-8")
        # Strip comments so a doc reference to the old call doesn't trip it.
        code = "\n".join(
            line.split("#", 1)[0] for line in src.splitlines()
        )
        self.assertNotIn(
            "set_z_from_current(", code,
            msg="calibration.py must not write the Z safety envelope; the "
                "device-setup range is authoritative.",
        )


if __name__ == "__main__":
    unittest.main()
