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
    QApplication / real CalibrationPage instantiation required.

    v7.17: ``_zoff_read_stage_xy_um`` / ``_zoff_push_plate_bottom_to_controller``
    / ``_plate_bottom_z_source`` were added to the production setter by v7.9.1's
    plate-bottom-anchor work and never added here, so this fake had been raising
    AttributeError rather than testing anything. Also drops ``_max_z``, retired
    in v7.17.
    """
    return SimpleNamespace(
        controller=SimpleNamespace(safety_limits=sl),
        _plate_bottom_z=None,
        _plate_bottom_anchor_xy_um=None,
        _plate_bottom_z_source=None,
        _zoff_lbl_plate_bottom_z=MagicMock(),
        _emit_calibration_data_changed=lambda: None,
        _zoff_read_stage_xy_um=lambda: (1000.0, 2000.0),
        _zoff_push_plate_bottom_to_controller=lambda z, src: None,
        # v7.5.x: the label now renders the value in the unified user frame;
        # the controller stub has no converter, so identity is fine here.
        _zoff_user_z=lambda z: z,
    )


class TestCalZEnvelopeNoClobber(unittest.TestCase):
    """v7.17 — Max Z is retired (it had no consumer), so the two tests that
    drove ``_zoff_set_max_z`` are gone. The invariant they guarded is unchanged
    and still covered: no Z-reference capture on this page may write the
    device-setup envelope."""

    def test_set_plate_bottom_z_does_not_touch_safety_limits(self):
        sl = SafetyLimits(z_min=0.0, z_max=60.0, enabled=True)
        page = _fake_page(sl)
        page._zoff_capture_current_z = lambda: 48.4

        CalibrationPage._zoff_set_plate_bottom_z(page)

        self.assertEqual(page._plate_bottom_z, 48.4)
        self.assertEqual(sl.z_min, 0.0)
        self.assertEqual(sl.z_max, 60.0)

    def test_a_low_reference_no_longer_collapses_the_envelope(self):
        """The bug-report value (plate_bottom_z=48.4 captured on a machine whose
        envelope is 10..60) must leave the device-setup envelope valid and
        non-inverted, so clamp_z stays a pass-through inside the range."""
        sl = SafetyLimits(z_min=10.0, z_max=60.0, enabled=True)
        page = _fake_page(sl)

        page._zoff_capture_current_z = lambda: 48.4
        CalibrationPage._zoff_set_plate_bottom_z(page)

        # Envelope unchanged + still valid (min < max).
        self.assertEqual((sl.z_min, sl.z_max), (10.0, 60.0))
        self.assertLess(sl.z_min, sl.z_max)
        # Jogs across the working range pass through unclamped (the bug
        # collapsed every one of these to 48.4).
        for z in (12.0, 48.28, 48.509, 55.0):
            self.assertEqual(sl.clamp_z(z), z)

    def test_the_contact_touch_off_records_taught_provenance(self):
        """v7.17: the clamp is armed only by a MEASURED bottom, so the contact
        touch-off must tag itself 'taught' — see
        StageController.print_floor_datum_zref."""
        page = _fake_page(SafetyLimits(z_min=0.0, z_max=60.0, enabled=True))
        page._zoff_capture_current_z = lambda: 48.4
        pushed = []
        page._zoff_push_plate_bottom_to_controller = \
            lambda z, src: pushed.append((z, src))

        CalibrationPage._zoff_set_plate_bottom_z(page)

        self.assertEqual(pushed, [(48.4, "taught")])
        self.assertEqual(page._plate_bottom_z_source, "taught")

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
