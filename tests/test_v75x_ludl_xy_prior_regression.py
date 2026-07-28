"""test_v75x_ludl_xy_prior_regression.py — freeze Prior XY behavior.

Written BEFORE the LEP MAC 5000 generalization of ``XYStage.py`` /
``ControllerProtocol.py`` and must stay green AFTER it. It pins the exact wire
strings the two Prior ProScan protocols format, the position-response parsing,
and the Prior-safe defaults of the new protocol getters — so adding a new
controller family can never silently change how a Prior stage is driven.

The parse assertions call ``_parse_position_response`` *through an instance*
(``mgr._parse_position_response(...)``), which binds identically whether the
method is a ``@staticmethod`` (old) or an instance method (new) — the whole
point being that the observable result is frozen across that refactor.
"""

import unittest

from SupportClasses.ControllerProtocol import ControllerProtocol
from SupportClasses.XYStage import XYStageManager

PROSCAN_II = "config/controllers/proscan_ii.json"
PROSCAN_III = "config/controllers/proscan_iii.json"


def _mgr(protocol=None):
    """Build an XYStageManager without touching serial/simulator."""
    mgr = XYStageManager.__new__(XYStageManager)
    mgr._protocol = protocol
    mgr.simulate = False
    return mgr


class TestPriorCommandStringsFrozen(unittest.TestCase):
    """The exact bytes-on-the-wire templates must not drift."""

    def test_proscan_iii_commands(self):
        p = ControllerProtocol.load(PROSCAN_III)
        self.assertEqual(p.format_command("move_absolute", x=1000, y=2000), "G 1000,2000")
        self.assertEqual(p.format_command("move_relative", dx=-5, dy=7), "GR -5,7")
        self.assertEqual(p.format_command("set_velocity", vx=10, vy=20), "VS,10,20")
        self.assertEqual(p.format_command("set_max_speed", speed=50), "SMS,50")
        self.assertEqual(p.format_command("set_acceleration", accel=80), "SAS,80")
        self.assertEqual(p.format_command("set_jerk", jerk=30), "SCS,30")
        self.assertEqual(p.format_command("set_home"), "Z")
        self.assertEqual(p.format_command("stop"), "I")
        self.assertEqual(p.format_command("position_query"), "P")

    def test_proscan_ii_commands(self):
        p = ControllerProtocol.load(PROSCAN_II)
        # ProScan II uses comma-delimited move syntax (no space).
        self.assertEqual(p.format_command("move_absolute", x=1000, y=2000), "G,1000,2000")
        self.assertEqual(p.format_command("move_relative", dx=-5, dy=7), "GR,-5,7")
        self.assertEqual(p.format_command("set_max_speed", speed=50), "SMS,50")
        self.assertEqual(p.format_command("set_acceleration", accel=80), "SAS,80")
        self.assertEqual(p.format_command("position_query"), "P")
        # Jerk is unsupported on ProScan II.
        self.assertIsNone(p.format_command("set_jerk", jerk=30))
        self.assertFalse(p.supports_jerk)


class TestPriorTerminatorsAndFraming(unittest.TestCase):
    def test_terminators(self):
        self.assertEqual(ControllerProtocol.load(PROSCAN_III).tx_terminator, b"\r\n")
        self.assertEqual(ControllerProtocol.load(PROSCAN_II).tx_terminator, b"\r")

    def test_stop_bits_default_one(self):
        for f in (PROSCAN_II, PROSCAN_III):
            self.assertEqual(ControllerProtocol.load(f).stop_bits, 1)


class TestPositionParseFrozen(unittest.TestCase):
    """Prior x,y,z comma parse + trailing-R strip must be unchanged."""

    def test_no_protocol_csv(self):
        mgr = _mgr(None)
        self.assertEqual(mgr._parse_position_response("1000,2000,3000"), (1000.0, 2000.0, 3000.0))

    def test_no_protocol_strips_trailing_R(self):
        mgr = _mgr(None)
        self.assertEqual(mgr._parse_position_response("1000,2000,3000R"), (1000.0, 2000.0, 3000.0))

    def test_no_protocol_bad_field_count(self):
        mgr = _mgr(None)
        self.assertEqual(mgr._parse_position_response("1000,2000"), (None, None, None))

    def test_prior_iii_protocol_csv(self):
        mgr = _mgr(ControllerProtocol.load(PROSCAN_III))
        self.assertEqual(mgr._parse_position_response("1000,2000,3000"), (1000.0, 2000.0, 3000.0))

    def test_prior_ii_protocol_csv(self):
        mgr = _mgr(ControllerProtocol.load(PROSCAN_II))
        self.assertEqual(mgr._parse_position_response("-5,7,0"), (-5.0, 7.0, 0.0))


class TestPriorGetterDefaults(unittest.TestCase):
    """New getters return Prior-compatible defaults for the Prior JSONs."""

    def test_defaults(self):
        for f in (PROSCAN_II, PROSCAN_III):
            p = ControllerProtocol.load(f)
            self.assertEqual(p.family, "prior")
            self.assertIsNone(p.command_mode_init_bytes)
            self.assertEqual(p.ack_success_token, "R")
            # v7.5.x: Prior's "E,<code>" error acks are now detected (previously
            # silently invisible — a rejected command looked identical to a
            # successful one). "R" (success) must never false-positive as an error.
            self.assertIsNotNone(p.match_ack_error("E,4"))
            self.assertIsNone(p.match_ack_error("R"))
            self.assertEqual(p.speed_model, "percentage")
            self.assertEqual(p.accel_model, "percentage")
            self.assertFalse(p.speed_is_per_axis)
            self.assertEqual(p.position_scale, 1.0)
            pp = p.get_position_parse()
            self.assertEqual(pp["format"], "csv")
            self.assertEqual(pp["axis_order"], ["x", "y", "z"])
            self.assertEqual(pp["strip_tokens"], ["R"])
            self.assertIsNone(pp["ack_prefix"])


if __name__ == "__main__":
    unittest.main()
