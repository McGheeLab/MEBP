"""test_v75x_ludl_protocol_semantics.py — ControllerProtocol getters for Ludl.

Verifies the new schema fields load correctly for the LEP MAC 5000 protocol and
default to Prior-compatible values for the two Prior JSONs.
"""

import unittest

from SupportClasses.ControllerProtocol import ControllerProtocol

MAC5000 = "config/controllers/mac5000.json"
PROSCAN_II = "config/controllers/proscan_ii.json"
PROSCAN_III = "config/controllers/proscan_iii.json"


class TestLudlProtocol(unittest.TestCase):
    def setUp(self):
        self.p = ControllerProtocol.load(MAC5000)

    def test_family_and_init_bytes(self):
        self.assertEqual(self.p.family, "ludl")
        self.assertEqual(self.p.command_mode_init_bytes, b"\xff\x41")

    def test_serial_framing(self):
        self.assertEqual(self.p.tx_terminator, b"\r")
        self.assertEqual(self.p.baud_rate, 9600)
        self.assertEqual(self.p.stop_bits, 2)

    def test_ack_semantics(self):
        self.assertEqual(self.p.ack_success_token, ":A")
        m = self.p.match_ack_error(":N -4")
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), "4")
        self.assertIsNone(self.p.match_ack_error(":A 100 200"))

    def test_position_parse_spec(self):
        pp = self.p.get_position_parse()
        self.assertEqual(pp["format"], "whitespace")
        self.assertEqual(pp["axis_order"], ["x", "y"])
        self.assertEqual(pp["ack_prefix"], ":A")
        self.assertEqual(pp["strip_tokens"], [])

    def test_speed_accel_models(self):
        self.assertEqual(self.p.speed_model, "absolute")
        self.assertEqual(self.p.speed_units, "counts_per_s")
        self.assertTrue(self.p.speed_is_per_axis)
        self.assertEqual(self.p.accel_model, "absolute")

    def test_position_scale(self):
        self.assertEqual(self.p.position_scale, 10.0)

    def test_command_templates(self):
        self.assertEqual(self.p.format_command("move_absolute", x=1, y=2), "MOVE X=1 Y=2")
        self.assertEqual(self.p.format_command("move_relative", dx=-5, dy=7), "MOVREL X=-5 Y=7")
        self.assertEqual(self.p.format_command("position_query"), "WHERE X Y")
        self.assertEqual(self.p.format_command("set_max_speed", sx=100, sy=100), "SPEED X=100 Y=100")
        self.assertEqual(self.p.format_command("set_acceleration", accel=50), "ACCEL X=50 Y=50")
        self.assertEqual(self.p.format_command("set_home"), "HERE X=0 Y=0")
        self.assertEqual(self.p.format_command("stop"), "HALT")
        # No continuous vector-jog in HLC.
        self.assertIsNone(self.p.format_command("set_velocity", vx=1, vy=1))

    def test_accel_range(self):
        self.assertEqual(self.p.get_acceleration_range(), (1, 255))

    def test_detection_matches_ludl_not_prior(self):
        import re
        det = self.p.get_detection_info()
        pat = det["response_pattern"]
        self.assertTrue(re.match(pat, ":A 9.34"))
        self.assertTrue(re.match(pat, ":N -1"))
        # Prior replies (bare R / version / E,) must NOT match the Ludl pattern.
        self.assertIsNone(re.match(pat, "R"))
        self.assertIsNone(re.match(pat, "ProScan III"))
        self.assertIsNone(re.match(pat, "E,4"))


class TestPriorDefaultsUnchanged(unittest.TestCase):
    def test_prior_families_default(self):
        for f in (PROSCAN_II, PROSCAN_III):
            p = ControllerProtocol.load(f)
            self.assertEqual(p.family, "prior")
            self.assertIsNone(p.command_mode_init_bytes)
            self.assertEqual(p.ack_success_token, "R")
            self.assertEqual(p.speed_model, "percentage")
            self.assertEqual(p.accel_model, "percentage")
            self.assertFalse(p.speed_is_per_axis)
            self.assertEqual(p.position_scale, 1.0)


if __name__ == "__main__":
    unittest.main()
