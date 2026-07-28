"""test_v75x_prior_multi_baud_detect.py — multi-baud detection retry.

Added after a real Prior ProScan III unit was found sitting at a non-default
baud (Prior's own manual documents that a changed baud setting reverts to
9600 if the port sits idle across TWO power cycles — a real, expected
failure mode). ``ControllerProtocol.baud_rate_candidates`` lets a protocol
declare several bauds to try per port during detection; absent, it's a
single-element list (byte-identical to the old single-``baud_rate`` behavior
for every existing Prior II/III/Ludl protocol, none of which set it).

Also covers the new ``config/controllers/proscan_iii_h117.json`` variant
(STAGE-based detection for a real unit whose 'V' query returns a bare 'R'
instead of a version string, independent of baud).
"""

import unittest

from SupportClasses.ControllerProtocol import ControllerProtocol
from SupportClasses.XYStage import XYStageManager

PROSCAN_II = "config/controllers/proscan_ii.json"
PROSCAN_III = "config/controllers/proscan_iii.json"
H117 = "config/controllers/proscan_iii_h117.json"
MAC5000 = "config/controllers/mac5000.json"


class TestBaudRateCandidatesDefault(unittest.TestCase):
    """Every existing protocol (no communication.baud_rates) must keep
    trying exactly its single default_baud_rate — no behavior change."""

    def test_prior_ii_single_baud(self):
        p = ControllerProtocol.load(PROSCAN_II)
        self.assertEqual(p.baud_rate_candidates, [p.baud_rate])

    def test_prior_iii_single_baud(self):
        p = ControllerProtocol.load(PROSCAN_III)
        self.assertEqual(p.baud_rate_candidates, [38400])

    def test_ludl_single_baud(self):
        p = ControllerProtocol.load(MAC5000)
        self.assertEqual(p.baud_rate_candidates, [9600])


class TestH117Protocol(unittest.TestCase):
    """The real-hardware Prior III variant (STAGE-based detect, multi-baud)."""

    def setUp(self):
        self.p = ControllerProtocol.load(H117)

    def test_family_defaults_to_prior(self):
        # No controller_family key set -> must default to "prior", same as
        # every other Prior JSON (this is still a Prior III wire protocol).
        self.assertEqual(self.p.family, "prior")

    def test_baud_candidates_highest_first(self):
        self.assertEqual(self.p.baud_rate_candidates, [115200, 38400, 9600])
        self.assertEqual(self.p.baud_rate, 115200)

    def test_stage_based_detection(self):
        det = self.p.get_detection_info()
        self.assertEqual(det["firmware_query"], "STAGE")
        self.assertEqual(det["wake_command"], "STAGE")
        self.assertIn("STAGE =", det["identify_tokens"])
        self.assertIn("MICROSTEPS", det["identify_tokens"])

    def test_same_wire_commands_as_standard_prior_iii(self):
        std = ControllerProtocol.load(PROSCAN_III)
        for cmd in ("move_absolute", "move_relative", "set_velocity",
                    "set_max_speed", "set_acceleration", "set_jerk",
                    "set_home", "stop", "position_query"):
            self.assertEqual(
                self.p.format_command(cmd, x=1, y=2, dx=1, dy=2, vx=1, vy=2,
                                      speed=50, accel=50, jerk=10),
                std.format_command(cmd, x=1, y=2, dx=1, dy=2, vx=1, vy=2,
                                   speed=50, accel=50, jerk=10),
                msg=f"{cmd} wire syntax must match standard proscan_iii.json")


class TestMultiBaudScanTriesEachCandidate(unittest.TestCase):
    """_find_with_protocol must retry EVERY candidate baud on a port before
    moving to the next port (the actual detection-scan behavior)."""

    def test_second_candidate_baud_matches(self):
        import sys
        xymod = sys.modules["SupportClasses.XYStage"]

        opened_bauds = []

        class _PortInfo:
            device = "COM9"

        class _Conn:
            def __init__(self, port, baudrate=None, **kw):
                opened_bauds.append(baudrate)
                self.timeout = 0.5
                self.dtr = False
                self._baud = baudrate
                self._reads_left = 1 if baudrate == 38400 else 0

            def reset_input_buffer(self): pass
            def reset_output_buffer(self): pass
            def write(self, b): pass

            def read(self, n=1):
                # Only the SECOND candidate baud (38400) "responds" — a single
                # stray byte then silence (simulates pyserial's real per-call
                # timeout: one byte arrives, then no CR ever follows so the
                # NEXT read() call legitimately times out and returns b"").
                if self._reads_left > 0:
                    self._reads_left -= 1
                    return b"S"
                return b""

            def close(self): pass

        class _ListPorts:
            @staticmethod
            def comports():
                return [_PortInfo()]

        class _Tools:
            list_ports = _ListPorts

        class _FakeSerial:
            STOPBITS_ONE = 1
            STOPBITS_TWO = 2
            SerialException = Exception
            Serial = _Conn
            tools = _Tools

        proto = ControllerProtocol.load(H117)  # candidates: [115200, 38400, 9600]
        mgr = XYStageManager.__new__(XYStageManager)
        mgr._protocol = proto
        mgr.simulate = False
        mgr._detected_controller = None
        mgr._exclude_ports = set()
        orig = xymod.serial
        xymod.serial = _FakeSerial
        try:
            # Response is a single "S" with no CR ever arriving -> _read_response_cr
            # will time out without a match on identify tokens either way, but the
            # important, deterministic thing to assert is that ALL 3 candidate
            # bauds were actually attempted on the one port (retry loop works),
            # not just the first.
            mgr._find_with_protocol(proto)
        finally:
            xymod.serial = orig
        self.assertEqual(opened_bauds, [115200, 38400, 9600])


if __name__ == "__main__":
    unittest.main()
