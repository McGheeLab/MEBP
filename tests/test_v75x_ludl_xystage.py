"""test_v75x_ludl_xystage.py — XYStageManager driven by the Ludl protocol.

Unit-level checks that the generalized XYStage behaves per-protocol:
  * position parse handles Ludl ':A x y' (whitespace, 2-axis, counts→µm) AND
    still handles Prior 'x,y,z' CSV,
  * moves apply the µm→count scale on the wire (Prior scale 1.0 = unchanged),
  * _drain_ack recognizes ':A' / ':N -<code>',
  * set_speed_mm_s / set_acceleration take the ABSOLUTE branch for Ludl and the
    PERCENTAGE branch for Prior.
"""

import unittest

from SupportClasses.ControllerProtocol import ControllerProtocol
from SupportClasses.XYStage import XYStageManager

MAC5000 = "config/controllers/mac5000.json"
PROSCAN_II = "config/controllers/proscan_ii.json"
PROSCAN_III = "config/controllers/proscan_iii.json"


class _FakeSerial:
    """Minimal pyserial-like object feeding bytes to _read_response_cr."""

    def __init__(self, data: bytes):
        self._data = data
        self.timeout = 0.5

    def read(self, n: int = 1) -> bytes:
        if not self._data:
            return b""
        b, self._data = self._data[:1], self._data[1:]
        return b

    def stop(self):
        pass

    def close(self):
        pass


def _mgr(protocol_path, *, simulate=True):
    mgr = XYStageManager.__new__(XYStageManager)
    mgr._protocol = ControllerProtocol.load(protocol_path)
    mgr.simulate = simulate
    mgr._apply_protocol_parameters()  # sets min/max_acceleration, ranges, etc.
    sent = []
    mgr._send_protocol_command = (
        lambda name, fallback_cmd=None, **kw: sent.append((name, dict(kw))))
    return mgr, sent


class TestPositionParse(unittest.TestCase):
    def test_ludl_whitespace_counts_to_um(self):
        mgr, _ = _mgr(MAC5000)
        # counts on the wire; scale 10 counts/µm ⇒ µm = counts/10
        self.assertEqual(mgr._parse_position_response(":A 10000 20000"), (1000.0, 2000.0, 0.0))

    def test_ludl_negative_and_z_default(self):
        mgr, _ = _mgr(MAC5000)
        self.assertEqual(mgr._parse_position_response(":A -500 0"), (-50.0, 0.0, 0.0))

    def test_prior_csv_still_parses(self):
        mgr, _ = _mgr(PROSCAN_III)
        self.assertEqual(mgr._parse_position_response("1000,2000,3000"), (1000.0, 2000.0, 3000.0))

    def test_prior_csv_strips_trailing_R(self):
        mgr, _ = _mgr(PROSCAN_III)
        self.assertEqual(mgr._parse_position_response("1000,2000,3000R"), (1000.0, 2000.0, 3000.0))


class TestMoveScaling(unittest.TestCase):
    def test_ludl_move_absolute_scales_um_to_counts(self):
        mgr, sent = _mgr(MAC5000)
        mgr.move_stage_to_position(1000, 2000)   # µm
        name, kw = sent[-1]
        self.assertEqual(name, "move_absolute")
        self.assertEqual((kw["x"], kw["y"]), (10000, 20000))  # ×10 counts/µm

    def test_ludl_move_relative_scales(self):
        mgr, sent = _mgr(MAC5000)
        mgr.move_stage_relative(-50, 7)
        name, kw = sent[-1]
        self.assertEqual(name, "move_relative")
        self.assertEqual((kw["dx"], kw["dy"]), (-500, 70))

    def test_prior_move_unscaled(self):
        mgr, sent = _mgr(PROSCAN_III)
        mgr.move_stage_to_position(1000, 2000)
        _, kw = sent[-1]
        self.assertEqual((kw["x"], kw["y"]), (1000, 2000))  # scale 1.0 = identity


class TestDrainAck(unittest.TestCase):
    def test_ludl_success_ack(self):
        mgr, _ = _mgr(MAC5000)
        mgr.spo = _FakeSerial(b":A\r")
        self.assertEqual(mgr._drain_ack(), ":A")

    def test_ludl_error_ack_returned_not_raised(self):
        mgr, _ = _mgr(MAC5000)
        mgr.spo = _FakeSerial(b":N -4\r")
        # logs a warning, does not raise, still drains the line
        self.assertEqual(mgr._drain_ack(), ":N -4")

    def test_prior_bare_R(self):
        mgr, _ = _mgr(PROSCAN_III)
        mgr.spo = _FakeSerial(b"R\r")
        self.assertEqual(mgr._drain_ack(), "R")


class TestSpeedModel(unittest.TestCase):
    def test_ludl_absolute_per_axis(self):
        mgr, sent = _mgr(MAC5000)
        mgr.set_speed_mm_s(5.0)   # 5 mm/s = 5000 µm/s → ×10 = 50000 counts/s
        name, kw = sent[-1]
        self.assertEqual(name, "set_max_speed")
        self.assertEqual(kw.get("sx"), 50000)
        self.assertEqual(kw.get("sy"), 50000)
        self.assertNotIn("speed", kw)   # per-axis, not the % 'speed' key

    def test_ludl_clamps_to_ceiling(self):
        mgr, sent = _mgr(MAC5000)
        mgr.set_speed_mm_s(999.0)  # 999 mm/s >> 50 mm/s ceiling (max_speed 50000 µm/s)
        _, kw = sent[-1]
        self.assertEqual(kw["sx"], 500000)   # 50000 µm/s × 10 = 500000 counts/s

    def test_ludl_velocity_percent_of_ceiling(self):
        mgr, sent = _mgr(MAC5000)
        mgr.set_velocity(50)       # 50% of 50000 µm/s = 25000 µm/s → 250000 counts/s
        _, kw = sent[-1]
        self.assertEqual(kw["sx"], 250000)

    def test_prior_percentage(self):
        mgr, sent = _mgr(PROSCAN_III)
        mgr.set_speed_mm_s(1.0)    # 1000/50000*100 = 2%
        name, kw = sent[-1]
        self.assertEqual(name, "set_max_speed")
        self.assertEqual(kw.get("speed"), 2)


class TestExcludePorts(unittest.TestCase):
    """The detection scan must NEVER open an excluded port (opening asserts DTR
    → resets an Arduino/Marlin ZP board)."""

    def _run_scan(self, exclude):
        import sys
        # NB: SupportClasses/__init__.py rebinds the name `XYStage` to the class
        # (`import XYStageManager as XYStage`), so `import SupportClasses.XYStage`
        # yields the CLASS, not the module — fetch the real module from sys.modules.
        xymod = sys.modules["SupportClasses.XYStage"]

        opened = []

        class _PortInfo:
            def __init__(self, device):
                self.device = device

        class _Conn:
            def __init__(self, port, **kw):
                opened.append(port)
                self.timeout = 0.5
                self.dtr = False

            def reset_input_buffer(self):
                pass

            def reset_output_buffer(self):
                pass

            def write(self, b):
                pass

            def read(self, n=1):
                return b""   # no reply → detection fails, scan continues

            def close(self):
                pass

        class _ListPorts:
            @staticmethod
            def comports():
                return [_PortInfo("COM4"), _PortInfo("COM5")]

        class _Tools:
            list_ports = _ListPorts

        class _FakeSerial:
            STOPBITS_ONE = 1
            STOPBITS_TWO = 2
            SerialException = Exception
            Serial = _Conn
            tools = _Tools

        mgr = XYStageManager.__new__(XYStageManager)
        mgr._protocol = ControllerProtocol.load(MAC5000)
        mgr.simulate = False
        mgr._detected_controller = None
        mgr._exclude_ports = {str(p).upper() for p in exclude}
        orig = xymod.serial
        xymod.serial = _FakeSerial
        try:
            mgr._find_with_protocol(mgr._protocol)
        finally:
            xymod.serial = orig
        return opened

    def test_excluded_port_never_opened(self):
        opened = self._run_scan({"COM4"})
        self.assertNotIn("COM4", opened)   # ZP/Marlin port never opened
        self.assertIn("COM5", opened)      # the real XY port still tried

    def test_no_exclude_opens_all(self):
        opened = self._run_scan(set())
        self.assertIn("COM4", opened)
        self.assertIn("COM5", opened)


class TestJogPulseFallback(unittest.TestCase):
    """Xbox-stick jog on a controller with no continuous-velocity command
    (Ludl HLC: set_velocity is null) must fall back to pulsed relative moves
    instead of silently doing nothing."""

    def test_ludl_velocity_delegates_to_jog_pulse(self):
        mgr, sent = _mgr(MAC5000)
        calls = []
        mgr._jog_pulse = lambda vx, vy: calls.append((vx, vy))
        mgr.move_stage_at_velocity(1234.0, -56.0)
        self.assertEqual(calls, [(1234.0, -56.0)])
        self.assertEqual(sent, [])  # never reaches the (unsupported) set_velocity path

    def test_prior_velocity_does_not_use_jog_pulse(self):
        mgr, sent = _mgr(PROSCAN_III)
        pulse_calls = []
        mgr._jog_pulse = lambda vx, vy: pulse_calls.append((vx, vy))
        mgr.move_stage_at_velocity(1234.0, -56.0)
        self.assertEqual(pulse_calls, [])
        self.assertEqual(sent, [("set_velocity", {"vx": 1234.0, "vy": -56.0})])

    def test_pulse_accumulates_and_fires_move_relative(self):
        mgr, _ = _mgr(MAC5000)
        moves = []
        mgr.move_stage_relative = lambda dx, dy: moves.append((dx, dy))
        t = [100.0]
        import sys
        # NB: SupportClasses/__init__.py rebinds `XYStage` to the class — fetch
        # the real module from sys.modules (see TestExcludePorts._run_scan).
        xymod = sys.modules["SupportClasses.XYStage"]
        orig_monotonic = xymod.time.monotonic
        xymod.time.monotonic = lambda: t[0]
        try:
            mgr._jog_pulse(2000.0, 0.0)   # first pulse: seeds the timer, no move yet
            self.assertEqual(moves, [])
            t[0] += 0.1                  # 100 ms later, holding the same velocity
            mgr._jog_pulse(2000.0, 0.0)   # accumulated 2000*0.1 = 200 µm >> 1 count threshold
            self.assertEqual(len(moves), 1)
            self.assertAlmostEqual(moves[0][0], 200.0, places=3)
            self.assertAlmostEqual(moves[0][1], 0.0, places=3)
        finally:
            xymod.time.monotonic = orig_monotonic

    def test_pulse_below_threshold_does_not_fire_yet(self):
        mgr, _ = _mgr(MAC5000)
        moves = []
        mgr.move_stage_relative = lambda dx, dy: moves.append((dx, dy))
        t = [100.0]
        import sys
        # NB: SupportClasses/__init__.py rebinds `XYStage` to the class — fetch
        # the real module from sys.modules (see TestExcludePorts._run_scan).
        xymod = sys.modules["SupportClasses.XYStage"]
        orig_monotonic = xymod.time.monotonic
        xymod.time.monotonic = lambda: t[0]
        try:
            mgr._jog_pulse(1.0, 0.0)   # seed
            t[0] += 0.1
            mgr._jog_pulse(1.0, 0.0)   # 1 um/s * 0.1s = 0.1 um << 1 count (0.1 um/count) threshold
            self.assertEqual(moves, [])  # too small to fire yet — accumulator keeps it
        finally:
            xymod.time.monotonic = orig_monotonic

    def test_stop_resets_accumulator(self):
        mgr, _ = _mgr(MAC5000)
        mgr.move_stage_relative = lambda dx, dy: None
        mgr._jog_pulse(1.0, 0.0)
        mgr._jog_pulse(0.0, 0.0)   # explicit stop
        self.assertIsNone(mgr._jog_pulse_last_t)
        self.assertEqual(mgr._jog_pulse_accum_x, 0.0)
        self.assertEqual(mgr._jog_pulse_accum_y, 0.0)


class TestReadback(unittest.TestCase):
    """Speed/acceleration read-back — added after a real-hardware incident
    where a bad command corrupted ONE axis's SPEED register on a Ludl MAC 5000
    while the other stayed normal, with no way to see the divergence from the
    app. Covers both families and the real reply quirks seen on the bench
    (Ludl single-axis '?' queries append trailing junk for the OTHER axis,
    e.g. ':A 20 N-2'; ':N' is an explicit error)."""

    def test_ludl_speed_readback_absolute(self):
        mgr, _ = _mgr(MAC5000)
        mgr._query_protocol_value = lambda name, axis="X", timeout=0.5: ":A 84 N-2"
        r = mgr.get_speed_readback("X")
        self.assertEqual(r["raw"], 84)
        self.assertEqual(r["model"], "absolute")
        self.assertIn("84", r["display"])
        self.assertIn("8", r["display"])  # 84/10 = 8.4 um/s, rounds to 8

    def test_ludl_accel_readback_absolute(self):
        mgr, _ = _mgr(MAC5000)
        mgr._query_protocol_value = lambda name, axis="X", timeout=0.5: ":A 255 N-2"
        r = mgr.get_acceleration_readback("X")
        self.assertEqual(r["raw"], 255)
        self.assertEqual(r["model"], "absolute")
        self.assertIn("LOWER = faster", r["display"])

    def test_ludl_error_reply_is_unavailable(self):
        mgr, _ = _mgr(MAC5000)
        mgr._query_protocol_value = lambda name, axis="X", timeout=0.5: ":N -1"
        r = mgr.get_speed_readback("X")
        self.assertIsNone(r["raw"])
        self.assertEqual(r["display"], "unavailable")

    def test_ludl_empty_reply_is_unavailable(self):
        mgr, _ = _mgr(MAC5000)
        mgr._query_protocol_value = lambda name, axis="X", timeout=0.5: ""
        r = mgr.get_acceleration_readback("X")
        self.assertIsNone(r["raw"])
        self.assertEqual(r["display"], "unavailable")

    def test_prior_speed_readback_percentage(self):
        mgr, _ = _mgr(PROSCAN_III)
        mgr._query_protocol_value = lambda name, axis="X", timeout=0.5: "37"
        r = mgr.get_speed_readback("X")
        self.assertEqual(r["raw"], 37)
        self.assertEqual(r["model"], "percentage")
        self.assertEqual(r["display"], "37%")

    def test_prior_accel_readback_percentage(self):
        mgr, _ = _mgr(PROSCAN_III)
        mgr._query_protocol_value = lambda name, axis="X", timeout=0.5: "65"
        r = mgr.get_acceleration_readback("X")
        self.assertEqual(r["raw"], 65)
        self.assertEqual(r["display"], "65%")

    def test_query_protocol_value_returns_none_when_unsupported(self):
        # set_jerk-style null command -> format_command returns None -> no I/O attempted.
        mgr, _ = _mgr(PROSCAN_II)  # ProScan II has no get_max_speed defined? still has it via our edit
        # Force an explicitly-unsupported command name to prove the None short-circuit.
        self.assertIsNone(mgr._query_protocol_value("nonexistent_command", axis="X"))

    def test_extract_first_int_tolerates_trailing_garbage(self):
        self.assertEqual(XYStageManager._extract_first_int(":A 20 N-2"), 20)
        self.assertEqual(XYStageManager._extract_first_int(":A -5"), -5)
        self.assertIsNone(XYStageManager._extract_first_int(":N -1"))
        self.assertIsNone(XYStageManager._extract_first_int(""))
        self.assertIsNone(XYStageManager._extract_first_int(None))


class TestAccelModel(unittest.TestCase):
    def test_ludl_accel_clamps_to_255(self):
        mgr, sent = _mgr(MAC5000)
        mgr.set_acceleration(999)   # clamps to protocol acceleration_range max 255
        name, kw = sent[-1]
        self.assertEqual(name, "set_acceleration")
        self.assertEqual(kw["accel"], 255)

    def test_prior_accel_clamps_to_100(self):
        mgr, sent = _mgr(PROSCAN_III)
        mgr.set_acceleration(999)
        _, kw = sent[-1]
        self.assertEqual(kw["accel"], 100)


if __name__ == "__main__":
    unittest.main()
