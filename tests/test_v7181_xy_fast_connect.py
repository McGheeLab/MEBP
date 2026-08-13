"""test_v7181_xy_fast_connect.py — XY connect stops re-scanning every time.

THE DEFECT (measured on ME3B_01, from the operator's own logs/app.log):

    16:22:42.33  Auto-detecting controller from 4 protocol files
    16:22:42.33  Trying COM3 @ 9600   (Ludl MAC 5000)          -> miss
    16:22:43.03  Trying COM3 @ 38400  (Prior ProScan II)       -> miss
    16:22:43.92  Trying COM3 @ 38400  (Prior ProScan III)      -> miss
    16:22:45.25  Trying COM3 @ 115200 (H117)                   -> miss
    16:22:46.76  Trying COM3 @ 38400  (H117)                   -> miss
    16:22:48.27  Trying COM3 @ 9600   (H117)                   -> MATCH
    16:22:49.09  XY stage connected (REAL)                     = 6.76 s

Only ONE candidate port even exists; the entire 6.76 s is spent re-proving
five protocol/baud combinations that were already ruled out on the previous
run — on the GUI thread, so the freeze watchdog fired on every connect.

The fix is the XY twin of the v7.4.2 ZP ``preferred_port`` hotfix: persist
the winning (protocol, port, baud) and probe THAT first. A miss falls
through to the full scan, so a moved cable or swapped controller still
self-heals — the hint can only make a connect faster, never fail one.

Also covers the trailing-reply drain: detection matches on the FIRST line of
a multi-line answer (the H117's ``STAGE`` reply is several lines), and the
rest was still arriving when the first position read ran — the source of
``Failed to parse XY position: Expected 3 values, got 1: D``.
"""

import sys
import unittest

from SupportClasses.ControllerProtocol import ControllerProtocol
from SupportClasses.XYStage import XYStageManager

H117 = "config/controllers/proscan_iii_h117.json"
PROSCAN_III = "config/controllers/proscan_iii.json"

# What the real unit answers the STAGE query with (first line matches the
# 'STAGE =' identify token; the rest keeps arriving afterwards).
STAGE_REPLY = b"STAGE = H117P1N4/F\rTYPE = 25\rMICROSTEPS = 250\rDONE\r"


class _Conn:
    """Minimal pyserial stand-in that answers only at one (port, baud)."""

    def __init__(self, port, baudrate=None, **kw):
        self.port = port
        self.timeout = 0.5
        self.dtr = False
        self.closed = False
        self._baud = baudrate
        self._buf = b""
        self._live = (port, baudrate) in _Conn.answers_at
        _Conn.opened.append((port, baudrate))

    # class-level probe recording
    opened: list = []
    answers_at: set = set()

    @classmethod
    def reset(cls, answers_at):
        cls.opened = []
        cls.answers_at = set(answers_at)

    def reset_input_buffer(self):
        self._buf = b""

    def reset_output_buffer(self):
        pass

    def write(self, b):
        if self._live:
            self._buf += STAGE_REPLY

    def read(self, n=1):
        if not self._buf:
            return b""
        out, self._buf = self._buf[:n], self._buf[n:]
        return out

    def close(self):
        self.closed = True


def _install_fake_serial(devices):
    """Patch SupportClasses.XYStage.serial; returns (module, original)."""
    class _PortInfo:
        def __init__(self, dev):
            self.device = dev
            self.description = dev
            self.hwid = dev

    class _ListPorts:
        @staticmethod
        def comports():
            return [_PortInfo(d) for d in devices]

    class _Tools:
        list_ports = _ListPorts

    class _FakeSerial:
        STOPBITS_ONE = 1
        STOPBITS_TWO = 2
        SerialException = Exception
        Serial = _Conn
        tools = _Tools

    xymod = sys.modules["SupportClasses.XYStage"]
    orig = xymod.serial
    xymod.serial = _FakeSerial
    return xymod, orig


def _manager(preferred=None, protocol=None, exclude=()):
    """A partial manager (no __init__ → no real serial) for scan tests."""
    mgr = XYStageManager.__new__(XYStageManager)
    mgr.simulate = False
    mgr._protocol = protocol
    mgr._detected_controller = None
    mgr._exclude_ports = {str(p).upper() for p in exclude}
    mgr._preferred = dict(preferred) if preferred else None
    mgr._connected_port = None
    mgr._connected_baud = None
    return mgr


class TestFastConnectSkipsTheScan(unittest.TestCase):
    """The load-bearing one: a good hint must cost exactly ONE port open."""

    def test_hit_opens_only_the_hinted_port_and_baud(self):
        _Conn.reset(answers_at={("COM3", 9600)})
        xymod, orig = _install_fake_serial(["COM3", "COM7", "COM9"])
        try:
            mgr = _manager(preferred={"protocol": H117,
                                      "port": "COM3", "baud": 9600})
            spo = mgr._find_controller()
        finally:
            xymod.serial = orig

        self.assertIsNotNone(spo, "the hinted probe should have connected")
        # The whole point: no other port, no other baud, no other protocol.
        self.assertEqual(_Conn.opened, [("COM3", 9600)])
        self.assertEqual(mgr.connected_port, "COM3")
        self.assertEqual(mgr.connected_baud, 9600)

    def test_without_a_hint_the_full_sweep_still_runs(self):
        """No hint ⇒ pre-v7.18.1 behaviour, unchanged."""
        _Conn.reset(answers_at={("COM3", 9600)})
        xymod, orig = _install_fake_serial(["COM3"])
        try:
            mgr = _manager()  # no hint, no protocol -> auto-detect
            spo = mgr._find_controller()
        finally:
            xymod.serial = orig

        self.assertIsNotNone(spo)
        # Every protocol/baud combination ahead of the winner is still probed.
        self.assertGreater(len(_Conn.opened), 1)
        self.assertEqual(_Conn.opened[-1], ("COM3", 9600))

    def test_the_naive_answer_would_have_been_slow(self):
        """Guard the guard: prove the un-hinted sweep really is the costly
        path, so the one-open assertion above cannot pass vacuously."""
        _Conn.reset(answers_at={("COM3", 9600)})
        xymod, orig = _install_fake_serial(["COM3"])
        try:
            _manager()._find_controller()
            unhinted = len(_Conn.opened)
        finally:
            xymod.serial = orig
        self.assertGreaterEqual(
            unhinted, 5,
            "the un-hinted scan must probe several combinations, else this "
            "suite proves nothing about the hint saving work")


class TestAMissAlwaysFallsBack(unittest.TestCase):
    """A hint can make a connect faster; it must never make one fail."""

    def test_wrong_port_in_hint_still_connects_via_the_scan(self):
        _Conn.reset(answers_at={("COM7", 9600)})       # really on COM7
        xymod, orig = _install_fake_serial(["COM3", "COM7"])
        try:
            mgr = _manager(preferred={"protocol": H117,
                                      "port": "COM3", "baud": 9600})
            spo = mgr._find_controller()
        finally:
            xymod.serial = orig

        self.assertIsNotNone(spo)
        self.assertEqual(mgr.connected_port, "COM7")
        self.assertEqual(mgr.connected_baud, 9600)

    def test_hinted_port_vanished_falls_back(self):
        _Conn.reset(answers_at={("COM7", 9600)})
        xymod, orig = _install_fake_serial(["COM7"])   # COM3 unplugged
        try:
            mgr = _manager(preferred={"protocol": H117,
                                      "port": "COM3", "baud": 9600})
            spo = mgr._find_controller()
        finally:
            xymod.serial = orig
        self.assertIsNotNone(spo)
        self.assertEqual(mgr.connected_port, "COM7")

    def test_baud_no_longer_offered_by_the_protocol_falls_back(self):
        """Prior's documented baud reversion: the stored baud may become one
        the protocol no longer lists. Must miss cleanly, not crash."""
        _Conn.reset(answers_at={("COM3", 9600)})
        xymod, orig = _install_fake_serial(["COM3"])
        try:
            mgr = _manager(preferred={"protocol": H117,
                                      "port": "COM3", "baud": 4800})
            spo = mgr._find_controller()
        finally:
            xymod.serial = orig
        self.assertIsNotNone(spo)
        self.assertEqual(mgr.connected_baud, 9600)

    def test_unloadable_protocol_path_falls_back(self):
        _Conn.reset(answers_at={("COM3", 9600)})
        xymod, orig = _install_fake_serial(["COM3"])
        try:
            mgr = _manager(preferred={"protocol": "config/controllers/nope.json",
                                      "port": "COM3", "baud": 9600})
            spo = mgr._find_controller()
        finally:
            xymod.serial = orig
        self.assertIsNotNone(spo)

    def test_full_scan_tries_the_hinted_port_first(self):
        """Even when the BAUD hint is stale, the port hint still saves the
        other ports' probe time."""
        _Conn.reset(answers_at={("COM9", 9600)})
        xymod, orig = _install_fake_serial(["COM3", "COM7", "COM9"])
        try:
            mgr = _manager(protocol=ControllerProtocol.load(H117),
                           preferred={"protocol": H117,
                                      "port": "COM9", "baud": 4800})
            mgr._find_controller()
        finally:
            xymod.serial = orig
        self.assertEqual(_Conn.opened[0][0], "COM9")


class TestExcludedPortIsNeverOpened(unittest.TestCase):
    """Opening a port DTR-resets whatever board is behind it — a stale hint
    pointing at the Marlin/ZP port must not reboot it mid-session."""

    def test_hint_at_an_excluded_port_opens_nothing(self):
        _Conn.reset(answers_at={("COM4", 9600)})
        xymod, orig = _install_fake_serial(["COM4"])
        try:
            mgr = _manager(preferred={"protocol": H117,
                                      "port": "COM4", "baud": 9600},
                           exclude=["COM4"])
            spo = mgr._find_controller()
        finally:
            xymod.serial = orig
        self.assertIsNone(spo)
        self.assertEqual(_Conn.opened, [],
                         "the excluded port must have ZERO opens — the open "
                         "IS the reset")


class TestDetectionHint(unittest.TestCase):
    """What gets persisted must describe something that really answered."""

    def test_hint_is_the_winning_triple(self):
        _Conn.reset(answers_at={("COM3", 9600)})
        xymod, orig = _install_fake_serial(["COM3"])
        try:
            mgr = _manager()
            mgr._find_controller()
        finally:
            xymod.serial = orig
        hint = mgr.detection_hint
        self.assertEqual(hint["port"], "COM3")
        self.assertEqual(hint["baud"], 9600)
        self.assertTrue(hint["protocol"].endswith("proscan_iii_h117.json"))

    def test_hint_round_trips_into_a_single_probe(self):
        """The persisted hint must be exactly what the fast path consumes."""
        _Conn.reset(answers_at={("COM3", 9600)})
        xymod, orig = _install_fake_serial(["COM3"])
        try:
            first = _manager()
            first._find_controller()
            hint = first.detection_hint
            _Conn.reset(answers_at={("COM3", 9600)})
            second = _manager(preferred=hint)
            self.assertIsNotNone(second._find_controller())
        finally:
            xymod.serial = orig
        self.assertEqual(_Conn.opened, [("COM3", 9600)])

    def test_no_hint_when_nothing_was_found(self):
        _Conn.reset(answers_at=set())
        xymod, orig = _install_fake_serial(["COM3"])
        try:
            mgr = _manager()
            self.assertIsNone(mgr._find_controller())
        finally:
            xymod.serial = orig
        self.assertIsNone(mgr.detection_hint,
                          "never persist a hint nothing answered on")

    def test_no_hint_when_an_explicit_protocol_found_nothing(self):
        """The case a protocol-is-None check alone would miss: with an
        explicit controller_json the protocol is loaded BEFORE detection, so
        a failed scan would otherwise publish {port: None, baud: None} and
        send the next launch at nothing."""
        _Conn.reset(answers_at=set())
        xymod, orig = _install_fake_serial(["COM3"])
        try:
            mgr = _manager(protocol=ControllerProtocol.load(H117))
            self.assertIsNone(mgr._find_controller())
        finally:
            xymod.serial = orig
        self.assertIsNotNone(mgr._protocol, "guard-the-guard: a protocol IS "
                                            "loaded, so this test would pass "
                                            "vacuously without it")
        self.assertIsNone(mgr.detection_hint)

    def test_no_hint_in_simulation(self):
        mgr = XYStageManager(simulate=True)
        try:
            self.assertIsNone(mgr.detection_hint)
        finally:
            mgr.stop()


class TestTrailingReplyIsDrained(unittest.TestCase):
    """The 'Expected 3 values, got 1: D' warning: detection matched the first
    line of a multi-line reply and left the rest on the wire."""

    def test_port_is_quiet_after_detection(self):
        _Conn.reset(answers_at={("COM3", 9600)})
        xymod, orig = _install_fake_serial(["COM3"])
        try:
            mgr = _manager(preferred={"protocol": H117,
                                      "port": "COM3", "baud": 9600})
            spo = mgr._find_controller()
        finally:
            xymod.serial = orig
        self.assertEqual(
            spo.read(64), b"",
            "the rest of the STAGE descriptor must be drained before the "
            "first position read sees it")

    def test_session_timeout_is_the_protocol_timeout(self):
        _Conn.reset(answers_at={("COM3", 9600)})
        xymod, orig = _install_fake_serial(["COM3"])
        try:
            mgr = _manager(preferred={"protocol": H117,
                                      "port": "COM3", "baud": 9600})
            spo = mgr._find_controller()
        finally:
            xymod.serial = orig
        self.assertEqual(spo.timeout,
                         ControllerProtocol.load(H117).timeout)


class TestControllerPlumbing(unittest.TestCase):
    """StageController must hand the hint down and take the result back."""

    def _controller(self):
        from SupportClasses.StageController import StageController
        return StageController.__new__(StageController)

    def test_set_preferred_xy_hint_copies(self):
        c = self._controller()
        c._preferred_xy_hint = None
        src = {"protocol": H117, "port": "COM3", "baud": 9600}
        c.set_preferred_xy_hint(src)
        src["port"] = "COM99"
        self.assertEqual(c._preferred_xy_hint["port"], "COM3")

    def test_set_preferred_xy_hint_rejects_junk(self):
        c = self._controller()
        c._preferred_xy_hint = {"port": "COM3"}
        c.set_preferred_xy_hint(None)
        self.assertIsNone(c._preferred_xy_hint)
        c.set_preferred_xy_hint("COM3")          # not a dict
        self.assertIsNone(c._preferred_xy_hint)

    def test_hint_is_none_while_simulating(self):
        # ``simulate_xy`` is derived from the live stage's own .simulate.
        class _SimStage:
            simulate = True
            detection_hint = None

        c = self._controller()
        c.xy_stage = _SimStage()
        self.assertIsNone(c.xy_connection_hint)

    def test_hint_reads_through_to_the_stage(self):
        class _Stage:
            simulate = False
            detection_hint = {"protocol": H117, "port": "COM3", "baud": 9600}

        c = self._controller()
        c.xy_stage = _Stage()
        self.assertEqual(c.xy_connection_hint["port"], "COM3")

    def test_connect_passes_the_hint_to_the_manager(self):
        """AST pin: the hint must actually reach XYStageManager — asserting
        on the attribute alone would pass with the kwarg deleted."""
        import ast
        import inspect

        scmod = sys.modules["SupportClasses.StageController"]
        src = inspect.getsource(scmod.StageController.connect_stages)
        tree = ast.parse(src.lstrip())
        found = [
            kw.arg
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and getattr(node.func, "id", "") == "XYStageManager"
            for kw in node.keywords
        ]
        self.assertIn("preferred", found,
                      "connect_stages must pass preferred= to XYStageManager")


if __name__ == "__main__":
    unittest.main()
