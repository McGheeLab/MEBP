"""
test_v718_incubator_shared_link.py — the shared-ZP transport, both halves:

  * ``ZPStageManager.transact`` / ``include_terminal_line`` /
    ``priority_write`` / ``exclude_ports`` (the motion-side hooks), driven
    through the REAL ``_read_until_ok`` against a scripted serial fake;
  * ``ZPSharedLink`` + ``IncubatorController.connect_shared`` against a fake
    ZP manager that answers like a Marlin board, including the refusal gates
    (autotune, board-side waits, simulated ZP) whose absence would starve the
    position poller into a false "ZP disconnected" on real hardware.

The single most load-bearing test is
``test_m105_data_would_be_LOST_without_the_terminal_line`` — it proves the
naive path (plain ``_txn``) drops the temperatures entirely, so the
invariance of everything downstream cannot be passing by accident.
"""

from __future__ import annotations

import os
import tempfile
import threading
import time
import unittest


def _isolate_env():
    td = tempfile.mkdtemp(prefix="incu_shared_")
    os.environ["MEBP_INCUBATOR_CONFIG_DIR"] = td
    os.environ["MEBP_INCUBATOR_CAL_DIR"] = td
    os.environ["MEBP_INCUBATOR_SIM_DIR"] = td
    os.environ["MEBP_INCUBATOR_LOG"] = "0"
    return td


# ═══════════════════════════════════════════════════════════════════
# Scripted serial fake for the REAL ZPStage read loop
# ═══════════════════════════════════════════════════════════════════

class _ScriptedSerial:
    """Feeds a scripted list of reply lines to ``readline``."""

    def __init__(self, replies):
        self._replies = list(replies)
        self.written = []
        self.is_open = True

    def write(self, data):
        self.written.append(data)

    def flush(self):
        pass

    def readline(self):
        if self._replies:
            return self._replies.pop(0)
        return b""

    def close(self):
        self.is_open = False


def _partial_zp(replies):
    """A ZPStageManager via ``__new__`` carrying only what the code under
    test touches — the repo's partial-object pattern."""
    from SupportClasses.ZPStage import ZPStageManager
    zp = ZPStageManager.__new__(ZPStageManager)
    zp._serial_lock = threading.RLock()
    zp.simulate = False
    zp.serial = _ScriptedSerial(replies)
    zp._board_reset_detected = False
    zp.emergency_parser = True
    zp.cmd_count = zp.ok_count = zp.ok_fail_count = zp.reset_count = 0
    return zp


class TestZPStageTransact(unittest.TestCase):

    def test_m105_data_would_be_LOST_without_the_terminal_line(self):
        """Marlin answers M105 with the data ON the ok line. The plain
        transaction discards it — which is exactly why include_terminal_line
        exists. Both halves asserted so neither can regress silently."""
        line = b"ok T:23.10 /0.00 B:24.20 /37.00 @:0 B@:64\n"
        zp = _partial_zp([line])
        ok, text, outcome = zp.transact("M105", ok_timeout=1.0)
        self.assertTrue(ok)
        self.assertEqual("ok", outcome)
        self.assertEqual("", text)          # the naive path loses the temps

        zp2 = _partial_zp([line])
        ok2, text2, _ = zp2.transact("M105", ok_timeout=1.0,
                                     include_terminal_line=True)
        self.assertTrue(ok2)
        self.assertIn("B:24.20", text2)
        from SupportClasses.incubator.marlin_gcode import parse_temp_line
        frame = parse_temp_line(text2)
        self.assertIsNotNone(frame)
        self.assertAlmostEqual(24.20, frame.fields["B"].value)

    def test_m115_reply_needs_allow_identity_lines(self):
        """The bug this flag exists for: an M115 reply contains
        FIRMWARE_NAME/Marlin, which the DEFAULT reset classifier reads as a
        boot banner — failing the probe AND poisoning
        _board_reset_detected. Both halves pinned."""
        replies = [b"FIRMWARE_NAME:Marlin 2.1\n", b"Cap:EEPROM:1\n", b"ok\n"]
        zp = _partial_zp(list(replies))
        ok, _text, outcome = zp.transact("M115", ok_timeout=1.0)
        self.assertFalse(ok)
        self.assertEqual("reset", outcome)
        self.assertTrue(zp._board_reset_detected)

        zp2 = _partial_zp(list(replies))
        ok2, text2, outcome2 = zp2.transact(
            "M115", ok_timeout=1.0, include_terminal_line=True,
            allow_identity_lines=True)
        self.assertTrue(ok2)
        self.assertEqual("ok", outcome2)
        self.assertFalse(zp2._board_reset_detected)
        lines = text2.splitlines()
        self.assertEqual(3, len(lines))
        self.assertEqual("ok", lines[-1])

    def test_a_genuine_reset_is_still_caught_during_an_identity_txn(self):
        """allow_identity_lines narrows, it does not disable: a real boot
        banner leads with a literal 'start' line."""
        zp = _partial_zp([b"start\n", b"Marlin 2.0.x\n"])
        ok, _text, outcome = zp.transact(
            "M115", ok_timeout=1.0, allow_identity_lines=True)
        self.assertFalse(ok)
        self.assertEqual("reset", outcome)
        self.assertTrue(zp._board_reset_detected)

    def test_error_outcome_carries_the_error_line(self):
        zp = _partial_zp([b"Error:checksum mismatch\n"])
        ok, text, outcome = zp.transact("M104 S37", ok_timeout=1.0,
                                        include_terminal_line=True)
        self.assertFalse(ok)
        self.assertEqual("error", outcome)
        self.assertIn("Error:checksum", text)

    def test_default_behaviour_is_byte_identical(self):
        """Every EXISTING caller goes through _txn with no new kwarg — the
        terminal line must stay excluded there."""
        zp = _partial_zp([b"X:1.0 Y:2.0 Z:3.0 E:0.0\n", b"ok\n"])
        ok, text = zp._txn("M114", ok_timeout=1.0, collect=True)
        self.assertTrue(ok)
        self.assertEqual("X:1.0 Y:2.0 Z:3.0 E:0.0", text)

    def test_simulated_and_disconnected_refuse(self):
        zp = _partial_zp([])
        zp.simulate = True
        self.assertEqual((False, "", "simulated"),
                         zp.transact("M105", ok_timeout=1.0))
        zp2 = _partial_zp([])
        zp2.simulate = False
        zp2.serial = None
        self.assertEqual((False, "", "not_connected"),
                         zp2.transact("M105", ok_timeout=1.0))

    def test_priority_write_writes_raw_when_lock_contended(self):
        zp = _partial_zp([])
        holder_ready = threading.Event()
        release = threading.Event()

        def hold():
            with zp._serial_lock:
                holder_ready.set()
                release.wait(timeout=5.0)

        t = threading.Thread(target=hold, daemon=True)
        t.start()
        self.assertTrue(holder_ready.wait(timeout=2.0))
        try:
            self.assertTrue(zp.priority_write("M112", lock_timeout_s=0.05))
            # Raw path: leading newline terminates any partial line.
            self.assertEqual(b"\nM112\n", zp.serial.written[-1])
        finally:
            release.set()
            t.join(timeout=2.0)

    def test_priority_write_under_lock_when_free(self):
        zp = _partial_zp([])
        self.assertTrue(zp.priority_write("M108"))
        self.assertEqual(b"M108\n", zp.serial.written[-1])

    def test_zp_scan_skips_excluded_ports(self):
        """exclude_ports must filter _initialise_serial BEFORE any open —
        a probe open DTR-resets the board behind the port."""
        import sys
        from SupportClasses.ZPStage import ZPStageManager

        class _P:
            def __init__(self, device):
                self.device = device
                self.description = "USB Serial Device"
                self.hwid = "USB VID:PID=0483:5740"

        class _FakeListPorts:
            @staticmethod
            def comports():
                return [_P("COM6"), _P("COM7")]

        zp = ZPStageManager.__new__(ZPStageManager)
        zp.preferred_port = None
        zp.connected_port = None
        zp._exclude_ports = {"COM6"}
        tried = []
        zp._try_open_marlin = lambda dev: tried.append(dev) or None

        import importlib
        # NOTE not `import SupportClasses.ZPStage as zpmod` — the package
        # __init__ rebinds the `ZPStage` attribute, so that form yields the
        # CLASS, not the module.
        zpmod = importlib.import_module("SupportClasses.ZPStage")
        real_serial = zpmod.serial

        class _SerialShim:
            class tools:
                list_ports = _FakeListPorts
        zpmod.serial = _SerialShim
        try:
            result = ZPStageManager._initialise_serial(zp)
        finally:
            zpmod.serial = real_serial
        self.assertIsNone(result)
        self.assertNotIn("COM6", tried)
        self.assertIn("COM7", tried)

    def test_ctor_accepts_and_normalises_exclusions(self):
        from SupportClasses.ZPStage import ZPStageManager
        zp = ZPStageManager(simulate=True, exclude_ports=["com6", None, ""])
        try:
            self.assertEqual({"COM6"}, zp._exclude_ports)
        finally:
            zp.stop()


# ═══════════════════════════════════════════════════════════════════
# A fake ZP manager that answers like a Marlin board
# ═══════════════════════════════════════════════════════════════════

class _FakeMarlinZP:
    """Just enough ZPStageManager surface for the shared link: transact,
    priority_write, and the attributes _usable_zp() inspects."""

    def __init__(self):
        self.simulate = False
        self.serial = object()
        self.connected_port = "COM6"
        self.commands: list[str] = []
        self.priority: list[str] = []
        self.timeouts: list[float] = []
        self.bed_target = 0.0

    def transact(self, command, *, ok_timeout=6.0, collect=True,
                 include_terminal_line=False, allow_identity_lines=False):
        self.commands.append(command)
        self.timeouts.append(ok_timeout)
        head = command.split()[0].upper()
        if head == "M115":
            body = ["FIRMWARE_NAME:Marlin 2.1 (FakeZP)",
                    "Cap:EEPROM:1", "Cap:AUTOREPORT_TEMP:1",
                    "Cap:EMERGENCY_PARSER:1"]
            term = "ok"
        elif head == "M105":
            term = (f"ok T:23.00 /0.00 B:24.00 /{self.bed_target:.2f} "
                    f"@:0 B@:0")
            body = []
        elif head == "M503":
            body = ["echo:  M301 P21.73 I1.54 D76.55",
                    "echo:  M304 P41.78 I7.32 D158.93"]
            term = "ok"
        elif head in ("M140", "M104"):
            for part in command.split():
                if part.startswith("S"):
                    self.bed_target = float(part[1:])
            body = []
            term = "ok"
        else:
            body = []
            term = "ok"
        lines = body + ([term] if include_terminal_line else [])
        return (True, "\n".join(lines), "ok")

    def priority_write(self, command):
        self.priority.append(command)
        return True


class TestZPSharedLink(unittest.TestCase):

    def _link(self, zp, **kw):
        from SupportClasses.incubator.zp_shared_link import ZPSharedLink
        return ZPSharedLink(lambda: zp, **kw)

    def test_m105_lines_reach_on_line(self):
        zp = _FakeMarlinZP()
        seen = []
        link = self._link(zp, on_line=lambda text, kind: seen.append(text))
        txn = link.send_and_wait("M105")
        self.assertTrue(txn.ok)
        self.assertTrue(any("B:24.00" in ln for ln in seen))
        self.assertTrue(any("B:24.00" in ln for ln in txn.lines))

    def test_refuses_when_zp_missing(self):
        link = self._link(None)
        txn = link.send_and_wait("M105")
        self.assertFalse(txn.ok)
        self.assertIn("not available", txn.error_text)

    def test_refuses_a_simulated_zp(self):
        zp = _FakeMarlinZP()
        zp.simulate = True
        link = self._link(zp)
        self.assertFalse(link.is_open())
        self.assertFalse(link.send_and_wait("M105").ok)

    def test_timeout_clamped_to_shared_maximum(self):
        """Defence-in-depth: no shared transaction may hold the motion lock
        for minutes, whatever a future caller asks for."""
        from SupportClasses.incubator.zp_shared_link import (
            MAX_SHARED_TIMEOUT_S,
        )
        zp = _FakeMarlinZP()
        link = self._link(zp)
        link.send_and_wait("M105", timeout_s=600.0)
        self.assertLessEqual(zp.timeouts[-1], MAX_SHARED_TIMEOUT_S)

    def test_close_never_touches_the_port(self):
        zp = _FakeMarlinZP()
        link = self._link(zp)
        link.close()   # would raise if it tried zp.serial.close()
        self.assertFalse(link.is_open())

    def test_priority_routes_to_priority_write(self):
        zp = _FakeMarlinZP()
        link = self._link(zp)
        self.assertTrue(link.send_priority("M112"))
        self.assertEqual(["M112"], zp.priority)

    def test_poll_gate(self):
        zp = _FakeMarlinZP()
        gate = {"open": True}
        link = self._link(zp, poll_gate=lambda: gate["open"])
        self.assertTrue(link.poll_allowed())
        gate["open"] = False
        self.assertFalse(link.poll_allowed())

    def test_disconnect_flagged_once(self):
        calls = []
        link = self._link(None, on_disconnect=lambda: calls.append(1))
        link.send_and_wait("M105")
        link.send_and_wait("M105")
        self.assertEqual(1, len(calls))


class TestControllerSharedTransport(unittest.TestCase):
    """connect_shared through the PRODUCTION controller + probe."""

    def setUp(self):
        _isolate_env()
        from SupportClasses.incubator.controller import IncubatorController
        self.zp = _FakeMarlinZP()
        self.ctrl = IncubatorController()
        self.msgs: list[str] = []
        self.ctrl.on_status(self.msgs.append)
        self.assertTrue(self.ctrl.connect_shared(lambda: self.zp))

    def tearDown(self):
        try:
            self.ctrl.disconnect()
        except Exception:
            pass

    def test_transport_and_probe(self):
        self.assertEqual("shared", self.ctrl.transport)
        self.assertTrue(self.ctrl.shared_transport)
        self.assertIn("FakeZP", self.ctrl.report.firmware_name)
        self.assertIn("COM6", self.ctrl.active_port)

    def test_autoreport_disabled_on_the_wire(self):
        """A previous dedicated session may have left M155 running —
        unsolicited pushes would interleave with M114/M400 parsing."""
        self.assertIn("M155 S0", self.zp.commands)

    def test_polling_forced(self):
        self.assertTrue(self.ctrl._use_polling)

    def test_set_target_flows_through_transact(self):
        self.ctrl.set_target("bed", 37.0)
        # Wait for THE setpoint command — the firmware probe issues its own
        # M140s (resolution probe + restore), so a bare startswith("M140")
        # break races them.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if any(c == "M140 S37" for c in self.zp.commands):
                break
            time.sleep(0.05)
        self.assertIn("M140 S37", self.zp.commands)

    def test_board_side_wait_degrades_to_non_blocking(self):
        """An M190 would hold the motion lock for the whole heat-up."""
        self.ctrl.set_target("bed", 37.0, board_side_wait=True)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if any(c.startswith("M140") for c in self.zp.commands):
                break
            time.sleep(0.05)
        sets = [c for c in self.zp.commands
                if c.startswith(("M140", "M190"))]
        self.assertTrue(sets)
        self.assertTrue(all(c.startswith("M140") for c in sets),
                        f"M190 leaked onto the shared link: {sets}")
        self.assertTrue(any("board-side waits are unavailable" in m
                            for m in self.msgs))

    def test_autotune_refused_with_the_remedy_named(self):
        self.assertFalse(self.ctrl.start_autotune("bed", 37.0))
        self.assertFalse(any(c.startswith("M303") for c in self.zp.commands))
        self.assertTrue(any("dedicated connection" in m for m in self.msgs))

    def test_poll_yields_while_gated(self):
        """With the gate closed (print in flight) no M105 poll is queued and
        the stale warning stays quiet."""
        self.ctrl._link._poll_gate = lambda: False
        submitted = []
        real_submit = self.ctrl.submit
        self.ctrl.submit = lambda fn, *a, **k: submitted.append(fn.__name__)
        try:
            self.assertFalse(self.ctrl._poll_permitted())
            self.ctrl._stale_warned = False
            self.ctrl._check_data_flowing([])
            self.assertNotIn("_do_poll_once", submitted)
            self.assertFalse(self.ctrl._stale_warned)
        finally:
            self.ctrl.submit = real_submit

    def test_disconnect_commands_heaters_off_but_not_the_port(self):
        # Snapshot first: the firmware probe's restore step legitimately
        # sends its own M140 S0 during connect.
        before = len(self.zp.commands)
        self.ctrl.disconnect()
        offs = [c for c in self.zp.commands[before:]
                if c in ("M140 S0", "M104 S0")]
        self.assertEqual(2, len(offs))
        self.assertEqual("", self.ctrl.transport)

    def test_emergency_stop_goes_out_of_band(self):
        self.ctrl.emergency_stop()
        self.assertIn("M112", self.zp.priority)


class TestConnectSharedRefusals(unittest.TestCase):

    def setUp(self):
        _isolate_env()

    def test_no_zp(self):
        from SupportClasses.incubator.controller import IncubatorController
        ctrl = IncubatorController()
        msgs = []
        ctrl.on_status(msgs.append)
        self.assertFalse(ctrl.connect_shared(lambda: None))
        self.assertFalse(ctrl.connected)
        self.assertTrue(any("not connected" in m for m in msgs))

    def test_simulated_zp_points_at_the_incubator_simulator(self):
        from SupportClasses.incubator.controller import IncubatorController
        zp = _FakeMarlinZP()
        zp.simulate = True
        ctrl = IncubatorController()
        msgs = []
        ctrl.on_status(msgs.append)
        self.assertFalse(ctrl.connect_shared(lambda: zp))
        self.assertTrue(any("SIMULATED" in m for m in msgs))


class TestStageControllerReservedPorts(unittest.TestCase):
    """StageController._incubator_reserved_ports — the ZP/XY scan guard."""

    def setUp(self):
        _isolate_env()
        from SupportClasses.incubator.config_store import reset_store
        from SupportClasses.incubator.service import reset_service
        reset_store()
        reset_service()

    def _sc(self, preferred=None):
        from SupportClasses.StageController import StageController
        sc = StageController.__new__(StageController)
        sc._preferred_zp_port = preferred
        return sc

    def test_saved_port_excluded_only_in_serial_transport(self):
        from SupportClasses.incubator.config_store import get_store
        st = get_store()
        st.set("dedicated_port", "COM9", save=False)
        st.set("transport", "shared")
        self.assertEqual([], self._sc()._incubator_reserved_ports())
        st.set("transport", "serial")
        self.assertEqual(["COM9"], self._sc()._incubator_reserved_ports())

    def test_zp_preferred_port_is_never_excluded(self):
        """A mistyped ZP port in the incubator config must not blind the ZP
        scan to its own board — the ZP wins its port, the incubator connect
        then fails with its own actionable message."""
        from SupportClasses.incubator.config_store import get_store
        st = get_store()
        st.set("dedicated_port", "COM6", save=False)
        st.set("transport", "serial")
        self.assertEqual([], self._sc("COM6")._incubator_reserved_ports())

    def test_live_serial_session_excluded(self):
        from SupportClasses.incubator.service import get_incubator
        ctrl = get_incubator()
        ctrl._connected = True
        ctrl._transport = "serial"
        ctrl.active_port = "COM8"
        try:
            self.assertIn("COM8", self._sc()._incubator_reserved_ports())
        finally:
            ctrl._connected = False
            ctrl._transport = ""

    def test_shared_session_reserves_nothing(self):
        from SupportClasses.incubator.service import get_incubator
        ctrl = get_incubator()
        ctrl._connected = True
        ctrl._transport = "shared"
        ctrl.active_port = "COM6 (shared ZP link)"
        try:
            self.assertEqual([], self._sc()._incubator_reserved_ports())
        finally:
            ctrl._connected = False
            ctrl._transport = ""


if __name__ == "__main__":
    unittest.main()
