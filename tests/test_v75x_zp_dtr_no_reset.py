"""
v7.5.x tests — ZP serial close-time reset mitigation.

SAFETY BUG: closing the app rebooted the Marlin (ZP) board, which
de-energized the steppers → a gravity-loaded Z dropped and backdrivable/
spring-loaded pumps drifted, so every active axis ran to its mechanical
extent (and, if firmware auto-homes on boot, was driven there). Nothing
in the shutdown path sends a motion command — the only board-touching
action is ``serial.close()``. On CH340/Arduino-class boards the DTR (and
on CH340, RTS) line is capacitively coupled to the MCU RESET pin, so the
OS dropping those lines on close pulses RESET.

A first attempt also pinned DTR/RTS low at OPEN to suppress the reset
entirely, but that broke connection on the ME3B V1 board — it needs the
open-time reset edge to start talking. So the open path is left at the
default (resetting) behavior, and the close-time reset is mitigated only
in ``stop()``: de-assert DTR/RTS *before* ``close()`` so the OS close
doesn't generate the resetting edge. (Whether this fully avoids the reset
is board-polarity dependent — the reliable cure is the hardware
auto-reset disable — but it never affects connectivity, since it runs
only at shutdown.)

Covered here:
  1. stop() de-asserts DTR/RTS BEFORE closing the handle (the fix).
  2. stop() is a no-op when there is no serial handle.
  3. stop() in simulate mode delegates to the simulator (no dtr/rts poke).
  4. A successful M115 probe still returns the live handle (open path
     intact / connectivity preserved).
  5. A non-Marlin port still returns None and is closed.
  6. An open failure returns None.
"""

import importlib
import unittest

from SupportClasses.ZPStage import ZPStageManager

# NB: `import SupportClasses.ZPStage as zp_mod` resolves to the *class*, not
# the module — SupportClasses/__init__.py re-exports ZPStageManager under the
# `ZPStage` attribute, shadowing the submodule. Pull the real module object so
# we can monkeypatch its module-level `serial`.
zp_mod = importlib.import_module(ZPStageManager.__module__)


class _FakeSerialException(Exception):
    pass


class _FakeSerialPort:
    """serial.Serial stand-in that logs DTR/RTS/open/close edges in order
    and answers an M115 probe with a configurable response. Mirrors the
    real (default) ctor: passing a port opens immediately."""

    def __init__(self, port=None, baudrate=None, timeout=None):
        self.events: list[tuple] = []
        # pyserial defaults: a port asserts DTR/RTS when opened.
        self._dtr = True
        self._rts = True
        self.is_open = False
        self._rx = b""
        self.m115_response = b"FIRMWARE_NAME:Marlin 2.1.2 (ME3B)\nok\n"
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        # serial.Serial(port, …) opens immediately (and resets the board).
        if port is not None:
            self.events.append(("ctor_open", port))
            self.is_open = True

    # — control lines —
    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value):
        self._dtr = bool(value)
        self.events.append(("dtr", bool(value)))

    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, value):
        self._rts = bool(value)
        self.events.append(("rts", bool(value)))

    # — lifecycle —
    def close(self):
        self.events.append(("close",))
        self.is_open = False

    # — io —
    def reset_input_buffer(self):
        self._rx = b""

    def reset_output_buffer(self):
        pass

    def write(self, data):
        if b"M115" in data:
            self._rx = self.m115_response
        return len(data)

    def flush(self):
        pass

    def read(self, n=1):
        chunk, self._rx = self._rx[:n], self._rx[n:]
        return chunk


def _fake_serial_module(port_factory):
    import types
    return types.SimpleNamespace(
        Serial=port_factory,
        SerialException=_FakeSerialException,
    )


def _bare_manager(baudrate=38400, simulate=False):
    """A ZPStageManager without running __init__ (no threads, no real
    serial). stop()/_try_open_marlin only need a few attributes."""
    mgr = ZPStageManager.__new__(ZPStageManager)
    mgr.baudrate = baudrate
    mgr.simulate = simulate
    mgr.serial = None
    mgr.connected_port = None
    return mgr


class _PatchSerial:
    """Context manager: swap ZPStage's module-level `serial` for a fake."""

    def __init__(self, port_factory):
        self._fake = _fake_serial_module(port_factory)
        self._orig = None

    def __enter__(self):
        self._orig = zp_mod.serial
        zp_mod.serial = self._fake
        return self._fake

    def __exit__(self, *exc):
        zp_mod.serial = self._orig
        return False


# ──────────────────────────────────────────────────────────────────────
# 1 + 2 + 3. stop() — the close-time mitigation.
# ──────────────────────────────────────────────────────────────────────
class TestStopNoReset(unittest.TestCase):
    def test_stop_de_asserts_before_close(self):
        port = _FakeSerialPort(port="COM7")  # a live session handle
        port.events.clear()                  # ignore ctor events
        mgr = _bare_manager()
        mgr.serial = port

        mgr.stop()

        self.assertIn(("close",), port.events, "stop() must close the port")
        idx_close = port.events.index(("close",))
        self.assertIn(("dtr", False), port.events[:idx_close],
                      "DTR must be de-asserted before close()")
        self.assertIn(("rts", False), port.events[:idx_close],
                      "RTS must be de-asserted before close()")
        self.assertFalse(port.is_open)

    def test_stop_noop_when_no_serial(self):
        mgr = _bare_manager()
        mgr.serial = None
        mgr.stop()  # must not raise

    def test_stop_simulate_delegates_to_simulator(self):
        class _Sim:
            def __init__(self):
                self.stopped = False
                self.poked = False

            def stop(self):
                self.stopped = True

            # If stop() wrongly poked dtr/rts on the sim, record it.
            @property
            def dtr(self):
                return False

            @dtr.setter
            def dtr(self, v):
                self.poked = True

        sim = _Sim()
        mgr = _bare_manager(simulate=True)
        mgr.serial = sim
        mgr.stop()
        self.assertTrue(sim.stopped, "simulate stop() must call sim.stop()")
        self.assertFalse(sim.poked, "simulate path must not poke dtr/rts")


# ──────────────────────────────────────────────────────────────────────
# 4. Open path intact — connectivity preserved (default reset-on-open).
# ──────────────────────────────────────────────────────────────────────
class TestOpenStillConnects(unittest.TestCase):
    def test_marlin_probe_returns_handle(self):
        with _PatchSerial(_FakeSerialPort):
            mgr = _bare_manager()
            ser = mgr._try_open_marlin("COM7", probe_timeout=0.2)
        self.assertIsNotNone(ser, "Marlin M115 probe should return the handle")
        # The board is opened the default way (immediate ctor open = the
        # reset edge this board needs to connect).
        self.assertIn(("ctor_open", "COM7"), ser.events)

    def test_port_configured_correctly(self):
        with _PatchSerial(_FakeSerialPort):
            mgr = _bare_manager(baudrate=250000)
            ser = mgr._try_open_marlin("COM3", probe_timeout=0.2)
        self.assertIsNotNone(ser)
        self.assertEqual(ser.port, "COM3")
        self.assertEqual(ser.baudrate, 250000)


# ──────────────────────────────────────────────────────────────────────
# 5 + 6. Failure paths.
# ──────────────────────────────────────────────────────────────────────
class TestProbeFailures(unittest.TestCase):
    def test_non_marlin_returns_none_and_closes(self):
        made = {}

        class _Silent(_FakeSerialPort):
            def __init__(self, *a, **k):
                super().__init__(*a, **k)
                # Answers, but not with FIRMWARE_NAME, and long enough to
                # trip the "ok without FIRMWARE_NAME → not Marlin" branch.
                self.m115_response = b"ok " + b"x" * 100 + b"\nok\n"
                made["p"] = self

        with _PatchSerial(_Silent):
            mgr = _bare_manager()
            ser = mgr._try_open_marlin("COM9", probe_timeout=0.2)

        self.assertIsNone(ser, "non-Marlin port must yield None")
        self.assertIn(("close",), made["p"].events,
                      "a rejected port must be closed")

    def test_open_failure_returns_none(self):
        class _Boom(_FakeSerialPort):
            def __init__(self, *a, **k):
                raise _FakeSerialException("Access is denied")

        with _PatchSerial(_Boom):
            mgr = _bare_manager()
            ser = mgr._try_open_marlin("COM9", probe_timeout=0.2)
        self.assertIsNone(ser)


if __name__ == "__main__":
    unittest.main()
