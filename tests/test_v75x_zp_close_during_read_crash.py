"""test_v75x_zp_close_during_read_crash.py — close-during-read crash fix.

Root cause of the "always crashes on the Z-retract-to-safe at the end of prints"
report: when the board goes silent during the end-of-print retract, the retract
thread blocks inside ``serial.readline()`` (flush_moves' M400 wait / the 'ok'
handshake) holding ``_serial_lock``, while the watchdog/poller thread
concurrently runs ``_handle_disconnect → disconnect_zp → zp_stage.stop() →
serial.close()``. Calling ``CloseHandle`` on a USB-serial port that another
thread is mid-read on is a hard crash on Windows.

Fix: ``ZPStageManager.stop()`` (and the XY twin) acquire ``_serial_lock`` before
closing, so the close waits for the in-flight read to finish instead of racing
it. These tests prove the serialization without real hardware.
"""

import threading
import time
import unittest

from SupportClasses.ZPStage import ZPStageManager
from SupportClasses.XYStage import XYStageManager


class _FakePort:
    def __init__(self):
        self.is_open = True
        self.closed = False
        self.dtr = True
        self.rts = True

    def close(self):
        self.closed = True


def _bare_zp(port):
    zp = ZPStageManager.__new__(ZPStageManager)
    zp.simulate = False
    zp._serial_lock = threading.RLock()
    zp.serial = port
    return zp


class TestZPStopSerializesClose(unittest.TestCase):
    def test_close_waits_for_inflight_read_lock(self):
        port = _FakePort()
        zp = _bare_zp(port)
        # Simulate a read in progress: another thread holds the serial lock.
        zp._serial_lock.acquire()
        t = threading.Thread(target=zp.stop, daemon=True)
        t.start()
        time.sleep(0.25)
        # Close MUST NOT happen while the read holds the lock (the crash case).
        self.assertFalse(port.closed,
                         "stop() closed the port mid-read — crash race!")
        # Releasing the lock (read finished) lets stop() close safely.
        zp._serial_lock.release()
        t.join(timeout=3.0)
        self.assertTrue(port.closed, "stop() should close once the lock frees")

    def test_close_happens_when_lock_free(self):
        port = _FakePort()
        zp = _bare_zp(port)
        zp.stop()
        self.assertTrue(port.closed)

    def test_stop_without_lock_attr_still_closes(self):
        # Defensive: a __new__-built stand-in without _serial_lock must still
        # close (getattr-safe), not raise.
        zp = ZPStageManager.__new__(ZPStageManager)
        zp.simulate = False
        zp.serial = _FakePort()
        zp.stop()
        self.assertTrue(zp.serial.closed)


class _FakeSpo:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class TestXYStopSerializesClose(unittest.TestCase):
    def _bare_xy(self, spo):
        xy = XYStageManager.__new__(XYStageManager)
        xy.simulate = False
        xy._serial_lock = threading.RLock()
        xy.spo = spo
        return xy

    def test_close_waits_for_inflight_read_lock(self):
        spo = _FakeSpo()
        xy = self._bare_xy(spo)
        xy._serial_lock.acquire()
        t = threading.Thread(target=xy.stop, daemon=True)
        t.start()
        time.sleep(0.25)
        self.assertFalse(spo.closed)
        xy._serial_lock.release()
        t.join(timeout=3.0)
        self.assertTrue(spo.closed)


if __name__ == "__main__":
    unittest.main()
