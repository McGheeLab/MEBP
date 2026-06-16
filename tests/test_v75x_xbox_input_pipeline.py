"""
v7.5.x tests — Xbox controller input pipeline overhaul
(MEBP_v75x_XBOX_INPUT_PIPELINE).

Field failures fixed:
  S1: while holding a trigger, everything reset every ~3 s (the worker
      heartbeat re-initialized the entire SDL joystick subsystem, wiping
      per-device axis state).
  S2: trigger "neutral" was not 0 — an idle controller never reports its
      trigger axes, so SDL's zero-initialized state (0.0) combined with
      the Windows -1.0 rest offset manufactured a phantom half-press that
      continuously drove the mapped axis.

The worker is exercised FOR REAL: ``xbox_polling_worker`` imports pygame
inside its body, so injecting a fake pygame module via ``sys.modules``
lets the genuine loop run in a thread against scripted joystick state.

Covered here:
  Worker:
    1.  Unarmed trigger at raw 0.0 → NO phantom dispatch; monitor says 0.
    2.  Armed trigger dispatches, and a release dispatches a zero.
    3.  Both triggers pulled (same command) → combined value cancels.
    4.  Steady-state heartbeat is non-destructive (no joystick.quit()),
        still emits "alive".
    5.  Device removal → zero-dispatch + reconnect; trigger gate re-arms
        closed after reconnect (no phantom from stale raw value).
    6.  Unmapped d-pad press on a hats==0 pad no longer crashes the loop
        (old NameError → spurious full reconnect).
    7.  Mapped d-pad-as-buttons press dispatches.
    8.  ctrl_queue deadzone update applies live (stick goes quiet).
    9.  stop_event stops the worker cleanly.
    10. Unmapping an axis group mid-hold sends a final zero.
    11. load_xbox_mapping keeps the previous mapping on a corrupt file.
  Consumer:
    12. XboxQueuePoller consumes "monitor" messages into last_axis.
    13. Poller fires on_lost on connected→reconnecting and on stale
        heartbeat.
    14. ZPJogHandler staleness watchdog zeroes a stranded velocity.
    15. XYJogHandler watchdog zeroes + sends the explicit (0,0) stop.
  GUI / files:
    16. TriggerBar.set_value treats input as pull amount in [0, 1].
    17. _atomic_save_mapping writes valid JSON with no temp leftovers.
"""

import json
import os
import queue
import sys
import threading
import time
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from SupportClasses.XboxController import load_xbox_mapping, xbox_polling_worker
from SupportClasses.Processor import Processor
from SupportClasses.StageController import (
    XboxQueuePoller, XYJogHandler, ZPJogHandler,
)


# ════════════════════════════════════════════════════════════════════
#  Fake pygame
# ════════════════════════════════════════════════════════════════════

JOYAXISMOTION = 1536
JOYDEVICEADDED = 1541
JOYDEVICEREMOVED = 1542


class _FakeEvent:
    def __init__(self, type_, **attrs):
        self.type = type_
        for k, v in attrs.items():
            setattr(self, k, v)


class _FakeJoystick:
    def __init__(self, state):
        self._state = state

    def init(self):
        pass

    def get_name(self):
        return "Fake Xbox Series X Controller"

    def get_instance_id(self):
        return self._state.instance_id

    def get_numaxes(self):
        return 6

    def get_numbuttons(self):
        return 16

    def get_numhats(self):
        return 0

    def get_axis(self, i):
        return self._state.axis_values.get(i, 0.0)

    def get_button(self, i):
        return self._state.button_state.get(i, 0)

    def get_hat(self, i):
        return (0, 0)


class _FakePygameState:
    """Mutable scripted controller state shared with the test."""

    def __init__(self):
        self.axis_values = {}      # axis index → raw value
        self.button_state = {}     # button index → 0/1
        self.joystick_count = 1
        self.instance_id = 7
        self.pending_events = []   # drained by event.get()
        self.joystick_quit_calls = 0
        self.lock = threading.Lock()

    def push_event(self, ev):
        with self.lock:
            self.pending_events.append(ev)


def _make_fake_pygame(state: _FakePygameState):
    pg = types.ModuleType("pygame")
    pg.JOYAXISMOTION = JOYAXISMOTION
    pg.JOYDEVICEADDED = JOYDEVICEADDED
    pg.JOYDEVICEREMOVED = JOYDEVICEREMOVED
    pg.init = lambda: None
    pg.quit = lambda: None

    joystick = types.SimpleNamespace()
    joystick.init = lambda: None

    def _quit():
        state.joystick_quit_calls += 1

    joystick.quit = _quit
    joystick.get_count = lambda: state.joystick_count
    joystick.Joystick = lambda idx: _FakeJoystick(state)
    pg.joystick = joystick

    event = types.SimpleNamespace()
    event.pump = lambda: None

    def _get():
        with state.lock:
            evs = state.pending_events
            state.pending_events = []
        return evs

    event.get = _get
    pg.event = event
    return pg


# ════════════════════════════════════════════════════════════════════
#  Worker harness
# ════════════════════════════════════════════════════════════════════

_DEFAULT_TEST_MAPPING = {
    "buttons": {"0": "zero_needle_pos"},
    "axes": {
        "0-1": "move_stage_at_velocity",
        "2-3": "None",
        "4": "move_z_at_velocity",
        "5": "move_z_at_velocity",
    },
    "axes_invert": {},
    "dpad": {"up": "None", "down": "None", "left": "None", "right": "None"},
}


class _WorkerHarness:
    """Runs the real xbox_polling_worker against a fake pygame."""

    def __init__(self, test, mapping=None, **worker_kwargs):
        self.test = test
        self.state = _FakePygameState()
        self.out_queue = queue.Queue()
        self.ctrl_queue = queue.Queue()
        self.stop_event = threading.Event()

        tmp = Path(test.id().replace(":", "_").replace(".", "_") + "_mapping.json")
        self.mapping_file = Path(__file__).parent / tmp
        self.write_mapping(mapping or _DEFAULT_TEST_MAPPING)

        self._old_pygame = sys.modules.get("pygame")
        sys.modules["pygame"] = _make_fake_pygame(self.state)

        kwargs = dict(
            mapping_file=str(self.mapping_file),
            avg_interval=0.05,
            ctrl_queue=self.ctrl_queue,
            stop_event=self.stop_event,
        )
        kwargs.update(worker_kwargs)
        self.thread = threading.Thread(
            target=xbox_polling_worker, args=(self.out_queue,),
            kwargs=kwargs, daemon=True)
        self.thread.start()
        test.addCleanup(self.close)

    def write_mapping(self, mapping):
        with open(self.mapping_file, "w") as f:
            json.dump(mapping, f)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=5.0)
        if self._old_pygame is not None:
            sys.modules["pygame"] = self._old_pygame
        else:
            sys.modules.pop("pygame", None)
        try:
            os.unlink(self.mapping_file)
        except OSError:
            pass

    # ── helpers ────────────────────────────────────────────────────

    def arm_trigger(self, axis):
        self.state.push_event(_FakeEvent(
            JOYAXISMOTION, axis=axis,
            instance_id=self.state.instance_id))

    def drain(self):
        msgs = []
        while True:
            try:
                msgs.append(self.out_queue.get_nowait())
            except queue.Empty:
                return msgs

    def wait_for(self, pred, timeout=3.0):
        """Wait for a message matching pred; returns (msg, all_seen)."""
        seen = []
        end = time.time() + timeout
        while time.time() < end:
            try:
                m = self.out_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            seen.append(m)
            if pred(m):
                return m, seen
        return None, seen


def _is_axis(msg):
    return isinstance(msg, dict) and "axis" in msg


def _nonzero_axis(msg):
    if not _is_axis(msg):
        return False
    avg = msg["average"]
    if isinstance(avg, tuple):
        return any(abs(v) > 1e-9 for v in avg)
    return abs(avg) > 1e-9


def _zero_axis_for(key):
    def _pred(msg):
        if not _is_axis(msg) or msg["axis"] != key:
            return False
        avg = msg["average"]
        if isinstance(avg, tuple):
            return all(abs(v) < 1e-9 for v in avg)
        return abs(avg) < 1e-9
    return _pred


# ════════════════════════════════════════════════════════════════════
#  1–11: Worker tests
# ════════════════════════════════════════════════════════════════════

class TestWorkerTriggerArming(unittest.TestCase):

    def test_unarmed_trigger_produces_no_phantom(self):
        """SDL zero-init: raw 0.0 on an unarmed trigger must NOT become a
        phantom half-press (the S2 root cause)."""
        h = _WorkerHarness(self)
        # raw 0.0 everywhere = exactly what SDL reports for an idle pad
        time.sleep(0.8)
        msgs = h.drain()
        phantom = [m for m in msgs if _nonzero_axis(m)]
        self.assertEqual(phantom, [],
                         f"phantom dispatches from idle controller: {phantom}")
        monitors = [m for m in msgs if "monitor" in m]
        self.assertTrue(monitors, "no monitor messages published")
        self.assertEqual(monitors[-1]["monitor"]["triggers"].get(5), 0.0)
        self.assertEqual(monitors[-1]["monitor"]["triggers"].get(4), 0.0)

    def test_armed_trigger_dispatches_then_zeroes_on_release(self):
        h = _WorkerHarness(self)
        h.arm_trigger(5)
        h.state.axis_values[5] = 1.0  # fully pulled (raw +1)
        msg, _ = h.wait_for(
            lambda m: _is_axis(m) and m["command"] == "move_z_at_velocity"
            and not isinstance(m["average"], tuple) and m["average"] > 0.9)
        self.assertIsNotNone(msg, "armed full pull was not dispatched")
        self.assertEqual(msg["axis"], "4+5")  # combined trigger key
        # Release: raw back to the true rest value
        h.state.axis_values[5] = -1.0
        msg, _ = h.wait_for(_zero_axis_for("4+5"))
        self.assertIsNotNone(msg, "release did not dispatch a zero")

    def test_combined_triggers_cancel(self):
        h = _WorkerHarness(self)
        h.arm_trigger(4)
        h.arm_trigger(5)
        h.state.axis_values[4] = 1.0  # LT fully pulled → -1 after negation
        h.state.axis_values[5] = 1.0  # RT fully pulled → +1
        time.sleep(0.6)
        msgs = [m for m in h.drain() if _nonzero_axis(m)]
        self.assertEqual(msgs, [],
                         f"opposing full pulls should cancel, got {msgs}")
        # Drop LT → RT should win cleanly
        h.state.axis_values[4] = -1.0
        msg, _ = h.wait_for(
            lambda m: _is_axis(m) and not isinstance(m["average"], tuple)
            and m["average"] > 0.9)
        self.assertIsNotNone(msg, "RT-only pull was not dispatched")


class TestWorkerHeartbeat(unittest.TestCase):

    def test_heartbeat_is_nondestructive(self):
        """The 3 s heartbeat must NOT re-init the joystick subsystem
        (the S1 root cause) but must still emit 'alive'."""
        h = _WorkerHarness(self)
        # initial _find_controller legitimately does one quit()+init()
        time.sleep(0.5)
        quits_after_connect = h.state.joystick_quit_calls
        msg, _ = h.wait_for(lambda m: m.get("status") == "alive",
                            timeout=4.5)
        self.assertIsNotNone(msg, "heartbeat 'alive' not emitted")
        self.assertEqual(
            h.state.joystick_quit_calls, quits_after_connect,
            "steady-state heartbeat re-initialized the joystick subsystem")


class TestWorkerDeviceLoss(unittest.TestCase):

    def test_removal_zeroes_then_gate_stays_closed_after_reconnect(self):
        h = _WorkerHarness(self)
        h.arm_trigger(5)
        h.state.axis_values[5] = 1.0
        msg, _ = h.wait_for(lambda m: _nonzero_axis(m))
        self.assertIsNotNone(msg, "no dispatch before removal")
        # Simulate silent loss: JOYDEVICEREMOVED for our instance id.
        # joystick_count stays 1 so reconnection succeeds immediately.
        h.state.push_event(_FakeEvent(
            JOYDEVICEREMOVED, instance_id=h.state.instance_id))
        # Safety: the held velocity must be zeroed BEFORE the worker even
        # reports "reconnecting" (zero-dispatch is the first thing the
        # loss path does) — so wait for the zero first, then the statuses.
        msg, seen = h.wait_for(_zero_axis_for("4+5"))
        self.assertIsNotNone(msg, f"no zero-dispatch on removal; saw {seen}")
        self.assertFalse(any(m.get("status") == "reconnecting" for m in seen),
                         "zero-dispatch should precede the reconnecting status")
        msg, _ = h.wait_for(lambda m: m.get("status") == "reconnecting")
        self.assertIsNotNone(msg, "no reconnecting status after removal")
        # Reconnected with the trigger still physically at raw +1.0 but the
        # arming gate closed again: no dispatch until a fresh axis event.
        msg, _ = h.wait_for(lambda m: m.get("status") == "connected")
        self.assertIsNotNone(msg, "worker did not reconnect")
        h.drain()
        time.sleep(0.6)
        phantom = [m for m in h.drain() if _nonzero_axis(m)]
        self.assertEqual(phantom, [],
                         f"phantom after reconnect (gate not re-closed): {phantom}")
        # A real axis event re-arms it
        h.arm_trigger(5)
        msg, _ = h.wait_for(lambda m: _nonzero_axis(m))
        self.assertIsNotNone(msg, "re-armed trigger did not dispatch")


class TestWorkerDpadAsButtons(unittest.TestCase):

    def test_unmapped_dpad_press_does_not_crash(self):
        """Old code: NameError (misindented debounce) → except → full
        reconnect. The loop must survive an unmapped d-pad press."""
        h = _WorkerHarness(self)
        time.sleep(0.4)
        h.drain()
        h.state.button_state[14] = 1  # d-pad right (hats==0 fallback)
        time.sleep(0.4)
        msgs = h.drain()
        self.assertFalse(
            any(m.get("status") == "reconnecting" for m in msgs),
            f"unmapped d-pad press caused a reconnect: {msgs}")
        self.assertFalse(
            any("Controller error" in str(m.get("debug", "")) for m in msgs),
            f"unmapped d-pad press raised in the loop: {msgs}")

    def test_mapped_dpad_press_dispatches(self):
        mapping = dict(_DEFAULT_TEST_MAPPING)
        mapping["dpad"] = {"up": "None", "down": "None",
                           "left": "None", "right": "increment_pspeed_down"}
        h = _WorkerHarness(self, mapping=mapping)
        time.sleep(0.4)
        h.state.button_state[14] = 1
        msg, _ = h.wait_for(lambda m: m.get("dpad") == "right")
        self.assertIsNotNone(msg, "mapped d-pad press not dispatched")
        self.assertEqual(msg["command"], "increment_pspeed_down")


class TestWorkerLiveTuning(unittest.TestCase):

    def test_ctrl_queue_deadzone_update_applies_live(self):
        h = _WorkerHarness(self)
        h.state.axis_values[0] = 0.3  # above default 0.2 deadzone
        msg, _ = h.wait_for(lambda m: _nonzero_axis(m))
        self.assertIsNotNone(msg, "stick above deadzone not dispatched")
        # Raise the deadzone past the stick value — live, no reconnect
        h.ctrl_queue.put({"axis_deadzones": {0: 0.5, 1: 0.5, 2: 0.5,
                                             3: 0.5, 4: 0.5, 5: 0.5}})
        deadline = time.time() + 2.0
        quiet = False
        while time.time() < deadline and not quiet:
            h.drain()
            time.sleep(0.4)
            quiet = not any(_nonzero_axis(m) for m in h.drain())
        self.assertTrue(quiet, "deadzone update did not silence the stick")


class TestWorkerLifecycle(unittest.TestCase):

    def test_stop_event_stops_worker(self):
        h = _WorkerHarness(self)
        time.sleep(0.4)
        h.stop_event.set()
        h.thread.join(timeout=3.0)
        self.assertFalse(h.thread.is_alive(), "worker did not stop")

    def test_unmap_mid_hold_sends_final_zero(self):
        h = _WorkerHarness(self, mapping_reload_interval=0.2)
        h.state.axis_values[0] = 0.5
        msg, _ = h.wait_for(
            lambda m: _is_axis(m) and m["axis"] == "0-1" and _nonzero_axis(m))
        self.assertIsNotNone(msg, "stick hold not dispatched")
        mapping = json.loads(json.dumps(_DEFAULT_TEST_MAPPING))
        mapping["axes"]["0-1"] = "None"
        h.write_mapping(mapping)
        msg, seen = h.wait_for(_zero_axis_for("0-1"))
        self.assertIsNotNone(
            msg, f"no final zero after unmapping mid-hold; saw {seen}")
        self.assertEqual(msg["command"], "move_stage_at_velocity")


class TestMappingLoad(unittest.TestCase):

    def test_corrupt_file_keeps_previous_mapping(self):
        p = Path(__file__).parent / "_corrupt_mapping.json"
        p.write_text("{ this is not json", encoding="utf-8")
        self.addCleanup(lambda: p.unlink(missing_ok=True))
        prev = {"buttons": {"1": "keep_me"}, "axes": {}, "dpad": {}}
        out = load_xbox_mapping(str(p), fallback=prev)
        self.assertIs(out, prev)
        # without a fallback: legacy empty-mapping behavior
        out = load_xbox_mapping(str(p))
        self.assertEqual(out, {"buttons": {}, "axes": {}, "dpad": {}})


# ════════════════════════════════════════════════════════════════════
#  12–13: XboxQueuePoller
# ════════════════════════════════════════════════════════════════════

class TestQueuePoller(unittest.TestCase):

    def _make(self, on_lost=None):
        q = queue.Queue()
        proc = Processor(name="TestProc")
        self.addCleanup(proc.stop)
        poller = XboxQueuePoller(q, proc, on_lost=on_lost)
        poller.start()
        self.addCleanup(poller.stop)
        return q, poller

    def test_monitor_messages_feed_last_axis(self):
        q, poller = self._make()
        q.put({"monitor": {"sticks": {0: 0.12, 1: -0.5},
                           "triggers": {4: 0.0, 5: 0.8}}})
        time.sleep(0.2)
        self.assertAlmostEqual(poller.last_axis.get(0), 0.12)
        self.assertAlmostEqual(poller.last_axis.get(1), -0.5)
        self.assertAlmostEqual(poller.last_axis.get(5), 0.8)

    def test_on_lost_fires_on_reconnecting_transition(self):
        fired = []
        q, poller = self._make(on_lost=lambda: fired.append(True))
        q.put({"status": "connected"})
        time.sleep(0.15)
        q.put({"status": "reconnecting"})
        time.sleep(0.25)
        self.assertTrue(fired, "on_lost not fired on connected→reconnecting")

    def test_on_lost_fires_on_stale_heartbeat(self):
        fired = []
        q, poller = self._make(on_lost=lambda: fired.append(True))
        q.put({"status": "alive"})
        time.sleep(0.15)
        poller._last_heartbeat_time = time.time() - 11.0
        time.sleep(0.25)
        self.assertEqual(poller._xbox_status, "disconnected")
        self.assertTrue(fired, "on_lost not fired on stale heartbeat")


# ════════════════════════════════════════════════════════════════════
#  14–15: Jog handler staleness watchdogs
# ════════════════════════════════════════════════════════════════════

class _FakeZPStage:
    def __init__(self):
        self.axis_map = {"Z": "Z", "P1": "X", "P2": "Y", "P3": "E"}
        self.moves = []

    def move_relative(self, deltas, feedrate):
        self.moves.append((dict(deltas or {}), feedrate))


class _FakeXYStage:
    def __init__(self):
        self.calls = []

    def move_stage_at_velocity(self, vx, vy):
        self.calls.append((vx, vy))


class TestJogWatchdogs(unittest.TestCase):

    def test_zp_watchdog_zeroes_stranded_velocity(self):
        proc = Processor(name="TestProcZP")
        self.addCleanup(proc.stop)
        stage = _FakeZPStage()
        jog = ZPJogHandler(proc, stage)
        jog.stale_timeout = 0.2
        jog._handle_z_vel(average=0.8)  # velocity arrives once, then silence
        self.assertNotEqual(jog.vel_z, 0.0)
        jog.start()
        self.addCleanup(jog.stop)
        time.sleep(0.8)
        self.assertEqual(jog.vel_z, 0.0,
                         "stranded Z velocity not zeroed by watchdog")
        self.assertTrue(stage.moves, "no segment was sent before timeout")
        moves_at_timeout = len(stage.moves)
        time.sleep(0.4)
        self.assertEqual(len(stage.moves), moves_at_timeout,
                         "segments kept flowing after watchdog zeroed")

    def test_zp_watchdog_not_triggered_while_fed(self):
        proc = Processor(name="TestProcZP2")
        self.addCleanup(proc.stop)
        stage = _FakeZPStage()
        jog = ZPJogHandler(proc, stage)
        jog.stale_timeout = 0.3
        jog.start()
        self.addCleanup(jog.stop)
        # Feed like the worker does (every 0.1 s) — must keep moving
        for _ in range(6):
            jog._handle_z_vel(average=0.8)
            time.sleep(0.1)
        self.assertNotEqual(jog.vel_z, 0.0,
                            "watchdog zeroed a continuously-fed velocity")

    def test_xy_watchdog_zeroes_and_sends_stop(self):
        proc = Processor(name="TestProcXY")
        self.addCleanup(proc.stop)
        stage = _FakeXYStage()
        jog = XYJogHandler(proc, stage)
        jog.stale_timeout = 0.2
        jog._handle_vel(average=(0.5, 0.4))
        self.assertNotEqual(jog.vel_x, 0.0)
        jog.start()
        self.addCleanup(jog.stop)
        time.sleep(0.8)
        self.assertEqual((jog.vel_x, jog.vel_y), (0.0, 0.0),
                         "stranded XY velocity not zeroed by watchdog")
        self.assertIn((0, 0), stage.calls,
                      "explicit zero-velocity stop was not sent")


# ════════════════════════════════════════════════════════════════════
#  16: TriggerBar display contract
# ════════════════════════════════════════════════════════════════════

class TestTriggerBarContract(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        try:
            from PySide6.QtWidgets import QApplication
        except Exception:
            raise unittest.SkipTest("PySide6 not available")
        cls._app = QApplication.instance() or QApplication([])

    def test_set_value_is_pull_amount(self):
        from gui.pages.hardware.xbox_panel import TriggerBar
        bar = TriggerBar("RT")
        bar.set_value(0.0)
        self.assertEqual(bar._value, 0.0)   # rest renders EMPTY, not 50%
        bar.set_value(0.5)
        self.assertEqual(bar._value, 0.5)
        bar.set_value(1.0)
        self.assertEqual(bar._value, 1.0)
        bar.set_value(-0.7)                 # LT-negated dispatch convention
        self.assertAlmostEqual(bar._value, 0.7)
        bar.set_value(5.0)
        self.assertEqual(bar._value, 1.0)   # clamped


# ════════════════════════════════════════════════════════════════════
#  17: Atomic mapping save
# ════════════════════════════════════════════════════════════════════

class TestAtomicMappingSave(unittest.TestCase):

    def test_atomic_save_writes_json_and_cleans_up(self):
        try:
            from gui.widgets.xbox_mapping_editor import _atomic_save_mapping
        except Exception:
            raise unittest.SkipTest("PySide6 not available")
        target = Path(__file__).parent / "_atomic_mapping.json"
        target.write_text('{"old": true}', encoding="utf-8")
        self.addCleanup(lambda: target.unlink(missing_ok=True))
        mapping = {"buttons": {"0": "zero_needle_pos"}, "axes": {},
                   "axes_invert": {}, "dpad": {}}
        _atomic_save_mapping(mapping, str(target))
        with open(target) as f:
            self.assertEqual(json.load(f), mapping)
        leftovers = list(target.parent.glob(target.name + ".*.tmp"))
        self.assertEqual(leftovers, [], f"temp files left behind: {leftovers}")


if __name__ == "__main__":
    unittest.main()
