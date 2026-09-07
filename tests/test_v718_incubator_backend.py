"""
test_v718_incubator_backend.py — the vendored incubator backend, driven
through the PRODUCTION controller against the thermal simulator.

Covers what the v7.18 integration changed or newly relies on:
  * transport bookkeeping ("" / simulated / serial), ceiling clamping,
    zone display labels;
  * the per-machine store paths (env-override isolation);
  * port-arbitration in device_config (exclusions filter the ranking, the
    suggestions AND the detect scan — an excluded port must never be OPENED,
    because opening DTR-resets the board behind it);
  * the config store's validation (a config file can only LOWER the ceiling,
    ramp steps stay watchdog-safe, unknown transports are refused).
"""

from __future__ import annotations

import os
import tempfile
import time
import unittest


def _isolate_env():
    td = tempfile.mkdtemp(prefix="incu_test_")
    os.environ["MEBP_INCUBATOR_CONFIG_DIR"] = td
    os.environ["MEBP_INCUBATOR_CAL_DIR"] = td
    os.environ["MEBP_INCUBATOR_SIM_DIR"] = td
    os.environ["MEBP_INCUBATOR_LOG"] = "0"
    return td


class _SimControllerTest(unittest.TestCase):
    """Shared harness: one controller against the accelerated simulator."""

    def setUp(self):
        self.tmp = _isolate_env()
        from SupportClasses.incubator.config_store import reset_store
        from SupportClasses.incubator.service import reset_service
        reset_store()
        reset_service()
        from SupportClasses.incubator.controller import IncubatorController
        self.ctrl = IncubatorController()
        self.assertTrue(self.ctrl.connect(simulate=True, sim_time_scale=600))

    def tearDown(self):
        try:
            self.ctrl.disconnect()
        except Exception:
            pass


class TestTransportBookkeeping(_SimControllerTest):

    def test_simulated_transport_reported(self):
        self.assertEqual("simulated", self.ctrl.transport)
        self.assertTrue(self.ctrl.simulated)
        self.assertFalse(self.ctrl.shared_transport)

    def test_disconnect_resets_transport(self):
        self.ctrl.disconnect()
        self.assertEqual("", self.ctrl.transport)
        self.assertFalse(self.ctrl.connected)

    def test_probe_populated(self):
        rep = self.ctrl.report
        self.assertIsNotNone(rep)
        self.assertTrue(rep.firmware_name)
        self.assertIn("B", rep.sensor_fields)


class TestCeilingAndLabels(_SimControllerTest):

    def test_ceiling_can_only_be_lowered(self):
        self.ctrl.set_max_setpoint_c(45.0)
        self.assertEqual(45.0, self.ctrl.MAX_SETPOINT_C)
        # A config file must never raise the code-level hard maximum.
        self.ctrl.set_max_setpoint_c(90.0)
        self.assertEqual(50.0, self.ctrl.MAX_SETPOINT_C)
        # Junk is ignored, not applied.
        self.ctrl.set_max_setpoint_c("junk")
        self.assertEqual(50.0, self.ctrl.MAX_SETPOINT_C)

    def test_lowered_ceiling_clamps_setpoints(self):
        self.ctrl.set_max_setpoint_c(40.0)
        chk = self.ctrl.set_target("bed", 49.0)
        self.assertTrue(chk.clamped)
        self.assertEqual(40.0, chk.allowed_c)

    def test_zone_labels_reach_the_sensor_source(self):
        self.ctrl.set_zone_labels({"bed": "Water bath", "hotend": ""})
        time.sleep(2.5)   # one autoreport/poll round refreshes the channels
        ch = self.ctrl.hub.marlin_channel("B")
        self.assertIsNotNone(ch)
        self.assertEqual("Water bath", ch.label)

    def test_unknown_zone_label_ignored(self):
        # Must not raise, must not corrupt the map.
        self.ctrl.set_zone_labels({"nonsense": "X", "bed": "A"})


class TestSetpointsAgainstSimulator(_SimControllerTest):

    def test_set_target_commands_integer(self):
        self.ctrl.set_target("bed", 37.0)
        rt = self.ctrl.zone_runtime("bed")
        self.assertEqual(37.0, rt.requested_c)
        self.assertEqual(37, rt.commanded_c)
        time.sleep(2.5)
        ch = self.ctrl.hub.marlin_channel("B")
        self.assertIsNotNone(ch)
        self.assertEqual(37.0, ch.target_c)

    def test_heater_off_clears_target(self):
        self.ctrl.set_target("bed", 37.0)
        self.ctrl.heater_off("bed")
        rt = self.ctrl.zone_runtime("bed")
        self.assertEqual(0.0, rt.requested_c)

    def test_fine_target_arms_dither(self):
        self.ctrl.set_fine_target("bed", 37.5, period_s=30.0)
        rt = self.ctrl.zone_runtime("bed")
        self.assertTrue(rt.dither_enabled)
        self.assertEqual(37.5, rt.dither_target_c)

    def test_ramp_step_is_watchdog_safe(self):
        # A caller-supplied oversized step must be capped (MAX_SAFE_STEP_C):
        # above ~5-6 °C Marlin's heat-up watchdog arms and a slow block
        # false-trips it.
        self.assertTrue(self.ctrl.start_ramp("bed", 37.0, step_c=10.0))
        ramp = self.ctrl._ramps["bed"]
        self.assertLessEqual(ramp.step_c, 4.0)
        self.ctrl.stop_ramp("bed")


class TestCalibrationStorePath(unittest.TestCase):

    def test_calibration_persists_under_env_override(self):
        td = _isolate_env()
        from SupportClasses.incubator.calibration import CalibrationStore
        store = CalibrationStore()
        self.assertTrue(str(store.path).startswith(td))
        cal = store.get("marlin:B")
        cal.set_single_point(36.4, 37.0)
        self.assertTrue(store.save())
        again = CalibrationStore()
        self.assertAlmostEqual(0.6, again.get("marlin:B").offset_c, places=6)


class TestConfigStoreValidation(unittest.TestCase):

    def setUp(self):
        _isolate_env()
        from SupportClasses.incubator.config_store import reset_store
        reset_store()

    def test_defaults(self):
        from SupportClasses.incubator.config_store import get_store
        st = get_store()
        self.assertEqual("shared", st.get("transport"))
        self.assertEqual(50.0, st.get("max_setpoint_c"))

    def test_ceiling_clamped_on_set(self):
        from SupportClasses.incubator.config_store import get_store
        st = get_store()
        st.set("max_setpoint_c", 90.0)
        self.assertEqual(50.0, st.get("max_setpoint_c"))
        st.set("max_setpoint_c", 40.0)
        self.assertEqual(40.0, st.get("max_setpoint_c"))

    def test_ramp_step_clamped(self):
        from SupportClasses.incubator.config_store import get_store
        st = get_store()
        st.set("ramp_step_c", 9.0)
        self.assertEqual(4.0, st.get("ramp_step_c"))

    def test_unknown_transport_refused(self):
        from SupportClasses.incubator.config_store import get_store
        st = get_store()
        st.set("transport", "telepathy")
        self.assertEqual("shared", st.get("transport"))

    def test_round_trip_including_zones(self):
        from SupportClasses.incubator.config_store import (
            IncubatorConfigStore, get_store, reset_store,
        )
        st = get_store()
        st.set_zone("bed", label="Bath", enabled=False, preset_c=36.5)
        st.set("transport", "serial", save=False)
        st.set("dedicated_port", "COM9")
        reset_store()
        again = get_store()
        self.assertEqual("serial", again.get("transport"))
        self.assertEqual("COM9", again.get("dedicated_port"))
        z = again.zone("bed")
        self.assertEqual("Bath", z["label"])
        self.assertFalse(z["enabled"])
        self.assertEqual(36.5, z["preset_c"])
        self.assertIsInstance(again, IncubatorConfigStore)

    def test_malformed_file_falls_back_to_blank(self):
        from SupportClasses.incubator.config_store import (
            get_store, reset_store, _default_path,
        )
        path = _default_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{not json", encoding="utf-8")
        reset_store()
        st = get_store()
        self.assertEqual("shared", st.get("transport"))


class TestServiceSingleton(unittest.TestCase):

    def setUp(self):
        _isolate_env()
        from SupportClasses.incubator.config_store import reset_store
        from SupportClasses.incubator.service import reset_service
        reset_store()
        reset_service()

    def test_peek_never_constructs(self):
        from SupportClasses.incubator.service import (
            get_incubator, peek_incubator,
        )
        self.assertIsNone(peek_incubator())
        ctrl = get_incubator()
        self.assertIs(ctrl, peek_incubator())
        self.assertIs(ctrl, get_incubator())

    def test_store_ceiling_applied_at_construction(self):
        from SupportClasses.incubator.config_store import get_store
        get_store().set("max_setpoint_c", 42.0)
        from SupportClasses.incubator.service import get_incubator
        self.assertEqual(42.0, get_incubator().MAX_SETPOINT_C)

    def test_shutdown_forgets_the_singleton(self):
        from SupportClasses.incubator.service import (
            get_incubator, peek_incubator, shutdown_incubator,
        )
        get_incubator()
        shutdown_incubator(heaters_off=False)
        self.assertIsNone(peek_incubator())

    def test_connect_from_store_simulator(self):
        from SupportClasses.incubator.config_store import get_store
        get_store().set("transport", "simulate")
        from SupportClasses.incubator.service import (
            connect_from_store, get_incubator, shutdown_incubator,
        )
        try:
            self.assertTrue(connect_from_store(stage_controller=None))
            self.assertEqual("simulated", get_incubator().transport)
        finally:
            shutdown_incubator(heaters_off=False)


class TestPortExclusions(unittest.TestCase):
    """The DTR-reset guard: an excluded port is filtered from the ranking,
    the suggestions AND the detect scan — and detect must never OPEN it."""

    def setUp(self):
        _isolate_env()

    def _with_fake_ports(self, ports):
        from SupportClasses.incubator import device_config
        self._dc = device_config
        self._orig = device_config.available_ports
        device_config.available_ports = lambda: ports
        self.addCleanup(
            lambda: setattr(device_config, "available_ports", self._orig))

    def test_ranked_ports_filter(self):
        self._with_fake_ports([
            {"device": "COM6", "description": "STM32 Virtual COM",
             "hwid": "USB VID:PID=0483:5740"},
            {"device": "COM7", "description": "USB Serial Device",
             "hwid": "USB VID:PID=1A86:7523"},
        ])
        devs = [d for d, _s, _x in self._dc.ranked_ports(
            exclude_ports=["com6"])]
        self.assertEqual(["COM7"], devs)

    def test_suggest_ports_filter(self):
        self._with_fake_ports([
            {"device": "COM6", "description": "x", "hwid": "0483:5740"},
            {"device": "COM7", "description": "x", "hwid": "1A86:7523"},
        ])
        self.assertEqual(
            ["COM7"], self._dc.suggest_ports(exclude_ports=["COM6"]))

    def test_detect_never_opens_an_excluded_port(self):
        """The load-bearing one: exclusion is enforced even over an EXPLICIT
        ports list, before any serial.Serial() call."""
        import sys
        opened = []

        class _FakeSerial:
            def __init__(self, dev, baud, **kw):
                opened.append(dev)
                raise OSError("port not really there")

        class _FakeListPorts:
            @staticmethod
            def comports():
                return []

        fake = type(sys)("serial")
        fake.Serial = _FakeSerial
        fake.tools = type(sys)("serial.tools")
        fake.tools.list_ports = _FakeListPorts
        real = {k: sys.modules.get(k)
                for k in ("serial", "serial.tools", "serial.tools.list_ports")}
        sys.modules["serial"] = fake
        sys.modules["serial.tools"] = fake.tools
        sys.modules["serial.tools.list_ports"] = fake.tools.list_ports
        try:
            from SupportClasses.incubator import device_config
            found = device_config.detect_marlin(
                ports=["COM6", "COM7"],
                exclude_ports=["COM6"],
                bauds=(38400,),
            )
            self.assertIsNone(found)
            self.assertNotIn("COM6", opened)
            self.assertIn("COM7", opened)
        finally:
            for k, v in real.items():
                if v is None:
                    sys.modules.pop(k, None)
                else:
                    sys.modules[k] = v

    def test_controller_refuses_dedicated_connect_to_an_excluded_port(self):
        from SupportClasses.incubator.controller import IncubatorController
        ctrl = IncubatorController()
        ctrl.exclude_ports_provider = lambda: ["COM6"]
        msgs = []
        ctrl.on_status(msgs.append)
        self.assertFalse(ctrl.connect("COM6", 38400, auto_detect=False))
        self.assertFalse(ctrl.connected)
        self.assertTrue(any("shared transport" in m for m in msgs),
                        f"refusal must name the alternative, got: {msgs}")


class _FakeTxn:
    """Just the Transaction surface `_send_zone_cmd` reads."""

    def __init__(self, *, ok=False, rejected=False, timed_out=False,
                 board_reset=False, error_text=""):
        self.ok = ok
        self.rejected = rejected
        self.timed_out = timed_out
        self.board_reset = board_reset
        self.error_text = error_text
        self.lines: list[str] = []


class TestRefusedHeaterCommand(_SimControllerTest):
    """v7.18 bench fault (2026-08-12): the firmware REFUSED `M104 S37`
    (`error`, rx=1) every ~minute for 48 minutes and the software showed
    NOTHING — no status, no log, no card state — while the hold loop kept
    re-asserting the refused command. These pin the fix: a refusal latches on
    the zone, stops the automatic re-assertion, and leaves evidence."""

    def _reject_setpoints(self, error_text="Error:heater does not exist"):
        """Patch `_send` to refuse heater set commands but pass everything
        else (M105 polls etc.) through to the real simulator link."""
        orig = self.ctrl._send
        counter = {"n": 0}

        def fake(cmd, **kw):
            if cmd.startswith(("M140", "M104")):
                counter["n"] += 1
                return _FakeTxn(rejected=True, error_text=error_text)
            return orig(cmd, **kw)

        self.ctrl._send = fake
        return counter

    def test_a_refusal_latches_logs_and_fires_status(self):
        msgs = []
        self.ctrl.on_status(msgs.append)
        self.ctrl._send = lambda cmd, **kw: _FakeTxn(
            rejected=True, error_text="Error:heater does not exist")
        with self.assertLogs("SupportClasses.incubator.controller",
                             level="WARNING") as cap:
            self.ctrl._send_zone_cmd("hotend", "M104 S37")
        rt = self.ctrl.zone_runtime("hotend")
        self.assertEqual("M104 S37 → Error:heater does not exist", rt.refused)
        self.assertTrue(any("REFUSED" in line and "M104 S37" in line
                            for line in cap.output),
                        f"app.log must carry the reply text, got {cap.output}")
        self.assertTrue(any("REFUSED" in m and "HE0" in m for m in msgs),
                        f"the status must name the refusal AND the heater "
                        f"port, got: {msgs}")

    def test_status_fires_once_not_per_retry(self):
        msgs = []
        self.ctrl.on_status(msgs.append)
        self.ctrl._send = lambda cmd, **kw: _FakeTxn(
            rejected=True, error_text="Error:nope")
        self.ctrl._send_zone_cmd("bed", "M140 S37")
        self.ctrl._send_zone_cmd("bed", "M140 S37")
        self.assertEqual(
            1, sum(1 for m in msgs if "REFUSED" in m),
            "a repeated refusal must not re-banner every retry")

    def test_a_timeout_or_reset_does_not_latch(self):
        # An unanswered command is a LINK problem (stale-data warning /
        # reset detection own it); only an answered "no" is a refusal.
        for kw in ({"timed_out": True, "error_text": "no reply within 8.0s"},
                   {"board_reset": True, "error_text": "board reset"}):
            self.ctrl._send = lambda cmd, _kw=kw, **k: _FakeTxn(**_kw)
            self.ctrl._send_zone_cmd("bed", "M140 S37")
            self.assertEqual("", self.ctrl.zone_runtime("bed").refused)

    def test_an_accepted_command_clears_the_latch(self):
        rt = self.ctrl.zone_runtime("bed")
        rt.refused = "M140 S37 → Error:old"
        self.ctrl._send = lambda cmd, **kw: _FakeTxn(ok=True)
        self.ctrl._send_zone_cmd("bed", "M140 S37")
        self.assertEqual("", rt.refused)

    def test_a_fresh_operator_action_clears_the_latch(self):
        rt = self.ctrl.zone_runtime("bed")
        for action in (lambda: self.ctrl.set_target("bed", 30.0),
                       lambda: self.ctrl.start_ramp("bed", 30.0),
                       lambda: self.ctrl.set_fine_target("bed", 30.4),
                       lambda: self.ctrl.heater_off("bed")):
            rt.refused = "M140 S30 → Error:old"
            action()
            self.assertEqual("", rt.refused)

    def test_the_refusal_STOPS_the_dither_reassertion(self):
        """THE 48-MINUTE BUG: a refused setpoint must not be silently
        re-asserted every fine period forever. The dither must (a) route
        through the checked sender and (b) stop once refused."""
        counter = self._reject_setpoints()
        rt = self.ctrl.zone_runtime("bed")
        # Long period so only the FIRST flip (next_flip=0) fires on its own.
        self.ctrl.set_fine_target("bed", 37.5, period_s=600.0)
        self.assertTrue(rt.dither_enabled)

        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            if counter["n"] >= 1 and not rt.dither_enabled:
                break
            time.sleep(0.05)
        self.assertGreaterEqual(counter["n"], 1,
                                "the dither never sent its setpoint")
        self.assertFalse(rt.dither_enabled,
                         "a refused setpoint left the dither re-asserting")
        self.assertIn("M140", rt.refused)

        # And it STAYS stopped: force what would have been the next flip and
        # give the sampler a couple of rounds — no further sends.
        sent = counter["n"]
        rt._dither_next_flip = 0.0
        time.sleep(2.5)
        self.assertEqual(sent, counter["n"],
                         "the refused setpoint was re-asserted")

    def test_a_refused_ramp_rung_latches_and_stops_the_ramp(self):
        counter = self._reject_setpoints()
        rt = self.ctrl.zone_runtime("bed")
        self.assertTrue(self.ctrl.start_ramp("bed", 37.0))
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and not rt.refused:
            time.sleep(0.05)
        self.assertIn("M140", rt.refused,
                      "a refused ramp rung never latched")
        self.assertGreaterEqual(counter["n"], 1)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and self.ctrl.ramp_active("bed"):
            time.sleep(0.05)
        self.assertFalse(self.ctrl.ramp_active("bed"),
                         "the ramp kept walking rungs the firmware refuses")

    def test_a_refused_heater_off_latches(self):
        # "Off" being refused matters MORE than a refused setpoint — the
        # operator believes the heater is off when it is not.
        self._reject_setpoints(error_text="Error:kill")
        rt = self.ctrl.zone_runtime("bed")
        self.ctrl.heater_off("bed")
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and not rt.refused:
            time.sleep(0.05)
        self.assertIn("S0", rt.refused)


class TestSetpointKeeper(_SimControllerTest):
    """v7.18 bench fault, round 4 (2026-08-13): the heater was proved good at
    the bench (a direct M140 S37 held 36.7-38.0 C for three minutes) yet the
    app still would not heat. A Marlin reset clears every heater target to 0,
    and this board resets on every ZP auto-reconnect (opening the port pulses
    DTR) -- three times in four minutes in the operator's own app.log. Nothing
    put the target back, because the ramp finishes as soon as it has commanded
    the final value. These pin the keeper that does."""

    def _fake_channel(self, target_c, *, stale=False, temp=25.0):
        from types import SimpleNamespace
        return SimpleNamespace(value_c=temp, target_c=target_c,
                               power_pct=0.0, stale=stale)

    def _arm(self, *, commanded=37, requested=37.0, board_target=0.0,
             stale=False):
        """Zone A wants `requested`; the board claims `board_target`."""
        rt = self.ctrl.zone_runtime("bed")
        rt.requested_c = requested
        rt.commanded_c = commanded
        rt.refused = ""
        rt.reasserts = 0
        rt._target_mismatch = 0
        self.ctrl.hub.marlin_channel = (
            lambda key, _t=board_target, _s=stale: self._fake_channel(
                _t, stale=_s))
        sent = []
        self.ctrl.submit = lambda fn, *a, **k: sent.append((fn, a, k))
        return rt, sent

    @staticmethod
    def _setpoint_cmds(sent):
        """Only the re-asserted SETPOINT commands.

        A detected reset now also re-asserts the PID gains (a Marlin reset
        clears those too), so `sent` legitimately carries a second entry per
        reset. Counting raw submissions would conflate the two.
        """
        return [e for e in sent if getattr(e[0], "__name__", "")
                == "_send_zone_cmd"]

    @staticmethod
    def _pid_reapplies(sent):
        return [e for e in sent if getattr(e[0], "__name__", "")
                == "_do_apply_configured_pid"]

    def test_the_board_forgetting_its_target_is_put_back(self):
        rt, sent = self._arm(board_target=0.0)
        msgs = []
        self.ctrl.on_status(msgs.append)
        with self.assertLogs("SupportClasses.incubator.controller",
                             level="WARNING") as cap:
            self.ctrl._service_setpoint_keeper()   # sample 1: debounce
            self.assertEqual([], sent, "one disagreeing sample must not act")
            self.ctrl._service_setpoint_keeper()   # sample 2: act
        cmds = self._setpoint_cmds(sent)
        self.assertEqual(1, len(cmds), "the setpoint was never re-asserted")
        _fn, args, _kw = cmds[0]
        self.assertEqual("bed", args[0])
        self.assertEqual("M140 S37", args[1])
        self.assertEqual(1, rt.reasserts)
        self.assertTrue(any("re-asserting" in ln for ln in cap.output))
        self.assertTrue(any("forgotten its setpoint" in m for m in msgs),
                        f"the operator must be told, got: {msgs}")

    def test_a_matching_board_target_is_left_alone(self):
        _rt, sent = self._arm(board_target=37.0)
        for _ in range(4):
            self.ctrl._service_setpoint_keeper()
        self.assertEqual([], sent,
                         "the keeper commanded a target that already matched")

    def test_a_single_lagging_sample_does_not_trigger(self):
        """A sample taken between issuing a command and the board applying it
        legitimately still shows the old target -- acting on one would put a
        command on the wire every time a setpoint changed."""
        rt, sent = self._arm(board_target=0.0)
        self.ctrl._service_setpoint_keeper()
        self.assertEqual([], sent)
        # ...and if the board then agrees, the count resets rather than
        # carrying over into a later, unrelated disagreement.
        self.ctrl.hub.marlin_channel = lambda key: self._fake_channel(37.0)
        self.ctrl._service_setpoint_keeper()
        self.ctrl.hub.marlin_channel = lambda key: self._fake_channel(0.0)
        self.ctrl._service_setpoint_keeper()
        self.assertEqual([], sent, "the mismatch counter did not reset")
        self.assertEqual(0, rt.reasserts)

    def test_a_REFUSED_zone_is_never_re_asserted(self):
        """The round-3 rule outranks the keeper: a refused command is a
        verdict about this zone, not a target to retry forever."""
        rt, sent = self._arm(board_target=0.0)
        rt.refused = "M140 S37 -> Error:heater does not exist"
        for _ in range(4):
            self.ctrl._service_setpoint_keeper()
        self.assertEqual([], sent)
        self.assertEqual(0, rt.reasserts)

    def test_an_idle_zone_is_never_re_asserted(self):
        _rt, sent = self._arm(commanded=0, requested=0.0, board_target=0.0)
        for _ in range(4):
            self.ctrl._service_setpoint_keeper()
        self.assertEqual([], sent)

    def test_a_stale_reading_cannot_trigger_it(self):
        """Judging the board's target from a reading the board never sent
        would re-command on every poll while the link is down."""
        _rt, sent = self._arm(board_target=0.0, stale=True)
        for _ in range(4):
            self.ctrl._service_setpoint_keeper()
        self.assertEqual([], sent)

    def test_a_latched_fault_stops_it(self):
        from SupportClasses.incubator.marlin_gcode import parse_fault
        _rt, sent = self._arm(board_target=0.0)
        fault = parse_fault(
            "Error:Thermal Runaway, system stopped! Heater_ID: bed")
        self.assertIsNotNone(fault, "the fixture's fault line no longer parses")
        self.ctrl.fault_latch.record(fault, "bed")
        self.assertTrue(self.ctrl.fault_latch.active)
        for _ in range(4):
            self.ctrl._service_setpoint_keeper()
        self.assertEqual([], sent)

    def test_the_keeper_is_actually_WIRED_into_the_sample_loop(self):
        """Guard the guard. Every other test in this class calls
        `_service_setpoint_keeper` directly, so all of them would still pass
        if nothing ever called it -- which is precisely the fault being fixed
        (a hold that no one re-asserts). Pin the call site by AST, the way
        this repo pins its other one-writer rules."""
        import ast
        import inspect
        from SupportClasses.incubator import controller as mod
        tree = ast.parse(inspect.getsource(mod))
        publish = next(
            (n for n in ast.walk(tree)
             if isinstance(n, ast.FunctionDef) and n.name == "_publish_sample"),
            None)
        self.assertIsNotNone(publish, "_publish_sample no longer exists")
        called = {
            n.func.attr for n in ast.walk(publish)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
        }
        self.assertIn("_service_setpoint_keeper", called,
                      "_publish_sample no longer services the setpoint "
                      "keeper, so a target the board forgets is never put "
                      "back -- the 2026-08-13 bench fault returns")

    def test_the_keeper_really_runs_on_the_live_sample_loop(self):
        """And prove it behaviourally too: with the board claiming a target
        of 0 while 37 was commanded, the running controller must put it back
        without anyone poking it."""
        rt, sent = self._arm(board_target=0.0)
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not sent:
            time.sleep(0.2)
        self.assertTrue(sent, "the live sample loop never re-asserted the "
                              "forgotten setpoint")
        self.assertGreaterEqual(rt.reasserts, 1)

    def test_the_status_stops_repeating_but_the_count_keeps_rising(self):
        """A board reset-looping must not bury every other message, but the
        operator still needs to see that it is happening."""
        rt, sent = self._arm(board_target=0.0)
        msgs = []
        self.ctrl.on_status(msgs.append)
        for _ in range(20):
            self.ctrl._service_setpoint_keeper()
        self.assertGreaterEqual(rt.reasserts, 5)
        said = sum(1 for m in msgs if "forgotten its setpoint" in m)
        self.assertEqual(3, said,
                         f"expected 3 spoken warnings, got {said}")
        self.assertEqual(rt.reasserts, len(self._setpoint_cmds(sent)),
                         "every re-assert must actually reach the board")


if __name__ == "__main__":
    unittest.main()
