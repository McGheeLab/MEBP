"""test_v717_microscope_illumination_and_interlock.py

v7.17 — the three Nikon Ti devices nothing was driving (epi shutter, dia lamp,
light-path drive) plus the shutter↔cassette interlock.

Covered here:

  1. ``MicroscopeConfigStore``: both policy flags default OFF and round-trip;
     the Micro-Manager accessory device names reach ``backend_kwargs()``.
  2. ``SimulatedMicroscopeBackend``: all three modelled, connect-gated, the
     discrete/continuous split (light path refuses, lamp level clamps), and the
     "body without this accessory" path.
  3. ``NikonTiSdkBackend``: the shutter's open/closed codes DERIVED from the
     SDK's declared range and refined by its ``DisplayString``; the lamp's
     probed on/off property; the light path refusing an out-of-range index
     (the hardware-verified silent-clamp lesson); accessories absent.
  4. ``MicroManagerBackend``: the accessory calls are safe while disconnected.
  5. ``MicroscopeController``: the new state fields, the operator's inversion
     override, and the four new operations.
  6. **The interlock** — the load-bearing class. Every test there asserts the
     shutter state AT THE MOMENT the cassette moves, because asserting on the
     final state cannot tell "closed during the move" from "never touched".
  7. ``MicroscopePanel`` / ``MicroscopeSetupPanel``: rows appear only for fitted
     devices, controls drive the controller, both flags round-trip.
"""

import os
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from SupportClasses.MicroscopeConfigStore import MicroscopeConfigStore
from SupportClasses.MicroscopeControl import (
    MicroManagerBackend, MicroscopeController, MicroscopeError,
    NikonTiSdkBackend, SimulatedMicroscopeBackend,
)

_app = QApplication.instance() or QApplication([])


class _StoreCase(unittest.TestCase):
    """Base giving each test its own on-disk store."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.path = Path(self._tmp.name) / "microscope.json"
        self.store = MicroscopeConfigStore(self.path)


# ── Fakes for the Ti SDK's object model ─────────────────────────────

class _MipParam:
    """Stands in for the SDK's ``IMipParameter`` wrapper.

    Same shape as the fake in ``test_v75x_nikon_ti_microscope``: the number is
    on ``RawValue``, bounds on ``RangeLowerLimit``/``RangeHigherLimit``, and
    ``DisplayString`` carries the human text — which for a shutter is what names
    its state.
    """

    def __init__(self, raw, *, lower=None, upper=None, unit=None,
                 display=None):
        self.RawValue = raw
        self.RangeLowerLimit = lower
        self.RangeHigherLimit = upper
        self.RangeIncrement = 1
        self.Unit = unit
        self.DisplayScale = None
        self.DisplayString = display if display is not None else str(raw)
        self.IsReadOnly = False


class _AccessoryDevice:
    """A Ti accessory: one value parameter, optionally an on/off switch.

    ``value_prop`` exists because the value-carrying property is not always
    ``Position`` on this family of devices — a lamp's level and a shutter's
    state are both commonly ``Value``, which is exactly what the production
    code probes for.
    """

    def __init__(self, raw=1, *, lower=None, upper=None, display=None,
                 value_prop="Position", switch=None, switch_prop="IsOn",
                 mounted=True, is_opened=None, actions=False,
                 interlock_enabled=None):
        setattr(self, value_prop, _MipParam(raw, lower=lower, upper=upper,
                                            display=display))
        self._value_prop = value_prop
        # ⚠ Hardware-measured: the SDK publishes a COM object for every device it
        # KNOWS ABOUT, fitted or not, and an unfitted one answers
        # IsMounted=0 'Device not available'. The default here stays True, but
        # the flag exists because a fake that is always mounted is what let the
        # real bug through.
        self.IsMounted = _MipParam(
            1 if mounted else 0,
            display="Device mounted" if mounted else "Device not available")
        if switch is not None:
            setattr(self, switch_prop, _MipParam(1 if switch else 0))
        if is_opened is not None:
            # -1 is the SDK's 'Status Unknown' sentinel.
            self.IsOpened = _MipParam(
                is_opened, display="Status Unknown" if is_opened < 0 else "")
        if interlock_enabled is not None:
            self.IsInterlockEnabled = _MipParam(interlock_enabled)
        if actions:
            self.opened = []
            self.Open = lambda: self.opened.append(True)
            self.Close = lambda: self.opened.append(False)
            self.On = lambda: self.opened.append("on")
            self.Off = lambda: self.opened.append("off")

    @property
    def raw(self):
        return getattr(self, self._value_prop).RawValue


def _ti(devices: dict) -> NikonTiSdkBackend:
    """A Nikon backend wired to fake devices, without touching COM."""
    backend = NikonTiSdkBackend()
    backend._scope = object()
    backend._devices = dict(devices)
    return backend


class _Spy(SimulatedMicroscopeBackend):
    """Records the shutter state at the instant the cassette is told to move.

    This is the whole point of the interlock tests: the final state cannot
    distinguish "the shutter was closed for the rotation" from "the shutter was
    never touched", so the observation has to be taken *during* the move.
    """

    def __init__(self, fail: bool = False, **kwargs):
        super().__init__(**kwargs)
        self.during = []
        self.shutter_writes = []
        self._fail = fail

    def set_filter(self, position: int) -> None:
        self.during.append(self.epi_shutter_open())
        if self._fail:
            raise MicroscopeError("simulated cassette failure")
        super().set_filter(position)

    def set_epi_shutter(self, open_: bool) -> None:
        self.shutter_writes.append(bool(open_))
        super().set_epi_shutter(open_)


def _controller(store, backend) -> MicroscopeController:
    """A synchronous controller already connected to ``backend``."""
    ctrl = MicroscopeController(store=store, threaded=False)
    backend.connect()
    ctrl._backend = backend
    ctrl._read_all()
    return ctrl


# ── 1. Store ───────────────────────────────────────────────────────

class TestStoreFlags(_StoreCase):
    def test_both_policy_flags_default_off(self):
        """The interlock ships OFF because the shutter's open/closed encoding is
        not verified on hardware: if it is inverted, an interlock would OPEN the
        shutter for the rotation — the exposure it exists to prevent."""
        self.assertFalse(self.store.filter_shutter_interlock())
        self.assertFalse(self.store.epi_shutter_invert())

    def test_flags_round_trip_to_disk(self):
        self.store.set_filter_shutter_interlock(True)
        self.store.set_epi_shutter_invert(True)
        again = MicroscopeConfigStore(self.path)
        self.assertTrue(again.filter_shutter_interlock())
        self.assertTrue(again.epi_shutter_invert())

    def test_accessory_device_names_reach_the_backend(self):
        self.store.set_backend("micromanager")
        self.store.set("mm_epi_shutter_device", "MyShutter")
        kwargs = self.store.backend_kwargs()
        self.assertEqual(kwargs["epi_shutter_device"], "MyShutter")
        self.assertEqual(kwargs["dia_lamp_device"], "TIDiaLamp")
        self.assertEqual(kwargs["light_path_device"], "TILightPath")

    def test_accessory_names_are_accepted_by_the_backend_constructor(self):
        """The store's kwargs must actually be constructor-compatible; a keyword
        mismatch here is invisible until a real connect."""
        self.store.set_backend("micromanager")
        MicroManagerBackend(**self.store.backend_kwargs())

    def test_simulated_kwargs_unchanged(self):
        """A simulated body takes no accessory kwargs from the store, so its
        construction is byte-identical to pre-v7.17."""
        self.assertEqual(set(self.store.backend_kwargs()),
                         {"filter_slots", "objective_slots"})


# ── 2. Simulated backend ───────────────────────────────────────────

class TestSimulatedAccessories(unittest.TestCase):
    def setUp(self):
        self.b = SimulatedMicroscopeBackend()
        self.b.connect()

    def test_all_three_are_modelled(self):
        self.assertTrue(self.b.has_epi_shutter())
        self.assertTrue(self.b.has_dia_lamp())
        self.assertEqual(self.b.light_path_count(), 4)

    def test_shutter_and_lamp_start_off(self):
        """Closed and off at rest: that is the state they should be found in, and
        it means a freshly connected simulator models no light on the sample."""
        self.assertFalse(self.b.epi_shutter_open())
        self.assertFalse(self.b.dia_lamp_on())

    def test_shutter_toggles(self):
        self.b.set_epi_shutter(True)
        self.assertTrue(self.b.epi_shutter_open())
        self.b.set_epi_shutter(False)
        self.assertFalse(self.b.epi_shutter_open())

    def test_reads_are_none_before_connect(self):
        fresh = SimulatedMicroscopeBackend()
        self.assertIsNone(fresh.epi_shutter_open())
        self.assertIsNone(fresh.dia_lamp_on())
        self.assertIsNone(fresh.get_light_path())

    def test_writes_require_a_connection(self):
        fresh = SimulatedMicroscopeBackend()
        for call in (lambda: fresh.set_epi_shutter(True),
                     lambda: fresh.set_dia_lamp_on(True),
                     lambda: fresh.set_dia_lamp_intensity(10.0),
                     lambda: fresh.set_light_path(2)):
            with self.assertRaises(MicroscopeError):
                call()

    def test_light_path_refuses_out_of_range(self):
        with self.assertRaises(MicroscopeError) as ctx:
            self.b.set_light_path(9)
        self.assertIn("1-4", str(ctx.exception))
        self.assertEqual(self.b.get_light_path(), 1)      # never moved

    def test_lamp_level_clamps_to_its_range(self):
        """A level is continuous — asking for 'full' and landing at the declared
        maximum is ordinary, unlike a discrete selector."""
        self.b.set_dia_lamp_intensity(1e6)
        self.assertEqual(self.b.dia_lamp_intensity(), 100.0)
        self.b.set_dia_lamp_intensity(-50.0)
        self.assertEqual(self.b.dia_lamp_intensity(), 0.0)

    def test_a_body_without_the_accessories(self):
        bare = SimulatedMicroscopeBackend(
            epi_shutter=False, dia_lamp=False, light_path_slots=0)
        bare.connect()
        self.assertFalse(bare.has_epi_shutter())
        self.assertFalse(bare.has_dia_lamp())
        self.assertEqual(bare.light_path_count(), 0)
        self.assertIsNone(bare.epi_shutter_open())
        self.assertIsNone(bare.dia_lamp_intensity_range())
        with self.assertRaises(MicroscopeError):
            bare.set_epi_shutter(True)
        with self.assertRaises(MicroscopeError):
            bare.set_light_path(1)

    def test_light_path_names_fall_back_past_the_known_ones(self):
        wide = SimulatedMicroscopeBackend(light_path_slots=6)
        wide.connect()
        self.assertEqual(wide.light_path_names()[-1], "port 6")

    def test_diagnostics_names_every_accessory(self):
        text = self.b.diagnostics()
        for expected in ("epi shutter", "dia lamp", "light path"):
            self.assertIn(expected, text)


# ── 3. Nikon Ti SDK plumbing ───────────────────────────────────────

class TestNikonPresenceIsNotAttributeResolution(unittest.TestCase):
    """⚠ HARDWARE-MEASURED on the Ti-E, 2026-08-12 — and this is where the first
    cut was WRONG.

    The Ti SDK publishes a COM object for every device it knows about whether or
    not the body has one. On this rig ``EpiShutter`` and ``DiaShutter`` both
    resolve and then report ``IsMounted = 0 ('Device not available')`` with
    ``IsOpened = -1 ('Status Unknown')``, while ``DiaLamp`` and
    ``LightPathDrive`` report ``IsMounted = 1``.

    So "the attribute resolved" claims hardware that is not there. For the
    shutter that is not cosmetic: ``epi_shutter_open()`` fell through to the
    generic ``Value`` (1, in a declared 1–2 range), read it as *closed*, and the
    interlock therefore concluded "already closed, nothing to protect" and
    **silently did nothing** while the UI showed a shutter row.
    """

    def test_an_unfitted_device_is_not_present(self):
        b = _ti({"epi_shutter": _AccessoryDevice(raw=1, lower=1, upper=2,
                                                 mounted=False, is_opened=-1)})
        self.assertFalse(b.has_epi_shutter())
        self.assertIsNone(b.epi_shutter_open())
        with self.assertRaises(MicroscopeError) as ctx:
            b.set_epi_shutter(True)
        self.assertIn("no epi shutter fitted", str(ctx.exception))

    def test_the_status_unknown_sentinel_is_not_a_state(self):
        """-1 must read as unknown. 'Closed' would tell the operator the sample
        is dark when we do not know that."""
        b = _ti({"epi_shutter": _AccessoryDevice(raw=1, lower=1, upper=2,
                                                 mounted=True, is_opened=-1)})
        self.assertTrue(b.has_epi_shutter())
        self.assertIsNone(b.epi_shutter_open())

    def test_an_unfitted_lamp_and_light_path_report_absent(self):
        b = _ti({"dia_lamp": _AccessoryDevice(raw=5, lower=0, upper=24,
                                              mounted=False),
                 "light_path": _AccessoryDevice(raw=3, lower=1, upper=4,
                                                mounted=False)})
        self.assertFalse(b.has_dia_lamp())
        self.assertIsNone(b.dia_lamp_on())
        self.assertEqual(b.light_path_count(), 0)
        self.assertIsNone(b.get_light_path())
        with self.assertRaises(MicroscopeError):
            b.set_light_path(2)

    def test_a_device_that_does_not_report_its_mount_state_is_usable(self):
        """``IsMounted`` absent means 'did not say', not 'absent'."""
        dev = _AccessoryDevice(raw=3, lower=1, upper=4)
        del dev.IsMounted
        b = _ti({"light_path": dev})
        self.assertEqual(b.light_path_count(), 4)

    def test_the_interlock_degrades_when_no_shutter_is_fitted(self):
        """The end-to-end consequence of the bug above, through the controller."""
        import tempfile as _tf
        store = MicroscopeConfigStore(Path(_tf.mkdtemp()) / "m.json")
        store.set_filter_shutter_interlock(True)
        spy = _Spy(epi_shutter=False)
        ctrl = _controller(store, spy)
        op = ctrl.set_filter(2)
        self.assertIsNone(op.error)
        self.assertEqual(spy.get_filter(), 2)
        self.assertFalse(ctrl.state().epi_shutter_present)


class TestStaleFirstRead(unittest.TestCase):
    """⚠ HARDWARE-MEASURED: a device's ``Position`` read as the FIRST COM access
    after connect returns a default.

    The light-path drive physically at 3 answered **1**, and kept answering 1 for
    as long as nothing else on the device was touched (6 reads over 0.9 s). One
    access to any other property primes it and the next read is correct — so it is
    the access, not elapsed time.

    Every position getter is primed today only *by accident* of calling
    ``_require_mounted`` first. ``_prime_devices`` makes it deliberate.
    """

    class _StaleDevice(_AccessoryDevice):
        """Answers Position with a default until something else is touched."""

        def __init__(self, true_value, **kwargs):
            super().__init__(raw=1, **kwargs)
            self._true = true_value
            self._primed = False

        @property
        def IsMounted(self):                       # noqa: N802 (COM name)
            self._primed = True
            return _MipParam(1, display="Device mounted")

        @IsMounted.setter
        def IsMounted(self, _value):
            pass                                   # base __init__ assigns it

        @property
        def Position(self):                        # noqa: N802 (COM name)
            return _MipParam(self._true if self._primed else 1,
                             lower=1, upper=4)

        @Position.setter
        def Position(self, _value):
            pass

    def test_the_fake_reproduces_the_stale_read(self):
        """Guard the guard: if this class does not model the defect, the test
        below proves nothing."""
        dev = self._StaleDevice(3)
        self.assertEqual(dev.Position.RawValue, 1)      # first access: stale
        dev.IsMounted                                    # noqa: B018 - primes it
        self.assertEqual(dev.Position.RawValue, 3)

    def test_priming_makes_the_first_published_read_truthful(self):
        b = NikonTiSdkBackend()
        b._scope = object()
        b._devices = {"light_path": self._StaleDevice(3)}
        b._prime_devices()
        self.assertEqual(b.get_light_path(), 3)

    def test_connect_primes_every_device(self):
        """Pinned by structure: the ordering that makes this work today is
        accidental, so the explicit call must stay in connect()."""
        import ast
        import inspect
        import textwrap
        src = textwrap.dedent(inspect.getsource(NikonTiSdkBackend.connect))
        tree = ast.parse(src)
        calls = [n.func.attr for n in ast.walk(tree)
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
        self.assertIn("_prime_devices", calls)

    def test_priming_survives_a_device_that_raises(self):
        class _Angry:
            @property
            def IsMounted(self):                   # noqa: N802
                raise RuntimeError("no")

        b = NikonTiSdkBackend()
        b._scope = object()
        b._devices = {"filter": _Angry()}
        b._prime_devices()                         # must not raise


class TestNikonUsesTheNamedMembers(unittest.TestCase):
    """⚠ HARDWARE-MEASURED: ``IEpiShutter`` carries a named boolean ``IsOpened``
    plus ``Open()``/``Close()``, and ``IDiaLamp`` carries ``IsOn`` plus
    ``On()``/``Off()``.

    ``Value`` on the shutter sits at 1 in a declared 1–2 range regardless of
    state, so it is NOT the state. Preferring the named members turns the
    riskiest guess in this module — the derived open/closed encoding — into no
    guess at all wherever they exist.
    """

    def test_is_opened_beats_the_generic_value(self):
        # Value=1 would decode as CLOSED under the derived codes; IsOpened says
        # open, and it must win.
        b = _ti({"epi_shutter": _AccessoryDevice(raw=1, lower=1, upper=2,
                                                 is_opened=1)})
        self.assertIs(b.epi_shutter_open(), True)

    def test_open_and_close_methods_are_used_in_preference(self):
        dev = _AccessoryDevice(raw=1, lower=1, upper=2, is_opened=0,
                               actions=True)
        b = _ti({"epi_shutter": dev})
        b.set_epi_shutter(True)
        b.set_epi_shutter(False)
        self.assertEqual(dev.opened, [True, False])
        self.assertEqual(dev.raw, 1)          # Value never written

    def test_the_derived_codes_remain_the_fallback(self):
        """An SDK generation with no IsOpened/Open still works."""
        dev = _AccessoryDevice(raw=1, lower=1, upper=2, display="")
        b = _ti({"epi_shutter": dev})
        self.assertIs(b.epi_shutter_open(), False)
        b.set_epi_shutter(True)
        self.assertEqual(dev.raw, 2)

    def test_lamp_on_off_methods_are_used_in_preference(self):
        dev = _AccessoryDevice(raw=5, lower=0, upper=24, switch=False,
                               actions=True)
        b = _ti({"dia_lamp": dev})
        b.set_dia_lamp_on(True)
        self.assertEqual(dev.opened, ["on"])
        b.set_dia_lamp_on(False)
        self.assertEqual(dev.opened, ["on", "off"])
        self.assertEqual(dev.IsOn.RawValue, 0)   # untouched by the method path

    def test_the_bodys_own_interlock_flag_is_read_only(self):
        b = _ti({"epi_shutter": _AccessoryDevice(raw=1, lower=1, upper=2,
                                                 is_opened=0,
                                                 interlock_enabled=1)})
        self.assertIs(b.epi_shutter_interlock_enabled(), True)
        unknown = _ti({"epi_shutter": _AccessoryDevice(
            raw=1, lower=1, upper=2, is_opened=-1, interlock_enabled=-1)})
        self.assertIsNone(unknown.epi_shutter_interlock_enabled())

    def test_is_on_is_probed_first_because_it_is_what_this_sdk_has(self):
        from SupportClasses.MicroscopeControl import NikonTiSdkBackend as N
        self.assertEqual(N._LAMP_SWITCH_PROPS[0], "IsOn")


class TestNikonShutter(unittest.TestCase):
    def test_codes_come_from_the_declared_range(self):
        b = _ti({"epi_shutter": _AccessoryDevice(raw=1, lower=1, upper=2,
                                                 display="")})
        self.assertEqual(b._shutter_codes(), (1, 2))

    def test_codes_are_refined_by_the_reported_state_text(self):
        """A shutter reports its state in words, so the raw value in force tells
        us which code that word belongs to — a measurement, not a convention.
        Here the body says it is OPEN while sitting at the range's LOWER code,
        which is the opposite of the convention, and the text must win."""
        b = _ti({"epi_shutter": _AccessoryDevice(raw=1, lower=1, upper=2,
                                                 display="Open")})
        self.assertEqual(b._shutter_codes(), (2, 1))
        self.assertIs(b.epi_shutter_open(), True)

    def test_the_state_text_can_also_confirm_the_convention(self):
        b = _ti({"epi_shutter": _AccessoryDevice(raw=1, lower=1, upper=2,
                                                 display="Closed")})
        self.assertEqual(b._shutter_codes(), (1, 2))
        self.assertIs(b.epi_shutter_open(), False)

    def test_falls_back_and_says_so_when_no_range_is_declared(self):
        b = _ti({"epi_shutter": _AccessoryDevice(raw=1, display="")})
        with self.assertLogs("SupportClasses.MicroscopeControl",
                             level="WARNING") as logs:
            self.assertEqual(b._shutter_codes(), (1, 2))
        self.assertIn("declares no range", "\n".join(logs.output))

    def test_writes_the_resolved_code_not_a_boolean(self):
        dev = _AccessoryDevice(raw=1, lower=1, upper=2, display="")
        b = _ti({"epi_shutter": dev})
        b.set_epi_shutter(True)
        self.assertEqual(dev.raw, 2)
        b.set_epi_shutter(False)
        self.assertEqual(dev.raw, 1)

    def test_a_value_matching_neither_code_is_unknown(self):
        """Not truthiness: neither endpoint is guaranteed to be 0/1, so a value
        matching neither is honestly unknown rather than guessed."""
        b = _ti({"epi_shutter": _AccessoryDevice(raw=7, lower=1, upper=2,
                                                 display="")})
        self.assertIsNone(b.epi_shutter_open())

    def test_value_property_is_probed_not_assumed(self):
        """The value lives on ``Value`` for some of these devices, not
        ``Position``."""
        dev = _AccessoryDevice(raw=1, lower=1, upper=2, display="",
                               value_prop="Value")
        b = _ti({"epi_shutter": dev})
        b.set_epi_shutter(True)
        self.assertEqual(dev.Value.RawValue, 2)

    def test_absent_shutter_reports_absent_and_refuses(self):
        b = _ti({})
        self.assertFalse(b.has_epi_shutter())
        self.assertIsNone(b.epi_shutter_open())
        with self.assertRaises(MicroscopeError):
            b.set_epi_shutter(True)

    def test_codes_are_cached_per_connection_then_cleared(self):
        b = _ti({"epi_shutter": _AccessoryDevice(raw=1, lower=1, upper=2,
                                                 display="")})
        b._shutter_codes()
        self.assertIsNotNone(b._shutter_codes_cache)
        b.disconnect()
        self.assertIsNone(b._shutter_codes_cache)

    def test_state_text_parser(self):
        parse = NikonTiSdkBackend._state_from_text
        self.assertIs(parse("Open"), True)
        self.assertIs(parse("CLOSED"), False)
        self.assertIs(parse("shutter shut"), False)
        self.assertIsNone(parse(""))
        self.assertIsNone(parse("Device not available"))


class TestNikonDiaLamp(unittest.TestCase):
    def test_switch_property_is_probed(self):
        """``IsOn`` is what this SDK has (hardware-confirmed), so it is the
        primary spelling; with no ``On()``/``Off()`` the write falls back to it."""
        dev = _AccessoryDevice(raw=40, lower=0, upper=100, switch=True)
        b = _ti({"dia_lamp": dev})
        self.assertTrue(b.has_dia_lamp())
        self.assertIs(b.dia_lamp_on(), True)
        b.set_dia_lamp_on(False)
        self.assertEqual(dev.IsOn.RawValue, 0)

    def test_an_alternative_switch_spelling_is_found(self):
        dev = _AccessoryDevice(raw=40, lower=0, upper=100, switch=False,
                               switch_prop="SwitchValue")
        b = _ti({"dia_lamp": dev})
        self.assertIs(b.dia_lamp_on(), False)

    def test_a_lamp_with_no_switch_says_so_rather_than_pretending(self):
        b = _ti({"dia_lamp": _AccessoryDevice(raw=40, lower=0, upper=100)})
        self.assertIsNone(b.dia_lamp_on())
        with self.assertRaises(MicroscopeError) as ctx:
            b.set_dia_lamp_on(True)
        self.assertIn("no on/off switch", str(ctx.exception))

    def test_level_and_range_come_from_the_device(self):
        b = _ti({"dia_lamp": _AccessoryDevice(raw=40, lower=0, upper=100)})
        self.assertEqual(b.dia_lamp_intensity(), 40.0)
        self.assertEqual(b.dia_lamp_intensity_range(), (0.0, 100.0))

    def test_level_clamps_to_the_declared_range(self):
        dev = _AccessoryDevice(raw=40, lower=0, upper=100)
        b = _ti({"dia_lamp": dev})
        b.set_dia_lamp_intensity(1e6)
        self.assertEqual(dev.raw, 100)
        b.set_dia_lamp_intensity(-20.0)
        self.assertEqual(dev.raw, 0)

    def test_range_is_none_when_undeclared_so_no_percentage_is_invented(self):
        b = _ti({"dia_lamp": _AccessoryDevice(raw=40)})
        self.assertIsNone(b.dia_lamp_intensity_range())
        self.assertEqual(b.dia_lamp_intensity(), 40.0)


class TestDiaLampControlMode(_StoreCase):
    """⚠ HARDWARE-MEASURED on the Ti-E: the dia lamp has two control modes and
    the SDK **refuses every write in the wrong one**.

    ``IsControlled = 0 ('MainMode')`` — the body's front-panel knob owns the lamp
    and level/switch writes come back ``0xE01004BB`` / ``0xE01004BE``.
    ``IsControlled = 1 ('RemoteMode')`` — writes are accepted (verified: level
    1 → 6 → 12, on/off toggled, all restored).

    This is the state the rig is in by default, so it is the first thing an
    operator hits — and a bare HRESULT tells them nothing about the knob in front
    of them.
    """

    def test_nikon_reports_the_mode(self):
        remote = _ti({"dia_lamp": _AccessoryDevice(
            raw=1, lower=0, upper=24, switch=True, switch_prop="IsControlled")})
        self.assertIs(remote.dia_lamp_remote(), True)
        main = _ti({"dia_lamp": _AccessoryDevice(
            raw=1, lower=0, upper=24, switch=False, switch_prop="IsControlled")})
        self.assertIs(main.dia_lamp_remote(), False)

    def test_nikon_refuses_before_writing_and_names_the_remedy(self):
        """Checked BEFORE the write, so the message can name the front panel
        instead of relaying an opaque HRESULT."""
        dev = _AccessoryDevice(raw=1, lower=0, upper=24, switch=False,
                               switch_prop="IsControlled")
        b = _ti({"dia_lamp": dev})
        for call in (lambda: b.set_dia_lamp_intensity(10.0),
                     lambda: b.set_dia_lamp_on(True)):
            with self.assertRaises(MicroscopeError) as ctx:
                call()
            self.assertIn("MainMode", str(ctx.exception))
        self.assertEqual(dev.raw, 1)          # nothing was written

    def test_nikon_takes_and_releases_control(self):
        dev = _AccessoryDevice(raw=1, lower=0, upper=24, switch=False,
                               switch_prop="IsControlled")
        b = _ti({"dia_lamp": dev})
        b.set_dia_lamp_remote(True)
        self.assertEqual(dev.IsControlled.RawValue, 1)
        b.set_dia_lamp_remote(False)
        self.assertEqual(dev.IsControlled.RawValue, 0)

    def test_an_unknown_mode_does_not_block_writes(self):
        """``None`` means the driver cannot tell — not a refusal. Gating on it
        would make every body without the property permanently un-writable."""
        dev = _AccessoryDevice(raw=1, lower=0, upper=24, switch=True)
        b = _ti({"dia_lamp": dev})
        self.assertIsNone(b.dia_lamp_remote())
        b.set_dia_lamp_intensity(10.0)        # must not raise
        self.assertEqual(dev.raw, 10)

    def test_controller_surfaces_the_mode_and_the_refusal(self):
        backend = SimulatedMicroscopeBackend(dia_lamp_remote=False)
        ctrl = _controller(self.store, backend)
        self.assertIs(ctrl.state().dia_lamp_remote, False)
        op = ctrl.set_dia_lamp_on(True)
        self.assertIn("MainMode", op.error or "")
        ctrl.set_dia_lamp_remote(True)
        self.assertIs(ctrl.state().dia_lamp_remote, True)
        op = ctrl.set_dia_lamp_on(True)
        self.assertIsNone(op.error)
        self.assertTrue(backend.dia_lamp_on())

    def test_the_panel_disables_the_lamp_controls_in_main_mode(self):
        from gui.widgets.microscope_panel import MicroscopePanel
        ctrl = _controller(self.store,
                           SimulatedMicroscopeBackend(dia_lamp_remote=False))
        panel = MicroscopePanel(controller=ctrl, store=self.store)
        panel._render(force=True)
        self.assertFalse(panel._lamp_btn.isEnabled())
        self.assertFalse(panel._lamp_spin.isEnabled())
        self.assertTrue(panel._lamp_remote_btn.isEnabled())
        self.assertEqual(panel._lamp_remote_btn.text(), "Man")
        self.assertIn("front panel", panel._lamp_btn.toolTip())
        # Taking control through the button enables them.
        panel._lamp_remote_btn.setChecked(True)
        panel._on_lamp_remote_clicked()
        panel._render(force=True)
        self.assertTrue(panel._lamp_btn.isEnabled())
        self.assertEqual(panel._lamp_remote_btn.text(), "Remote")

    def test_the_setup_line_names_the_mode(self):
        from gui.pages.hardware.microscope_setup_panel import (
            MicroscopeSetupPanel)
        ctrl = _controller(self.store,
                           SimulatedMicroscopeBackend(dia_lamp_remote=False))
        panel = MicroscopeSetupPanel(store=self.store, controller=ctrl)
        panel._refresh_live()
        self.assertIn("FRONT-PANEL control", panel._illum_live_lbl.text())


class TestNikonLightPath(unittest.TestCase):
    def test_reads_position_and_count(self):
        b = _ti({"light_path": _AccessoryDevice(raw=2, lower=1, upper=4)})
        self.assertEqual(b.get_light_path(), 2)
        self.assertEqual(b.light_path_count(), 4)

    def test_out_of_range_is_refused_not_clamped(self):
        """⚠ HARDWARE-VERIFIED in v7.5.x on the cassette: this SDK silently
        clamps an out-of-range discrete index and reports SUCCESS. A light path
        is the same kind of device, so it must refuse too — otherwise a stale
        index quietly sends the light somewhere else."""
        dev = _AccessoryDevice(raw=1, lower=1, upper=4)
        b = _ti({"light_path": dev})
        with self.assertRaises(MicroscopeError) as ctx:
            b.set_light_path(99)
        self.assertIn("1-4", str(ctx.exception))
        self.assertEqual(dev.raw, 1)                      # never written

    def test_in_range_still_moves(self):
        dev = _AccessoryDevice(raw=1, lower=1, upper=4)
        b = _ti({"light_path": dev})
        b.set_light_path(3)
        self.assertEqual(dev.raw, 3)

    def test_no_invented_names(self):
        """There is no per-position name table in the SDK, so none is fabricated;
        the current position's own text is surfaced separately."""
        b = _ti({"light_path": _AccessoryDevice(raw=2, lower=1, upper=4,
                                                display="R100")})
        self.assertEqual(b.light_path_names(), ())
        self.assertEqual(b.light_path_name_now(), "R100")

    def test_absent_light_path(self):
        b = _ti({})
        self.assertEqual(b.light_path_count(), 0)
        self.assertIsNone(b.get_light_path())
        with self.assertRaises(MicroscopeError):
            b.set_light_path(1)


class TestNikonDeviceDiscovery(unittest.TestCase):
    def test_accessory_aliases_are_the_sdk_class_names(self):
        from SupportClasses.MicroscopeControl import _TI_DEVICE_ALIASES
        self.assertEqual(_TI_DEVICE_ALIASES["epi_shutter"][0], "EpiShutter")
        self.assertEqual(_TI_DEVICE_ALIASES["dia_lamp"][0], "DiaLamp")
        self.assertEqual(_TI_DEVICE_ALIASES["light_path"][0], "LightPathDrive")

    def test_only_the_core_devices_are_required(self):
        from SupportClasses.MicroscopeControl import _TI_CORE_DEVICES
        self.assertEqual(_TI_CORE_DEVICES, {"filter", "objective", "focus"})

    def test_a_missing_accessory_does_not_warn(self):
        """A body without an epi shutter is normal; warning on every connect
        would train the operator to ignore the log."""
        b = NikonTiSdkBackend()
        scope = type("S", (), {"Nosepiece": _AccessoryDevice(raw=1)})()
        with self.assertLogs("SupportClasses.MicroscopeControl",
                             level="INFO") as logs:
            found = b._discover_devices(scope)
        self.assertIn("objective", found)
        self.assertNotIn("epi_shutter", found)
        text = "\n".join(logs.output)
        self.assertIn("no epi_shutter accessory", text)
        self.assertNotIn("WARNING:SupportClasses.MicroscopeControl:"
                         "Nikon Ti: no epi_shutter", text)

    def test_diagnostics_reports_the_derived_semantics(self):
        """What a bench session needs before enabling the interlock."""
        b = _ti({"epi_shutter": _AccessoryDevice(raw=1, lower=1, upper=2,
                                                 display="Closed"),
                 "dia_lamp": _AccessoryDevice(raw=40, lower=0, upper=100,
                                              switch=True),
                 "light_path": _AccessoryDevice(raw=2, lower=1, upper=4,
                                                display="R100")})
        text = b.diagnostics()
        self.assertIn("1=closed", text)
        self.assertIn("IsOn", text)
        self.assertIn("R100", text)
        self.assertIn("[focus] not present on this body", text)
        # FITTED is the first thing a bench session must read.
        self.assertIn("FITTED", text)


# ── 4. Micro-Manager backend ───────────────────────────────────────

class TestMicroManagerAccessories(unittest.TestCase):
    def test_disconnected_reads_are_safe(self):
        b = MicroManagerBackend(config_path="")
        self.assertFalse(b.has_epi_shutter())
        self.assertFalse(b.has_dia_lamp())
        self.assertIsNone(b.epi_shutter_open())
        self.assertIsNone(b.dia_lamp_on())
        self.assertIsNone(b.dia_lamp_intensity())
        self.assertIsNone(b.dia_lamp_intensity_range())
        self.assertEqual(b.light_path_count(), 0)
        self.assertIsNone(b.get_light_path())
        self.assertEqual(b.light_path_names(), ())

    def test_disconnected_writes_refuse(self):
        b = MicroManagerBackend(config_path="")
        for call in (lambda: b.set_epi_shutter(True),
                     lambda: b.set_dia_lamp_on(True),
                     lambda: b.set_dia_lamp_intensity(5.0),
                     lambda: b.set_light_path(1)):
            with self.assertRaises(MicroscopeError):
                call()


# ── 5. Controller state + operations ───────────────────────────────

class TestControllerAccessories(_StoreCase):
    def setUp(self):
        super().setUp()
        self.backend = SimulatedMicroscopeBackend()
        self.ctrl = _controller(self.store, self.backend)

    def test_state_carries_every_accessory(self):
        st = self.ctrl.state()
        self.assertTrue(st.epi_shutter_present)
        self.assertIs(st.epi_shutter_open, False)
        self.assertTrue(st.dia_lamp_present)
        self.assertIs(st.dia_lamp_on, False)
        self.assertEqual(st.dia_lamp_intensity, 50.0)
        self.assertEqual((st.dia_lamp_min, st.dia_lamp_max), (0.0, 100.0))
        self.assertEqual(st.light_path_position, 1)
        self.assertEqual(st.light_path_count, 4)
        self.assertTrue(st.has_light_path)

    def test_operations_drive_the_backend(self):
        self.ctrl.set_epi_shutter(True)
        self.assertTrue(self.backend.epi_shutter_open())
        self.ctrl.set_dia_lamp_on(True)
        self.assertTrue(self.backend.dia_lamp_on())
        self.ctrl.set_dia_lamp_intensity(80.0)
        self.assertEqual(self.backend.dia_lamp_intensity(), 80.0)
        self.ctrl.set_light_path(3)
        self.assertEqual(self.backend.get_light_path(), 3)
        self.assertEqual(self.ctrl.state().light_path_position, 3)

    def test_inversion_override_flips_both_directions(self):
        """A body that disagrees with the derived codes is a checkbox, not a code
        change — so it must flip the READ as well as the WRITE."""
        self.store.set_epi_shutter_invert(True)
        self.ctrl.set_epi_shutter(True)
        # "open" in operator terms reached the backend as its opposite …
        self.assertFalse(self.backend.epi_shutter_open())
        # … and reads back as open again.
        self.ctrl._read_all()
        self.assertIs(self.ctrl.state().epi_shutter_open, True)

    def test_a_body_without_accessories_reports_absent(self):
        bare = SimulatedMicroscopeBackend(
            epi_shutter=False, dia_lamp=False, light_path_slots=0)
        ctrl = _controller(self.store, bare)
        st = ctrl.state()
        self.assertFalse(st.epi_shutter_present)
        self.assertFalse(st.dia_lamp_present)
        self.assertEqual(st.light_path_count, 0)
        self.assertFalse(st.has_light_path)

    def test_operations_fail_cleanly_when_disconnected(self):
        ctrl = MicroscopeController(store=self.store, threaded=False)
        for op in (ctrl.set_epi_shutter(True), ctrl.set_dia_lamp_on(True),
                   ctrl.set_dia_lamp_intensity(5.0), ctrl.set_light_path(2)):
            self.assertIn("not connected", op.error or "")

    def test_out_of_range_light_path_is_reported_not_raised(self):
        op = self.ctrl.set_light_path(99)
        self.assertIn("out of range", op.error or "")

    def test_disconnect_clears_the_accessory_readout(self):
        """A reconnect to a different body must not inherit the previous one's
        shutter or lamp state."""
        self.ctrl.disconnect()
        st = self.ctrl.state()
        self.assertFalse(st.epi_shutter_present)
        self.assertIsNone(st.epi_shutter_open)
        self.assertIsNone(st.dia_lamp_intensity)
        self.assertEqual(st.light_path_count, 0)
        self.assertEqual(st.native_light_path_names, ())

    def test_apply_invert_is_total(self):
        """Pure and total on purpose: it runs inside _read_all, whose reads are
        exception-swallowed, so anything that could raise here would turn a
        policy flag into a silently missing readout."""
        f = MicroscopeController._apply_invert
        self.assertIsNone(f(None, True))
        self.assertIs(f(True, False), True)
        self.assertIs(f(True, True), False)
        self.assertIs(f(False, True), True)


# ── 6. The interlock ───────────────────────────────────────────────

class TestShutterInterlock(_StoreCase):
    """Every test here observes the shutter DURING the cassette move.

    Asserting on the state afterwards cannot distinguish "closed for the
    rotation" from "never touched" — both end with the shutter as it started.
    """

    def _run(self, *, interlock: bool, start_open: bool, fail: bool = False,
             **kwargs):
        self.store.set_filter_shutter_interlock(interlock)
        spy = _Spy(fail=fail, **kwargs)
        ctrl = _controller(self.store, spy)
        if start_open and spy.has_epi_shutter():
            spy.set_epi_shutter(start_open)
            spy.shutter_writes.clear()
            ctrl._read_all()
        op = ctrl.set_filter(3)
        return spy, op

    def test_shutter_is_closed_during_the_move_and_reopened_after(self):
        spy, op = self._run(interlock=True, start_open=True)
        self.assertIsNone(op.error)
        self.assertEqual(spy.during, [False])       # closed while rotating
        self.assertTrue(spy.epi_shutter_open())     # restored
        self.assertEqual(spy.shutter_writes, [False, True])

    def test_off_by_default_the_shutter_is_never_touched(self):
        spy, op = self._run(interlock=False, start_open=True)
        self.assertIsNone(op.error)
        self.assertEqual(spy.during, [True])        # left open — legacy behaviour
        self.assertEqual(spy.shutter_writes, [])

    def test_an_already_closed_shutter_stays_closed(self):
        """Restore the PREVIOUS state, never 'open': reopening a deliberately
        darkened path would illuminate a sample the operator had gone dark on."""
        spy, op = self._run(interlock=True, start_open=False)
        self.assertIsNone(op.error)
        self.assertEqual(spy.during, [False])
        self.assertFalse(spy.epi_shutter_open())
        self.assertEqual(spy.shutter_writes, [])    # nothing to protect or undo

    def test_a_failed_move_still_reopens_the_shutter(self):
        """Otherwise a failed cube change leaves the light path closed and the
        next acquisition is black with nothing on screen explaining why."""
        spy, op = self._run(interlock=True, start_open=True, fail=True)
        self.assertIn("simulated cassette failure", op.error or "")
        self.assertEqual(spy.during, [False])
        self.assertTrue(spy.epi_shutter_open())     # restored despite the failure

    def test_a_body_with_no_shutter_just_moves(self):
        spy, op = self._run(interlock=True, start_open=False, epi_shutter=False)
        self.assertIsNone(op.error)
        self.assertEqual(spy.get_filter(), 3)
        self.assertEqual(spy.during, [None])        # nothing to report

    def test_an_unreadable_shutter_skips_the_interlock_and_says_so(self):
        """A state we cannot read is one we cannot restore, so closing on the way
        in would leave the body in a state we invented rather than found."""
        class _Opaque(_Spy):
            def epi_shutter_open(self):
                return None

        self.store.set_filter_shutter_interlock(True)
        spy = _Opaque()
        ctrl = _controller(self.store, spy)
        with self.assertLogs("SupportClasses.MicroscopeControl",
                             level="WARNING") as logs:
            op = ctrl.set_filter(2)
        self.assertIsNone(op.error)
        self.assertEqual(spy.get_filter(), 2)
        self.assertEqual(spy.shutter_writes, [])
        self.assertIn("interlock skipped", "\n".join(logs.output))

    def test_the_inversion_override_applies_inside_the_interlock(self):
        """The interlock must speak the same operator-facing language as the
        manual toggle, or ticking 'inverted' would fix one and break the other."""
        self.store.set_epi_shutter_invert(True)
        self.store.set_filter_shutter_interlock(True)
        spy = _Spy()
        ctrl = _controller(self.store, spy)
        spy.set_epi_shutter(False)          # raw closed == operator OPEN
        spy.shutter_writes.clear()
        ctrl._read_all()
        self.assertIs(ctrl.state().epi_shutter_open, True)
        ctrl.set_filter(4)
        # Raw writes are the inverse of the operator-facing close/reopen pair.
        self.assertEqual(spy.shutter_writes, [True, False])

    def test_a_failed_reopen_after_a_good_move_is_reported(self):
        """The cube changed but the light is still off — every following image
        would be black, so that deserves an error of its own."""
        class _StuckOpen(_Spy):
            def set_epi_shutter(self, open_):
                if open_:
                    raise MicroscopeError("shutter jammed")
                super().set_epi_shutter(open_)

        self.store.set_filter_shutter_interlock(True)
        spy = _StuckOpen()
        ctrl = _controller(self.store, spy)
        spy._epi_open = True                # start open without going through set
        ctrl._read_all()
        op = ctrl.set_filter(2)
        self.assertIn("reopening the epi shutter failed", op.error or "")
        self.assertEqual(spy.get_filter(), 2)   # the move itself did happen

    def test_a_failed_reopen_after_a_failed_move_keeps_the_move_error(self):
        """The move's own error is the more useful one; the restore failure is
        logged rather than replacing it."""
        class _Stuck(_Spy):
            def set_epi_shutter(self, open_):
                if open_:
                    raise MicroscopeError("shutter jammed")
                super().set_epi_shutter(open_)

        self.store.set_filter_shutter_interlock(True)
        spy = _Stuck(fail=True)
        ctrl = _controller(self.store, spy)
        spy._epi_open = True
        ctrl._read_all()
        with self.assertLogs("SupportClasses.MicroscopeControl",
                             level="ERROR"):
            op = ctrl.set_filter(2)
        self.assertIn("simulated cassette failure", op.error or "")

    def test_the_interlock_is_one_op_so_nothing_can_interleave(self):
        """Three separate ops would let another surface's refresh land between
        the close and the move. Proven by counting submissions."""
        self.store.set_filter_shutter_interlock(True)
        spy = _Spy()
        ctrl = _controller(self.store, spy)
        spy.set_epi_shutter(True)
        ctrl._read_all()
        submitted = []
        original = ctrl._submit

        def _spy_submit(name, fn):
            submitted.append(name)
            return original(name, fn)

        ctrl._submit = _spy_submit
        ctrl.set_filter(5)
        self.assertEqual(submitted, ["set_filter"])

    def test_the_interlock_covers_the_shared_chokepoint(self):
        """Hardware Setup's per-slot Go button calls controller.set_filter, the
        same method the jog card's combo uses — so it is protected too. A
        separate set_filter_interlocked() would have left it out."""
        import inspect
        from gui.pages.hardware.microscope_setup_panel import (
            MicroscopeSetupPanel)
        source = inspect.getsource(MicroscopeSetupPanel._go_filter)
        self.assertIn("set_filter", source)
        self.assertNotIn("interlock", source)


# ── 7. GUI surfaces ────────────────────────────────────────────────

class TestPanelIllumination(_StoreCase):
    def _panel(self, backend=None):
        from gui.widgets.microscope_panel import MicroscopePanel
        backend = backend or SimulatedMicroscopeBackend()
        ctrl = _controller(self.store, backend)
        panel = MicroscopePanel(controller=ctrl, store=self.store)
        panel._render(force=True)
        return panel, ctrl, backend

    def test_controls_reflect_the_hardware(self):
        panel, _ctrl, backend = self._panel()
        self.assertEqual(panel._shutter_btn.text(), "Excitation closed")
        self.assertEqual(panel._lamp_btn.text(), "Dia lamp off")
        self.assertEqual(panel._lamp_spin.value(), 50.0)
        self.assertEqual(
            [panel._light_combo.itemData(i)
             for i in range(panel._light_combo.count())], [1, 2, 3, 4])
        self.assertEqual(panel._light_combo.currentData(), 1)
        del backend

    def test_toggling_the_shutter_drives_the_body(self):
        panel, _ctrl, backend = self._panel()
        panel._shutter_btn.setChecked(True)
        panel._on_shutter_clicked()
        self.assertTrue(backend.epi_shutter_open())
        self.assertEqual(panel._shutter_btn.text(), "Excitation open")

    def test_lamp_controls_drive_the_body(self):
        panel, _ctrl, backend = self._panel()
        panel._lamp_btn.setChecked(True)
        panel._on_lamp_clicked()
        self.assertTrue(backend.dia_lamp_on())
        panel._lamp_spin.setValue(75.0)
        panel._on_lamp_level()
        self.assertEqual(backend.dia_lamp_intensity(), 75.0)

    def test_selecting_a_light_path_drives_the_body(self):
        panel, _ctrl, backend = self._panel()
        idx = panel._light_combo.findData(3)
        panel._light_combo.setCurrentIndex(idx)
        self.assertEqual(backend.get_light_path(), 3)

    def test_rows_are_hidden_when_the_body_has_no_such_device(self):
        """A visible but permanently dead control reads as broken software, not
        as an accessory this microscope was never fitted with."""
        bare = SimulatedMicroscopeBackend(
            epi_shutter=False, dia_lamp=False, light_path_slots=0)
        panel, _ctrl, _b = self._panel(bare)
        self.assertTrue(panel._shutter_btn.isHidden())
        self.assertTrue(panel._lamp_btn.isHidden())
        self.assertTrue(panel._light_combo.isHidden())
        self.assertTrue(panel._path_caption.isHidden())

    def test_an_unknown_shutter_state_is_shown_as_unknown(self):
        """Rendering unknown as 'closed' would tell the operator the sample is
        dark when we do not actually know that."""
        class _Opaque(SimulatedMicroscopeBackend):
            def epi_shutter_open(self):
                return None

        panel, _ctrl, _b = self._panel(_Opaque())
        self.assertEqual(panel._shutter_btn.text(), "Excitation —")

    def test_a_disconnected_panel_is_inert(self):
        from gui.widgets.microscope_panel import MicroscopePanel
        ctrl = MicroscopeController(store=self.store, threaded=False)
        panel = MicroscopePanel(controller=ctrl, store=self.store)
        panel._render(force=True)
        panel._shutter_btn.setChecked(True)
        panel._on_shutter_clicked()          # must not raise
        self.assertIsNone(ctrl.state().epi_shutter_open)

    def test_the_level_is_not_overwritten_while_being_typed(self):
        """The ~1 s poll would otherwise fight the keyboard.

        ``hasFocus`` is forced rather than requested: offscreen Qt will not
        reliably give a widget focus, and skipping would leave the guard
        unverified on every machine that runs the suite headless.
        """
        panel, _ctrl, _backend = self._panel()
        panel._lamp_spin.setValue(12.0)
        panel._lamp_spin.hasFocus = lambda: True
        panel._render(force=True)
        self.assertEqual(panel._lamp_spin.value(), 12.0)   # left alone
        # …and released, the poll takes over again.
        panel._lamp_spin.hasFocus = lambda: False
        panel._render(force=True)
        self.assertEqual(panel._lamp_spin.value(), 50.0)


class TestSetupPanelIllumination(_StoreCase):
    def _panel(self):
        from gui.pages.hardware.microscope_setup_panel import (
            MicroscopeSetupPanel)
        ctrl = _controller(self.store, SimulatedMicroscopeBackend())
        return MicroscopeSetupPanel(store=self.store, controller=ctrl), ctrl

    def test_flags_round_trip_through_commit_and_reload(self):
        panel, ctrl = self._panel()
        self.assertFalse(panel._interlock_chk.isChecked())
        panel._interlock_chk.setChecked(True)
        panel._shutter_invert_chk.setChecked(True)
        self.assertTrue(panel.commit())
        self.assertTrue(self.store.filter_shutter_interlock())
        self.assertTrue(self.store.epi_shutter_invert())
        from gui.pages.hardware.microscope_setup_panel import (
            MicroscopeSetupPanel)
        again = MicroscopeSetupPanel(store=self.store, controller=ctrl)
        self.assertTrue(again._interlock_chk.isChecked())
        self.assertTrue(again._shutter_invert_chk.isChecked())

    def test_editing_without_commit_leaves_the_store_untouched(self):
        panel, _ctrl = self._panel()
        panel._interlock_chk.setChecked(True)
        self.assertFalse(self.store.filter_shutter_interlock())

    def test_mm_accessory_device_names_round_trip(self):
        panel, ctrl = self._panel()
        panel._mm_light_path_edit.setText("MyPath")
        self.assertTrue(panel.commit())
        self.assertEqual(self.store.get("mm_light_path_device"), "MyPath")

    def test_the_live_line_distinguishes_absent_from_unknown(self):
        panel, _ctrl = self._panel()
        panel._refresh_live()
        text = panel._illum_live_lbl.text()
        self.assertIn("excitation shutter: closed", text)
        self.assertIn("dia lamp: off at 50", text)
        self.assertIn("light path: 1 of 4", text)

        bare = _controller(
            self.store,
            SimulatedMicroscopeBackend(epi_shutter=False, dia_lamp=False,
                                       light_path_slots=0))
        from gui.pages.hardware.microscope_setup_panel import (
            MicroscopeSetupPanel)
        bare_panel = MicroscopeSetupPanel(store=self.store, controller=bare)
        bare_panel._refresh_live()
        bare_text = bare_panel._illum_live_lbl.text()
        self.assertIn("excitation shutter: not fitted", bare_text)
        self.assertIn("light path: not fitted", bare_text)

    def test_the_page_warns_about_verifying_the_direction_first(self):
        """The interlock's whole risk is an inverted encoding, so the page has to
        say which check comes first."""
        panel, _ctrl = self._panel()
        tip = panel._interlock_chk.toolTip()
        self.assertIn("Verify the shutter direction", tip)


if __name__ == "__main__":
    unittest.main()
