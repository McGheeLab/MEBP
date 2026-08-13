"""test_v75x_nikon_ti_microscope.py — Nikon Ti Eclipse body control.

Manual control of the microscope's three motorized devices — filter-cube
cassette, nosepiece and focus drive — surfaced as a card in the jog panel.
Covered here:

  1. ``MicroscopeConfigStore``: 1-based slot assignments, slot counts, focus
     preferences + soft limits, unknown-key tolerance, on-disk round-trip.
  2. ``SimulatedMicroscopeBackend``: connect gating, range checks, focus clamp.
  3. ``NikonTiSdkBackend``: the probed ``Position``/``Position.Value`` plumbing
     and — the number that scales every focus move — device-units → µm.
  4. ``MicroManagerBackend``: refuses cleanly without a configuration.
  5. ``MicroscopeController``: state snapshots, turret + focus operations,
     operator soft-limit clamping, errors when disconnected, and the threaded
     worker path (one thread, serialized ops).
  6. ``MicroscopePanel``: builds headless, labels slots from the store, drives
     the turrets on selection, honours the focus-direction convention, and
     stays inert while disconnected.
  7. The ``microscope`` section is registered in the custom-panel catalog.
"""

import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from gui.pages.hardware import microscope_setup_panel as setup_panel_mod

from SupportClasses.MicroscopeConfigStore import MicroscopeConfigStore
from SupportClasses.MicroscopeControl import (
    MicroManagerBackend, MicroscopeBackend, MicroscopeController,
    MicroscopeError, NikonTiSdkBackend, SimulatedMicroscopeBackend,
    build_backend,
)

_app = QApplication.instance() or QApplication([])


def _store(tmpdir) -> MicroscopeConfigStore:
    return MicroscopeConfigStore(Path(tmpdir) / "microscope.json")


class _StoreCase(unittest.TestCase):
    """Base giving each test its own on-disk store."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.path = Path(self._tmp.name) / "microscope.json"
        self.store = MicroscopeConfigStore(self.path)


# ── 1. Config store ────────────────────────────────────────────────

class TestMicroscopeConfigStore(_StoreCase):
    def test_defaults(self):
        self.assertEqual(self.store.get_backend(), "simulated")
        self.assertEqual(self.store.filter_slots(), 6)
        self.assertEqual(self.store.objective_slots(), 6)
        self.assertEqual(self.store.filter_labels(), {})
        self.assertTrue(self.store.focus_up_is_positive())
        self.assertEqual(self.store.focus_soft_limits_um(), (None, None))

    def test_slot_labels_are_one_based(self):
        self.store.set_filter_label(1, "DAPI")
        self.store.set_filter_label(3, "mCherry")
        self.assertEqual(self.store.filter_labels(), {1: "DAPI", 3: "mCherry"})
        self.assertEqual(self.store.filter_label(3), "mCherry")
        self.assertEqual(self.store.filter_label(2), "")

    def test_zero_position_rejected(self):
        with self.assertRaises(ValueError):
            self.store.set_filter_label(0, "nope")

    def test_blank_clears_assignment(self):
        self.store.set_objective_label(2, "10x Plan Fluor")
        self.store.set_objective_label(2, "   ")
        self.assertEqual(self.store.objective_labels(), {})

    def test_labels_beyond_slot_count_are_hidden(self):
        self.store.set_filter_label(9, "extra")
        self.store.set_filter_slots(6)
        self.assertNotIn(9, self.store.filter_labels())
        # …but not destroyed: raising the count brings it back.
        self.store.set_filter_slots(10)
        self.assertEqual(self.store.filter_labels()[9], "extra")

    def test_bulk_set_replaces(self):
        self.store.set_filter_labels({1: "DAPI", 2: "FITC"})
        self.store.set_filter_labels({2: "GFP"})
        self.assertEqual(self.store.filter_labels(), {2: "GFP"})

    def test_focus_prefs_round_trip(self):
        self.store.set_focus_step_um(25.0)
        self.store.set_focus_up_is_positive(False)
        self.store.set_focus_soft_limits_um(6000.0, 4000.0)   # swapped input
        self.assertEqual(self.store.focus_step_um(), 25.0)
        self.assertFalse(self.store.focus_up_is_positive())
        self.assertEqual(self.store.focus_soft_limits_um(), (4000.0, 6000.0))

    def test_persists_to_disk(self):
        self.store.set_filter_label(4, "Cy5")
        self.store.set_backend("nikon_ti")
        self.store.set_focus_step_um(2.5)
        reloaded = MicroscopeConfigStore(self.path)
        self.assertEqual(reloaded.filter_label(4), "Cy5")
        self.assertEqual(reloaded.get_backend(), "nikon_ti")
        self.assertEqual(reloaded.focus_step_um(), 2.5)

    def test_unknown_keys_and_bad_backend_tolerated(self):
        self.path.write_text(
            '{"backend": "not-a-backend", "who": "dis", "filter_slots": 99}',
            encoding="utf-8")
        st = MicroscopeConfigStore(self.path)
        self.assertEqual(st.get_backend(), "simulated")   # falls back
        self.assertEqual(st.filter_slots(), 12)           # clamped to the ceiling

    def test_backend_kwargs_track_selection(self):
        self.store.set_backend("nikon_ti")
        self.store.set("z_units_per_um", 50.0)
        self.assertEqual(self.store.backend_kwargs()["z_units_per_um"], 50.0)
        self.store.set_backend("simulated")
        self.assertIn("filter_slots", self.store.backend_kwargs())

    def test_rejects_unknown_backend(self):
        with self.assertRaises(ValueError):
            self.store.set_backend("confocal")


# ── 2. Simulated backend ───────────────────────────────────────────

class TestSimulatedBackend(unittest.TestCase):
    def test_requires_connect(self):
        b = SimulatedMicroscopeBackend()
        self.assertIsNone(b.get_filter())
        with self.assertRaises(MicroscopeError):
            b.set_filter(2)

    def test_turret_moves_and_range_check(self):
        b = SimulatedMicroscopeBackend(filter_slots=6, objective_slots=5)
        b.connect()
        b.set_filter(4)
        b.set_objective(5)
        self.assertEqual(b.get_filter(), 4)
        self.assertEqual(b.get_objective(), 5)
        with self.assertRaises(MicroscopeError):
            b.set_objective(6)
        with self.assertRaises(MicroscopeError):
            b.set_filter(0)

    def test_focus_moves_and_clamps_to_travel(self):
        b = SimulatedMicroscopeBackend(focus_um=100.0,
                                       focus_range_um=(0.0, 500.0))
        b.connect()
        b.move_focus_um(50.0)
        self.assertAlmostEqual(b.get_focus_um(), 150.0)
        b.set_focus_um(9999.0)
        self.assertAlmostEqual(b.get_focus_um(), 500.0)   # hardware travel limit

    def test_factory(self):
        self.assertIsInstance(build_backend("simulated"),
                              SimulatedMicroscopeBackend)
        with self.assertRaises(MicroscopeError):
            build_backend("nope")

    def test_optic_write_support_is_unsupported(self):
        """Neither the base contract nor the simulator claims a writable
        optics database — that would be a lie no hardware has confirmed."""
        self.assertEqual(MicroscopeBackend().probe_optic_write_support(), {})
        self.assertEqual(
            SimulatedMicroscopeBackend().probe_optic_write_support(), {})

    def test_set_optic_name_is_refused_by_default(self):
        """A backend that cannot tell the body its optics must say so, not
        silently accept a rename that goes nowhere."""
        for backend in (MicroscopeBackend(), SimulatedMicroscopeBackend()):
            with self.assertRaises(MicroscopeError):
                backend.set_optic_name("filter", 4, "CY5")


# ── 3. Nikon Ti SDK plumbing (no hardware) ─────────────────────────

class _FlatDevice:
    """Variant exposing ``Position`` as a plain number."""

    def __init__(self, value):
        self.Position = value


class _MipParam:
    """Stands in for the Ti SDK's ``IMipParameter`` wrapper.

    Shape confirmed by introspecting a live ``Nikon.TiScope.NikonTi``
    (SDK 4.4.1.714): the number is on ``RawValue``, bounds on
    ``RangeLowerLimit``/``RangeHigherLimit``, physical unit on ``Unit``, and
    ``DisplayString`` carries human text such as "Device not available".
    """

    def __init__(self, raw, *, lower=None, upper=None, unit=None,
                 display=None, display_scale=None):
        self.RawValue = raw
        self.RangeLowerLimit = lower
        self.RangeHigherLimit = upper
        self.RangeIncrement = 1
        self.Unit = unit
        # display = RawValue × DisplayScale, expressed in Unit.
        self.DisplayScale = display_scale
        self.DisplayOffset = 0.0
        self.DisplayString = display if display is not None else str(raw)
        self.IsReadOnly = False
        self.Name = "Position"


class _FakeOptic:
    """One mounted (or empty) optic.

    Modelled on the real ``IObjective`` / ``IFilterBlock``: an EMPTY slot has
    ``Code == 0`` and every other field **raises** "No database code is
    associated with this optical element."
    """

    def __init__(self, code=0, name=None, mag=None, na=None, wd=None):
        self.Code = code
        self._name = name
        self._mag, self._na, self._wd = mag, na, wd

    def _field(self, value):
        if not self.Code:
            raise RuntimeError(
                "No database code is associated with this optical element.")
        return value

    @property
    def Name(self):
        return self._name if self.Code else self._name  # placeholder survives

    @property
    def Magnification(self):
        return self._field(self._mag)

    @property
    def NumericalAperture(self):
        return self._field(self._na)

    @property
    def WorkingDistance(self):
        return self._field(self._wd)


class _ReadOnlyOptic:
    """An optic whose Name/Code are get-only ``property`` objects.

    This is how comtypes represents a COM property the typelib declares with a
    ``propget`` and no ``propput`` — i.e. what the Ti SDK is expected to look
    like if (as the "No database code is associated with this optical element"
    error implies) these values are resolved from a hardware-sensed code.
    """

    def __init__(self, code, name):
        self._code, self._name = code, name

    @property
    def Code(self):
        return self._code

    @property
    def Name(self):
        return self._name


class _WritableNameOptic:
    """Name declares a setter (propget + propput); Code stays get-only."""

    def __init__(self, code, name):
        self._code, self._name = code, name

    @property
    def Code(self):
        return self._code

    @property
    def Name(self):
        return self._name

    @Name.setter
    def Name(self, value):
        self._name = value


class _IgnoringNameOptic(_WritableNameOptic):
    """Accepts a Name write and silently keeps the old value.

    The SDK's documented failure mode, in a different place: it accepted filter
    slot 999, clamped it to 6 and reported success.
    """

    @_WritableNameOptic.Name.setter
    def Name(self, value):
        pass


class _RecordingOptic:
    """Records every attribute write instead of performing it.

    Lets a test assert exactly WHICH fields a code path wrote — including
    "none at all" — in a way a broad ``except`` around a reintroduced write
    cannot hide. ``Name`` declares a setter so writability checks pass and the
    write is actually attempted; ``Code`` is get-only, as the real body's is
    believed to be.
    """

    def __init__(self, code, name, writes):
        object.__setattr__(self, "_code", code)
        object.__setattr__(self, "_name", name)
        object.__setattr__(self, "_writes", writes)

    @property
    def Code(self):
        return self._code

    @property
    def Name(self):
        return self._name

    @Name.setter
    def Name(self, value):        # declared, but never reached (see below)
        object.__setattr__(self, "_name", value)

    def __setattr__(self, key, value):
        # Intercepts EVERY assignment, property setters included, so the write
        # is recorded and deliberately not performed — the read-back then shows
        # the old value, which is also how a silently-ignored SDK write looks.
        self._writes.append((key, value))


class _LockedOptic(_RecordingOptic):
    """The real Ti-E: declares a Name setter but reports ``CanModify == 0``."""

    @property
    def CanModify(self):
        return 0


class _RefusingOptic(_WritableNameOptic):
    """Declares a Name setter, then refuses the write with a real ``COMError``.

    This is the REAL Ti-E, hardware-verified 2026-08-12: the typelib advertises
    ``propput`` on Name and Code, and the SDK rejects every write at runtime
    with *"Database entry cannot be modified."*
    """

    @_WritableNameOptic.Name.setter
    def Name(self, value):
        from SupportClasses.MicroscopeControl import COMError
        raise COMError(
            -2147352567, "Exception occurred.",
            ("Database entry cannot be modified.",
             "Nikon.TiScope.FilterBlock.1", None, 0, None))


class _FakeCollection:
    """The SDK's ``IObjectives`` / ``IFilterBlocks``: Count + 1-based Item()."""

    def __init__(self, items):
        self._items = list(items)
        self.Count = len(self._items)

    def Item(self, index):
        return self._items[index - 1]


class _TiDevice:
    """A Ti SDK device: every property is an ``IMipParameter``, never a number."""

    def __init__(self, position=1, mounted=True, lower=1, upper=6, unit=None,
                 mount_text=None, display_scale=None):
        self.Position = _MipParam(position, lower=lower, upper=upper, unit=unit,
                                  display_scale=display_scale)
        self.Value = self.Position
        self.IsMounted = _MipParam(
            1 if mounted else 0,
            display=mount_text if mount_text is not None
            else ("Mounted" if mounted else "Device not available"))
        self.Name = "Fake Ti device"


class TestNikonTiPlumbing(unittest.TestCase):
    def test_reads_flat_position(self):
        self.assertEqual(NikonTiSdkBackend._read_position(_FlatDevice(3)), 3)

    def test_missing_position_is_a_clear_error(self):
        with self.assertRaises(MicroscopeError):
            NikonTiSdkBackend._read_position(object())

    def test_unknown_device_reports_clearly(self):
        b = NikonTiSdkBackend()
        b._scope = object()
        b._devices = {}
        with self.assertRaises(MicroscopeError):
            b.set_filter(1)
        self.assertEqual(b.filter_count(), 0)

    def test_not_connected_refuses(self):
        with self.assertRaises(MicroscopeError):
            NikonTiSdkBackend().set_objective(1)

    def test_diagnostics_without_connection(self):
        self.assertIn("not connected", NikonTiSdkBackend().diagnostics())

    # ── Hardware-session regressions (2026-07-30, real Ti-E + SDK 4.4.1) ──

    def test_com_error_text_prefers_the_sdk_message(self):
        """The SDK's own words ('No available instruments.') beat a traceback."""
        from SupportClasses.MicroscopeControl import _com_message

        class _Fake(Exception):
            hresult = -535821127
            details = ("No available instruments.",
                       "Nikon.TiScope.Nosepiece.1", None, 0, None)

        msg = _com_message(_Fake())
        self.assertIn("No available instruments.", msg)
        self.assertIn("Nikon.TiScope.Nosepiece.1", msg)

    def test_com_error_text_without_a_description(self):
        from SupportClasses.MicroscopeControl import _com_message

        class _Fake(Exception):
            hresult = -535821121
            details = ("", "Nikon.TiScope.FilterBlockCassette1.1", None, 0, None)

        msg = _com_message(_Fake())
        self.assertIn("FilterBlockCassette1", msg)
        self.assertNotIn("Traceback", msg)

    def test_junk_position_is_diagnosed_not_int_crashed(self):
        """With no live instrument the SDK returned raw pointer BYTES, and a
        bare int() died with 'invalid literal for int()'. Explain instead."""
        with self.assertRaises(MicroscopeError) as ctx:
            NikonTiSdkBackend._coerce_int(b"\xb0{n%<\x02\x00\x00", "Position")
        self.assertIn("no live instrument", str(ctx.exception))

    def test_implausible_turret_index_rejected(self):
        with self.assertRaises(MicroscopeError):
            NikonTiSdkBackend._coerce_int(123456, "Position", max_abs=99)

    def test_focus_units_are_not_bounded_by_the_turret_ceiling(self):
        """A Z position is in device units and legitimately reaches 100000s —
        it must NOT be judged against the turret index ceiling."""
        self.assertEqual(
            NikonTiSdkBackend._coerce_int(500000, "Position"), 500000)

    def test_unmounted_device_reports_the_sdk_wording(self):
        dev = _TiDevice(position=1, mounted=False,
                        mount_text="Device not available")
        with self.assertRaises(MicroscopeError) as ctx:
            NikonTiSdkBackend._read_position(dev)
        msg = str(ctx.exception)
        self.assertIn("not available", msg)
        self.assertIn("Device not available", msg)   # the SDK's own words

    def test_reads_rawvalue_off_the_parameter_wrapper(self):
        """The SDK returns IMipParameter, never a bare number — the value is
        on RawValue. Reading the wrapper itself is what produced the pointer
        bytes that crashed int() on hardware."""
        dev = _TiDevice(position=4, mounted=True)
        self.assertEqual(NikonTiSdkBackend._read_position(dev), 4)

    def test_writes_go_to_rawvalue(self):
        dev = _TiDevice(position=1, mounted=True)
        NikonTiSdkBackend._write_position(dev, 5)
        self.assertEqual(dev.Position.RawValue, 5)

    def test_range_comes_from_the_declared_range(self):
        dev = _TiDevice(position=1, mounted=True, lower=1, upper=6)
        self.assertEqual(NikonTiSdkBackend._read_range(dev, 99), 6)

    def test_reads_mounted_objectives_with_magnification_lookup(self):
        """``Objective.Magnification`` is an INDEX into the SDK's table, not a
        magnification — hardware codes 5/7/9 mean 4x/10x/20x."""
        coll = _FakeCollection([
            _FakeOptic(code=106, name="MRH20040", mag=5, na=0.13, wd=16.4),
            _FakeOptic(code=109, name="MRH20100", mag=7, na=0.30, wd=15.2),
            _FakeOptic(code=0),          # empty: every field raises
        ])
        b = NikonTiSdkBackend()
        b._scope = object()
        b._devices = {"objective": _TiDevice(position=1, mounted=True)}
        b._devices["objective"].Objectives = coll

        optics = b.mounted_objectives()
        self.assertEqual(len(optics), 3)
        self.assertTrue(optics[0].present)
        self.assertEqual(optics[0].label, "4x")
        self.assertEqual(optics[0].code, "MRH20040")
        self.assertIn("NA 0.13", optics[0].detail)
        self.assertIn("WD 16.4 mm", optics[0].detail)
        self.assertEqual(optics[1].label, "10x")
        self.assertFalse(optics[2].present)      # empty slot, no crash
        self.assertEqual(optics[2].detail, "empty")
        self.assertEqual(b.objective_names(), ("4x", "10x", ""))

    def test_reads_mounted_filter_cubes(self):
        coll = _FakeCollection([
            _FakeOptic(code=4, name="DAPI"),
            _FakeOptic(code=15, name="FITC"),
            _FakeOptic(code=0, name="-----"),    # the real 'empty' marker
        ])
        b = NikonTiSdkBackend()
        b._scope = object()
        b._devices = {"filter": _TiDevice(position=1, mounted=True)}
        b._devices["filter"].FilterBlocks = coll

        optics = b.mounted_filters()
        self.assertEqual([o.label for o in optics], ["DAPI", "FITC", ""])
        self.assertEqual([o.present for o in optics], [True, True, False])

    def test_mounted_is_empty_when_the_body_cannot_report(self):
        b = NikonTiSdkBackend()
        b._scope = object()
        b._devices = {"filter": _TiDevice(position=1, mounted=True)}
        self.assertEqual(b.mounted_filters(), ())   # no FilterBlocks attribute

    # ── Optics write-support check (READ-ONLY) ──────────────────────

    def _optic_backend(self, logical, items):
        b = NikonTiSdkBackend()
        b._scope = object()
        dev = _TiDevice(position=1, mounted=True)
        setattr(dev, "FilterBlocks" if logical == "filter" else "Objectives",
                _FakeCollection(items))
        b._devices = {logical: dev}
        return b

    def test_write_support_reads_the_declared_setters(self):
        """comtypes exposes a COM property as a Python ``property`` — get-only
        when the typelib declares no propput. ``_ReadOnlyOptic`` models a body
        that declares neither field settable."""
        b = self._optic_backend("objective", [_ReadOnlyOptic(106, "MRH20040")])
        self.assertEqual(b.probe_optic_write_support(),
                         {"objective": {"Name": False, "Code": False}})

    def test_write_support_detects_a_settable_name(self):
        b = self._optic_backend("filter", [_WritableNameOptic(4, "DAPI")])
        self.assertEqual(b.probe_optic_write_support()["filter"]["Name"], True)

    def test_write_support_is_undeterminable_without_type_information(self):
        """A late-bound wrapper exposes no descriptor at all. That must report
        None (unknown), never a guess in either direction."""
        b = self._optic_backend("objective",
                                [_FakeOptic(code=106, name="MRH20040")])
        self.assertIsNone(b.probe_optic_write_support()["objective"]["Code"])

    def test_write_support_check_writes_nothing_at_all(self):
        """⚠ THE LOAD-BEARING TEST. An earlier cut settled writability by
        writing each field's own current value back to itself, on the theory
        that a same-value write is a no-op. ``tucam_backend._capa_set`` records
        a hardware-verified case where exactly that was DESTRUCTIVE. Here the
        analogous field (``Code``) resolves the working distance that bounds a
        focus sweep, so this check must never write.

        Asserts against a RECORDED list rather than a raising fake, so a
        reintroduced write cannot hide inside a broad ``except``."""
        writes: list = []
        b = self._optic_backend("objective",
                                [_RecordingOptic(106, "MRH20040", writes)])
        b.probe_optic_write_support()
        self.assertEqual(writes, [])

    def test_write_support_empty_when_nothing_enumerable(self):
        b = NikonTiSdkBackend()
        b._scope = object()
        b._devices = {}
        self.assertEqual(b.probe_optic_write_support(), {})

    # ── set_optic_name: the ONE genuine write ───────────────────────

    def test_set_optic_name_writes_and_verifies(self):
        optic = _WritableNameOptic(4, "-----")
        b = self._optic_backend("filter", [_WritableNameOptic(1, "DAPI"),
                                           _WritableNameOptic(2, "FITC"),
                                           _WritableNameOptic(3, "TxRed"),
                                           optic])
        self.assertEqual(b.set_optic_name("filter", 4, "CY5"), "CY5")
        self.assertEqual(optic.Name, "CY5")

    def test_set_optic_name_never_touches_code(self):
        """Code resolves an objective's NA/working distance, which bound a
        focus sweep — a rename must not be able to disturb it."""
        writes: list = []
        b = self._optic_backend("filter", [_RecordingOptic(4, "DAPI", writes)])
        try:
            b.set_optic_name("filter", 1, "CY5")
        except MicroscopeError:
            pass
        self.assertEqual([k for k, _v in writes], ["Name"])

    def test_set_optic_name_refuses_a_declared_readonly_field(self):
        b = self._optic_backend("filter", [_ReadOnlyOptic(4, "DAPI")])
        with self.assertRaises(MicroscopeError) as ctx:
            b.set_optic_name("filter", 1, "CY5")
        self.assertIn("read-only", str(ctx.exception))

    def test_set_optic_name_raises_when_the_write_does_not_take(self):
        """⚠ This SDK has already been caught accepting an out-of-range turret
        index, clamping it and reporting SUCCESS. A silently-ignored rename is
        the same failure mode, so the value is read back and disagreement is an
        error — never a reported success."""
        b = self._optic_backend("filter", [_IgnoringNameOptic(4, "DAPI")])
        with self.assertRaises(MicroscopeError) as ctx:
            b.set_optic_name("filter", 1, "CY5")
        self.assertIn("did not take", str(ctx.exception))

    def test_set_optic_name_honours_the_bodys_own_canmodify_flag(self):
        """``CanModify`` is Nikon's own advertised gate — *"Determines if
        properties such as 'Name' can be modified for this optical element"* —
        and it reads 0 on every slot of the real Ti-E. Asking it turns a COM
        error into a plain explanation, and it must be asked BEFORE any write
        (assert on the recorded writes, not just the message)."""
        writes: list = []
        b = self._optic_backend("filter", [_LockedOptic(0, "-----", writes)])
        with self.assertRaises(MicroscopeError) as ctx:
            b.set_optic_name("filter", 1, "CY5")
        self.assertIn("CanModify=0", str(ctx.exception))
        self.assertEqual(writes, [])          # refused without touching it

    def test_set_optic_name_still_proceeds_when_canmodify_is_true(self):
        """The gate must not become a blanket refusal — a body that permits it
        (or does not expose the flag) still goes through."""
        optic = _WritableNameOptic(4, "-----")
        optic.CanModify = 1
        b = self._optic_backend("filter", [optic])
        self.assertEqual(b.set_optic_name("filter", 1, "CY5"), "CY5")

    def test_set_optic_name_surfaces_the_ti_e_runtime_refusal(self):
        """⚠ THE HARDWARE FINDING, pinned (real Ti-E, SDK 4.4.1.714,
        2026-08-12). The body DECLARES Name and Code settable and then refuses
        at runtime — so a declared setter is a FALSE POSITIVE and only a real
        attempt settles it. The operator must see Nikon's own words, not a
        traceback and not a success."""
        b = self._optic_backend("filter", [_RefusingOptic(0, "-----")])
        with self.assertRaises(MicroscopeError) as ctx:
            b.set_optic_name("filter", 1, "CY5")
        msg = str(ctx.exception)
        self.assertIn("Database entry cannot be modified.", msg)
        self.assertIn("Nikon.TiScope.FilterBlock.1", msg)
        self.assertNotIn("Traceback", msg)
        # ...and the optimistic typelib claim is exactly what makes this trap.
        self.assertEqual(b.probe_optic_write_support()["filter"]["Name"], True)

    def test_filter_catalogue_indices_match_this_rig(self):
        """The names come from Nikon's own 0-based FilterBlockNames.txt, which
        this rig's live readings pin: codes 4/15/23 -> DAPI/FITC/TxRed, and Cy5
        is 25. Guards the "empty slot" rule those readings also confirmed."""
        coll = _FakeCollection([
            _FakeOptic(code=4, name="DAPI"),
            _FakeOptic(code=15, name="FITC"),
            _FakeOptic(code=23, name="TxRed"),
            _FakeOptic(code=0, name="-----"),   # slot 4 as the body reports it
        ])
        b = NikonTiSdkBackend()
        b._scope = object()
        b._devices = {"filter": _TiDevice(position=1, mounted=True)}
        b._devices["filter"].FilterBlocks = coll
        optics = b.mounted_filters()
        self.assertEqual([o.label for o in optics],
                         ["DAPI", "FITC", "TxRed", ""])
        self.assertEqual([o.present for o in optics],
                         [True, True, True, False])

    def test_set_optic_name_rejects_a_bad_slot_and_group(self):
        b = self._optic_backend("filter", [_WritableNameOptic(4, "DAPI")])
        with self.assertRaises(MicroscopeError):
            b.set_optic_name("filter", 7, "CY5")      # only 1 slot enumerable
        with self.assertRaises(MicroscopeError):
            b.set_optic_name("turntable", 1, "CY5")   # unknown group

    def test_magnification_table_covers_the_known_codes(self):
        table = NikonTiSdkBackend._MAGNIFICATIONS
        self.assertEqual(table[5], "4")
        self.assertEqual(table[7], "10")
        self.assertEqual(table[9], "20")

    def test_range_falls_back_when_undeclared(self):
        dev = _TiDevice(position=1, mounted=True, lower=None, upper=None)
        self.assertEqual(NikonTiSdkBackend._read_range(dev, 6), 6)

    def test_focus_scale_uses_displayscale_not_the_unit_alone(self):
        """⚠ REGRESSION (operator, 2026-07-30: "1000 µm only moved 10 µm").

        ``Unit`` describes the DISPLAY value, not ``RawValue``; the conversion
        is ``display = RawValue × DisplayScale``. Reading Unit=='um' as
        "µm-native" gave factor 1.0 and made every move 40× too small.
        Real Ti-E values: DisplayScale 0.025 µm/raw ⇒ 40 units/µm."""
        b = NikonTiSdkBackend(z_units_per_um=100.0)
        b._scope = object()
        b._devices = {"focus": _TiDevice(position=30844, mounted=True,
                                         unit="um", display_scale=0.025)}
        self.assertEqual(b.focus_units_per_um(), 40.0)
        # 30844 raw × 0.025 = 771.100 µm — matches the SDK's own DisplayString.
        self.assertAlmostEqual(b.get_focus_um(), 771.1, places=3)

    def test_focus_move_of_1000um_writes_the_right_raw_delta(self):
        """The operator's exact case: ask for 1000 µm, get 1000 µm."""
        b = NikonTiSdkBackend()
        b._scope = object()
        dev = _TiDevice(position=0, mounted=True, unit="um",
                        display_scale=0.025, lower=0, upper=400000)
        b._devices = {"focus": dev}
        b.set_focus_um(1000.0)
        self.assertEqual(dev.Position.RawValue, 40000)   # 40000 × 0.025 = 1000
        self.assertAlmostEqual(b.get_focus_um(), 1000.0)

    def test_focus_scale_handles_a_nanometre_display_unit(self):
        b = NikonTiSdkBackend()
        b._scope = object()
        b._devices = {"focus": _TiDevice(position=5000, mounted=True,
                                         unit="nm", display_scale=1.0)}
        self.assertEqual(b.focus_units_per_um(), 1000.0)
        self.assertAlmostEqual(b.get_focus_um(), 5.0)

    def test_focus_scale_falls_back_without_displayscale(self):
        b = NikonTiSdkBackend(z_units_per_um=100.0)
        b._scope = object()
        b._devices = {"focus": _TiDevice(position=12345, mounted=True,
                                         unit="um", display_scale=None)}
        self.assertEqual(b.focus_units_per_um(), 100.0)
        self.assertAlmostEqual(b.get_focus_um(), 123.45)

    def test_focus_travel_converts_through_displayscale(self):
        """400000 raw is 10 mm of travel, NOT 400 mm."""
        b = NikonTiSdkBackend()
        b._scope = object()
        b._devices = {"focus": _TiDevice(position=0, mounted=True, unit="um",
                                         display_scale=0.025,
                                         lower=0, upper=400000)}
        self.assertEqual(b.focus_limits_um(), (0.0, 10000.0))

    def test_out_of_range_turret_index_is_refused(self):
        """⚠ HARDWARE-VERIFIED 2026-07-30: asked for filter slot 999, the Ti
        SDK silently CLAMPED to 6 and reported SUCCESS. For a discrete selector
        that is the worst failure mode — imaging through the wrong cube while
        the app says the move was fine. Must refuse, not write."""
        b = NikonTiSdkBackend()
        b._scope = object()
        dev = _TiDevice(position=1, mounted=True, lower=1, upper=6)
        b._devices = {"filter": dev}
        with self.assertRaises(MicroscopeError) as ctx:
            b.set_filter(999)
        self.assertIn("1-6", str(ctx.exception))
        self.assertEqual(dev.Position.RawValue, 1)      # never written

    def test_in_range_turret_index_still_moves(self):
        b = NikonTiSdkBackend()
        b._scope = object()
        dev = _TiDevice(position=1, mounted=True, lower=1, upper=6)
        b._devices = {"objective": dev}
        b.set_objective(6)
        self.assertEqual(dev.Position.RawValue, 6)

    def test_focus_clamps_rather_than_refusing(self):
        """A continuous axis differs from a turret: hitting the end of travel
        during a jog is normal, so it clamps instead of erroring."""
        b = NikonTiSdkBackend()
        b._scope = object()
        dev = _TiDevice(position=0, mounted=True, unit="um",
                        lower=0, upper=400000)
        b._devices = {"focus": dev}
        b.set_focus_um(999999.0)
        self.assertEqual(dev.Position.RawValue, 400000)
        b.set_focus_um(-500.0)
        self.assertEqual(dev.Position.RawValue, 0)

    def test_focus_limits_scale_with_the_display_unit(self):
        b = NikonTiSdkBackend()
        b._scope = object()
        b._devices = {"focus": _TiDevice(position=0, mounted=True, unit="nm",
                                         display_scale=1.0,
                                         lower=0, upper=1_000_000)}
        self.assertEqual(b.focus_limits_um(), (0.0, 1000.0))

    def test_progid_order_matches_the_shipped_sdk(self):
        """Read out of NikonTi.dll v4.4.1.714 (TiSDKRedist64): the Ti/Ti-E SDK
        publishes Nikon.TiScope.* and contains NO 'LvMic' string at all, so the
        TiScope ProgID must be tried FIRST."""
        from SupportClasses.MicroscopeControl import TI_PROG_IDS
        self.assertEqual(TI_PROG_IDS[0], "Nikon.TiScope.NikonTi")
        self.assertIn("Nikon.LvMic.NikonTi", TI_PROG_IDS)   # kept as fallback

    def test_device_aliases_match_the_sdk_class_names(self):
        """Nosepiece / FilterBlockCassette1 / ZDrive are the SDK's own names."""
        from SupportClasses.MicroscopeControl import _TI_DEVICE_ALIASES
        self.assertEqual(_TI_DEVICE_ALIASES["objective"][0], "Nosepiece")
        self.assertEqual(
            _TI_DEVICE_ALIASES["filter"][0].format(n=1), "FilterBlockCassette1")
        self.assertEqual(_TI_DEVICE_ALIASES["focus"][0], "ZDrive")


# ── 4. Micro-Manager backend ───────────────────────────────────────

class TestMicroManagerBackend(unittest.TestCase):
    def test_refuses_without_config(self):
        b = MicroManagerBackend(config_path="")
        with self.assertRaises(MicroscopeError) as ctx:
            b.connect()
        self.assertIn("configuration", str(ctx.exception))

    def test_reads_are_safe_when_disconnected(self):
        b = MicroManagerBackend(config_path="x.cfg")
        self.assertIsNone(b.get_focus_um())
        self.assertEqual(b.filter_count(), 0)
        self.assertEqual(b.filter_names(), ())


# ── 5. Controller ──────────────────────────────────────────────────

class TestMicroscopeController(_StoreCase):
    def _connected(self):
        ctrl = MicroscopeController(store=self.store, threaded=False)
        ctrl.connect()
        return ctrl

    def test_connect_populates_state(self):
        ctrl = self._connected()
        st = ctrl.state()
        self.assertTrue(st.connected)
        self.assertEqual(st.backend, "simulated")
        self.assertEqual(st.filter_position, 1)
        self.assertEqual(st.objective_position, 1)
        self.assertEqual(st.filter_count, 6)
        self.assertIsNotNone(st.focus_um)
        self.assertIsNone(st.error)

    def test_set_filter_and_objective(self):
        ctrl = self._connected()
        ctrl.set_filter(3)
        ctrl.set_objective(2)
        st = ctrl.state()
        self.assertEqual(st.filter_position, 3)
        self.assertEqual(st.objective_position, 2)

    def test_focus_relative_and_absolute(self):
        ctrl = self._connected()
        start = ctrl.state().focus_um
        ctrl.move_focus_um(-25.0)
        self.assertAlmostEqual(ctrl.state().focus_um, start - 25.0)
        ctrl.set_focus_um(1234.0)
        self.assertAlmostEqual(ctrl.state().focus_um, 1234.0)

    def test_operator_soft_limits_clamp_focus(self):
        self.store.set_focus_soft_limits_um(4000.0, 6000.0)
        ctrl = self._connected()
        ctrl.set_focus_um(9999.0)
        self.assertAlmostEqual(ctrl.state().focus_um, 6000.0)
        ctrl.move_focus_um(-9999.0)
        self.assertAlmostEqual(ctrl.state().focus_um, 4000.0)

    def test_operations_fail_cleanly_when_disconnected(self):
        ctrl = MicroscopeController(store=self.store, threaded=False)
        op = ctrl.set_filter(2)
        self.assertIsNotNone(op.error)
        self.assertIn("not connected", ctrl.state().error)
        self.assertFalse(ctrl.state().connected)

    def test_error_clears_on_next_success(self):
        ctrl = MicroscopeController(store=self.store, threaded=False)
        ctrl.set_filter(2)                       # fails: not connected
        self.assertIsNotNone(ctrl.state().error)
        ctrl.connect()
        ctrl.set_filter(2)
        self.assertIsNone(ctrl.state().error)

    def test_bad_slot_is_reported_not_raised(self):
        ctrl = self._connected()
        op = ctrl.set_filter(99)
        self.assertIsNotNone(op.error)
        self.assertEqual(ctrl.state().filter_position, 1)   # unchanged

    def test_disconnect_clears_positions(self):
        ctrl = self._connected()
        ctrl.disconnect()
        st = ctrl.state()
        self.assertFalse(st.connected)
        self.assertIsNone(st.filter_position)
        self.assertIsNone(st.focus_um)

    def test_probe_optic_write_support_updates_state(self):
        """Wiring only — the simulated backend truthfully reports {} (see
        TestSimulatedBackend); this proves the op reaches the backend and the
        result lands on the cached state, not that hardware supports it."""
        ctrl = self._connected()
        self.assertEqual(ctrl.state().optic_write_support, {})
        op = ctrl.probe_optic_write_support()
        self.assertIsNone(op.error)
        self.assertEqual(ctrl.state().optic_write_support, {})
        ctrl.disconnect()
        self.assertEqual(ctrl.state().optic_write_support, {})

    def test_probe_optic_write_support_fails_cleanly_when_disconnected(self):
        ctrl = MicroscopeController(store=self.store, threaded=False)
        op = ctrl.probe_optic_write_support()
        self.assertIsNotNone(op.error)

    def test_set_optic_name_reports_the_refusal(self):
        """The simulated body cannot be told its optics; the op must carry the
        error, and ``result`` must stay None rather than implying success."""
        ctrl = self._connected()
        op = ctrl.set_optic_name("filter", 4, "CY5")
        self.assertIsNotNone(op.error)
        self.assertIsNone(op.result)

    def test_set_optic_name_carries_the_verified_value_inline(self):
        """⚠ REGRESSION: the op's fn closes over the op itself to report a
        result. Built naively (``op = self._submit(...)`` with ``_do``
        referencing ``op``), the INLINE threaded=False path — the one the whole
        suite uses — runs ``_do`` before ``op`` is bound and dies with a
        NameError swallowed into op.error."""
        ctrl = MicroscopeController(store=self.store, threaded=False)
        ctrl.connect()

        class _Body(SimulatedMicroscopeBackend):
            def set_optic_name(self, logical, position, name):
                return name.upper()

        ctrl._backend = _Body()
        ctrl._backend.connect()
        op = ctrl.set_optic_name("filter", 4, "cy5")
        self.assertIsNone(op.error)
        self.assertEqual(op.result, "CY5")

    def test_set_optic_name_carries_the_verified_value_threaded(self):
        ctrl = MicroscopeController(store=self.store, threaded=True)
        self.addCleanup(ctrl.shutdown)
        ctrl.connect()
        ctrl.wait_idle(timeout=10.0)

        class _Body(SimulatedMicroscopeBackend):
            def set_optic_name(self, logical, position, name):
                return name.upper()

        ctrl._backend = _Body()
        ctrl._backend.connect()
        op = ctrl.set_optic_name("filter", 4, "cy5")
        self.assertTrue(ctrl.wait_idle(timeout=10.0))
        self.assertIsNone(op.error)
        self.assertEqual(op.result, "CY5")

    def test_reconnect_replaces_previous_backend(self):
        ctrl = self._connected()
        first = ctrl._backend
        ctrl.connect()
        self.assertIsNot(ctrl._backend, first)
        self.assertFalse(first.is_connected)

    def test_threaded_worker_runs_ops_in_order(self):
        ctrl = MicroscopeController(store=self.store, threaded=True)
        self.addCleanup(ctrl.shutdown)
        ctrl.connect()
        ctrl.set_filter(5)
        ctrl.set_objective(3)
        ctrl.set_focus_um(777.0)
        self.assertTrue(ctrl.wait_idle(timeout=10.0))
        st = ctrl.state()
        self.assertTrue(st.connected)
        self.assertEqual(st.filter_position, 5)
        self.assertEqual(st.objective_position, 3)
        self.assertAlmostEqual(st.focus_um, 777.0)
        self.assertFalse(st.busy)          # busy clears once the queue drains

    def test_threaded_shutdown_disconnects(self):
        ctrl = MicroscopeController(store=self.store, threaded=True)
        ctrl.connect()
        self.assertTrue(ctrl.wait_idle(timeout=10.0))
        ctrl.shutdown()
        self.assertFalse(ctrl.state().connected)

    def test_diagnostics_without_backend(self):
        ctrl = MicroscopeController(store=self.store, threaded=False)
        self.assertIn("not connected", ctrl.diagnostics())


# ── 6. Panel widget ────────────────────────────────────────────────

class TestMicroscopePanel(_StoreCase):
    def _panel(self, *, connect=True):
        from gui.widgets.microscope_panel import MicroscopePanel
        ctrl = MicroscopeController(store=self.store, threaded=False)
        if connect:
            ctrl.connect()
        panel = MicroscopePanel(controller=ctrl, store=self.store)
        panel._render(force=True)
        return panel, ctrl

    def test_combo_shows_assigned_names(self):
        self.store.set_filter_labels({1: "DAPI", 2: "FITC"})
        self.store.set_objective_labels({3: "20x"})
        panel, _ = self._panel()
        panel._rebuild_slot_combos()
        self.assertEqual(panel._filter_combo.itemText(0), "1 · DAPI")
        self.assertEqual(panel._objective_combo.itemText(2), "3 · 20x")
        # Item data is the 1-based physical slot number.
        self.assertEqual(panel._filter_combo.itemData(0), 1)

    def test_unnamed_slot_falls_back_to_what_the_body_reports(self):
        """The operator name wins; otherwise show the microscope's own name
        (a motorised body knows its cubes), and only then '(empty)'."""
        self.store.set_filter_labels({1: "my DAPI cube"})
        panel, _ = self._panel()
        panel._rebuild_slot_combos()
        # 1 = operator name overrides the body's "DAPI"
        self.assertEqual(panel._filter_combo.itemText(0), "1 · my DAPI cube")
        # 3 = unnamed, but the body reports TxRed
        self.assertEqual(panel._filter_combo.itemText(2), "3 · TxRed")
        # 6 = unnamed and the body reports nothing fitted
        self.assertEqual(panel._filter_combo.itemText(5), "6 · (empty)")

    def test_selecting_a_cube_switches_the_turret(self):
        panel, ctrl = self._panel()
        panel._filter_combo.setCurrentIndex(3)      # slot 4
        self.assertEqual(ctrl.state().filter_position, 4)

    def test_selecting_an_objective_switches_the_nosepiece(self):
        panel, ctrl = self._panel()
        panel._objective_combo.setCurrentIndex(1)   # position 2
        self.assertEqual(ctrl.state().objective_position, 2)

    def test_current_position_is_reflected_without_commanding(self):
        panel, ctrl = self._panel()
        ctrl.set_filter(5)
        panel._render(force=True)
        self.assertEqual(panel._filter_combo.currentData(), 5)
        # Rendering the hardware's own position must not re-command it.
        self.assertEqual(ctrl.state().filter_position, 5)

    def test_focus_jog_uses_step_and_direction(self):
        panel, ctrl = self._panel()
        start = ctrl.state().focus_um
        panel._step_spin.setValue(10.0)
        panel._jog_focus(+1)
        self.assertAlmostEqual(ctrl.state().focus_um, start + 10.0)
        panel._jog_focus(-1)
        self.assertAlmostEqual(ctrl.state().focus_um, start)

    def test_focus_direction_flag_inverts_the_buttons(self):
        self.store.set_focus_up_is_positive(False)
        panel, ctrl = self._panel()
        start = ctrl.state().focus_um
        panel._step_spin.setValue(10.0)
        panel._jog_focus(+1)          # "Up" now counts down on this body
        self.assertAlmostEqual(ctrl.state().focus_um, start - 10.0)

    def test_focus_jog_stays_live_during_a_move(self):
        """Regression: busy-gating the jog buttons silently DROPPED clicks.

        Repeated small focus steps are the most common microscope interaction,
        and queued deltas are additive, so the buttons must stay live while a
        move is in flight. Absolute go-to and the turrets stay gated — those are
        not additive."""
        from dataclasses import replace
        panel, ctrl = self._panel()
        ctrl._state = replace(ctrl._state, busy=True)
        panel._render(force=True)
        self.assertTrue(panel._btn_up.isEnabled())
        self.assertTrue(panel._btn_down.isEnabled())
        self.assertFalse(panel._btn_goto.isEnabled())
        self.assertFalse(panel._filter_combo.isEnabled())
        self.assertFalse(panel._objective_combo.isEnabled())

    def test_repeated_jogs_accumulate(self):
        panel, ctrl = self._panel()
        start = ctrl.state().focus_um
        panel._step_spin.setValue(5.0)
        for _ in range(3):
            panel._btn_down.click()
        self.assertAlmostEqual(ctrl.state().focus_um, start - 15.0)

    def test_background_refresh_keeps_the_error_readable(self):
        """A refresh fires every ~1 s; clearing the error there would wipe the
        message before the operator could read it."""
        panel, ctrl = self._panel()
        ctrl.set_filter(99)                      # refused: out of range
        self.assertIsNotNone(ctrl.state().error)
        ctrl.refresh()
        self.assertIsNotNone(ctrl.state().error)
        ctrl.set_filter(2)                       # next successful command clears
        self.assertIsNone(ctrl.state().error)

    # ── Dropdown vs the background poll (operator, 2026-07-30) ────────

    def test_background_poll_is_paused_while_a_dropdown_is_open(self):
        """⚠ REGRESSION: "every time it reads it cancels the dropdown box".

        The ~1 s refresh sets busy=True, which disabled the combo mid-selection
        and snapped the list shut under the operator's cursor."""
        panel, ctrl = self._panel()
        calls = []
        ctrl.refresh = lambda: calls.append(1)
        panel._last_refresh = 0.0            # a poll would otherwise be due
        panel._any_popup_open = lambda: True
        panel._tick()
        self.assertEqual(calls, [], "polled the body with a drop-down open")

    def test_open_combo_keeps_its_selection_during_a_render(self):
        panel, ctrl = self._panel()
        ctrl.set_filter(1)
        panel._render(force=True)
        # Operator opens the filter list; hardware meanwhile reports slot 5.
        panel._popup_open = lambda combo: combo is panel._filter_combo
        ctrl.set_filter(5)
        panel._render(force=True)
        # The open list is untouched…
        self.assertEqual(panel._filter_combo.currentData(), 1)
        self.assertTrue(panel._filter_combo.isEnabled())
        # …while the closed one still tracks the hardware.
        self.assertEqual(panel._objective_combo.currentData(),
                         ctrl.state().objective_position)

    def test_closed_combo_still_follows_the_hardware(self):
        panel, ctrl = self._panel()
        ctrl.set_filter(4)
        panel._render(force=True)
        self.assertEqual(panel._filter_combo.currentData(), 4)

    def test_step_change_persists(self):
        panel, _ = self._panel()
        panel._step_spin.setValue(42.0)
        self.assertEqual(self.store.focus_step_um(), 42.0)

    def test_absolute_goto(self):
        panel, ctrl = self._panel()
        panel._goto_spin.setValue(321.0)
        panel._goto_focus()
        self.assertAlmostEqual(ctrl.state().focus_um, 321.0)

    def test_disconnected_panel_is_inert(self):
        panel, ctrl = self._panel(connect=False)
        self.assertFalse(panel._filter_combo.isEnabled())
        self.assertFalse(panel._btn_up.isEnabled())
        panel._jog_focus(+1)                       # must not raise or command
        panel._filter_combo.setCurrentIndex(2)
        self.assertIsNone(ctrl.state().filter_position)

    def test_connect_button_toggles(self):
        panel, ctrl = self._panel(connect=False)
        self.assertEqual(panel._btn_connect.text(), "Connect")
        panel._toggle_connect()
        self.assertTrue(ctrl.state().connected)
        self.assertEqual(panel._btn_connect.text(), "Disconnect")
        panel._toggle_connect()
        self.assertFalse(ctrl.state().connected)

    def test_error_is_surfaced(self):
        panel, ctrl = self._panel(connect=False)
        ctrl.set_filter(2)                         # fails: not connected
        panel._render(force=True)
        self.assertTrue(panel._error_lbl.isVisibleTo(panel))
        self.assertIn("not connected", panel._error_lbl.text())


# ── 7. Setup dialog ────────────────────────────────────────────────

class TestMicroscopeSetupPanel(_StoreCase):
    """The ONE setup surface — hosted by both the Hardware Setup tab and the
    jog card's ⚙ dialog."""

    def _panel(self, *, connect=False):
        from gui.pages.hardware.microscope_setup_panel import (
            MicroscopeSetupPanel)
        ctrl = MicroscopeController(store=self.store, threaded=False)
        if connect:
            ctrl.connect()
        return MicroscopeSetupPanel(self.store, controller=ctrl), ctrl

    def _name(self, table, pos):
        return table._rows[pos]["edit"].text()

    def _set_name(self, table, pos, text):
        table._rows[pos]["edit"].setText(text)

    def test_loads_current_configuration(self):
        self.store.set_backend("nikon_ti")
        self.store.set_filter_label(2, "FITC")
        self.store.set_focus_step_um(7.5)
        panel, _ = self._panel()
        self.assertEqual(panel._backend_combo.currentData(), "nikon_ti")
        self.assertEqual(self._name(panel._filter_table, 2), "FITC")
        self.assertEqual(panel._focus_step_spin.value(), 7.5)

    def test_commit_writes_everything(self):
        panel, _ = self._panel()
        panel._backend_combo.setCurrentIndex(1)          # Nikon Ti SDK
        panel._z_units_spin.setValue(50.0)
        self._set_name(panel._filter_table, 1, "DAPI")
        self._set_name(panel._objective_table, 4, "40x")
        panel._focus_step_spin.setValue(3.0)
        panel._focus_dir_chk.setChecked(False)
        self.assertTrue(panel.commit())
        self.assertEqual(self.store.get_backend(), "nikon_ti")
        self.assertEqual(self.store.get("z_units_per_um"), 50.0)
        self.assertEqual(self.store.filter_label(1), "DAPI")
        self.assertEqual(self.store.objective_label(4), "40x")
        self.assertEqual(self.store.focus_step_um(), 3.0)
        self.assertFalse(self.store.focus_up_is_positive())

    def test_editing_without_commit_leaves_the_store_untouched(self):
        self.store.set_filter_label(1, "DAPI")
        panel, _ = self._panel()
        panel._backend_combo.setCurrentIndex(1)
        self._set_name(panel._filter_table, 1, "something else")
        panel._focus_step_spin.setValue(99.0)
        # No commit() — e.g. the dialog was cancelled.
        self.assertEqual(self.store.get_backend(), "simulated")
        self.assertEqual(self.store.filter_label(1), "DAPI")
        self.assertNotEqual(self.store.focus_step_um(), 99.0)

    def test_slot_count_change_keeps_typed_names(self):
        panel, _ = self._panel()
        self._set_name(panel._filter_table, 6, "Cy5")
        panel._filter_slots_spin.setValue(3)     # shrink — row 6 is gone
        panel._filter_slots_spin.setValue(6)     # grow back
        self.assertEqual(self._name(panel._filter_table, 6), "Cy5")

    def test_soft_limits_are_opt_in(self):
        panel, _ = self._panel()
        panel._focus_limits_chk.setChecked(True)
        panel._focus_min_spin.setValue(100.0)
        panel._focus_max_spin.setValue(900.0)
        panel.commit()
        self.assertEqual(self.store.focus_soft_limits_um(), (100.0, 900.0))

        panel2, _ = self._panel()
        panel2._focus_limits_chk.setChecked(False)
        panel2.commit()
        self.assertEqual(self.store.focus_soft_limits_um(), (None, None))

    def test_micromanager_requires_a_config(self):
        panel, _ = self._panel()
        panel._backend_combo.setCurrentIndex(2)   # Micro-Manager, no .cfg
        # The refusal shows a modal warning; stub it so the test can run
        # headless (an unpatched QMessageBox blocks forever under offscreen Qt).
        with mock.patch.object(setup_panel_mod.QMessageBox, "warning") as warn:
            self.assertFalse(panel.commit())
        warn.assert_called_once()
        self.assertEqual(self.store.get_backend(), "simulated")

    # ── Read from microscope ──────────────────────────────────────

    def test_read_from_microscope_fills_names(self):
        panel, ctrl = self._panel(connect=True)
        with mock.patch.object(setup_panel_mod.QMessageBox, "information"):
            panel._read_filters()
            panel._read_objectives()
        # The simulated body reports DAPI/FITC/TxRed/Cy5 and 4x/10x/20x.
        self.assertEqual(self._name(panel._filter_table, 1), "DAPI")
        self.assertEqual(self._name(panel._filter_table, 4), "Cy5")
        self.assertEqual(self._name(panel._objective_table, 2), "10x")
        # Unfitted slots are left alone rather than blanked.
        self.assertEqual(self._name(panel._filter_table, 6), "")
        panel.commit()
        self.assertEqual(self.store.filter_label(3), "TxRed")
        self.assertEqual(self.store.objective_label(3), "20x")

    def test_read_from_microscope_needs_a_connection(self):
        panel, _ = self._panel(connect=False)
        with mock.patch.object(setup_panel_mod.QMessageBox,
                               "information") as info:
            panel._read_filters()
        info.assert_called_once()
        self.assertEqual(self._name(panel._filter_table, 1), "")

    def test_mounted_column_shows_what_is_fitted(self):
        panel, _ = self._panel(connect=True)
        panel._refresh_live()
        self.assertIn("DAPI", panel._filter_table._rows[1]["fitted"].text())
        self.assertEqual(panel._filter_table._rows[6]["fitted"].text(), "empty")

    def test_go_buttons_are_gated_on_connection(self):
        panel, ctrl = self._panel(connect=False)
        panel._refresh_live()
        self.assertFalse(panel._filter_table._rows[1]["go"].isEnabled())
        panel._go_filter(3)                     # must be inert, not raise
        self.assertIsNone(ctrl.state().filter_position)

        panel2, ctrl2 = self._panel(connect=True)
        panel2._refresh_live()
        self.assertTrue(panel2._filter_table._rows[1]["go"].isEnabled())
        panel2._go_filter(3)
        self.assertEqual(ctrl2.state().filter_position, 3)

    # ── Optics write-support probe ──────────────────────────────────

    def test_probe_write_support_needs_a_connection(self):
        panel, _ = self._panel(connect=False)
        with mock.patch.object(setup_panel_mod.QMessageBox,
                               "information") as info:
            panel._probe_write_support()
        info.assert_called_once()

    def test_probe_write_support_runs_when_connected(self):
        panel, ctrl = self._panel(connect=True)
        # The result dialog is modal — patch exec() so this can't block under
        # offscreen Qt (the same trap already recorded for QMessageBox above).
        with mock.patch.object(setup_panel_mod.QDialog, "exec"):
            panel._probe_write_support()
        # The simulated backend has no notion of a writable optics database —
        # this only proves the probe reaches it and the result lands on state.
        self.assertEqual(ctrl.state().optic_write_support, {})

    def test_format_write_support_with_no_result(self):
        text = setup_panel_mod.MicroscopeSetupPanel._format_write_support({})
        self.assertIn("no notion", text)

    def test_format_write_support_read_only(self):
        text = setup_panel_mod.MicroscopeSetupPanel._format_write_support(
            {"objective": {"Name": False, "Code": False}})
        self.assertIn("cannot be set from software", text)
        self.assertIn("read-only", text)
        self.assertNotIn("WRITABLE", text)
        self.assertIn("nothing was written", text.lower())

    def test_format_write_support_writable_carries_the_false_positive_warning(self):
        """A declared setter is hardware-verified to be a FALSE POSITIVE on the
        Ti-E. This report must not promise that a rename will reach the body."""
        text = setup_panel_mod.MicroscopeSetupPanel._format_write_support(
            {"objective": {"Name": True, "Code": False}})
        self.assertIn("WRITABLE", text)
        self.assertIn("FALSE POSITIVE", text)
        self.assertIn("Database entry cannot be modified.", text)
        self.assertNotIn("MAY be pushable", text)

    def test_format_write_support_undeterminable(self):
        text = setup_panel_mod.MicroscopeSetupPanel._format_write_support(
            {"objective": {"Name": None, "Code": None}})
        self.assertIn("Undeterminable", text)
        self.assertNotIn("cannot be set from software", text)

    def test_probe_surfaces_a_refusal_instead_of_a_stale_result(self):
        """A lease-blocked / dropped op must not render as "this driver has no
        such notion" — that would read as a hardware answer."""
        panel, ctrl = self._panel(connect=True)
        with mock.patch.object(ctrl, "probe_optic_write_support") as probe:
            probe.return_value = mock.Mock(error="microscope is reserved by x")
            with mock.patch.object(setup_panel_mod.QMessageBox,
                                   "warning") as warn:
                with mock.patch.object(setup_panel_mod.QDialog, "exec") as ex:
                    panel._probe_write_support()
        warn.assert_called_once()
        ex.assert_not_called()


class TestMicroscopeSettingsDialogWrapper(_StoreCase):
    """The dialog must be a wrapper, not a second implementation."""

    def _dialog(self):
        from gui.dialogs.microscope_settings_dialog import (
            MicroscopeSettingsDialog)
        ctrl = MicroscopeController(store=self.store, threaded=False)
        return MicroscopeSettingsDialog(self.store, controller=ctrl)

    def test_hosts_the_shared_panel(self):
        from gui.pages.hardware.microscope_setup_panel import (
            MicroscopeSetupPanel)
        dlg = self._dialog()
        self.assertIsInstance(dlg.panel, MicroscopeSetupPanel)

    def test_ok_commits(self):
        dlg = self._dialog()
        dlg.panel._filter_table._rows[1]["edit"].setText("DAPI")
        dlg.accept()
        self.assertEqual(self.store.filter_label(1), "DAPI")

    def test_cancel_discards(self):
        self.store.set_filter_label(1, "DAPI")
        dlg = self._dialog()
        dlg.panel._filter_table._rows[1]["edit"].setText("changed")
        dlg.reject()
        self.assertEqual(self.store.filter_label(1), "DAPI")


class TestHardwareSetupMicroscopeTab(unittest.TestCase):
    def test_microscope_sub_page_is_registered(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        page = HardwareSetupPage()
        self.assertTrue(hasattr(page, "_microscope_panel"))
        self.assertIn("microscope", page._sub_scrolls)
        # The plate/rosette index bookkeeping must survive the insertion.
        self.assertEqual(page._sub_scrolls["plate"],
                         page._sub_scrolls["plate"])
        self.assertIsInstance(page._plate_sub_index, int)


# ── 8. Jog-panel integration ───────────────────────────────────────

class TestJogPanelHostsMicroscope(unittest.TestCase):
    def test_panel_has_a_microscope_card_and_forwards_ticks(self):
        from gui.widgets.standard_jog_context import StandardJogContextPanel
        panel = StandardJogContextPanel()
        self.assertTrue(hasattr(panel, "_microscope"))
        # The card drives the shared controller, not the stage controller.
        self.assertIsNotNone(panel._microscope.microscope())
        panel.on_status_update()   # must not raise with no hardware attached


# ── 9. Custom-panel registration ───────────────────────────────────

class TestSectionRegistration(unittest.TestCase):
    def test_registered_in_catalog(self):
        from gui.widgets.context_sections import catalog, known_types
        self.assertIn("microscope", known_types())
        entry = [c for c in catalog() if c[0] == "microscope"]
        self.assertEqual(len(entry), 1)
        _type, label, icon = entry[0]
        self.assertIn("Microscope", label)
        self.assertEqual(icon, "🔬")

    def test_builds_headless_and_ticks(self):
        from gui.widgets.context_sections import build_section, SectionContext
        w = build_section("microscope", SectionContext())
        self.assertTrue(hasattr(w, "on_status_update"))
        w.on_status_update()   # must not raise


# ── 10. Connect Hardware row (shared hardware control panel) ───────

class TestConnectHardwareRow(_StoreCase):
    """The body connects where the stages connect.

    Operator: *"move the microscope connect button to the connection area for
    xy stage, zp stage"* — it is hardware, so it belongs in the one place
    hardware is opened, not on the page that only assigns cube names.
    """

    def _panel(self):
        """The real Connect Hardware card, wired to a private controller so a
        test never touches the app-wide singleton or the operator's config."""
        from gui.pages.hardware.control_panel import HardwareControlPanel
        panel = HardwareControlPanel()
        ctrl = MicroscopeController(store=self.store, threaded=False)
        panel._microscope = lambda: ctrl
        return panel, ctrl

    def test_row_sits_with_the_stage_rows(self):
        panel, _ = self._panel()
        for attr in ("btn_connect_xy", "btn_connect_zp", "btn_connect_xbox",
                     "btn_connect_scope", "btn_simulate_scope",
                     "btn_disconnect_scope", "badge_scope"):
            self.assertTrue(hasattr(panel, attr), attr)
        # Same Connect Hardware card as the stage buttons — one area.
        self.assertIs(panel.btn_connect_scope.parent(),
                      panel.btn_connect_xy.parent())

    def test_connect_and_disconnect(self):
        panel, ctrl = self._panel()
        panel._connect_microscope()
        self.assertTrue(ctrl.state().connected)
        panel._sync_microscope_badge()
        self.assertEqual(panel.badge_scope.text(), "Simulated")
        panel._disconnect_microscope()
        self.assertFalse(ctrl.state().connected)
        self.assertEqual(panel.badge_scope.text(), "Not connected")

    def test_simulate_does_not_rewrite_the_saved_driver(self):
        self.store.set_backend("nikon_ti")
        panel, ctrl = self._panel()
        panel._simulate_microscope()
        self.assertTrue(ctrl.state().connected)
        self.assertEqual(ctrl.state().backend, "simulated")
        # The operator's real driver choice must survive a simulator run.
        self.assertEqual(self.store.get_backend(), "nikon_ti")

    def test_badge_reports_a_failed_connect(self):
        panel, ctrl = self._panel()
        with mock.patch.object(ctrl, "connect",
                               side_effect=RuntimeError("no body")):
            panel._connect_microscope()
        self.assertIn("no body", panel.badge_scope.text())

    def test_status_tick_syncs_the_badge_without_a_stage_controller(self):
        # The microscope is its own singleton, so its badge must not depend on
        # a StageController having been injected.
        panel, ctrl = self._panel()
        self.assertIsNone(panel._controller)
        ctrl.connect()
        panel.on_status_update()
        self.assertEqual(panel.badge_scope.text(), "Simulated")

    def test_panel_built_without_the_connect_card_is_unaffected(self):
        # The calibration variant hides Connect Hardware entirely.
        from gui.pages.hardware.control_panel import HardwareControlPanel
        panel = HardwareControlPanel(show_connect=False)
        self.assertFalse(hasattr(panel, "badge_scope"))
        panel.on_status_update()        # must not raise

    def test_setup_panel_no_longer_carries_its_own_connect_button(self):
        from gui.pages.hardware.microscope_setup_panel import (
            MicroscopeSetupPanel)
        ctrl = MicroscopeController(store=self.store, threaded=False)
        setup = MicroscopeSetupPanel(self.store, controller=ctrl)
        self.assertFalse(hasattr(setup, "_btn_connect"))
        self.assertFalse(hasattr(setup, "_toggle_connect"))
        # It still REPORTS the live link — that is what its Read/Go need.
        ctrl.connect()
        setup._refresh_live()
        self.assertIn("connected", setup._status_lbl.text())


if __name__ == "__main__":
    unittest.main()
