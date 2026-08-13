"""
"Put optic X in the light path" — the four rules that are not negotiable.

Everything here drives the **real** ``MicroscopeController`` over a fake
*backend*, so the ``_Op`` / ``.error`` / ``STALE_OP_S`` / ``_read_all`` /
lease contract is exercised rather than modelled. A hand-rolled fake ``_Op``
would agree with whatever this code believes and prove nothing.

THE LOAD-BEARING TEST is ``TestAckWithoutMovingIsCaught``: a backend that accepts
``set_filter`` and does not move leaves ``op.error`` None and ``op.done`` set, so
the ONLY thing that can catch it is the position read-back. That is not
hypothetical — ``NikonTiSdkBackend._set_turret`` documents this SDK clamping an
out-of-range slot and *reporting success*.

Close behind it is ``TestLeaseIsNotStolenFromAnOuterHolder``: ``try_acquire`` is
re-entrant and returns True for the same thread, while ``release`` clears the
lease unconditionally — so a service nested inside a workflow that already holds
the lease will hand the body to anyone unless it releases only what it took.
"""

import os
import shutil
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.MicroscopeConfigStore import (        # noqa: E402
    MicroscopeConfigStore)
from SupportClasses.MicroscopeControl import (            # noqa: E402
    MicroscopeBackend, MicroscopeController, MicroscopeError)
from SupportClasses.OpticsRegistry import FILTER, OBJECTIVE   # noqa: E402
from SupportClasses.OpticsService import (                # noqa: E402
    OpticsService, wait_for_op)

RIG_CUBES = {"1": "DAPI", "2": "FITC", "3": "TxRed", "4": "Cy5"}
RIG_OBJECTIVES = {"1": "4X", "2": "10X", "3": "20x"}
GLASS = 5000.0

#: (label, mag, NA, WD mm) for the three objectives actually on this rig.
OBJ_SPECS = (("4x", 4.0, 0.13, 16.4), ("10x", 10.0, 0.30, 4.0),
             ("20x", 20.0, 0.45, 1.0))


class _Optic:
    def __init__(self, position, label, present=True, code="",
                 mag=None, na=None, wd=None):
        self.position, self.label, self.present, self.code = (
            position, label, present, code)
        self.magnification, self.numerical_aperture = mag, na
        self.working_distance_mm = wd


class _ScriptedTiBackend(MicroscopeBackend):
    """A Ti-shaped backend that can be told to misbehave in specific ways.

    ``ack_without_moving`` is the important one: it is the failure ``.error``
    cannot see.
    """

    display_name = "scripted"
    name = "nikon_ti"

    def __init__(self, *, ack_without_moving=(), refuse_on=(), slow_by_s=0.0,
                 focus_limits=(0.0, 10000.0), focus_floor_um=None,
                 filters_present=(True, True, True, True, False, False)):
        self._conn = False
        self._filter, self._objective = 2, 1
        self._focus = GLASS
        self._ack_without_moving = set(ack_without_moving)
        self._refuse_on = set(refuse_on)
        self._slow_by_s = float(slow_by_s)
        self._focus_limits = focus_limits
        #: A floor the drive silently refuses to go below — how a real clamp
        #: makes a commanded move arrive SHORT of the request.
        self._focus_floor_um = focus_floor_um
        self._filters_present = tuple(filters_present)
        self.calls = []

    # -- lifecycle
    def connect(self):
        self._conn = True

    def disconnect(self):
        self._conn = False

    def is_connected(self):
        return self._conn

    def _guard(self, what):
        self.calls.append(what)
        if self._slow_by_s:
            time.sleep(self._slow_by_s)
        if what in self._refuse_on:
            raise MicroscopeError(f"{what} refused by the scripted backend")

    # -- filter
    def filter_count(self):
        return 6

    def get_filter(self):
        return self._filter

    def set_filter(self, position):
        self._guard("set_filter")
        if "set_filter" not in self._ack_without_moving:
            self._filter = int(position)

    def filter_names(self):
        return ("DAPI", "FITC", "TxRed", "Cy5", "-----", "-----")

    def mounted_filters(self):
        names = self.filter_names()
        return tuple(_Optic(i, n, present=self._filters_present[i - 1],
                            code=f"C{i}")
                     for i, n in enumerate(names, start=1))

    # -- objective
    def objective_count(self):
        return 5

    def get_objective(self):
        return self._objective

    def set_objective(self, position):
        self._guard("set_objective")
        if "set_objective" not in self._ack_without_moving:
            self._objective = int(position)

    def objective_names(self):
        return ("4x", "10x", "20x", "", "")

    def mounted_objectives(self):
        return tuple(_Optic(i, lbl, present=True, code=f"MRH{i}",
                            mag=m, na=na, wd=wd)
                     for i, (lbl, m, na, wd) in enumerate(OBJ_SPECS, start=1))

    # -- focus
    def get_focus_um(self):
        return self._focus

    def set_focus_um(self, value_um):
        self._guard("set_focus_um")
        v = float(value_um)
        if self._focus_floor_um is not None:
            v = max(v, float(self._focus_floor_um))
        lo, hi = self._focus_limits
        self._focus = max(lo, min(hi, v))

    def focus_limits_um(self):
        return self._focus_limits

    def diagnostics(self):
        return "scripted"


class _Case(unittest.TestCase):
    """One real controller (unthreaded) + one real config store per test."""

    THREADED = False

    def setUp(self):
        self._dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self._dir, ignore_errors=True)
        self.cfg = MicroscopeConfigStore(Path(self._dir) / "microscope.json")
        self.cfg.set_filter_labels(dict(RIG_CUBES))
        self.cfg.set_objective_labels(dict(RIG_OBJECTIVES))
        self.cfg.set_focus_soft_limits_um(0.0, 10000.0)

    def build(self, backend=None, backend_name="nikon_ti", **kw):
        """Bring the REAL controller up through its REAL connect() path.

        The fake is injected at the ``build_backend`` seam rather than by
        assigning ``_backend``, so ``connect()`` runs for real and the state's
        ``backend`` / ``connected`` fields are genuine. Setting ``_backend``
        directly leaves ``backend='none'``, which silently disables the
        simulated-backend flag — the harness would then be testing a state the
        app can never be in.
        """
        import SupportClasses.MicroscopeControl as mc
        self.backend = backend if backend is not None else _ScriptedTiBackend(**kw)
        original = mc.build_backend
        mc.build_backend = lambda name, **kwargs: self.backend
        try:
            self.ctrl = MicroscopeController(store=self.cfg,
                                            threaded=self.THREADED)
            self.addCleanup(self.ctrl.shutdown)
            op = self.ctrl.connect(backend_name)
            if getattr(op, "done", None):
                op.done.wait(10.0)
        finally:
            mc.build_backend = original
        self.assertTrue(self.ctrl.state().connected, "the fake did not connect")
        self.svc = OpticsService(self.ctrl, self.cfg, owner="test")
        return self.svc


class TestTheFilterHappyPath(_Case):
    def test_it_switches_and_verifies(self):
        svc = self.build()
        r = svc.ensure_filter("DAPI")
        self.assertTrue(r.ok, r.why_not)
        self.assertEqual((r.from_position, r.to_position), (2, 1))
        self.assertEqual(r.resolved_name, "DAPI")
        self.assertEqual(self.backend.get_filter(), 1)
        self.assertIn("verified", r.describe())

    def test_already_there_commands_NOTHING(self):
        """The idempotence that makes it safe to call before every channel."""
        svc = self.build()
        r = svc.ensure_filter("FITC")           # already position 2
        self.assertTrue(r.ok)
        self.assertTrue(r.already)
        self.assertNotIn("set_filter", self.backend.calls)
        self.assertIn("already", r.describe())

    def test_a_HAND_ROTATED_turret_is_noticed_before_the_no_op_check(self):
        """The cached state is up to ~1 s stale, and this is the case the whole
        service exists for: the operator turned the cassette themselves.

        Without a refresh first, the stale cache still says position 2 and the
        service would command a move to a cube that is ALREADY in the light path —
        or, worse, report "already there" about a position the body has left.
        """
        svc = self.build()
        self.backend._filter = 4              # turned by hand, cache says 2
        self.assertEqual(self.ctrl.state().filter_position, 2)

        r = svc.ensure_filter("Cy5")          # Cy5 IS position 4
        self.assertTrue(r.ok, r.why_not)
        self.assertTrue(r.already,
                        "the service acted on a stale cached position")
        self.assertEqual(self.backend.calls.count("set_filter"), 0,
                         "it commanded a move to where the body already was")

    def test_a_hand_rotated_turret_still_MOVES_when_it_must(self):
        """The counterweight: refreshing must not make it think it is done."""
        svc = self.build()
        self.backend._filter = 4
        r = svc.ensure_filter("DAPI")         # position 1
        self.assertTrue(r.ok, r.why_not)
        self.assertFalse(r.already)
        self.assertEqual(r.from_position, 4, "it reported the stale position")
        self.assertEqual(self.backend.get_filter(), 1)

    def test_a_cube_change_never_touches_the_focus(self):
        """Rotating the cassette moves no objective, so it needs no guard —
        and must not invent one."""
        svc = self.build()
        svc.ensure_filter("Cy5")
        self.assertNotIn("set_focus_um", self.backend.calls)

    def test_an_alias_resolves_and_is_reported_as_such(self):
        svc = self.build()
        self.cfg.set_optic_alias("filter", "mCherry", "TxRed")
        r = svc.ensure_filter("mCherry")
        self.assertTrue(r.ok, r.why_not)
        self.assertEqual(r.to_position, 3)
        self.assertEqual(r.resolved_name, "TxRed")
        self.assertEqual(r.how, "alias")

    def test_mcherry_without_an_alias_refuses_and_moves_nothing(self):
        svc = self.build()
        r = svc.ensure_filter("mCherry")
        self.assertFalse(r.ok)
        self.assertNotIn("set_filter", self.backend.calls)
        self.assertIn("TxRed", r.why_not)


class TestAckWithoutMovingIsCaught(_Case):
    """The only failure ``.error`` cannot see."""

    def test_a_backend_that_acks_and_does_not_move_is_a_FAILURE(self):
        svc = self.build(ack_without_moving=("set_filter",))
        r = svc.ensure_filter("DAPI")
        self.assertFalse(r.ok, "an ack without motion was reported as success")
        self.assertIn("did not move", r.why_not)
        self.assertIn("2", r.why_not)        # names where it actually is
        self.assertIn("1", r.why_not)        # and where it was told to go

    def test_the_same_for_the_objective(self):
        svc = self.build(ack_without_moving=("set_objective",))
        r = svc.ensure_objective("20x", glass_focus_um=GLASS)
        self.assertFalse(r.ok)
        self.assertIn("did not move", r.why_not)


class TestRefusalsFromTheDriver(_Case):
    def test_a_refused_op_is_not_success(self):
        svc = self.build(refuse_on=("set_filter",))
        r = svc.ensure_filter("DAPI")
        self.assertFalse(r.ok)
        self.assertIn("refused", r.why_not)

    def test_a_disconnected_body_degrades_and_says_to_do_it_by_hand(self):
        svc = self.build()
        # Through the controller, not the driver: `connected` records that WE
        # connected, and `refresh()` deliberately does not clear it — so a
        # driver-level disconnect is not the state the app can reach.
        op = self.ctrl.disconnect()
        if getattr(op, "done", None):
            op.done.wait(10.0)
        r = svc.ensure_filter("DAPI")
        self.assertFalse(r.ok)
        self.assertTrue(r.degraded)
        self.assertIn("by hand", r.why_not)
        self.assertNotIn("set_filter", self.backend.calls)


class TestStaleDropAbortsAndDoesNotRetry(unittest.TestCase):
    """Rule 2, against the REAL queue: a retry re-queues behind the same backlog."""

    def test_wait_for_op_reports_a_stale_drop_as_fatal(self):
        class _Op:
            def __init__(self):
                self.done = threading.Event()
                self.done.set()
                self.error = "dropped (stale)"

        why = wait_for_op(_Op(), "switching the cube")
        self.assertTrue(why)
        self.assertIn("DROPPED", why)
        self.assertIn("not retried", why.lower().replace("nothing was retried",
                                                         "not retried"))

    def test_a_none_op_is_a_refusal_not_a_pass(self):
        self.assertTrue(wait_for_op(None, "switching the cube"))

    def test_a_timeout_is_a_refusal(self):
        class _Op:
            done = threading.Event()          # never set
            error = None

        why = wait_for_op(_Op(), "switching the cube", timeout_s=0.05)
        self.assertIn("did not complete", why)

    def test_a_clean_op_returns_empty(self):
        class _Op:
            def __init__(self):
                self.done = threading.Event()
                self.done.set()
                self.error = None

        self.assertEqual(wait_for_op(_Op(), "switching the cube"), "")


class TestStaleDropThroughTheRealController(_Case):
    """Threaded, so STALE_OP_S is the real controller's own bookkeeping."""

    THREADED = True

    def test_a_REAL_dropped_op_is_fatal_and_never_executed(self):
        """The drop path end-to-end: a real ``_Op``, dropped by the real queue.

        ``STALE_OP_S`` is a CLASS attribute (20 s in production), shortened here
        so the drop is reachable. The op is put through the production
        ``wait_for_op`` rather than the whole ``ensure_filter`` because
        ``ensure_filter`` deliberately drains the queue first (see the sibling
        test), which is what makes a drop unlikely for its own ops — the contract
        under test is what happens WHEN one is dropped.
        """
        svc = self.build(slow_by_s=0.5)
        original = MicroscopeController.STALE_OP_S
        MicroscopeController.STALE_OP_S = 0.01
        try:
            # A MOVE is what occupies the worker — reads are fast on real
            # hardware too, so a refresh creates no backlog to queue behind.
            self.ctrl.set_focus_um(GLASS)
            op = self.ctrl.set_filter(1)        # queues behind it, ages out
            why = wait_for_op(op, "switching the cube", timeout_s=5.0)
        finally:
            MicroscopeController.STALE_OP_S = original
        self.assertTrue(why, "a dropped op was reported as success")
        self.assertIn("DROPPED", why)
        self.assertEqual(self.backend.calls.count("set_filter"), 0,
                         "a dropped op must never reach the driver")
        self.assertEqual(self.backend.get_filter(), 2, "the cube moved anyway")

    def test_the_service_drains_the_queue_before_it_submits(self):
        """Its own mitigation: a switch is not gratuitously queued behind a
        backlog it could have waited out."""
        svc = self.build(slow_by_s=0.2)
        self.ctrl.refresh()                     # a backlog exists
        r = svc.ensure_filter("DAPI")
        self.assertTrue(r.ok, r.why_not)
        self.assertEqual(self.backend.get_filter(), 1)


class TestTheWizardAndTheServiceCannotDrift(unittest.TestCase):
    """``plate_level_wizard._scope_op`` predates ``wait_for_op`` and is KEPT.

    It is deliberately not re-pointed: its wording is more specific (the stale
    case names the jog panel and Hardware Setup, which is what an operator needs),
    and 17 existing tests pin that text. Delegating and then re-deriving the case
    to restore the wording would be longer code for no extra safety.

    What matters is that the two never disagree about **whether** an op succeeded.
    That is what this pins — the decision, not the phrasing.
    """

    @staticmethod
    def _ops():
        def _op(error=None, set_done=True):
            class _O:
                pass
            o = _O()
            o.done = threading.Event()
            if set_done:
                o.done.set()
            o.error = error
            return o

        return {
            "clean": _op(),
            "stale": _op("dropped (stale)"),
            "reserved": _op("microscope is reserved by plate_level"),
            "driver": _op("TIFilterBlock1: COM error"),
            "timeout": _op(set_done=False),
            "none": None,
        }

    def test_they_agree_on_every_outcome(self):
        from gui.widgets.plate_level_wizard import PlateLevelSurveyWorker, _Abort

        for label, op in self._ops().items():
            service_why = wait_for_op(op, "switching the cube", timeout_s=0.05)
            wizard_failed = False
            try:
                # Called unbound with `self=None`: `_scope_op` reads only its
                # arguments, and PlateLevelSurveyWorker is a QObject that cannot
                # be instantiated without a live parent here.
                PlateLevelSurveyWorker._scope_op(
                    None, op, "switching the cube", 0.05)
            except _Abort:
                wizard_failed = True
            self.assertEqual(
                bool(service_why), wizard_failed,
                f"{label}: wait_for_op says {bool(service_why)} but the wizard "
                f"says {wizard_failed} — the two decisions have drifted")


class TestLeaseIsNotStolenFromAnOuterHolder(_Case):
    def test_a_nested_call_leaves_the_outer_lease_held(self):
        """try_acquire is re-entrant and release is unconditional, so releasing a
        lease this call did not take hands the body to anyone mid-run."""
        svc = self.build()
        self.assertTrue(self.ctrl.try_acquire("plate_level"))
        nested = OpticsService(self.ctrl, self.cfg, owner="plate_level")
        r = nested.ensure_filter("DAPI")
        self.assertTrue(r.ok, r.why_not)
        self.assertEqual(self.ctrl.lease_owner(), "plate_level",
                         "the nested call released its caller's lease")
        self.ctrl.release("plate_level")

    def test_it_releases_a_lease_it_did_take(self):
        svc = self.build()
        r = svc.ensure_filter("DAPI")
        self.assertTrue(r.ok, r.why_not)
        self.assertIsNone(self.ctrl.lease_owner(), "the lease leaked")

    def test_someone_elses_lease_refuses_and_names_them(self):
        svc = self.build()
        held = threading.Event()
        done = threading.Event()

        def _holder():
            self.ctrl.try_acquire("plate_level")
            held.set()
            done.wait(5.0)
            self.ctrl.release("plate_level")

        t = threading.Thread(target=_holder, daemon=True)
        t.start()
        held.wait(2.0)
        try:
            r = svc.ensure_filter("DAPI")
        finally:
            done.set()
            t.join(5.0)
        self.assertFalse(r.ok)
        self.assertIn("plate_level", r.why_not)
        self.assertEqual(self.backend.calls.count("set_filter"), 0)


class TestObjectiveSafety(_Case):
    def test_a_lethal_focus_height_retreats_BEFORE_the_rotation(self):
        """Ordering is the whole point: the crash is at the instant of rotation."""
        svc = self.build()
        self.backend._focus = GLASS + 3000.0
        self.ctrl.refresh()
        r = svc.ensure_objective("20x", glass_focus_um=GLASS)
        self.assertTrue(r.ok, r.why_not)
        calls = [c for c in self.backend.calls
                 if c in ("set_focus_um", "set_objective")]
        self.assertEqual(calls[0], "set_focus_um",
                         f"the turret moved before the focus retreated: {calls}")
        self.assertLess(self.backend.get_focus_um(), GLASS + 3000.0)

    def test_no_glass_datum_refuses_and_moves_NOTHING(self):
        svc = self.build()
        self.backend._focus = GLASS + 3000.0
        self.ctrl.refresh()
        r = svc.ensure_objective("20x", glass_focus_um=None)
        self.assertFalse(r.ok)
        self.assertNotIn("set_objective", self.backend.calls)
        self.assertNotIn("set_focus_um", self.backend.calls)

    def test_a_clear_height_rotates_without_touching_the_focus(self):
        svc = self.build()
        r = svc.ensure_objective("20x", glass_focus_um=GLASS)
        self.assertTrue(r.ok, r.why_not)
        self.assertNotIn("set_focus_um", self.backend.calls)
        self.assertEqual(self.backend.get_objective(), 3)

    def test_the_needle_gate_refuses_before_anything_moves(self):
        svc = self.build()
        r = svc.ensure_objective("20x", glass_focus_um=GLASS,
                                 needle_retracted=False)
        self.assertFalse(r.ok)
        self.assertIn("retract", r.why_not.lower())
        self.assertEqual(self.backend.calls, [])

    def test_the_needle_gate_does_not_block_a_no_op(self):
        """Nothing moves, so nothing is unsafe."""
        svc = self.build()
        r = svc.ensure_objective("4X", glass_focus_um=GLASS,
                                 needle_retracted=False)
        self.assertTrue(r.ok, r.why_not)
        self.assertTrue(r.already)

    def test_the_service_never_arms_the_print_floor(self):
        """The plate-level precedent: arming a refcount it cannot need risks an
        unbalanced decrement against a concurrent print."""
        src = (Path(__file__).resolve().parent.parent
               / "SupportClasses/OpticsService.py").read_text(encoding="utf-8")
        for forbidden in ("set_print_floor_active", "safe_travel_to",
                          "move_xy", "move_z"):
            self.assertNotIn(forbidden + "(", src,
                             f"OpticsService must not command the stage "
                             f"({forbidden})")


class TestParfocal(_Case):
    def test_it_is_off_unless_auto_apply_is_set(self):
        svc = self.build()
        self.cfg.set_parfocal(
            "tucam:0", reference="4x", offsets_um={"4x": 0.0, "20x": -180.0})
        r = svc.ensure_objective("20x", glass_focus_um=GLASS,
                                 camera_identity="tucam:0")
        self.assertTrue(r.ok, r.why_not)
        self.assertNotIn("set_focus_um", self.backend.calls)
        self.assertIsNone(r.focus_applied_um)

    def test_the_delta_is_relative_not_absolute(self):
        """offset[to] - offset[from]. Using offset[to] alone is a full-magnitude
        focus error that still looks plausible."""
        svc = self.build()
        self.cfg.set_parfocal_auto_apply(True)
        self.cfg.set_parfocal(
            "tucam:0", reference="4x",
            offsets_um={"4X": -40.0, "20x": -180.0})
        r = svc.ensure_objective("20x", glass_focus_um=GLASS,
                                 camera_identity="tucam:0")
        self.assertTrue(r.ok, r.why_not)
        self.assertAlmostEqual(self.backend.get_focus_um(), GLASS - 140.0,
                               places=3)

    def test_an_untaught_offset_still_SUCCEEDS_but_says_so(self):
        """The optic IS correct; focus is a separate fact. This rig ships with no
        offsets at all, so this is the default path."""
        svc = self.build()
        self.cfg.set_parfocal_auto_apply(True)
        r = svc.ensure_objective("20x", glass_focus_um=GLASS,
                                 camera_identity="tucam:0")
        self.assertTrue(r.ok, r.why_not)
        self.assertIsNone(r.focus_applied_um)
        self.assertIn("parfocal", r.focus_note.lower())

    def test_a_swapped_objective_invalidates_the_offset_by_product_code(self):
        svc = self.build()
        self.cfg.set_parfocal_auto_apply(True)
        self.cfg.set_parfocal(
            "tucam:0", reference="4x", offsets_um={"4X": 0.0, "20x": -180.0},
            product_codes={"20x": "SOMETHING-ELSE"})
        r = svc.ensure_objective("20x", glass_focus_um=GLASS,
                                 camera_identity="tucam:0")
        self.assertTrue(r.ok, r.why_not)
        self.assertIsNone(r.focus_applied_um)
        self.assertIn("swapped", r.focus_note.lower())

    def test_a_clamped_focus_reports_where_it_LANDED(self):
        """set_focus_um clamps before queueing, so a move can arrive SHORT of the
        request — the read-back is what makes the reported number honest."""
        svc = self.build(focus_floor_um=GLASS - 10.0)
        self.cfg.set_parfocal_auto_apply(True)
        self.cfg.set_parfocal(
            "tucam:0", reference="4x", offsets_um={"4X": 0.0, "20x": -180.0})
        r = svc.ensure_objective("20x", glass_focus_um=GLASS,
                                 camera_identity="tucam:0")
        self.assertTrue(r.ok, r.why_not)
        # Asked for 4820, the drive floors at 4990 — the RESULT must say 4990.
        self.assertAlmostEqual(r.focus_applied_um, GLASS - 10.0, places=3)
        self.assertIn("clamped", r.focus_note.lower())


class TestCancellation(_Case):
    def test_cancelled_up_front_commands_nothing(self):
        svc = self.build()
        r = svc.ensure_filter("DAPI", cancelled=lambda: True)
        self.assertFalse(r.ok)
        self.assertTrue(r.cancelled)
        self.assertEqual(self.backend.calls, [])

    def test_a_raising_predicate_is_not_a_cancellation(self):
        def _boom():
            raise RuntimeError("no")

        svc = self.build()
        r = svc.ensure_filter("DAPI", cancelled=_boom)
        self.assertTrue(r.ok, r.why_not)


class TestSimulatedIsFlagged(_Case):
    def test_a_simulated_switch_is_reported_as_simulated(self):
        """A simulated switch reported as real is a fabricated fact."""
        from SupportClasses.MicroscopeControl import SimulatedMicroscopeBackend
        svc = self.build(backend=SimulatedMicroscopeBackend(),
                         backend_name="simulated")
        r = svc.ensure_position(FILTER, 3)
        self.assertTrue(r.ok, r.why_not)
        self.assertTrue(r.simulated)
        self.assertIn("SIMULATED", r.describe())


class TestEnsurePosition(_Case):
    def test_it_drives_a_position_through_the_same_verified_path(self):
        svc = self.build()
        r = svc.ensure_position(FILTER, 4)
        self.assertTrue(r.ok, r.why_not)
        self.assertEqual(self.backend.get_filter(), 4)

    def test_an_impossible_position_refuses_without_moving(self):
        svc = self.build()
        r = svc.ensure_position(FILTER, 99)
        self.assertFalse(r.ok)
        self.assertEqual(self.backend.calls, [])

    def test_it_can_drive_an_unnamed_slot_so_the_operator_can_LOOK(self):
        svc = self.build(filters_present=(True,) * 6)
        self.cfg.set_filter_labels({"1": "DAPI"})
        r = svc.ensure_position(FILTER, 5)
        self.assertTrue(r.ok, r.why_not)
        self.assertEqual(self.backend.get_filter(), 5)


class TestRestore(_Case):
    def test_focus_goes_back_BEFORE_the_turret(self):
        svc = self.build()
        entry = svc.capture_entry()
        self.assertEqual(entry["objective"], 1)
        svc.ensure_objective("20x", glass_focus_um=GLASS)
        self.backend.calls.clear()
        problems = svc.restore(entry)
        self.assertEqual(problems, "")
        calls = [c for c in self.backend.calls
                 if c in ("set_focus_um", "set_objective")]
        self.assertEqual(calls[0], "set_focus_um",
                         f"restore rotated before returning the focus: {calls}")
        self.assertEqual(self.backend.get_objective(), 1)

    def test_it_issues_exactly_one_focus_command(self):
        svc = self.build()
        entry = svc.capture_entry()
        svc.ensure_objective("20x", glass_focus_um=GLASS)
        self.backend.calls.clear()
        svc.restore(entry)
        self.assertEqual(self.backend.calls.count("set_focus_um"), 1)

    def test_it_never_raises_and_reports_what_it_could_not_do(self):
        svc = self.build(refuse_on=("set_objective",))
        entry = svc.capture_entry()
        problems = svc.restore(entry)
        self.assertIn("objective", problems)

    def test_an_empty_entry_is_a_no_op(self):
        svc = self.build()
        self.assertEqual(svc.restore({}), "")
        self.assertEqual(self.backend.calls, [])


class TestTheWizardRestoresFocusBeforeTheTurret(unittest.TestCase):
    """v7.18 B5, in the wizard's OWN ``_restore``.

    ⚠ This test exists because a mutation SURVIVED without it: the service's
    ``restore`` was covered but the wizard's was not, so reverting B5 there —
    rotating the objective back before returning the focus — passed everything.
    If the entry objective is the 20x (WD ~1 mm) and the run ended at a height
    that is legal under the 4x, that order rotates the 20x in AT that height,
    into the plate, with no move in flight to abort.
    """

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _restore_calls(self, entry_obj=3, entry_focus=5000.0):
        from tests.test_v711_plate_level_worker import make_worker
        w, scope, _ = make_worker()
        scope.calls.clear()
        w._restore(entry_obj, entry_focus)
        return [c[0] for c in scope.calls]

    def test_focus_comes_first(self):
        kinds = self._restore_calls()
        self.assertIn("focus", kinds)
        self.assertIn("objective", kinds)
        self.assertLess(kinds.index("focus"), kinds.index("objective"),
                        f"the objective rotated before the focus returned: {kinds}")

    def test_exactly_one_focus_command(self):
        """Re-issuing after the rotation adds nothing and doubles what a caller
        has to reason about after an abort."""
        self.assertEqual(self._restore_calls().count("focus"), 1)

    def test_no_focus_entry_still_restores_the_objective(self):
        self.assertEqual(self._restore_calls(entry_focus=None), ["objective"])


class TestTheJogPanelDoesNotFightARunningWorkflow(_Case):
    """v7.18 lease hygiene on the manual Microscope card.

    ``_submit`` fails a lease-blocked op fast and does NOT publish an error, so a
    refused poll is invisible: the card would render off whatever the lease
    holder's own ops read back and then silently go stale. Worse, a combo change
    submitted just BEFORE a workflow takes the lease is not blocked at all and
    executes at an arbitrary point inside the run.
    """

    def setUp(self):
        super().setUp()
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        self.app = QApplication.instance() or QApplication([])

    def _panel(self):
        from gui.widgets.microscope_panel import MicroscopePanel
        p = MicroscopePanel(controller=self.ctrl)
        self.addCleanup(p.deleteLater)
        p._render(force=True)
        return p

    def test_the_combos_are_live_when_nothing_holds_the_body(self):
        self.build()
        p = self._panel()
        self.assertTrue(p._filter_combo.isEnabled())
        self.assertTrue(p._objective_combo.isEnabled())

    def test_they_are_disabled_and_name_the_owner_while_leased(self):
        self.build()
        p = self._panel()
        self.assertTrue(self.ctrl.try_acquire("plate_level"))
        try:
            p._render(force=True)
            self.assertFalse(p._filter_combo.isEnabled())
            self.assertFalse(p._objective_combo.isEnabled())
            self.assertIn("plate level", p._status_lbl.text())
            self.assertIn("plate level", p._filter_combo.toolTip())
        finally:
            self.ctrl.release("plate_level")
        p._render(force=True)
        self.assertTrue(p._filter_combo.isEnabled())

    def test_it_does_not_poll_under_someone_elses_lease(self):
        """A refused poll is invisible, so it must not be attempted."""
        self.build()
        p = self._panel()
        p._last_refresh = 0.0
        self.assertTrue(self.ctrl.try_acquire("plate_level"))
        try:
            self.backend.calls.clear()
            before = self.ctrl.state()
            p._tick()
            self.assertIs(self.ctrl.state(), before,
                          "the card polled a body it does not own")
        finally:
            self.ctrl.release("plate_level")

    def test_it_does_poll_once_the_lease_is_released(self):
        """The guard must not permanently stop the card from tracking hardware."""
        self.build()
        p = self._panel()
        p._last_refresh = 0.0
        self.backend._filter = 4                 # hand-rotated behind our back
        p._tick()
        self.assertEqual(self.ctrl.state().filter_position, 4)


if __name__ == "__main__":
    unittest.main()
