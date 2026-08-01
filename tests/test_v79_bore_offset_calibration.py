"""test_v79_bore_offset_calibration.py — per-MOUNT bore offsets.

Stage 8 of v7.9. Without these offsets "assign a target per bore" cannot work:
the software knows exactly ONE needle position (``needle_origin_um``), so with a
multi-bore assembly whose bores sit 100-500 µm apart (decision D7) the non-datum
bores miss their target by more than a cell's width, silently.

The test that matters most is :class:`TestSignConventionComposedWithTheExecutor`:
it composes the MEASUREMENT path (this store's subtraction of two centred stage
positions) with the CONSUMPTION path (``PickPlaceExecutor._bore_target_xy_um``)
and asserts the bore ends up ON the target. A mis-signed offset is a
*right-distance-wrong-way* error — it lands the correct distance away on the wrong
side, which reads as a calibration problem rather than a bug, exactly like the
plate-orientation defects recorded in CLAUDE.md.
"""

from __future__ import annotations

import ast
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

from SupportClasses.CalibrationSnapshotStore import CalibrationSnapshotStore
from SupportClasses.NeedleBoreCalibrationStore import (
    BoreOffset, NeedleBoreCalibrationStore, build_fingerprint, fingerprint_diff,
    get_store, offset_from_centred_positions, z_offset_from_centred_heights,
)
from SupportClasses.PhysicalModels import (
    NEEDLE_FORM_BACKPACK, NEEDLE_FORM_TRIPLE, NeedleBore, NeedleSpec,
)
from SupportClasses.PickAndPlaceManager import PickPlaceExecutor, PickPlaceTarget

REPO = Path(__file__).resolve().parent.parent


# ── fixtures ─────────────────────────────────────────────────────────

def _backpack(offsets=((0.0, 0.0), (0.0, 0.0))) -> NeedleSpec:
    """Bore 0 = coarse (P1, datum). Bore 1 = fine (P2). Offsets default to
    UNMEASURED so a test has to put them there through the store."""
    return NeedleSpec(
        needle_form=NEEDLE_FORM_BACKPACK,
        bores=[
            NeedleBore(gauge=22, od_um=718, id_um=413, wall_um=152,
                       length_mm=50.8, pump_id="P1", label="coarse",
                       offset_um=offsets[0]),
            NeedleBore(gauge=30, od_um=311, id_um=159, wall_um=76,
                       length_mm=50.8, pump_id="P2", label="fine",
                       offset_um=offsets[1]),
        ],
    )


def _triple() -> NeedleSpec:
    return NeedleSpec(
        needle_form=NEEDLE_FORM_TRIPLE,
        bores=[
            NeedleBore(gauge=22, od_um=718, id_um=413, wall_um=152,
                       length_mm=50.8, pump_id="P1"),
            NeedleBore(gauge=27, od_um=413, id_um=210, wall_um=102,
                       length_mm=50.8, pump_id="P2"),
            NeedleBore(gauge=30, od_um=311, id_um=159, wall_um=76,
                       length_mm=50.8, pump_id="P3"),
        ],
    )


class _IsolatedStore(unittest.TestCase):
    """Base: a store backed by a throwaway file, never config/hardware."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="mebp_bore_cal_")
        self.addCleanup(self._tmp.cleanup)
        self.path = Path(self._tmp.name) / "needle_bore_calibration.json"

    def store(self) -> NeedleBoreCalibrationStore:
        return NeedleBoreCalibrationStore(self.path)


# ════════════════════════════════════════════════════════════════════
#  A. The sign convention, in isolation
# ════════════════════════════════════════════════════════════════════

class TestSignConvention(unittest.TestCase):

    def test_offset_is_datum_minus_bore(self):
        """A bore protruding +320 µm in X is centred at a stage 320 µm LOWER in
        X, so its offset comes out POSITIVE."""
        datum = (100000.0, 50000.0)
        bore = (100000.0 - 320.0, 50000.0)
        self.assertEqual(offset_from_centred_positions(datum, bore),
                         (320.0, 0.0))

    def test_offset_is_signed_per_axis(self):
        off = offset_from_centred_positions((1000.0, 2000.0),
                                            (680.0, 2140.0))
        self.assertEqual(off, (320.0, -140.0))

    def test_datum_against_itself_is_zero(self):
        self.assertEqual(
            offset_from_centred_positions((1.5, -2.5), (1.5, -2.5)), (0.0, 0.0))

    def test_z_offset_positive_means_reaches_lower(self):
        """A longer bore is centred with the stage HIGHER, so bore_h > datum_h."""
        self.assertAlmostEqual(
            z_offset_from_centred_heights(22.500, 22.540), 0.040, places=9)

    def test_z_offset_negative_for_a_shorter_bore(self):
        self.assertAlmostEqual(
            z_offset_from_centred_heights(22.500, 22.470), -0.030, places=9)


# ════════════════════════════════════════════════════════════════════
#  B. THE ONE THAT MATTERS: measurement ∘ consumption
# ════════════════════════════════════════════════════════════════════

class _FakeCtrl:
    """Just enough controller for the executor's geometry helpers."""

    def __init__(self, z_up_sign=1.0):
        self._sign = z_up_sign
        self.is_zp_connected = True

    def z_up_sign(self):
        return self._sign


def _executor(needle) -> PickPlaceExecutor:
    """A ``__new__``-partial executor — the pattern the v7.9 executor suite uses
    (a real one needs a whole hardware stack)."""
    ex = PickPlaceExecutor.__new__(PickPlaceExecutor)
    ex.controller = _FakeCtrl()
    hw = MagicMock()
    hw.needle = needle
    ex.hw_config = hw
    return ex


class TestSignConventionComposedWithTheExecutor(_IsolatedStore):
    """Measure → store → apply → command, end to end.

    This is the test the whole stage exists for. Each half can be self-consistent
    and wrong; only the composition proves the bore lands on the cell.
    """

    def _measure(self, store, needle, datum_stage, bore_stages,
                 datum_h=20.0, bore_heights=None):
        """Simulate the GUI flow: centre the datum, then each other bore."""
        store.set_bore(0, (0.0, 0.0), 0.0, stage_um=datum_stage,
                       z_user_mm=datum_h, needle=needle)
        for k, stage in enumerate(bore_stages, start=1):
            h = None if bore_heights is None else bore_heights[k - 1]
            off = offset_from_centred_positions(datum_stage, stage)
            dz = (0.0 if h is None
                  else z_offset_from_centred_heights(datum_h, h))
            store.set_bore(k, off, dz, stage_um=stage, z_user_mm=h,
                           needle=needle)
        self.assertEqual(store.apply_to_needle(needle), len(bore_stages))

    def test_a_bore_measured_at_plus_320um_lands_ON_the_target(self):
        """The failure mode this pins: a flipped sign puts the bore 640 µm away
        (2× the offset, on the wrong side) while still 'looking calibrated'."""
        needle = _backpack()
        store = self.store()
        # Bore 1 physically sticks out +320 µm in X, −140 µm in Y, so it is
        # centred with the stage backed off by exactly that.
        datum = (100000.0, 50000.0)
        self._measure(store, needle, datum, [(100000.0 - 320.0,
                                              50000.0 + 140.0)])
        self.assertEqual(needle.bore_offset_um(1), (320.0, -140.0))

        ex = _executor(needle)
        target = PickPlaceTarget(target_id="C1", x_um=70000.0, y_um=30000.0,
                                 well_name="A1")
        sx, sy = ex._bore_target_xy_um(target, 1)
        # Where bore 1 actually ends up = stage + its physical offset.
        ox, oy = needle.bore_offset_um(1)
        self.assertAlmostEqual(sx + ox, target.x_um, places=6,
                               msg="bore 1 missed the target in X — sign?")
        self.assertAlmostEqual(sy + oy, target.y_um, places=6,
                               msg="bore 1 missed the target in Y — sign?")
        # And explicitly: the commanded stage is 320 µm BELOW the target in X.
        self.assertAlmostEqual(sx, target.x_um - 320.0, places=6)
        self.assertAlmostEqual(sy, target.y_um + 140.0, places=6)

    def test_the_flipped_sign_would_be_twice_the_spacing_away(self):
        """Quantifies why this cannot be caught by eye: the wrong sign is a
        640 µm miss, i.e. the needle sits a whole extra bore-spacing off."""
        needle = _backpack()
        store = self.store()
        datum = (100000.0, 50000.0)
        self._measure(store, needle, datum, [(100000.0 - 320.0, 50000.0)])
        ex = _executor(needle)
        target = PickPlaceTarget(target_id="C1", x_um=70000.0, y_um=30000.0,
                                 well_name="A1")
        sx, _sy = ex._bore_target_xy_um(target, 1)
        wrong_sx = target.x_um + 320.0        # what a flipped sign would command
        self.assertAlmostEqual(abs(wrong_sx - sx), 640.0, places=6)

    def test_every_bore_of_a_triple_lands_on_the_target(self):
        needle = _triple()
        store = self.store()
        datum = (100000.0, 50000.0)
        # A triangle: bore 2 at (+250, +0), bore 3 at (+125, +216).
        self._measure(store, needle, datum,
                      [(100000.0 - 250.0, 50000.0 - 0.0),
                       (100000.0 - 125.0, 50000.0 - 216.0)])
        ex = _executor(needle)
        target = PickPlaceTarget(target_id="C", x_um=12345.0, y_um=-6789.0,
                                 well_name="A1")
        for k in range(needle.bore_count):
            sx, sy = ex._bore_target_xy_um(target, k)
            ox, oy = needle.bore_offset_um(k)
            self.assertAlmostEqual(sx + ox, target.x_um, places=6, msg=f"bore {k}")
            self.assertAlmostEqual(sy + oy, target.y_um, places=6, msg=f"bore {k}")

    def test_a_longer_bore_gets_a_HIGHER_stage_z(self):
        """Composed with ``_bore_z_mm``: a bore centred with the stage 40 µm
        higher must be commanded 40 µm higher, not lower (that is the glass)."""
        needle = _backpack()
        store = self.store()
        self._measure(store, needle, (100000.0, 50000.0),
                      [(99680.0, 50140.0)],
                      datum_h=20.000, bore_heights=[20.040])
        self.assertAlmostEqual(needle.bore(1).z_offset_mm, 0.040, places=9)
        ex = _executor(needle)                       # z_up_sign = +1
        self.assertAlmostEqual(ex._bore_z_mm(10.0, 1), 10.040, places=9)

    def test_z_offset_follows_the_machine_polarity(self):
        needle = _backpack()
        store = self.store()
        self._measure(store, needle, (0.0, 0.0), [(0.0, 0.0)],
                      datum_h=20.0, bore_heights=[20.040])
        ex = _executor(needle)
        ex.controller = _FakeCtrl(z_up_sign=-1.0)
        self.assertAlmostEqual(ex._bore_z_mm(10.0, 1), 10.0 - 0.040, places=9)

    def test_an_unmeasured_bore_is_a_no_op_not_a_guess(self):
        """The fail-safe direction: an unmeasured bore targets the datum (the
        pre-v7.9 behaviour) rather than a made-up position."""
        needle = _backpack()
        store = self.store()
        store.set_bore(0, (0.0, 0.0), 0.0, stage_um=(1.0, 2.0),
                       z_user_mm=20.0, needle=needle)
        store.apply_to_needle(needle)
        ex = _executor(needle)
        t = PickPlaceTarget(target_id="C", x_um=5.0, y_um=6.0, well_name="A1")
        self.assertEqual(ex._bore_target_xy_um(t, 1), (5.0, 6.0))


# ════════════════════════════════════════════════════════════════════
#  C. Bore 0 is always the datum
# ════════════════════════════════════════════════════════════════════

class TestDatumIsAlwaysZero(_IsolatedStore):

    def test_set_bore_zero_forces_a_zero_offset(self):
        """Bore 0 IS ``needle_origin_um``; a non-zero datum offset would be
        double-counted by every consumer, which all assume it is (0, 0)."""
        store = self.store()
        rec = store.set_bore(0, (123.0, -456.0), 0.789)
        self.assertEqual(rec.offset_um, (0.0, 0.0))
        self.assertEqual(rec.z_offset_mm, 0.0)
        self.assertEqual(store.offset_um(0), (0.0, 0.0))
        self.assertEqual(store.z_offset_mm(0), 0.0)

    def test_provenance_is_still_kept_for_the_datum(self):
        store = self.store()
        store.set_bore(0, (0.0, 0.0), 0.0, stage_um=(105617.6, 65890.4),
                       z_user_mm=22.564)
        rec = store.get_bore(0)
        self.assertEqual(rec.stage_um, (105617.6, 65890.4))
        self.assertAlmostEqual(rec.z_user_mm, 22.564, places=4)
        self.assertTrue(rec.is_datum)

    def test_apply_never_writes_bore_zero(self):
        needle = _backpack()
        store = self.store()
        # Hand-forge a corrupt file claiming a non-zero datum.
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps({
            "version": "1.0",
            "fingerprint": build_fingerprint(needle),
            "bores": [{"bore_index": 0, "offset_um": [99.0, 99.0],
                       "z_offset_mm": 9.0}],
        }), encoding="utf-8")
        store = self.store()
        store.apply_to_needle(needle)
        self.assertEqual(needle.bores[0].offset_um, (0.0, 0.0))
        self.assertEqual(needle.bores[0].z_offset_mm, 0.0)

    def test_datum_only_store_is_not_calibrated(self):
        """Bore 0's offset carries no information, so a store holding only the
        datum must keep asking for the rest."""
        store = self.store()
        store.set_bore(0, (0.0, 0.0), 0.0, stage_um=(1.0, 2.0))
        self.assertFalse(store.is_calibrated())
        store.set_bore(1, (320.0, 0.0), 0.0)
        self.assertTrue(store.is_calibrated())


# ════════════════════════════════════════════════════════════════════
#  D. Round-trip, atomicity, isolation, GUI-free
# ════════════════════════════════════════════════════════════════════

class TestPersistence(_IsolatedStore):

    def test_round_trip_through_disk(self):
        needle = _backpack()
        store = self.store()
        store.set_bore(0, (0.0, 0.0), 0.0, stage_um=(1000.0, 2000.0),
                       z_user_mm=20.0, needle=needle)
        store.set_bore(1, (320.0, -140.0), 0.040, stage_um=(680.0, 2140.0),
                       z_user_mm=20.04, needle=needle)
        again = self.store()
        self.assertEqual(again.offset_um(1), (320.0, -140.0))
        self.assertAlmostEqual(again.z_offset_mm(1), 0.040, places=9)
        self.assertEqual(again.measured_bore_indices(), [0, 1])
        self.assertEqual(again.get_fingerprint(), build_fingerprint(needle))

    def test_bore_offset_dataclass_round_trip(self):
        rec = BoreOffset(bore_index=2, offset_um=(125.0, 216.0),
                         z_offset_mm=-0.012, stage_um=(9.0, 8.0),
                         z_user_mm=21.5, measured_at="2026-08-01T10:00:00")
        back = BoreOffset.from_dict(rec.to_dict())
        self.assertEqual(back, rec)

    def test_records_are_replaced_not_appended(self):
        store = self.store()
        store.set_bore(1, (100.0, 0.0))
        store.set_bore(1, (320.0, 0.0))
        self.assertEqual(len(store.all_bores()), 1)
        self.assertEqual(store.offset_um(1), (320.0, 0.0))

    def test_records_are_sorted_by_bore_index(self):
        store = self.store()
        store.set_bore(2, (1.0, 0.0))
        store.set_bore(1, (2.0, 0.0))
        store.set_bore(0, (0.0, 0.0))
        self.assertEqual([b.bore_index for b in store.all_bores()], [0, 1, 2])

    def test_write_is_atomic_and_leaves_no_temp_file(self):
        store = self.store()
        store.set_bore(1, (320.0, 0.0))
        self.assertTrue(self.path.exists())
        leftovers = [p.name for p in self.path.parent.iterdir()
                     if p.suffix == ".tmp"]
        self.assertEqual(leftovers, [])
        json.loads(self.path.read_text(encoding="utf-8"))   # valid JSON

    def test_a_truncated_file_degrades_to_uncalibrated(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text('{"version": "1.0", "bores": [', encoding="utf-8")
        store = self.store()
        self.assertFalse(store.is_calibrated())
        self.assertEqual(store.offset_um(1), (0.0, 0.0))

    def test_a_garbage_record_degrades_to_the_datum(self):
        """A corrupt offset must read as 'unmeasured' (0, 0), never as NaN or a
        crash — an unmeasured bore is safe, a NaN command is not."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps({
            "version": "1.0", "bores": [
                {"bore_index": 1, "offset_um": "nonsense",
                 "z_offset_mm": "nope"},
            ]}), encoding="utf-8")
        store = self.store()
        self.assertEqual(store.offset_um(1), (0.0, 0.0))
        self.assertEqual(store.z_offset_mm(1), 0.0)

    def test_non_finite_values_are_rejected(self):
        store = self.store()
        store.set_bore(1, (float("nan"), float("inf")), float("nan"))
        self.assertEqual(store.offset_um(1), (0.0, 0.0))
        self.assertEqual(store.z_offset_mm(1), 0.0)

    def test_clear_forgets_everything(self):
        store = self.store()
        store.set_bore(0, (0.0, 0.0))
        store.set_bore(1, (320.0, 0.0))
        store.clear()
        self.assertEqual(store.all_bores(), [])
        self.assertFalse(store.is_calibrated())
        self.assertEqual(self.store().all_bores(), [])   # and on disk

    def test_clear_bore_removes_only_that_one(self):
        store = self.store()
        store.set_bore(1, (320.0, 0.0))
        store.set_bore(2, (125.0, 216.0))
        self.assertTrue(store.clear_bore(1))
        self.assertEqual(store.measured_bore_indices(), [2])
        self.assertFalse(store.clear_bore(1))    # already gone

    def test_set_offsets_replaces_the_whole_set(self):
        needle = _backpack()
        store = self.store()
        store.set_bore(3, (9.0, 9.0))
        store.set_offsets({
            0: BoreOffset(0, (0.0, 0.0), 0.0, stage_um=(1.0, 2.0)),
            1: BoreOffset(1, (320.0, -140.0), 0.04),
        }, needle=needle)
        self.assertEqual(store.measured_bore_indices(), [0, 1])
        self.assertEqual(store.get_fingerprint(), build_fingerprint(needle))


class TestEnvIsolation(unittest.TestCase):

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="mebp_bore_cal_env_")
        self.addCleanup(self._tmp.cleanup)
        self._prev = os.environ.get("MEBP_NEEDLE_BORE_CAL_PATH")

        def _restore():
            if self._prev is None:
                os.environ.pop("MEBP_NEEDLE_BORE_CAL_PATH", None)
            else:
                os.environ["MEBP_NEEDLE_BORE_CAL_PATH"] = self._prev
        self.addCleanup(_restore)

    def test_env_redirects_the_file(self):
        target = Path(self._tmp.name) / "sub" / "bore.json"
        os.environ["MEBP_NEEDLE_BORE_CAL_PATH"] = str(target)
        store = NeedleBoreCalibrationStore()        # no explicit path
        store.set_bore(1, (320.0, 0.0))
        self.assertTrue(target.exists())
        self.assertEqual(NeedleBoreCalibrationStore().offset_um(1), (320.0, 0.0))

    def test_explicit_path_beats_the_env(self):
        os.environ["MEBP_NEEDLE_BORE_CAL_PATH"] = str(
            Path(self._tmp.name) / "env.json")
        explicit = Path(self._tmp.name) / "explicit.json"
        store = NeedleBoreCalibrationStore(explicit)
        self.assertEqual(store.path, explicit)
        store.set_bore(1, (1.0, 0.0))
        self.assertTrue(explicit.exists())
        self.assertFalse((Path(self._tmp.name) / "env.json").exists())

    def test_get_store_is_a_singleton(self):
        import SupportClasses.NeedleBoreCalibrationStore as mod
        prev = mod._store
        mod._store = None
        try:
            os.environ["MEBP_NEEDLE_BORE_CAL_PATH"] = str(
                Path(self._tmp.name) / "singleton.json")
            self.assertIs(get_store(), get_store())
        finally:
            mod._store = prev


class TestGuiFree(unittest.TestCase):

    def test_no_gui_or_image_dependency(self):
        src = (REPO / "SupportClasses" /
               "NeedleBoreCalibrationStore.py").read_text(encoding="utf-8")
        banned = ("PySide6", "gui", "cv2", "numpy")
        for node in ast.walk(ast.parse(src)):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            for name in names:
                head = name.split(".")[0]
                self.assertNotIn(head, banned,
                                 f"store must not import {name}")

    def test_not_part_of_hardware_config(self):
        """A setup file copied from another rig must never carry this mount's
        bore geometry — the CAMERA_CAL_PERSIST_STORE lesson.

        Checked STRUCTURALLY (imports + referenced names) rather than by
        searching the source text: the prose has to be able to explain *why* the
        setup file is the hazard, and a plain string search made the rule
        unstatable — it failed on the sentence describing it.
        """
        src = (REPO / "SupportClasses" /
               "NeedleBoreCalibrationStore.py").read_text(encoding="utf-8")
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                self.assertNotIn("HardwareConfig", node.module or "")
                for a in node.names:
                    self.assertNotEqual(a.name, "HardwareConfig")
            elif isinstance(node, ast.Import):
                for a in node.names:
                    self.assertNotIn("HardwareConfig", a.name)
            elif isinstance(node, ast.Name):
                self.assertNotEqual(node.id, "HardwareConfig")
            elif isinstance(node, ast.Attribute):
                self.assertNotEqual(node.attr, "HardwareConfig")


# ════════════════════════════════════════════════════════════════════
#  E. Assembly fingerprint
# ════════════════════════════════════════════════════════════════════

class TestFingerprint(_IsolatedStore):

    def test_fingerprint_records_form_count_and_per_bore_geometry(self):
        fp = build_fingerprint(_backpack())
        self.assertEqual(fp["needle_form"], NEEDLE_FORM_BACKPACK)
        self.assertEqual(fp["bore_count"], 2)
        self.assertEqual([r["pump_id"] for r in fp["bores"]], ["P1", "P2"])
        self.assertEqual([r["orifice_id_um"] for r in fp["bores"]],
                         [413.0, 159.0])

    def test_identical_assemblies_match(self):
        self.assertEqual(
            fingerprint_diff(build_fingerprint(_backpack()),
                             build_fingerprint(_backpack())), [])

    def test_bore_count_change_is_flagged(self):
        diffs = fingerprint_diff(build_fingerprint(_backpack()),
                                 build_fingerprint(_triple()))
        self.assertTrue(any("bore count" in d for d in diffs), diffs)
        self.assertTrue(any("needle form" in d for d in diffs), diffs)

    def test_a_bore_replumbed_to_another_pump_is_flagged(self):
        a = _backpack()
        b = _backpack()
        b.bores[1].pump_id = "P3"
        diffs = fingerprint_diff(build_fingerprint(a), build_fingerprint(b))
        self.assertTrue(any("bore geometry" in d for d in diffs), diffs)

    def test_a_bore_swapped_for_a_different_size_is_flagged(self):
        a = _backpack()
        b = _backpack()
        b.bores[1].id_um = 305.0
        diffs = fingerprint_diff(build_fingerprint(a), build_fingerprint(b))
        self.assertTrue(any("bore geometry" in d for d in diffs), diffs)

    def test_no_fingerprint_reports_no_diff(self):
        """Older file / undescribable needle: report nothing rather than
        manufacture a scary warning out of absent data."""
        self.assertEqual(fingerprint_diff({}, build_fingerprint(_backpack())), [])
        self.assertEqual(fingerprint_diff(None, None), [])
        self.assertEqual(fingerprint_diff(build_fingerprint(_backpack()), {}), [])

    def test_dimension_added_after_the_file_was_written_is_skipped(self):
        saved = {"bore_count": 2}                    # an older, thinner file
        current = build_fingerprint(_backpack())
        self.assertEqual(fingerprint_diff(saved, current), [])

    def test_a_stub_needle_yields_an_empty_fingerprint(self):
        self.assertEqual(build_fingerprint(None), {})
        self.assertEqual(build_fingerprint(object()), {})

    def test_swapped_assembly_reads_as_uncalibrated(self):
        store = self.store()
        store.set_bore(0, (0.0, 0.0), needle=_backpack())
        store.set_bore(1, (320.0, -140.0), needle=_backpack())
        self.assertTrue(store.is_calibrated(_backpack()))
        self.assertFalse(store.is_calibrated(_triple()))

    def test_apply_refuses_on_a_mismatch(self):
        """Pushing one assembly's geometry onto a different one would drive the
        stage a real, confidently-wrong distance — worse than not applying.

        The needle arrives ALREADY carrying offsets, because that is the only
        way this can bite: a fresh ``NeedleSpec`` is already (0, 0), so asserting
        against one would pass with the refusal removed entirely.
        """
        store = self.store()
        store.set_bore(0, (0.0, 0.0), needle=_backpack())
        store.set_bore(1, (320.0, -140.0), needle=_backpack())
        other = _triple()
        other.bores[1].offset_um = (777.0, -888.0)      # from a foreign setup
        other.bores[1].z_offset_mm = 0.5
        self.assertEqual(store.apply_to_needle(other), 0)
        self.assertEqual(other.bore_offset_um(1), (0.0, 0.0))
        self.assertEqual(other.bore(1).z_offset_mm, 0.0)

    def test_force_overrides_the_refusal(self):
        store = self.store()
        store.set_bore(0, (0.0, 0.0), needle=_backpack())
        store.set_bore(1, (320.0, -140.0), needle=_backpack())
        other = _triple()
        self.assertEqual(store.apply_to_needle(other, force=True), 1)
        self.assertEqual(other.bore_offset_um(1), (320.0, -140.0))


# ════════════════════════════════════════════════════════════════════
#  F. apply_to_needle
# ════════════════════════════════════════════════════════════════════

class TestApplyToNeedle(_IsolatedStore):

    def test_offsets_reach_the_live_needle(self):
        needle = _backpack()
        store = self.store()
        store.set_bore(0, (0.0, 0.0), 0.0, needle=needle)
        store.set_bore(1, (320.0, -140.0), 0.040, needle=needle)
        self.assertEqual(store.apply_to_needle(needle), 1)
        self.assertEqual(needle.bores[1].offset_um, (320.0, -140.0))
        self.assertAlmostEqual(needle.bores[1].z_offset_mm, 0.040, places=9)
        # And through the public accessors the executor uses.
        self.assertEqual(needle.bore_offset_um(1), (320.0, -140.0))
        self.assertAlmostEqual(needle.max_bore_z_offset_mm, 0.040, places=9)

    def test_a_single_bore_needle_is_a_no_op(self):
        needle = NeedleSpec(gauge=27, id_um=210, od_um=413, wall_um=102)
        store = self.store()
        store.set_bore(1, (320.0, 0.0))
        self.assertEqual(store.apply_to_needle(needle), 0)

    def test_none_needle_is_a_no_op(self):
        self.assertEqual(self.store().apply_to_needle(None), 0)

    def test_a_record_beyond_the_bore_count_is_ignored(self):
        """A stale record from a 3-bore assembly must not raise on a 2-bore one."""
        needle = _backpack()
        store = self.store()
        store.set_bore(0, (0.0, 0.0), needle=needle)
        store.set_bore(1, (320.0, 0.0), needle=needle)
        store.set_bore(2, (999.0, 999.0), needle=None)   # keeps the fingerprint
        self.assertEqual(store.apply_to_needle(needle), 1)
        self.assertEqual(needle.bore_offset_um(1), (320.0, 0.0))

    def test_apply_is_idempotent(self):
        needle = _backpack()
        store = self.store()
        store.set_bore(0, (0.0, 0.0), needle=needle)
        store.set_bore(1, (320.0, -140.0), 0.04, needle=needle)
        store.apply_to_needle(needle)
        store.apply_to_needle(needle)
        self.assertEqual(needle.bore_offset_um(1), (320.0, -140.0))


class TestUnvouchedOffsetsCannotSurvive(_IsolatedStore):
    """A mount offset is per-MACHINE, but ``NeedleBore`` serializes it, so it
    round-trips through ``HardwareConfig`` — the SWAPPABLE setup file (and its
    ``settings.json`` mirror). A setup saved on another rig, or before a re-seat,
    therefore ARRIVES carrying that mount's offsets, which this machine's store
    cannot vouch for. Declining to apply is not enough: the foreign number stays
    live and the executor commands a real 100-500 µm error while
    ``is_calibrated()`` and the whole Needle-Location UI report the bore as
    unmeasured. That is the CAMERA_CAL_PERSIST_STORE failure mode.
    """

    @staticmethod
    def _swapped_in_via_a_setup_file(needle) -> NeedleSpec:
        """Round-trip through the real setup-file path, not a hand-built spec."""
        from SupportClasses.HardwareConfig import HardwareConfig
        cfg = HardwareConfig()
        cfg.needle = needle
        blob = json.loads(json.dumps(cfg.to_dict()))
        return HardwareConfig.from_dict(blob).needle

    def test_the_setup_file_really_does_carry_the_offsets(self):
        """Premise check — if this ever stops being true the two tests below stop
        testing anything, so it is asserted rather than assumed."""
        measured = _backpack()
        measured.bores[1].offset_um = (320.0, -140.0)
        measured.bores[1].z_offset_mm = 0.04
        arrived = self._swapped_in_via_a_setup_file(measured)
        self.assertEqual(arrived.bore_offset_um(1), (320.0, -140.0))

    def test_an_empty_store_clears_a_foreign_offset(self):
        measured = _backpack()
        measured.bores[1].offset_um = (320.0, -140.0)
        measured.bores[1].z_offset_mm = 0.04
        arrived = self._swapped_in_via_a_setup_file(measured)

        fresh_machine = self.store()                 # never calibrated here
        self.assertFalse(fresh_machine.is_calibrated(arrived))
        self.assertEqual(fresh_machine.apply_to_needle(arrived), 0)
        # The UI says "not measured"; motion must agree with the UI.
        self.assertEqual(arrived.bore_offset_um(1), (0.0, 0.0))
        self.assertEqual(arrived.bore(1).z_offset_mm, 0.0)

    def test_a_partly_measured_store_clears_only_the_unvouched_bores(self):
        """Bore 1 measured here, bore 2 inherited from the file: the measured one
        is applied, the inherited one is discarded."""
        measured = _triple()
        measured.bores[1].offset_um = (320.0, -140.0)
        measured.bores[2].offset_um = (0.0, 500.0)
        arrived = self._swapped_in_via_a_setup_file(measured)

        store = self.store()
        store.set_bore(0, (0.0, 0.0), needle=arrived)
        store.set_bore(1, (111.0, 222.0), needle=arrived)   # THIS rig's value
        self.assertEqual(store.apply_to_needle(arrived), 1)
        self.assertEqual(arrived.bore_offset_um(1), (111.0, 222.0))
        self.assertEqual(arrived.bore_offset_um(2), (0.0, 0.0))

    def test_a_cleared_store_zeroes_the_needle_on_the_next_config_push(self):
        """After a re-seat the operator clicks Clear. The next ``set_hardware_config``
        (which calls ``apply_to_needle``) must not resurrect the offsets from the
        config blob the app keeps mirroring to settings.json."""
        needle = _backpack()
        store = self.store()
        store.set_bore(0, (0.0, 0.0), needle=needle)
        store.set_bore(1, (320.0, -140.0), 0.04, needle=needle)
        store.apply_to_needle(needle)
        arrived = self._swapped_in_via_a_setup_file(needle)   # persisted mirror
        store.clear()
        self.assertEqual(store.apply_to_needle(arrived), 0)
        self.assertEqual(arrived.bore_offset_um(1), (0.0, 0.0))

    def test_the_executor_targets_the_datum_for_an_unvouched_bore(self):
        """Composed with the consumption path: the fail-safe is a MISS AT THE
        DATUM (pre-v7.9 behaviour), never a confidently-wrong 350 µm offset."""
        measured = _backpack()
        measured.bores[1].offset_um = (320.0, -140.0)
        arrived = self._swapped_in_via_a_setup_file(measured)
        self.store().apply_to_needle(arrived)

        ex = _executor(arrived)
        tgt = PickPlaceTarget(target_id="t", x_um=90000.0, y_um=40000.0,
                              well_name="")
        self.assertEqual(ex._bore_target_xy_um(tgt, 1), (90000.0, 40000.0))


# ════════════════════════════════════════════════════════════════════
#  G. CalibrationSnapshotStore invalidation
# ════════════════════════════════════════════════════════════════════

class _Settings:
    def __init__(self, values):
        self._values = values

    def get(self, key, default=None):
        return self._values.get(key, default)


def _fp(**needle_fields) -> dict:
    values = {f"hardware_config.needle.{k}": v
              for k, v in needle_fields.items()}
    return CalibrationSnapshotStore.build_fingerprint(_Settings(values))


class TestSnapshotFingerprintInvalidation(unittest.TestCase):

    def test_bore_count_change_invalidates(self):
        """Every other needle dimension in the fingerprint describes bore 0, so
        swapping a single needle for a backpack was previously invisible."""
        single = _fp(gauge=27, id_um=210.0, num_channels=1)
        backpack = _fp(gauge=27, id_um=210.0, num_channels=1,
                       needle_form=NEEDLE_FORM_BACKPACK,
                       bores=[{"id_um": 413.0}, {"id_um": 159.0}])
        diffs = CalibrationSnapshotStore.fingerprint_diff(single, backpack)
        self.assertTrue(any("bore count" in d for d in diffs), diffs)
        self.assertTrue(any("needle form" in d for d in diffs), diffs)

    def test_backpack_to_triple_invalidates(self):
        a = _fp(needle_form=NEEDLE_FORM_BACKPACK,
                bores=[{"id_um": 413.0}, {"id_um": 159.0}])
        b = _fp(needle_form=NEEDLE_FORM_TRIPLE,
                bores=[{"id_um": 413.0}, {"id_um": 210.0}, {"id_um": 159.0}])
        diffs = CalibrationSnapshotStore.fingerprint_diff(a, b)
        self.assertTrue(any("bore count" in d for d in diffs), diffs)

    def test_bore_count_falls_back_to_num_channels(self):
        """``bores`` is conditional-emit, so a pre-v7.9 needle must still report
        a count rather than None."""
        self.assertEqual(_fp(gauge=27, num_channels=1)["needle_bore_count"], 1)
        self.assertEqual(_fp(gauge=27, num_channels=3)["needle_bore_count"], 3)

    def test_bores_list_wins_over_num_channels(self):
        fp = _fp(num_channels=1, bores=[{"id_um": 1.0}, {"id_um": 2.0}])
        self.assertEqual(fp["needle_bore_count"], 2)

    def test_no_needle_section_leaves_the_count_absent(self):
        fp = CalibrationSnapshotStore.build_fingerprint(_Settings({}))
        self.assertIsNone(fp["needle_bore_count"])
        self.assertIsNone(fp["needle_form"])

    def test_added_dimensions_produce_no_spurious_diff(self):
        """THE regression guard: an old snapshot has neither new key, so it must
        not report 'bore count: None → 1' for every existing user on first
        launch. Same rule test_v76:540 pins for the v7.6 tip dimensions."""
        saved = {"device": "ME3B V1", "needle_gauge": 27}
        current = _fp(gauge=27, num_channels=1)
        current["device"] = "ME3B V1"
        current["needle_gauge"] = 27
        self.assertEqual(
            CalibrationSnapshotStore.fingerprint_diff(saved, current), [])

    def test_unchanged_single_bore_setup_reports_no_diff(self):
        fp = _fp(gauge=27, id_um=210.0, num_channels=1)
        self.assertEqual(CalibrationSnapshotStore.fingerprint_diff(fp, dict(fp)),
                         [])


# ════════════════════════════════════════════════════════════════════
#  H. Vocabulary (v7.9: the bare word "channel" is banned)
# ════════════════════════════════════════════════════════════════════

class TestVocabulary(unittest.TestCase):

    def test_no_bare_channel_identifier(self):
        for rel in ("SupportClasses/NeedleBoreCalibrationStore.py",
                    "tests/test_v79_bore_offset_calibration.py"):
            tree = ast.parse((REPO / rel).read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    for arg in list(node.args.args) + list(node.args.kwonlyargs):
                        self.assertNotEqual(arg.arg, "channel",
                                            f"{rel}: {node.name}(channel=)")
                elif isinstance(node, ast.keyword):
                    self.assertNotEqual(node.arg, "channel", rel)


# ════════════════════════════════════════════════════════════════════
#  I. The Needle Location tab flow
# ════════════════════════════════════════════════════════════════════
#
# The real (unbound) CalibrationPage methods are called against duck-typed stubs
# — the pattern test_v75x_needle_location_quick_move.py uses — so no Qt widget,
# camera or event loop is needed. ``QMessageBox`` MUST be patched: it blocks
# forever under offscreen Qt.

import gui.pages.calibration as calmod                       # noqa: E402
from gui.pages.calibration import CalibrationPage            # noqa: E402
from types import SimpleNamespace                            # noqa: E402


class _FakeMB:
    """Stand-in for QMessageBox (the real one blocks forever headless)."""

    class StandardButton:
        Yes = 1
        No = 0

    warned: list = []
    questions: list = []
    answer = 1                    # StandardButton.Yes

    @classmethod
    def reset(cls):
        cls.warned = []
        cls.questions = []
        cls.answer = cls.StandardButton.Yes

    @classmethod
    def warning(cls, _parent, _title, text, *a, **k):
        cls.warned.append(text)

    @classmethod
    def information(cls, _parent, _title, text, *a, **k):
        cls.warned.append(text)

    @classmethod
    def question(cls, _parent, _title, text, *a, **k):
        cls.questions.append(text)
        return cls.answer


class _FakeLabel:
    def __init__(self):
        self.text = ""

    def setText(self, t):
        self.text = t

    def setStyleSheet(self, *a):
        pass


class _BoreFlowBase(_IsolatedStore):
    """A page stub wired to a throwaway store, with the modals patched."""

    def setUp(self):
        super().setUp()
        _FakeMB.reset()
        self._orig_mb = calmod.QMessageBox
        calmod.QMessageBox = _FakeMB
        self.addCleanup(lambda: setattr(calmod, "QMessageBox", self._orig_mb))

    def _stub(self, needle=None, *, xy=(100000.0, 50000.0), raw_z=-20.0,
              xy_connected=True, cams_ok=True, picks=0, store=None,
              z_dz_um=None):
        if needle is None:
            needle = _backpack()
        self._store = store if store is not None else self.store()
        self._recorded_origin = []
        self._resets = []
        stub = SimpleNamespace(
            _hardware_config=SimpleNamespace(needle=needle),
            controller=SimpleNamespace(
                is_xy_connected=xy_connected,
                get_xy_position=lambda cached=False: xy,
                capture_current_z_raw=lambda: raw_z,
                raw_to_user_z=lambda raw: -float(raw),   # z_up_sign = -1 machine
            ),
            _bore_cal_status=_FakeLabel(),
            _needle_loc_step=picks,
            _needle_loc_z_center_chk=SimpleNamespace(
                isChecked=lambda: z_dz_um is not None),
            # Stand in for the two-camera aligner: a fixed residual.
            _needle_loc_compute_offset_um=lambda: (7.0, -3.0),
            _needle_loc_compute_z_offset_um=lambda: z_dz_um,
            _needle_loc_camera_info=(
                (lambda role: (1.67, 640, 45.0)) if cams_ok
                else (lambda role: None)),
            _needle_loc_record_origin_here=(
                lambda: (self._recorded_origin.append(True) or True)),
            _needle_loc_reset=lambda: self._resets.append(True),
            _bore_cal_refresh=lambda: None,     # the widget rebuild is not under test
        )
        _bind = lambda *names: [
            setattr(stub, n, getattr(CalibrationPage, n).__get__(stub))
            for n in names]
        _bind("_bore_cal_needle", "_bore_cal_bore_count", "_bore_cal_gate",
              "_bore_cal_capture", "_bore_cal_clear", "_bore_cal_apply_stored",
              "_bore_cal_centred_position")
        stub._bore_cal_store = lambda: self._store
        return stub


class TestBoreFlowGate(_BoreFlowBase):
    """Refuse CLEARLY. A capture that looked fine but recorded garbage would
    drive the needle a confidently-wrong few hundred µm on every run."""

    def test_single_bore_needle_is_refused(self):
        stub = self._stub(NeedleSpec(gauge=27, id_um=210, od_um=413))
        ok, why = stub._bore_cal_gate()
        self.assertFalse(ok)
        self.assertIn("multi-bore", why)

    def test_no_needle_is_refused(self):
        stub = self._stub()
        stub._hardware_config = None
        ok, why = stub._bore_cal_gate()
        self.assertFalse(ok)
        self.assertIn("multi-bore", why)

    def test_xy_disconnected_is_refused(self):
        stub = self._stub(xy_connected=False)
        ok, why = stub._bore_cal_gate()
        self.assertFalse(ok)
        self.assertIn("XY stage", why)

    def test_uncalibrated_needle_cameras_are_refused_by_name(self):
        stub = self._stub(cams_ok=False)
        ok, why = stub._bore_cal_gate()
        self.assertFalse(ok)
        self.assertIn("Needle cam 1", why)
        self.assertIn("Needle cam 2", why)
        self.assertIn("µm/px", why)

    def test_a_refused_capture_warns_and_records_nothing(self):
        stub = self._stub(xy_connected=False)
        stub._bore_cal_capture(1)
        self.assertEqual(len(_FakeMB.warned), 1)
        self.assertEqual(self._store.all_bores(), [])

    def test_a_ready_rig_passes(self):
        ok, why = self._stub()._bore_cal_gate()
        self.assertTrue(ok, why)


class TestBoreFlowCapture(_BoreFlowBase):

    def test_datum_capture_reuses_the_existing_origin_recorder(self):
        """Bore 1 IS ``needle_origin_um`` — one writer for one datum."""
        stub = self._stub()
        stub._bore_cal_capture(0)
        self.assertEqual(len(_FakeMB.questions), 1)      # confirmed, not silent
        self.assertEqual(self._recorded_origin, [True])
        rec = self._store.get_bore(0)
        self.assertIsNotNone(rec)
        self.assertEqual(rec.offset_um, (0.0, 0.0))
        self.assertEqual(rec.stage_um, (100000.0, 50000.0))

    def test_declining_the_datum_confirm_records_nothing(self):
        _FakeMB.answer = _FakeMB.StandardButton.No
        stub = self._stub()
        stub._bore_cal_capture(0)
        self.assertEqual(self._store.all_bores(), [])
        self.assertEqual(self._recorded_origin, [])

    def test_a_bore_before_the_datum_is_refused(self):
        stub = self._stub()
        stub._bore_cal_capture(1)
        self.assertTrue(any("bore 1" in w.lower() for w in _FakeMB.warned),
                        _FakeMB.warned)
        self.assertEqual(self._store.all_bores(), [])

    def test_offset_is_the_difference_of_the_two_centred_positions(self):
        stub = self._stub(xy=(100000.0, 50000.0))
        stub._bore_cal_capture(0)
        # Bore 2 needed the stage backed off 320 µm in X / forward 140 in Y.
        stub.controller.get_xy_position = lambda cached=False: (99680.0, 50140.0)
        stub._bore_cal_capture(1)
        self.assertEqual(self._store.offset_um(1), (320.0, -140.0))
        # ...and it reached the LIVE needle, so the executor uses it at once.
        self.assertEqual(stub._hardware_config.needle.bore_offset_um(1),
                         (320.0, -140.0))

    def test_z_offset_uses_the_height_frame(self):
        """raw_to_user_z here is ``-raw`` (a z_up_sign=-1 machine), so a bore
        centred at a LOWER raw Z is HIGHER in the height frame and its offset
        must come out positive = 'reaches lower'."""
        stub = self._stub(raw_z=-20.000)
        stub._bore_cal_capture(0)
        stub.controller.capture_current_z_raw = lambda: -20.040
        stub._bore_cal_capture(1)
        self.assertAlmostEqual(self._store.z_offset_mm(1), 0.040, places=6)

    def test_no_z_reading_leaves_the_z_offset_at_zero(self):
        stub = self._stub()
        stub.controller.capture_current_z_raw = lambda: None
        stub._bore_cal_capture(0)
        stub.controller.get_xy_position = lambda cached=False: (99680.0, 50000.0)
        stub._bore_cal_capture(1)
        self.assertEqual(self._store.z_offset_mm(1), 0.0)
        self.assertEqual(self._store.offset_um(1), (320.0, 0.0))

    def test_the_edge_picks_refine_the_position_when_present(self):
        """Four picks ⇒ apply the same (dx, dy) Center & Save would move by."""
        stub = self._stub(picks=4, xy=(100000.0, 50000.0))
        stub._bore_cal_capture(0)
        self.assertEqual(self._store.get_bore(0).stage_um,
                         (100007.0, 49997.0))         # + the (7, -3) residual

    def test_the_refinement_is_skipped_without_a_full_pick_set(self):
        stub = self._stub(picks=2, xy=(100000.0, 50000.0))
        stub._bore_cal_capture(0)
        self.assertEqual(self._store.get_bore(0).stage_um, (100000.0, 50000.0))

    def test_the_picks_are_reset_after_every_capture(self):
        """Otherwise bore 2 would be 'refined' by bore 1's stale clicks — a
        silently wrong offset."""
        stub = self._stub(picks=4)
        stub._bore_cal_capture(0)
        self.assertEqual(self._resets, [True])

    def test_re_measuring_the_datum_discards_the_old_offsets(self):
        """They were taken against a different datum position, so they no longer
        mean anything — in the store AND on the live needle the executor reads."""
        stub = self._stub()
        stub._bore_cal_capture(0)
        stub.controller.get_xy_position = lambda cached=False: (99680.0, 50140.0)
        stub._bore_cal_capture(1)
        self.assertEqual(self._store.measured_bore_indices(), [0, 1])
        self.assertEqual(stub._hardware_config.needle.bore_offset_um(1),
                         (320.0, -140.0))
        stub.controller.get_xy_position = lambda cached=False: (80000.0, 40000.0)
        stub._bore_cal_capture(0)
        self.assertEqual(self._store.measured_bore_indices(), [0])
        # The store forgetting is not enough: the offset lives on the needle too,
        # and a re-based datum makes it meaningless rather than merely unproven.
        self.assertEqual(stub._hardware_config.needle.bore_offset_um(1),
                         (0.0, 0.0))

    def test_an_unreadable_stage_warns_and_records_nothing(self):
        stub = self._stub()
        stub.controller.get_xy_position = lambda cached=False: (None, None)
        stub._bore_cal_capture(0)
        self.assertEqual(self._store.all_bores(), [])
        self.assertTrue(_FakeMB.warned)

    def test_no_stage_motion_is_commanded(self):
        """The operator jogs; these buttons only READ. A controller stub with no
        motion methods at all must complete the whole flow — which is the
        structural proof that the retract-before-XY rule cannot be violated."""
        stub = self._stub()
        stub._bore_cal_capture(0)
        stub.controller.get_xy_position = lambda cached=False: (99680.0, 50140.0)
        stub._bore_cal_capture(1)
        for banned in ("move_xy_absolute_um", "move_xy_relative_um",
                       "safe_travel_to", "move_z_absolute",
                       "move_z_user_relative", "ensure_retracted_to"):
            self.assertFalse(hasattr(stub.controller, banned))


class TestBoreFlowClearAndApply(_BoreFlowBase):

    def test_clear_forgets_the_store_and_zeroes_the_live_needle(self):
        stub = self._stub()
        stub._bore_cal_capture(0)
        stub.controller.get_xy_position = lambda cached=False: (99680.0, 50140.0)
        stub._bore_cal_capture(1)
        self.assertEqual(stub._hardware_config.needle.bore_offset_um(1),
                         (320.0, -140.0))
        stub._bore_cal_clear()
        self.assertEqual(self._store.all_bores(), [])
        self.assertEqual(stub._hardware_config.needle.bore_offset_um(1),
                         (0.0, 0.0))

    def test_declining_the_clear_confirm_keeps_everything(self):
        stub = self._stub()
        stub._bore_cal_capture(0)
        _FakeMB.answer = _FakeMB.StandardButton.No
        stub._bore_cal_clear()
        self.assertEqual(self._store.measured_bore_indices(), [0])

    def test_apply_is_a_no_op_without_a_needle(self):
        stub = self._stub()
        stub._hardware_config = None
        self.assertEqual(stub._bore_cal_apply_stored(), 0)

    def test_apply_pushes_a_previously_saved_measurement(self):
        """The restart path: measurements taken last session reach a needle
        object built fresh from the config."""
        needle = _backpack()
        seeded = self.store()
        seeded.set_bore(0, (0.0, 0.0), needle=needle)
        seeded.set_bore(1, (320.0, -140.0), 0.04, needle=needle)
        fresh = _backpack()                       # as rebuilt from settings.json
        self.assertEqual(fresh.bore_offset_um(1), (0.0, 0.0))
        stub = self._stub(fresh, store=self.store())
        self.assertEqual(stub._bore_cal_apply_stored(), 1)
        self.assertEqual(fresh.bore_offset_um(1), (320.0, -140.0))


class TestBoreGroupOnTheRealPage(unittest.TestCase):
    """One offscreen build of the real page: the group exists, is hidden for a
    single-bore setup, and appears for a backpack."""

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        import sys
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="mebp_bore_cal_page_")
        self.addCleanup(self._tmp.cleanup)
        self._prev = os.environ.get("MEBP_NEEDLE_BORE_CAL_PATH")
        os.environ["MEBP_NEEDLE_BORE_CAL_PATH"] = str(
            Path(self._tmp.name) / "bore.json")
        import SupportClasses.NeedleBoreCalibrationStore as mod
        self._prev_singleton = mod._store
        mod._store = None

        def _restore():
            mod._store = self._prev_singleton
            if self._prev is None:
                os.environ.pop("MEBP_NEEDLE_BORE_CAL_PATH", None)
            else:
                os.environ["MEBP_NEEDLE_BORE_CAL_PATH"] = self._prev
        self.addCleanup(_restore)

    def _page(self):
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (1000.0, 2000.0)
        ctrl.z_up_sign.return_value = 1.0
        return CalibrationPage(ctrl, settings=None)

    def test_group_is_built_and_hidden_by_default(self):
        page = self._page()
        self.assertTrue(hasattr(page, "_bore_cal_group"))
        self.assertFalse(page._bore_cal_group.isVisibleTo(page))

    def test_group_stays_hidden_for_a_single_bore_needle(self):
        page = self._page()
        page._hardware_config = SimpleNamespace(
            needle=NeedleSpec(gauge=27, id_um=210, od_um=413))
        page._bore_cal_refresh()
        self.assertFalse(page._bore_cal_group.isVisibleTo(page))
        self.assertEqual(page._bore_cal_row_widgets, [])

    @staticmethod
    def _with_cameras(page):
        """Pretend both needle cams are live + µm/px-calibrated.

        There are no cameras in a headless build, so without this the gate
        refuses and its message (correctly) wins the status line — which is
        itself asserted by :meth:`test_missing_cameras_disable_the_row_buttons`.
        """
        page._needle_loc_camera_info = lambda role: (1.67, 640, 45.0)
        page.controller.is_xy_connected = True

    def test_a_backpack_gets_one_row_per_bore(self):
        page = self._page()
        page._hardware_config = SimpleNamespace(needle=_backpack())
        self._with_cameras(page)
        page._bore_cal_refresh()
        self.assertTrue(page._bore_cal_group.isVisibleTo(page))
        self.assertEqual(len(page._bore_cal_row_widgets), 2)
        # Nothing measured yet → point at the datum, since bore 2's offset is
        # meaningless until bore 1 has been recorded.
        self.assertIn("start with bore 1",
                      page._bore_cal_status.text().lower())

    def test_status_lists_the_bores_still_needed(self):
        import SupportClasses.NeedleBoreCalibrationStore as mod
        needle = _triple()
        store = mod.get_store()
        store.set_bore(0, (0.0, 0.0), needle=needle)
        store.set_bore(1, (250.0, 0.0), needle=needle)
        page = self._page()
        page._hardware_config = SimpleNamespace(needle=needle)
        self._with_cameras(page)
        page._bore_cal_refresh()
        text = page._bore_cal_status.text()
        self.assertIn("1 of 2", text)
        self.assertIn("bore 3", text)

    def test_missing_cameras_disable_the_row_buttons(self):
        """The refusal reaches the operator BOTH ways: the status names what is
        missing and the per-bore buttons cannot be pressed."""
        page = self._page()
        page._hardware_config = SimpleNamespace(needle=_backpack())
        page._bore_cal_refresh()
        self.assertIn("Needle cam 1", page._bore_cal_status.text())
        from PySide6.QtWidgets import QPushButton
        buttons = [b for w in page._bore_cal_row_widgets
                   for b in w.findChildren(QPushButton)]
        self.assertEqual(len(buttons), 2)
        self.assertFalse(any(b.isEnabled() for b in buttons))

    def test_rows_are_replaced_not_appended_when_the_form_changes(self):
        page = self._page()
        self._with_cameras(page)
        page._hardware_config = SimpleNamespace(needle=_backpack())
        page._bore_cal_refresh()
        page._hardware_config = SimpleNamespace(needle=_triple())
        page._bore_cal_refresh()
        self.assertEqual(len(page._bore_cal_row_widgets), 3)

    def test_status_reports_a_stale_assembly(self):
        import SupportClasses.NeedleBoreCalibrationStore as mod
        store = mod.get_store()
        store.set_bore(0, (0.0, 0.0), needle=_backpack())
        store.set_bore(1, (320.0, -140.0), needle=_backpack())
        page = self._page()
        page._hardware_config = SimpleNamespace(needle=_triple())
        self._with_cameras(page)
        page._bore_cal_refresh()
        self.assertIn("DIFFERENT assembly", page._bore_cal_status.text())

    def _row_texts(self, page):
        from PySide6.QtWidgets import QLabel
        return [lbl.text() for w in page._bore_cal_row_widgets
                for lbl in w.findChildren(QLabel)]

    def test_a_stale_row_says_the_offset_is_not_in_use(self):
        """Otherwise the row prints an offset the executor is (correctly)
        refusing to apply, which reads as 'calibrated' when it is not."""
        import SupportClasses.NeedleBoreCalibrationStore as mod
        store = mod.get_store()
        store.set_bore(0, (0.0, 0.0), needle=_backpack())
        store.set_bore(1, (320.0, -140.0), needle=_backpack())
        page = self._page()
        page._hardware_config = SimpleNamespace(needle=_triple())
        self._with_cameras(page)
        page._bore_cal_refresh()
        self.assertTrue(any("not in use" in t for t in self._row_texts(page)),
                        self._row_texts(page))

    def test_rows_report_datum_measured_and_unmeasured(self):
        import SupportClasses.NeedleBoreCalibrationStore as mod
        needle = _triple()
        store = mod.get_store()
        store.set_bore(0, (0.0, 0.0), needle=needle)
        store.set_bore(1, (250.0, 0.0), 0.02, needle=needle)
        page = self._page()
        page._hardware_config = SimpleNamespace(needle=needle)
        self._with_cameras(page)
        page._bore_cal_refresh()
        texts = self._row_texts(page)
        self.assertEqual(len(texts), 3)
        self.assertIn("datum", texts[0])
        self.assertIn("+250", texts[1])
        self.assertIn("+0.020", texts[1])
        self.assertIn("not measured", texts[2])
        # The offset must appear exactly once per row (it used to be printed
        # twice — once by assembly_summary_lines and once by the store).
        self.assertEqual(texts[1].count("offset"), 1)

    def test_status_reports_a_complete_measurement(self):
        import SupportClasses.NeedleBoreCalibrationStore as mod
        needle = _backpack()
        store = mod.get_store()
        store.set_bore(0, (0.0, 0.0), needle=needle)
        store.set_bore(1, (320.0, -140.0), needle=needle)
        page = self._page()
        page._hardware_config = SimpleNamespace(needle=needle)
        self._with_cameras(page)
        page._bore_cal_refresh()
        self.assertIn("✓", page._bore_cal_status.text())

    def test_set_hardware_config_applies_the_stored_offsets(self):
        """The restart path through the real page: a config push must put the
        measured offsets onto the needle the executor will read."""
        import SupportClasses.NeedleBoreCalibrationStore as mod
        from SupportClasses.HardwareConfig import HardwareConfig
        needle = _backpack()
        store = mod.get_store()
        store.set_bore(0, (0.0, 0.0), needle=needle)
        store.set_bore(1, (320.0, -140.0), 0.04, needle=needle)
        page = self._page()
        cfg = HardwareConfig()
        cfg.needle = _backpack()                  # fresh, unmeasured
        self.assertEqual(cfg.needle.bore_offset_um(1), (0.0, 0.0))
        page.set_hardware_config(cfg)
        self.assertEqual(cfg.needle.bore_offset_um(1), (320.0, -140.0))
        self.assertAlmostEqual(cfg.needle.max_bore_z_offset_mm, 0.04, places=9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
