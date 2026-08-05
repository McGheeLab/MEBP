"""test_v79_other_workflows_bore_safety.py — D12 + D7.

D12: Spheroid Pick & Place and Cell Labeling let the operator choose a PUMP but
always position **bore 1** and always size their volumes from **bore 1's**
orifice area — they were never taught about bores. On the reference backpack the
bore0/bore1 orifice-area ratio is 6.75×, so choosing the second bore's pump
over-doses by that ratio AND lands 100-500 µm off the region, silently. Per the
operator's decision these two workflows are RESTRICTED to bore 1 for now
(per-bore support is a separate job); this file pins the restriction and its
Start backstop.

D7: an abort between the dose and the recovery pull leaves reagent digesting a
live cell. There is deliberately no automatic recovery aspirate — the operator is
TOLD, and given a manual clean. These tests pin the disclosure, and pin that no
pump move is added to the abort path.
"""

from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from SupportClasses.PhysicalModels import (
    NeedleSpec, NeedleBore, NEEDLE_FORM_BACKPACK, needle_orifice_area_mm2,
)
from gui.pages.workflows import _reagent_prep


def _backpack():
    """Bore 0 = 22G (P1, datum). Bore 1 = 30G (P2). Areas differ ~6.75×."""
    return NeedleSpec(
        needle_form=NEEDLE_FORM_BACKPACK,
        bores=[
            NeedleBore(gauge=22, od_um=718, id_um=413, wall_um=152,
                       length_mm=50.8, pump_id="P1"),
            NeedleBore(gauge=30, od_um=311, id_um=159, wall_um=76,
                       length_mm=50.8, pump_id="P2", offset_um=(320.0, -140.0)),
        ],
    )


def _single():
    return NeedleSpec(gauge=27, id_um=210, od_um=413)


def _hw(needle):
    return SimpleNamespace(needle=needle, pumps={})


class TestTheStakeIsReal(unittest.TestCase):
    def test_the_two_bores_differ_by_the_ratio_that_motivates_this(self):
        n = _backpack()
        a0 = needle_orifice_area_mm2(n.bore(0))
        a1 = needle_orifice_area_mm2(n.bore(1))
        self.assertGreater(a0 / a1, 6.0,
                           "fixture must exhibit the dose-error ratio")
        self.assertLess(a0 / a1, 7.5)


class TestDatumBorePumpResolution(unittest.TestCase):
    def test_a_multi_bore_assembly_resolves_bore_1s_pump(self):
        self.assertEqual(_reagent_prep.datum_bore_pump(_hw(_backpack())), "P1")

    def test_a_single_bore_assembly_imposes_NO_restriction(self):
        """Unknown/irrelevant data must never forbid work."""
        self.assertIsNone(_reagent_prep.datum_bore_pump(_hw(_single())))

    def test_no_needle_imposes_no_restriction(self):
        self.assertIsNone(_reagent_prep.datum_bore_pump(_hw(None)))

    def test_no_hardware_config_imposes_no_restriction(self):
        self.assertIsNone(_reagent_prep.datum_bore_pump(None))

    def test_a_mock_config_imposes_no_restriction(self):
        """The partial-page test pattern must not trip the restriction."""
        self.assertIsNone(_reagent_prep.datum_bore_pump(MagicMock()))

    def test_a_datum_bore_with_no_pump_imposes_no_restriction(self):
        n = NeedleSpec(
            needle_form=NEEDLE_FORM_BACKPACK,
            bores=[NeedleBore(gauge=22, od_um=718, id_um=413, pump_id=None),
                   NeedleBore(gauge=30, od_um=311, id_um=159, pump_id="P2")])
        self.assertIsNone(_reagent_prep.datum_bore_pump(_hw(n)))

    def test_pump_ids_are_normalised(self):
        n = NeedleSpec(
            needle_form=NEEDLE_FORM_BACKPACK,
            bores=[NeedleBore(gauge=22, od_um=718, id_um=413, pump_id=" p2 "),
                   NeedleBore(gauge=30, od_um=311, id_um=159, pump_id="P3")])
        self.assertEqual(_reagent_prep.datum_bore_pump(_hw(n)), "P2")

    def test_the_note_names_the_bore_count_and_the_pump(self):
        note = _reagent_prep.multi_bore_restriction_note(_hw(_backpack()))
        self.assertIn("bore 1", note)
        self.assertIn("P1", note)
        self.assertIn("2 bores", note)
        self.assertIn("Cell Targeting", note, "must point at the capable workflow")

    def test_no_note_for_a_single_bore(self):
        self.assertEqual(
            _reagent_prep.multi_bore_restriction_note(_hw(_single())), "")


class TestAbortLeavesNoMotionOnThePumpPath(unittest.TestCase):
    """The reason there is no automatic dose recovery: an abort must not command
    new pump moves. That is what keeps Abort responsive during a long drain."""

    def test_no_pump_move_is_emitted_once_the_abort_flag_is_set(self):
        from SupportClasses.PickAndPlaceManager import (
            AbortException, CellRemovalConfig, OperationType,
            PickPlaceExecutor, PickPlaceOperation, PickPlaceTarget,
        )

        class _Ctrl:
            def __init__(self):
                self.calls = []
                self.is_zp_connected = True
                self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

            def z_up_sign(self):
                return 1.0

            def safe_travel_to(self, **k):
                self.calls.append("travel")
                return True

            def move_xy_absolute_um(self, x, y):
                self.calls.append("xy")

            def wait_for_xy_arrival(self, *a, **k):
                return True

            def wait_for_z_arrival(self, *a, **k):
                return True

            def move_z_user_relative(self, dz):
                pass

            def move_z_absolute(self, z, from_zero_ref=False):
                pass

            def ensure_retracted_to(self, z, *a, **k):
                self.calls.append("retract")
                return True

            def move_pump_uL(self, pump, volume_uL, **k):
                self.calls.append(f"pump:{pump}")

        ctrl = _Ctrl()
        ex = PickPlaceExecutor.__new__(PickPlaceExecutor)
        ex.controller = ctrl
        hw = MagicMock()
        hw.needle = _backpack()
        ex.hw_config = hw
        for attr, val in (("on_sub_step", None), ("on_dwell_tick", None),
                          ("safe_z_mm", 40.0), ("operating_z_mm", 10.0),
                          ("pick_z_mm", 10.0), ("place_z_mm", 12.0),
                          ("reagent_well_pos", (1.0, 2.0)),
                          ("reagent_dip_z_mm", 15.0),
                          ("wash_after_pickup", False),
                          ("intra_well_retract_mm", 1.0),
                          ("z_timeout_s", 5.0), ("xy_timeout_s", 5.0),
                          ("_current_well", None), ("bore_area_mm2", 0.0),
                          ("bore_profile", None), ("_well_positions", {}),
                          ("_pending_dose", None)):
            setattr(ex, attr, val)
        import threading
        ex._abort_flag = threading.Event()
        ex._pause_event = threading.Event()
        ex._pause_event.set()
        ex._abort_flag.set()          # abort already in flight

        cfg = CellRemovalConfig(reagent_bore="P1", release_depth_mm=0.10,
                                dwell_time_s=0.0)
        t = PickPlaceTarget(target_id="T1", x_um=1.0, y_um=2.0, well_name="A1")
        op = PickPlaceOperation(op_id="OP1",
                                op_type=OperationType.CELL_TARGET_REMOVAL,
                                source_target=t, dest_target=t, config=cfg)
        with self.assertRaises(AbortException):
            ex._execute_cell_removal(op)
        self.assertEqual([c for c in ctrl.calls if c.startswith("pump")], [],
                         "an in-flight abort must command no pump motion")

    def test_the_warning_names_the_target_the_volume_and_the_remedy(self):
        from SupportClasses.PickAndPlaceManager import PickPlaceExecutor
        ex = PickPlaceExecutor.__new__(PickPlaceExecutor)
        ex._pending_dose = {"target_id": "T012", "bore": 2, "pump": "P2",
                            "volume_uL": 0.00071}
        msg = ex.pending_dose_warning()
        self.assertIn("T012", msg)
        self.assertIn("bore 2", msg)
        self.assertIn("0.00071", msg)
        self.assertIn("serum", msg, "must state the operator's actual remedy")
        self.assertIn("Clean needle", msg)

    def test_no_warning_when_nothing_is_pending(self):
        from SupportClasses.PickAndPlaceManager import PickPlaceExecutor
        ex = PickPlaceExecutor.__new__(PickPlaceExecutor)
        ex._pending_dose = None
        self.assertIsNone(ex.pending_dose_warning())

    def test_a_partial_executor_does_not_crash_the_check(self):
        from SupportClasses.PickAndPlaceManager import PickPlaceExecutor
        ex = PickPlaceExecutor.__new__(PickPlaceExecutor)
        self.assertIsNone(ex.pending_dose_warning())


if __name__ == "__main__":
    unittest.main()
