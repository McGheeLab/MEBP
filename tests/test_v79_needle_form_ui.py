"""
v7.9 — Hardware Setup → Needle: assembly FORM + per-bore geometry rows.

The operator uses three needle forms: a single needle, a **backpack** (two
needles of DIFFERENT sizes bound together) and a **triple** (three fused). Before
v7.9 the Needle card held ONE geometry plus a bare bore-count spin, so a
backpack's second diameter was literally unrepresentable.

What these tests pin, in order of how much damage the failure would do:

1. **Byte-identity for a single needle.** ``NeedleSpec.to_dict()`` must still
   emit exactly the seven legacy keys, and all six real on-disk setups must load
   and re-save through the page unchanged. A regression here silently rewrites
   every saved setup on the operator's machine.
2. **A backpack round-trips two DIFFERENT diameters** through
   ``_rebuild_config`` → ``to_dict`` → ``from_dict`` → ``_apply_config_to_ui``.
   That is the whole point of the feature.
3. **Duplicate-pump assignment is reported** — one pump can only push one
   volume, so a shared pump drives the second bore blind.
4. **The per-bore flow ceilings differ.** I measured 45× between a 22G and a 30G
   bore and ~2000× between a 22G bore and a 30 µm pulled tip; applying the
   coarse bore's ceiling to the fine bore's pump over-pressures it and shatters
   glass.
5. **Unmeasured offsets read "not measured"** rather than a plausible-looking
   zero. Mount offsets are a per-MOUNT calibration (the assembly's rotation in
   the holder is arbitrary), so an unmeasured bore is positioned as if it sat
   exactly where bore 1 does — off target by 100–500 µm, larger than a cell.
6. **A form switch does not silently destroy bore geometry.** The bore-count spin
   used to wipe the pump map and immediately persist the emptied dict.

⚠ ``QMessageBox`` blocks forever under offscreen Qt, so nothing here may take a
path that opens one (none of the form/bore handlers do).
"""

from __future__ import annotations

import json
import os
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.PhysicalModels import (          # noqa: E402
    NeedleSpec, NeedleBore,
    NEEDLE_TYPE_HYPODERMIC, NEEDLE_TYPE_CAPILLARY,
    TIP_PROFILE_CYLINDER, TIP_PROFILE_CONE,
    NEEDLE_FORM_SINGLE, NEEDLE_FORM_BACKPACK, NEEDLE_FORM_TRIPLE,
    NEEDLE_FORM_BORE_COUNT,
)
from SupportClasses.HardwareConfig import HardwareConfig  # noqa: E402

CONFIG_HARDWARE_DIR = (
    Path(__file__).resolve().parent.parent / "config" / "hardware")

# The exact legacy key set — same list the v7.6 byte-identity tests assert.
LEGACY_KEYS = {
    "gauge", "od_um", "id_um", "wall_um", "length_inches",
    "num_channels", "channel_pump_map",
}


def _page():
    """The real Hardware Setup page, built once per class."""
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    from gui.pages.hardware_setup import HardwareSetupPage
    return app, HardwareSetupPage()


def _select(combo, data):
    idx = combo.findData(data)
    assert idx >= 0, f"{data!r} not offered by {combo}"
    combo.setCurrentIndex(idx)
    return idx


def _enable_pumps(page, *pump_ids):
    """Enable the given pumps so the per-bore pump combos offer them."""
    from SupportClasses.PhysicalModels import SyringeSpec
    for pid in pump_ids:
        pw = page._pump_widgets[pid]
        pw.enable_check.setChecked(True)
        # A pump needs a syringe to count as configured; pick whatever the
        # catalog offers first.
        if pw.syringe_combo.count() > 1 and not pw.syringe_combo.currentData():
            pw.syringe_combo.setCurrentIndex(1)
    page._refresh_channel_map_pump_options()


# ════════════════════════════════════════════════════════════════════
#  A. Single form stays byte-identical
# ════════════════════════════════════════════════════════════════════

class TestSingleFormByteIdentity(unittest.TestCase):
    """A single needle must serialize exactly as it did before v7.9."""

    @classmethod
    def setUpClass(cls):
        cls.app, cls.page = _page()

    def test_default_form_is_single(self):
        self.assertEqual(self.page._needle_form_combo.currentData(),
                         NEEDLE_FORM_SINGLE)
        self.assertEqual(self.page._form_bore_count(), 1)

    def test_single_needle_emits_exactly_the_seven_legacy_keys(self):
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec(gauge=27, od_um=413.0, id_um=210.0,
                                wall_um=102.0, length_inches=1.0)
        self.page.set_config(cfg)
        got = self.page.get_config().needle
        self.assertEqual(set(got.to_dict()), LEGACY_KEYS)
        # Neither v7.9 key may appear.
        self.assertNotIn("needle_form", got.to_dict())
        self.assertNotIn("bores", got.to_dict())
        self.assertIsNone(got.bores)

    def test_bore_group_is_hidden_for_a_single_needle(self):
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec(gauge=27, od_um=413.0, id_um=210.0,
                                wall_um=102.0)
        self.page.set_config(cfg)
        self.assertTrue(self.page._bore_group.isHidden())
        self.assertTrue(self.page._needle_datum_note.isHidden())
        # The legacy bore→pump card is the pump surface for a single bore.
        self.assertFalse(self.page.channel_map_group.isHidden())

    def test_bore_count_spin_is_a_hidden_mirror_of_the_form(self):
        # One quantity, one writer. Two independently editable controls for the
        # bore count is how a count/geometry mismatch gets shipped.
        self.assertTrue(self.page.channels_spin.isHidden())
        _select(self.page._needle_form_combo, NEEDLE_FORM_TRIPLE)
        self.assertEqual(self.page.channels_spin.value(), 3)
        _select(self.page._needle_form_combo, NEEDLE_FORM_SINGLE)
        self.assertEqual(self.page.channels_spin.value(), 1)

    def test_every_on_disk_setup_round_trips_through_the_page(self):
        """The six real setup files must survive set_config → get_config."""
        files = sorted(CONFIG_HARDWARE_DIR.glob("*.json"))
        checked = 0
        for path in files:
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
            except Exception:
                continue
            if not isinstance(payload, dict) or "needle" not in payload:
                continue
            needle_payload = payload.get("needle")
            if not isinstance(needle_payload, dict):
                continue
            cfg = HardwareConfig.from_dict(payload)
            if cfg.needle is None:
                continue
            before = cfg.needle.to_dict()
            self.page.set_config(cfg)
            after = self.page.get_config().needle.to_dict()
            with self.subTest(path.name):
                self.assertEqual(set(after), set(before))
                for key in set(before) - {"channel_pump_map"}:
                    self.assertEqual(after[key], before[key], key)
            checked += 1
        # Six real setups exist today; the point of the number is that a glob
        # that silently matched nothing would make this test vacuous.
        self.assertGreaterEqual(checked, 6,
                                f"only {checked} on-disk setup(s) exercised")


# ════════════════════════════════════════════════════════════════════
#  B. Backpack: two DIFFERENT diameters round-trip
# ════════════════════════════════════════════════════════════════════

class TestBackpackRoundTrip(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.app, cls.page = _page()

    def setUp(self):
        _enable_pumps(self.page, "P1", "P2", "P3")

    def _build_backpack_in_ui(self, *, bore2_gauge=30):
        """22G bore 1 on P1 + a different-gauge bore 2 on P2, via the widgets."""
        page = self.page
        _select(page._needle_form_combo, NEEDLE_FORM_BACKPACK)
        _select(page._needle_type_combo, NEEDLE_TYPE_HYPODERMIC)
        _select(page.gauge_combo, 22)
        _select(page.length_combo, 1.0)
        _select(page._bore_rows[0]["pump"], "P1")
        page._bore_rows[0]["label"].setText("carrier")

        row = page._bore_rows[1]
        _select(row["type"], NEEDLE_TYPE_HYPODERMIC)
        _select(row["gauge"], bore2_gauge)
        _select(row["length"], 1.0)
        _select(row["pump"], "P2")
        row["label"].setText("trypsin")
        return page.get_config()

    def test_form_combo_creates_one_row_per_bore(self):
        page = self.page
        for form in (NEEDLE_FORM_SINGLE, NEEDLE_FORM_BACKPACK,
                     NEEDLE_FORM_TRIPLE):
            _select(page._needle_form_combo, form)
            with self.subTest(form):
                self.assertEqual(len(page._bore_rows),
                                 NEEDLE_FORM_BORE_COUNT[form])

    def test_backpack_carries_two_different_diameters(self):
        cfg = self._build_backpack_in_ui()
        n = cfg.needle
        self.assertEqual(n.needle_form, NEEDLE_FORM_BACKPACK)
        self.assertEqual(n.bore_count, 2)
        b0, b1 = n.bores_resolved()
        self.assertNotAlmostEqual(b0.id_um, b1.id_um, places=3)
        self.assertEqual(b0.gauge, 22)
        self.assertEqual(b1.gauge, 30)
        self.assertEqual(b0.pump_id, "P1")
        self.assertEqual(b1.pump_id, "P2")
        self.assertEqual(b1.label, "trypsin")
        # Bore 0 mirrors the flat fields, so every legacy reader still sees a
        # REAL bore's number rather than a fictional aggregate.
        self.assertAlmostEqual(n.id_um, b0.id_um, places=6)
        self.assertAlmostEqual(n.cross_section_area_mm2, b0.orifice_area_mm2,
                               places=9)

    def test_backpack_survives_json_and_returns_to_the_ui(self):
        cfg = self._build_backpack_in_ui()
        payload = json.loads(json.dumps(cfg.to_dict()))
        reloaded = HardwareConfig.from_dict(payload)

        self.assertIn("bores", payload["needle"])
        self.assertEqual(payload["needle"]["needle_form"], NEEDLE_FORM_BACKPACK)

        self.page.set_config(reloaded)
        # The FORM combo, both rows and both diameters must all come back.
        self.assertEqual(self.page._needle_form_combo.currentData(),
                         NEEDLE_FORM_BACKPACK)
        self.assertEqual(len(self.page._bore_rows), 2)
        self.assertEqual(self.page._bore_rows[1]["gauge"].currentData(), 30)
        self.assertEqual(self.page._bore_rows[1]["pump"].currentData(), "P2")
        self.assertEqual(self.page._bore_rows[1]["label"].text(), "trypsin")
        self.assertEqual(self.page._bore_rows[0]["pump"].currentData(), "P1")

        # …and a second rebuild must not drift.
        again = self.page.get_config().needle
        self.assertEqual([b.gauge for b in again.bores_resolved()], [22, 30])
        self.assertEqual([b.pump_id for b in again.bores_resolved()],
                         ["P1", "P2"])

    def test_each_row_shows_the_geometry_block_for_its_own_type(self):
        page = self.page
        _select(page._needle_form_combo, NEEDLE_FORM_TRIPLE)
        _select(page._bore_rows[1]["type"], NEEDLE_TYPE_CAPILLARY)
        _select(page._bore_rows[2]["type"], NEEDLE_TYPE_HYPODERMIC)
        self.assertTrue(page._bore_rows[1]["hypo"].isHidden())
        self.assertFalse(page._bore_rows[1]["cap"].isHidden())
        self.assertFalse(page._bore_rows[2]["hypo"].isHidden())
        self.assertTrue(page._bore_rows[2]["cap"].isHidden())

    def test_bore_one_has_no_duplicate_geometry_editor(self):
        """Bore 1's geometry + taper live in the top block; a second editor here
        would be two controls for one value."""
        page = self.page
        _select(page._needle_form_combo, NEEDLE_FORM_BACKPACK)
        row = page._bore_rows[0]
        self.assertTrue(row["hypo"].isHidden())
        self.assertTrue(row["cap"].isHidden())
        self.assertTrue(row["type"].isHidden())
        # …but its label and pump ARE editable here.
        self.assertFalse(row["label"].isHidden())
        self.assertFalse(row["pump"].isHidden())
        self.assertIn("configured above", row["geom"].text())

    def test_mixed_taper_backpack_hypodermic_plus_capillary(self):
        """The FORM is orthogonal to the TYPE — a mixed assembly is legitimate."""
        page = self.page
        _select(page._needle_form_combo, NEEDLE_FORM_BACKPACK)
        _select(page._needle_type_combo, NEEDLE_TYPE_HYPODERMIC)
        _select(page.gauge_combo, 22)
        _select(page._bore_rows[0]["pump"], "P1")

        row = page._bore_rows[1]
        _select(row["type"], NEEDLE_TYPE_CAPILLARY)
        row["cap_spins"]["barrel_id"].setValue(1000.0)
        row["cap_spins"]["barrel_od"].setValue(1500.0)
        row["cap_spins"]["barrel_len"].setValue(100.0)
        row["cap_spins"]["tip_id"].setValue(30.0)
        row["cap_spins"]["tip_len"].setValue(5.0)
        _select(row["profile"], TIP_PROFILE_CONE)
        _select(row["pump"], "P2")

        n = page.get_config().needle
        b0, b1 = n.bores_resolved()
        self.assertFalse(b0.is_capillary)
        self.assertTrue(b1.is_capillary)
        self.assertAlmostEqual(b1.tip_id_um, 30.0, places=6)
        self.assertEqual(b1.tip_profile, TIP_PROFILE_CONE)
        # The whole-assembly needle_type mirrors BORE 0, so a legacy reader is
        # not told the assembly is a capillary because one bore is.
        self.assertEqual(n.needle_type, NEEDLE_TYPE_HYPODERMIC)

        # Round-trip the mixed assembly.
        reloaded = HardwareConfig.from_dict(
            json.loads(json.dumps(page.get_config().to_dict())))
        rb0, rb1 = reloaded.needle.bores_resolved()
        self.assertEqual(rb0.needle_type, NEEDLE_TYPE_HYPODERMIC)
        self.assertEqual(rb1.needle_type, NEEDLE_TYPE_CAPILLARY)
        self.assertAlmostEqual(rb1.tip_id_um, 30.0, places=6)

    def test_capillary_bore_count_now_restores(self):
        """A saved multi-bore CAPILLARY used to come back as one bore.

        The bore count was only restored on the hypodermic branch of
        `_apply_config_to_ui`, so the capillary path silently collapsed to 1.
        """
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec(
            needle_form=NEEDLE_FORM_BACKPACK,
            bores=[
                NeedleBore(id_um=1000.0, od_um=1500.0, length_mm=100.0,
                           needle_type=NEEDLE_TYPE_CAPILLARY,
                           tip_id_um=30.0, tip_length_mm=5.0, pump_id="P1"),
                NeedleBore(id_um=1000.0, od_um=1500.0, length_mm=100.0,
                           needle_type=NEEDLE_TYPE_CAPILLARY,
                           tip_id_um=80.0, tip_length_mm=5.0, pump_id="P2"),
            ])
        self.page.set_config(cfg)
        self.assertEqual(self.page._form_bore_count(), 2)
        self.assertEqual(self.page.channels_spin.value(), 2)
        self.assertEqual(len(self.page._bore_rows), 2)
        self.assertEqual(self.page.get_config().needle.bore_count, 2)

    def test_form_and_bore_list_disagreeing_keeps_every_saved_bore(self):
        """A hand-edited file whose form says "single" but stores 3 bores must
        not lose two of them — the resolved bore list is the authority."""
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec(
            needle_form=NEEDLE_FORM_SINGLE,     # deliberately inconsistent
            bores=[NeedleBore(id_um=413.0, od_um=718.0, length_mm=25.4,
                              gauge=22, pump_id="P1"),
                   NeedleBore(id_um=159.0, od_um=311.0, length_mm=25.4,
                              gauge=30, pump_id="P2"),
                   NeedleBore(id_um=108.0, od_um=241.0, length_mm=25.4,
                              gauge=32, pump_id="P3")])
        self.page.set_config(cfg)
        self.assertEqual(len(self.page._bore_rows), 3)
        self.assertEqual(self.page._needle_form_combo.currentData(),
                         NEEDLE_FORM_TRIPLE)
        self.assertEqual(self.page.get_config().needle.bore_count, 3)


# ════════════════════════════════════════════════════════════════════
#  C. Duplicate pump assignment is reported
# ════════════════════════════════════════════════════════════════════

class TestDuplicatePumpReported(unittest.TestCase):
    """One pump can only push one volume — a shared pump drives the second bore
    blind, so it must be surfaced, and by the SAME rule the config validates
    with (a second independently-written rule is how two surfaces diverge)."""

    @classmethod
    def setUpClass(cls):
        cls.app, cls.page = _page()

    def setUp(self):
        _enable_pumps(self.page, "P1", "P2", "P3")
        _select(self.page._needle_form_combo, NEEDLE_FORM_BACKPACK)
        _select(self.page._needle_type_combo, NEEDLE_TYPE_HYPODERMIC)
        _select(self.page.gauge_combo, 22)

    def test_two_bores_on_one_pump_is_flagged_on_the_page(self):
        _select(self.page._bore_rows[0]["pump"], "P1")
        row = self.page._bore_rows[1]
        _select(row["gauge"], 30)
        _select(row["pump"], "P1")            # collision

        text = self.page._bore_status.text()
        self.assertIn("P1", text)
        self.assertIn("more than one bore", text)

    def test_the_page_and_hardware_config_agree(self):
        _select(self.page._bore_rows[0]["pump"], "P1")
        row = self.page._bore_rows[1]
        _select(row["gauge"], 30)
        _select(row["pump"], "P1")

        cfg = self.page.get_config()
        _ok, issues = cfg.validate()
        self.assertTrue(any("more than one bore" in i for i in issues), issues)

    def test_distinct_pumps_are_reported_clean(self):
        _select(self.page._bore_rows[0]["pump"], "P1")
        row = self.page._bore_rows[1]
        _select(row["gauge"], 30)
        _select(row["pump"], "P2")
        text = self.page._bore_status.text()
        self.assertTrue(text.startswith("✓"), text)
        self.assertNotIn("more than one bore", text)

    def test_a_bore_with_no_pump_is_flagged(self):
        _select(self.page._bore_rows[0]["pump"], "P1")
        row = self.page._bore_rows[1]
        _select(row["gauge"], 30)
        _select(row["pump"], None)
        self.assertIn("without a pump", self.page._bore_status.text())

    def test_per_bore_pump_reaches_the_serialized_map(self):
        """`needle_channel_pump_map` is DERIVED from the bores, so the per-bore
        picker must land in it — a stale map is what `PrintPlanOfAction` reads."""
        _select(self.page._bore_rows[0]["pump"], "P1")
        row = self.page._bore_rows[1]
        _select(row["gauge"], 30)
        _select(row["pump"], "P3")
        cfg = self.page.get_config()
        self.assertEqual(cfg.resolved_bore_pump_map(), {0: "P1", 1: "P3"})
        payload = cfg.to_dict()
        self.assertEqual(payload["needle_channel_pump_map"],
                         {"0": "P1", "1": "P3"})


# ════════════════════════════════════════════════════════════════════
#  D. Per-bore flow ceilings differ — the safety payoff
# ════════════════════════════════════════════════════════════════════

class TestPerBoreFlowCeilings(unittest.TestCase):
    """Applying a coarse bore's ceiling to a fine bore's pump over-pressures it
    and shatters a pulled glass tip, so each row must show its OWN number."""

    @classmethod
    def setUpClass(cls):
        cls.app, cls.page = _page()

    def setUp(self):
        _enable_pumps(self.page, "P1", "P2", "P3")

    def test_ceiling_is_computed_per_bore_not_per_assembly(self):
        page = self.page
        coarse = NeedleBore(id_um=413.0, od_um=718.0, length_mm=25.4, gauge=22)
        fine = NeedleBore(id_um=159.0, od_um=311.0, length_mm=25.4, gauge=30)
        q_coarse = page._bore_flow_ceiling_uL_s(coarse)
        q_fine = page._bore_flow_ceiling_uL_s(fine)
        self.assertGreater(q_coarse, 0.0)
        self.assertGreater(q_fine, 0.0)
        # K ∝ L/d⁴ → a 22G bore beats a 30G bore by ~(413/159)⁴ ≈ 45×.
        # Measured through this helper: 5.62e3 vs 124 µL/s = 45.3×.
        ratio = q_coarse / q_fine
        self.assertGreater(ratio, 30.0, ratio)
        self.assertLess(ratio, 70.0, ratio)

    def test_pulled_tip_ceiling_is_orders_of_magnitude_below_a_barrel(self):
        page = self.page
        barrel = NeedleBore(id_um=413.0, od_um=718.0, length_mm=25.4, gauge=22)
        pulled = NeedleBore(id_um=1000.0, od_um=1500.0, length_mm=100.0,
                            needle_type=NEEDLE_TYPE_CAPILLARY,
                            tip_id_um=30.0, tip_length_mm=5.0)
        q_barrel = page._bore_flow_ceiling_uL_s(barrel)
        q_pulled = page._bore_flow_ceiling_uL_s(pulled)
        # Measured: 5.62e3 vs 0.795 µL/s ≈ 7000×. Driving the pulled tip's pump
        # at the barrel's ceiling is what shatters glass, so the two numbers must
        # never be collapsed into one assembly-level figure.
        self.assertGreater(q_barrel / max(q_pulled, 1e-12), 1000.0)
        self.assertLess(q_pulled, 1.0)

    def test_each_row_shows_its_own_ceiling(self):
        page = self.page
        _select(page._needle_form_combo, NEEDLE_FORM_BACKPACK)
        _select(page._needle_type_combo, NEEDLE_TYPE_HYPODERMIC)
        _select(page.gauge_combo, 22)
        _select(page._bore_rows[0]["pump"], "P1")
        row = page._bore_rows[1]
        _select(row["gauge"], 30)
        _select(row["pump"], "P2")

        t0 = page._bore_rows[0]["flow"].text()
        t1 = page._bore_rows[1]["flow"].text()
        self.assertIn("Max safe flow", t0)
        self.assertIn("Max safe flow", t1)
        self.assertNotEqual(t0, t1)

    def test_incomplete_geometry_says_so_rather_than_showing_zero(self):
        page = self.page
        _select(page._needle_form_combo, NEEDLE_FORM_BACKPACK)
        _select(page._needle_type_combo, NEEDLE_TYPE_HYPODERMIC)
        _select(page.gauge_combo, 22)
        _select(page._bore_rows[1]["gauge"], None)   # nothing entered yet
        self.assertIn("geometry incomplete",
                      page._bore_rows[1]["flow"].text())

    def test_ceiling_helper_never_raises_on_junk(self):
        # It runs inside a config rebuild; a throw there would break the page.
        class _Junk:
            pass
        self.assertEqual(self.page._bore_flow_ceiling_uL_s(_Junk()), 0.0)


# ════════════════════════════════════════════════════════════════════
#  E. Mount offsets are read-only and honestly labelled
# ════════════════════════════════════════════════════════════════════

class TestMountOffsetReadout(unittest.TestCase):
    """Offsets are a per-MOUNT calibration measured on Calibration → Needle
    Location, never typed here and never stored in a preset library (the
    CAMERA_CAL_PERSIST_STORE lesson)."""

    @classmethod
    def setUpClass(cls):
        cls.app, cls.page = _page()

    def setUp(self):
        # Start from a blank config: offsets deliberately SURVIVE a geometry edit
        # (see `test_measured_offsets_survive_a_geometry_edit`), so a sibling test
        # that loaded measured offsets would otherwise leak them into this one.
        self.page.set_config(HardwareConfig())
        _enable_pumps(self.page, "P1", "P2", "P3")

    def test_unmeasured_offset_reads_not_measured(self):
        page = self.page
        _select(page._needle_form_combo, NEEDLE_FORM_BACKPACK)
        _select(page._needle_type_combo, NEEDLE_TYPE_HYPODERMIC)
        _select(page.gauge_combo, 22)
        _select(page._bore_rows[0]["pump"], "P1")
        _select(page._bore_rows[1]["gauge"], 30)
        _select(page._bore_rows[1]["pump"], "P2")
        self.assertIn("not measured", page._bore_rows[1]["flow"].text())
        # Bore 1 IS the datum, so it is never "unmeasured".
        self.assertIn("datum", page._bore_rows[0]["flow"].text())

    def test_a_measured_offset_is_shown(self):
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec(
            needle_form=NEEDLE_FORM_BACKPACK,
            bores=[NeedleBore(id_um=413.0, od_um=718.0, length_mm=25.4,
                              gauge=22, pump_id="P1"),
                   NeedleBore(id_um=159.0, od_um=311.0, length_mm=25.4,
                              gauge=30, pump_id="P2",
                              offset_um=(320.0, -140.0), z_offset_mm=0.02)])
        self.page.set_config(cfg)
        text = self.page._bore_rows[1]["flow"].text()
        self.assertIn("+320", text)
        self.assertIn("-140", text)
        self.assertIn("Z +0.020 mm", text)
        self.assertNotIn("not measured", text)

    def test_the_group_says_where_offsets_are_measured(self):
        self.assertIn("Needle Location", self.page._bore_offset_note.text())

    def test_no_widget_lets_the_operator_type_an_offset(self):
        _select(self.page._needle_form_combo, NEEDLE_FORM_TRIPLE)
        for row in self.page._bore_rows:
            self.assertNotIn("offset", row)
            self.assertNotIn("z_offset", row)

    def test_measured_offsets_survive_a_geometry_edit(self):
        """A geometry edit rebuilds the NeedleSpec; losing the calibrated offsets
        there would silently un-calibrate the assembly."""
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec(
            needle_form=NEEDLE_FORM_BACKPACK,
            bores=[NeedleBore(id_um=413.0, od_um=718.0, length_mm=25.4,
                              gauge=22, pump_id="P1"),
                   NeedleBore(id_um=159.0, od_um=311.0, length_mm=25.4,
                              gauge=30, pump_id="P2",
                              offset_um=(320.0, -140.0), z_offset_mm=0.02)])
        self.page.set_config(cfg)
        _enable_pumps(self.page, "P1", "P2", "P3")
        # Change bore 2's gauge — the offsets belong to the MOUNT, not the gauge.
        _select(self.page._bore_rows[1]["gauge"], 32)
        got = self.page.get_config().needle
        self.assertEqual(got.bores_resolved()[1].gauge, 32)
        self.assertEqual(tuple(got.bores_resolved()[1].offset_um),
                         (320.0, -140.0))
        self.assertAlmostEqual(got.bores_resolved()[1].z_offset_mm, 0.02,
                               places=9)
        # Bore 0 is the datum BY DEFINITION and must stay pinned at the origin.
        self.assertEqual(tuple(got.bores_resolved()[0].offset_um), (0.0, 0.0))

    def test_measured_offsets_survive_a_form_round_trip(self):
        """A mis-clicked form combo must not force a re-calibration — the offsets
        are the most expensive thing on this page to lose."""
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec(
            needle_form=NEEDLE_FORM_BACKPACK,
            bores=[NeedleBore(id_um=413.0, od_um=718.0, length_mm=25.4,
                              gauge=22, pump_id="P1"),
                   NeedleBore(id_um=159.0, od_um=311.0, length_mm=25.4,
                              gauge=30, pump_id="P2",
                              offset_um=(320.0, -140.0), z_offset_mm=0.02)])
        self.page.set_config(cfg)
        _enable_pumps(self.page, "P1", "P2", "P3")
        _select(self.page._needle_form_combo, NEEDLE_FORM_SINGLE)
        _select(self.page._needle_form_combo, NEEDLE_FORM_BACKPACK)
        got = self.page.get_config().needle
        self.assertEqual(tuple(got.bores_resolved()[1].offset_um),
                         (320.0, -140.0))
        self.assertIn("+320", self.page._bore_rows[1]["flow"].text())

    def test_a_calibration_written_in_place_is_picked_up(self):
        """The calibration writes offsets straight onto `config.needle.bores`; a
        later geometry edit rebuilds the spec and must not drop them."""
        cfg = HardwareConfig()
        cfg.needle = NeedleSpec(
            needle_form=NEEDLE_FORM_BACKPACK,
            bores=[NeedleBore(id_um=413.0, od_um=718.0, length_mm=25.4,
                              gauge=22, pump_id="P1"),
                   NeedleBore(id_um=159.0, od_um=311.0, length_mm=25.4,
                              gauge=30, pump_id="P2")])
        self.page.set_config(cfg)
        _enable_pumps(self.page, "P1", "P2", "P3")
        live = self.page.get_config().needle
        live.bores[1].offset_um = (-210.0, 88.0)
        live.bores[1].z_offset_mm = -0.031
        # Now edit geometry — the rebuild must carry the fresh calibration.
        _select(self.page._bore_rows[1]["gauge"], 32)
        got = self.page.get_config().needle
        self.assertEqual(tuple(got.bores_resolved()[1].offset_um), (-210.0, 88.0))
        self.assertAlmostEqual(got.bores_resolved()[1].z_offset_mm, -0.031,
                               places=9)


# ════════════════════════════════════════════════════════════════════
#  F. A form switch does not silently destroy bore geometry
# ════════════════════════════════════════════════════════════════════

class TestFormSwitchPreservesGeometry(unittest.TestCase):
    """The bore-count spin used to wipe the pump map and immediately persist the
    emptied dict. Shrinking the form must be recoverable, and any real loss must
    be stated rather than silent."""

    @classmethod
    def setUpClass(cls):
        cls.app, cls.page = _page()

    def setUp(self):
        _enable_pumps(self.page, "P1", "P2", "P3")
        self.page._bore_cache.clear()

    def _fill_triple(self):
        page = self.page
        _select(page._needle_form_combo, NEEDLE_FORM_TRIPLE)
        _select(page._needle_type_combo, NEEDLE_TYPE_HYPODERMIC)
        _select(page.gauge_combo, 22)
        _select(page._bore_rows[0]["pump"], "P1")
        _select(page._bore_rows[1]["gauge"], 30)
        _select(page._bore_rows[1]["pump"], "P2")
        page._bore_rows[1]["label"].setText("trypsin")
        _select(page._bore_rows[2]["gauge"], 32)
        _select(page._bore_rows[2]["pump"], "P3")
        page._bore_rows[2]["label"].setText("dye")

    def test_triple_to_single_to_triple_restores_both_bores(self):
        page = self.page
        self._fill_triple()
        _select(page._needle_form_combo, NEEDLE_FORM_SINGLE)
        self.assertEqual(len(page._bore_rows), 1)
        _select(page._needle_form_combo, NEEDLE_FORM_TRIPLE)
        self.assertEqual(len(page._bore_rows), 3)
        self.assertEqual(page._bore_rows[1]["gauge"].currentData(), 30)
        self.assertEqual(page._bore_rows[1]["pump"].currentData(), "P2")
        self.assertEqual(page._bore_rows[1]["label"].text(), "trypsin")
        self.assertEqual(page._bore_rows[2]["gauge"].currentData(), 32)
        self.assertEqual(page._bore_rows[2]["label"].text(), "dye")

    def test_backpack_bore_survives_a_detour_through_triple(self):
        page = self.page
        _select(page._needle_form_combo, NEEDLE_FORM_BACKPACK)
        _select(page._needle_type_combo, NEEDLE_TYPE_HYPODERMIC)
        _select(page.gauge_combo, 22)
        _select(page._bore_rows[1]["gauge"], 30)
        _select(page._needle_form_combo, NEEDLE_FORM_TRIPLE)
        self.assertEqual(page._bore_rows[1]["gauge"].currentData(), 30)
        _select(page._needle_form_combo, NEEDLE_FORM_BACKPACK)
        self.assertEqual(page._bore_rows[1]["gauge"].currentData(), 30)

    def test_shrinking_states_the_loss_explicitly(self):
        page = self.page
        self._fill_triple()
        _select(page._needle_form_combo, NEEDLE_FORM_BACKPACK)
        text = page._bore_status.text()
        # Bore 3 is hidden by the Backpack form — say so, and say what saving
        # now would cost.
        self.assertIn("3", text)
        self.assertIn("hidden", text)
        self.assertIn("NOT saved", text)

    def test_the_loss_note_is_not_crowded_out_by_a_validation_warning(self):
        """A data loss the operator is about to save must not be hidden behind an
        unrelated warning."""
        page = self.page
        self._fill_triple()
        _select(page._needle_form_combo, NEEDLE_FORM_BACKPACK)
        # Now also make the assembly invalid, so both messages compete.
        _select(page._bore_rows[1]["pump"], None)
        text = page._bore_status.text()
        self.assertIn("without a pump", text)
        self.assertIn("NOT saved", text)

    def test_an_untouched_row_is_not_reported_as_a_loss(self):
        page = self.page
        _select(page._needle_form_combo, NEEDLE_FORM_TRIPLE)
        _select(page._needle_type_combo, NEEDLE_TYPE_HYPODERMIC)
        _select(page.gauge_combo, 22)
        _select(page._bore_rows[0]["pump"], "P1")
        # bores 2 and 3 left completely untouched
        _select(page._needle_form_combo, NEEDLE_FORM_SINGLE)
        self.assertEqual(page._bore_cache, {})

    def test_loading_a_config_does_not_graft_a_cached_bore(self):
        """A cache entry from the OUTGOING setup restored on top of an incoming
        one would silently splice one machine's bore onto another's assembly."""
        page = self.page
        self._fill_triple()
        _select(page._needle_form_combo, NEEDLE_FORM_SINGLE)
        self.assertTrue(page._bore_cache)            # parked

        cfg = HardwareConfig()
        cfg.needle = NeedleSpec(
            needle_form=NEEDLE_FORM_BACKPACK,
            bores=[NeedleBore(id_um=413.0, od_um=718.0, length_mm=25.4,
                              gauge=22, pump_id="P1"),
                   NeedleBore(id_um=108.0, od_um=241.0, length_mm=25.4,
                              gauge=32, pump_id="P3")])
        page.set_config(cfg)
        self.assertEqual(page._bore_cache, {})
        self.assertEqual(page._bore_rows[1]["gauge"].currentData(), 32)
        self.assertEqual(page._bore_rows[1]["pump"].currentData(), "P3")
        self.assertEqual(page._bore_rows[1]["label"].text(), "")

    def test_shrinking_to_single_emits_no_bores_key(self):
        page = self.page
        self._fill_triple()
        _select(page._needle_form_combo, NEEDLE_FORM_SINGLE)
        payload = page.get_config().needle.to_dict()
        self.assertEqual(set(payload), LEGACY_KEYS)

    def test_shrinking_to_single_keeps_bore_ones_pump(self):
        """Bore 1's pump was picked in the per-bore row; on the way back to a
        single needle the legacy bore→pump card becomes the surface again and
        must still hold it (the count spin used to wipe the map)."""
        page = self.page
        self._fill_triple()
        _select(page._needle_form_combo, NEEDLE_FORM_SINGLE)
        self.assertFalse(page.channel_map_group.isHidden())
        self.assertEqual(page._channel_map_widgets[0][1].currentData(), "P1")
        self.assertEqual(page._bore_rows[0]["pump"].currentData(), "P1")
        self.assertEqual(page.get_config().needle_channel_pump_map, {0: "P1"})

    def test_the_two_pump_surfaces_are_never_both_visible(self):
        """Two pickers for one decision is how they end up disagreeing."""
        page = self.page
        for form, multi in ((NEEDLE_FORM_SINGLE, False),
                            (NEEDLE_FORM_BACKPACK, True),
                            (NEEDLE_FORM_TRIPLE, True)):
            _select(page._needle_form_combo, form)
            with self.subTest(form):
                self.assertEqual(page._bore_group.isHidden(), not multi)
                self.assertEqual(page.channel_map_group.isHidden(), multi)

    def test_single_bore_map_row_still_drives_the_bore(self):
        """With one bore the legacy card is the editor, so its selection must
        reach `NeedleBore.pump_id` — otherwise the two surfaces disagree the
        moment the operator switches to a backpack."""
        page = self.page
        _select(page._needle_form_combo, NEEDLE_FORM_SINGLE)
        _select(page._needle_type_combo, NEEDLE_TYPE_HYPODERMIC)
        _select(page.gauge_combo, 22)
        _select(page._channel_map_widgets[0][1], "P2")
        self.assertEqual(page._bore_rows[0]["pump"].currentData(), "P2")
        _select(page._needle_form_combo, NEEDLE_FORM_BACKPACK)
        self.assertEqual(page._bore_rows[0]["pump"].currentData(), "P2")
        self.assertEqual(
            page.get_config().needle.bores_resolved()[0].pump_id, "P2")


# ════════════════════════════════════════════════════════════════════
#  G. Shared capillary spin definitions (anti-divergence)
# ════════════════════════════════════════════════════════════════════

class TestSharedCapillarySpecs(unittest.TestCase):
    """A backpack's second bore must not get different ranges, defaults or
    tooltips than the first — so both come from ONE table."""

    @classmethod
    def setUpClass(cls):
        cls.app, cls.page = _page()
        _select(cls.page._needle_form_combo, NEEDLE_FORM_BACKPACK)

    def test_row_spins_match_the_single_needle_card(self):
        from gui.pages.hardware_setup import _CAP_SPIN_SPECS
        card = {
            "barrel_id": self.page._cap_barrel_id_spin,
            "barrel_od": self.page._cap_barrel_od_spin,
            "barrel_len": self.page._cap_barrel_len_spin,
            "tip_id": self.page._cap_tip_id_spin,
            "tip_od": self.page._cap_tip_od_spin,
            "tip_len": self.page._cap_tip_len_spin,
        }
        row_spins = self.page._bore_rows[1]["cap_spins"]
        self.assertEqual(set(row_spins), set(_CAP_SPIN_SPECS))
        for key, spin in card.items():
            with self.subTest(key):
                other = row_spins[key]
                self.assertAlmostEqual(spin.minimum(), other.minimum(), places=9)
                self.assertAlmostEqual(spin.maximum(), other.maximum(), places=9)
                self.assertEqual(spin.decimals(), other.decimals())
                self.assertEqual(spin.suffix(), other.suffix())
                self.assertEqual(spin.toolTip(), other.toolTip())

    def test_tip_profile_options_match(self):
        card = self.page._cap_tip_profile_combo
        row = self.page._bore_rows[1]["profile"]
        self.assertEqual(
            [card.itemData(i) for i in range(card.count())],
            [row.itemData(i) for i in range(row.count())])
        self.assertEqual(card.toolTip(), row.toolTip())


# ════════════════════════════════════════════════════════════════════
#  H. COLD LOAD — the pumps arrive from the config, not from the UI
# ════════════════════════════════════════════════════════════════════

def _backpack_config(*, bore2_pump="P2", enabled=("P1", "P2")):
    """A saved backpack: 22G on P1 + 30G on P2, pumps enabled BY THE CONFIG."""
    from SupportClasses.PhysicalModels import SyringeSpec, InkSpec
    from SupportClasses.HardwareConfig import PumpChannelConfig
    cfg = HardwareConfig()
    cfg.ink_library["GelMA"] = InkSpec(name="GelMA")
    for pid in enabled:
        cfg.pumps[pid] = PumpChannelConfig(
            pump_id=pid, enabled=True,
            syringe=SyringeSpec(volume_uL=250, stroke_length_mm=30.0,
                                barrel_id_mm=3.26),
            inks=[cfg.ink_library["GelMA"]])
    cfg.needle = NeedleSpec(needle_form=NEEDLE_FORM_BACKPACK, bores=[
        NeedleBore(id_um=413.0, od_um=718.0, wall_um=152.0, length_mm=25.4,
                   gauge=22, pump_id="P1", label="carrier"),
        NeedleBore(id_um=305.0, od_um=508.0, wall_um=101.0, length_mm=25.4,
                   gauge=30, pump_id=bore2_pump, label="trypsin")])
    return cfg


class TestColdLoadKeepsBorePumps(unittest.TestCase):
    """A saved multi-bore setup must survive being LOADED, not just authored.

    Every other class here enables the pumps through the widgets before calling
    ``set_config``, which is the opposite order from a real load — and it hid a
    defect: ``_apply_config_to_ui`` builds the bore rows in its needle section,
    which runs BEFORE the pump widgets are restored, so the enabled-pump list was
    empty, the per-bore pump combos held only "— Unassigned —" and every saved
    bore→pump binding was dropped by ``findData``. The damage was not cosmetic:
    ``NeedleBore.pump_id`` is the authority the per-PUMP flow ceiling resolves
    through, so both pumps fell back to one bore's ceiling, and re-saving
    persisted the loss. Each test here builds a FRESH page and lets the pumps
    arrive from the config.
    """

    def setUp(self):
        self.app, self.page = _page()

    def test_bore_pumps_survive_a_cold_load(self):
        self.page.set_config(_backpack_config())
        bores = self.page.get_config().needle.bores
        self.assertEqual([b.pump_id for b in bores], ["P1", "P2"])

    def test_the_derived_map_and_validate_survive_a_cold_load(self):
        self.page.set_config(_backpack_config())
        out = self.page.get_config()
        self.assertEqual(out.resolved_bore_pump_map(), {0: "P1", 1: "P2"})
        # The legacy map is mirrored from the bores, so it must not be emptied.
        self.assertEqual(out.needle_channel_pump_map, {0: "P1", 1: "P2"})
        self.assertNotIn("Needle has 2 bore(s) but 0 mapped", out.validate()[1])

    def test_reloading_a_saved_setup_is_idempotent(self):
        self.page.set_config(_backpack_config())
        first = self.page.get_config().to_dict()
        page2 = _page()[1]
        page2.set_config(HardwareConfig.from_dict(json.loads(json.dumps(first))))
        self.assertEqual(page2.get_config().to_dict()["needle"], first["needle"])

    def test_each_pump_keeps_its_own_bores_flow_ceiling(self):
        """The safety-critical consequence: without the binding both pumps
        resolve through a fallback bore, so the coarse bore's pump is capped at
        the fine bore's ceiling (or, on another ordering, the reverse)."""
        from SupportClasses.SafetyLimits import SafetyLimits
        self.page.set_config(_backpack_config())
        limits = SafetyLimits()
        limits.update_from_hardware_config(self.page.get_config())
        coarse = limits.clamp_flow_rate(1e9, "P1")      # 22G
        fine = limits.clamp_flow_rate(1e9, "P2")        # 30G
        self.assertGreater(coarse, fine * 10,
                           f"22G ceiling {coarse} not distinct from 30G {fine}")

    def test_a_bore_on_a_disabled_pump_keeps_its_binding_and_is_reported(self):
        """Silently forgetting the binding hides which bore that pump feeds;
        keeping it makes `validate()` name the operator's next action."""
        self.page.set_config(_backpack_config(bore2_pump="P3",
                                              enabled=("P1",)))
        out = self.page.get_config()
        self.assertEqual([b.pump_id for b in out.needle.bores], ["P1", "P3"])
        self.assertIn("Bore 2 → P3 but P3 is not enabled", out.validate()[1])

    def test_repeated_pump_refreshes_do_not_accumulate_combo_entries(self):
        self.page.set_config(_backpack_config(bore2_pump="P3",
                                              enabled=("P1",)))
        combo = self.page._bore_rows[1]["pump"]
        before = combo.count()
        for _ in range(5):
            self.page._refresh_bore_pump_options()
        self.assertEqual(combo.count(), before)
        self.assertEqual(combo.currentData(), "P3")


if __name__ == "__main__":
    unittest.main(verbosity=2)
