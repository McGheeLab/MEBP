"""
test_session_a.py — Unit tests for Session A: Foundation layer.

Tests:
- PhysicalModels: NeedleSpec, SyringeSpec, InkSpec, FluidColumn,
  PumpLoadout, RosetteInsert, WorkspaceConfig, JSON loaders
- FlowPhysics: Hagen-Poiseuille, Reynolds, granular regime,
  shear stress, safety calculator
- ControllerProtocol: JSON loading, command formatting, detection
"""

import json
import math
import os
import tempfile
import unittest
from pathlib import Path

# Adjust path for imports
import sys
sys.path.insert(0, str(Path(__file__).parent))

from SupportClasses.PhysicalModels import (
    NeedleSpec, SyringeSpec, InkSpec, FluidColumn, PumpLoadout,
    PrintingMode, RosetteSubWell, RosetteInsert, WorkspaceConfig,
    WellRole, ROLE_COLORS,
    load_needle_catalog, load_syringe_catalog,
)
from SupportClasses.FlowPhysics import (
    hagen_poiseuille_pressure, reynolds_number, classify_flow_regime,
    wall_shear_stress, classify_granular_regime,
    max_safe_flow_rate_uL_s, extrusion_flow_rate, extrusion_pump_speed,
    flow_rate_to_pump_speed, pump_speed_to_flow_rate,
    calculate_flow_safety, calculate_print_safety,
    generate_compatibility_report,
    FlowRegime, GranularRegime,
)
from SupportClasses.ControllerProtocol import (
    ControllerProtocol, discover_controller_files, load_all_protocols,
)


# ---------------------------------------------------------------------------
# Test PhysicalModels
# ---------------------------------------------------------------------------

class TestNeedleSpec(unittest.TestCase):
    """Test NeedleSpec dataclass."""

    def setUp(self):
        self.needle_22g = NeedleSpec(
            gauge=22, od_um=718, id_um=413, wall_um=152
        )

    def test_basic_properties(self):
        n = self.needle_22g
        self.assertEqual(n.gauge, 22)
        self.assertAlmostEqual(n.od_mm, 0.718, places=3)
        self.assertAlmostEqual(n.id_mm, 0.413, places=3)
        self.assertAlmostEqual(n.wall_mm, 0.152, places=3)

    def test_length_conversion(self):
        n = self.needle_22g
        self.assertAlmostEqual(n.length_mm, 25.4, places=1)
        n.length_inches = 2.0
        self.assertAlmostEqual(n.length_mm, 50.8, places=1)

    def test_id_meters(self):
        """SI conversion for flow physics."""
        n = self.needle_22g
        self.assertAlmostEqual(n.id_m, 413e-6, places=9)

    def test_cross_section_area(self):
        n = self.needle_22g
        expected = math.pi * (0.413 / 2) ** 2
        self.assertAlmostEqual(n.cross_section_area_mm2, expected, places=6)

    def test_default_channel_pump_map(self):
        n = self.needle_22g
        self.assertEqual(n.channel_pump_map, {1: "P1"})

    def test_serialization(self):
        d = self.needle_22g.to_dict()
        n2 = NeedleSpec.from_dict(d)
        self.assertEqual(n2.gauge, 22)
        self.assertAlmostEqual(n2.od_um, 718)


class TestSyringeSpec(unittest.TestCase):
    """Test SyringeSpec dataclass."""

    def setUp(self):
        self.syringe_100 = SyringeSpec(
            volume_uL=100, stroke_length_mm=30.0,
            barrel_id_mm=2.060, part_number="1710",
        )

    def test_uL_per_mm(self):
        s = self.syringe_100
        self.assertAlmostEqual(s.uL_per_mm, 100 / 30, places=3)

    def test_mm_per_uL(self):
        s = self.syringe_100
        self.assertAlmostEqual(s.mm_per_uL, 30 / 100, places=3)

    def test_round_trip_conversion(self):
        s = self.syringe_100
        self.assertAlmostEqual(s.uL_to_mm(50.0), 15.0, places=3)
        self.assertAlmostEqual(s.mm_to_uL(15.0), 50.0, places=3)

    def test_cross_section_area(self):
        s = self.syringe_100
        expected = math.pi * (2.060 / 2) ** 2
        self.assertAlmostEqual(s.cross_section_area_mm2, expected, places=4)

    def test_serialization(self):
        d = self.syringe_100.to_dict()
        s2 = SyringeSpec.from_dict(d)
        self.assertEqual(s2.volume_uL, 100)
        self.assertEqual(s2.part_number, "1710")


class TestInkSpec(unittest.TestCase):
    """Test InkSpec with needle compatibility."""

    def setUp(self):
        self.needle_22g = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
        self.needle_30g = NeedleSpec(gauge=30, od_um=312, id_um=159, wall_um=76)

    def test_pure_liquid_compatible(self):
        ink = InkSpec(name="DPBS", ink_type="buffer", viscosity_cP=1.0)
        self.assertEqual(ink.can_flow_through(self.needle_22g), "compatible")

    def test_free_flow(self):
        """Cells at 15µm through 22G (413µm ID) → ratio ~27.5×"""
        ink = InkSpec(name="HEK293", ink_type="cells", cell_diameter_um=15)
        self.assertEqual(ink.can_flow_through(self.needle_22g), "free_flow")

    def test_risk_clogging(self):
        """Large organoids through 30G (159µm ID) → ratio < 4"""
        ink = InkSpec(name="Organoid", ink_type="cells", cell_diameter_um=100)
        status = ink.can_flow_through(self.needle_30g)
        self.assertEqual(status, "risk_clogging")

    def test_pick_and_place(self):
        """500µm organoid through 30G (159µm) → ratio < 1"""
        ink = InkSpec(name="Big Organoid", ink_type="cells", cell_diameter_um=500)
        self.assertEqual(ink.can_flow_through(self.needle_30g), "pick_and_place")

    def test_compatibility_detail(self):
        ink = InkSpec(name="MSC", ink_type="cells", cell_diameter_um=20)
        detail = ink.flow_compatibility_detail(self.needle_22g)
        self.assertEqual(detail["severity"], "ok")
        self.assertIn("flows freely", detail["message"])

    def test_viscosity_conversion(self):
        ink = InkSpec(name="Gel", viscosity_cP=50)
        self.assertAlmostEqual(ink.viscosity_Pa_s, 0.05, places=4)

    def test_serialization(self):
        ink = InkSpec(name="Test", ink_type="hydrogel", viscosity_cP=10, color="#ff0000")
        d = ink.to_dict()
        ink2 = InkSpec.from_dict(d)
        self.assertEqual(ink2.name, "Test")
        self.assertEqual(ink2.color, "#ff0000")


class TestFluidColumn(unittest.TestCase):
    """Test FluidColumn state tracking."""

    def setUp(self):
        self.ink = InkSpec(name="TestInk", ink_type="hydrogel")
        self.fc = FluidColumn(
            oil_volume_uL=50.0,
            buffer_volume_uL=5.0,
            dead_volume_uL=2.0,
        )

    def test_initial_state(self):
        self.assertTrue(self.fc.is_empty)
        self.assertTrue(self.fc.has_buffer)
        self.assertAlmostEqual(self.fc.total_volume_uL, 55.0)

    def test_aspirate_and_dispense(self):
        self.fc.aspirate_ink(10.0, self.ink)
        self.assertFalse(self.fc.is_empty)
        self.assertAlmostEqual(self.fc.ink_volume_uL, 10.0)
        self.assertEqual(self.fc.ink_spec.name, "TestInk")

        # Can dispense
        self.assertTrue(self.fc.can_dispense(8.0))
        actual = self.fc.dispense(8.0)
        self.assertAlmostEqual(actual, 8.0)
        self.assertAlmostEqual(self.fc.ink_volume_uL, 2.0)

    def test_dispense_more_than_available(self):
        self.fc.aspirate_ink(3.0, self.ink)
        actual = self.fc.dispense(5.0)
        self.assertAlmostEqual(actual, 3.0)
        self.assertAlmostEqual(self.fc.ink_volume_uL, 0.0)

    def test_waste_ink(self):
        self.fc.aspirate_ink(10.0, self.ink)
        ejected = self.fc.waste_ink()
        self.assertAlmostEqual(ejected, 10.0)
        self.assertTrue(self.fc.is_empty)
        self.assertIsNone(self.fc.ink_spec)

    def test_refresh_buffer(self):
        self.fc.aspirate_ink(10.0, self.ink)
        self.fc.refresh_buffer(8.0)
        self.assertTrue(self.fc.is_empty)
        self.assertAlmostEqual(self.fc.buffer_volume_uL, 8.0)

    def test_volume_fractions(self):
        self.fc.aspirate_ink(10.0, self.ink)
        syringe = SyringeSpec(volume_uL=100, stroke_length_mm=30.0)
        fracs = self.fc.volume_fractions(syringe)
        self.assertAlmostEqual(fracs["oil"], 0.50, places=2)
        self.assertAlmostEqual(fracs["buffer"], 0.05, places=2)
        self.assertAlmostEqual(fracs["ink"], 0.10, places=2)
        self.assertAlmostEqual(fracs["empty"], 0.35, places=2)

    def test_serialization(self):
        self.fc.aspirate_ink(10.0, self.ink)
        d = self.fc.to_dict()
        fc2 = FluidColumn.from_dict(d)
        self.assertAlmostEqual(fc2.ink_volume_uL, 10.0)
        self.assertEqual(fc2.ink_spec.name, "TestInk")


class TestPumpLoadout(unittest.TestCase):
    """Test PumpLoadout."""

    def test_position_conversion(self):
        syringe = SyringeSpec(volume_uL=100, stroke_length_mm=30.0,
                              barrel_id_mm=2.060)
        pump = PumpLoadout(pump_id="P1", syringe=syringe,
                           current_position_mm=15.0)
        self.assertAlmostEqual(pump.current_position_uL, 50.0, places=1)

    def test_needs_refill(self):
        pump = PumpLoadout(pump_id="P1")
        pump.fluid_column.aspirate_ink(3.0, InkSpec(name="X"))
        self.assertTrue(pump.needs_refill(threshold_uL=5.0))
        pump.fluid_column.aspirate_ink(10.0, InkSpec(name="X"))
        self.assertFalse(pump.needs_refill(threshold_uL=5.0))

    def test_serialization(self):
        syringe = SyringeSpec(volume_uL=250, stroke_length_mm=30.0,
                              barrel_id_mm=3.256)
        pump = PumpLoadout(
            pump_id="P2", syringe=syringe,
            printing_mode=PrintingMode.CONTINUOUS,
            current_position_mm=10.0,
        )
        d = pump.to_dict()
        pump2 = PumpLoadout.from_dict(d)
        self.assertEqual(pump2.pump_id, "P2")
        self.assertEqual(pump2.printing_mode, PrintingMode.CONTINUOUS)
        self.assertEqual(pump2.syringe.volume_uL, 250)


class TestRosetteInsert(unittest.TestCase):
    """Test RosetteInsert geometry."""

    def test_create_standard(self):
        rosette = RosetteInsert.create_standard(
            name="Ink-6", well_format=24, num_ring=6, has_center=True,
        )
        self.assertEqual(rosette.num_subwells, 7)  # 6 ring + 1 center
        self.assertEqual(len(rosette.subwells), 7)

    def test_center_well_at_origin(self):
        rosette = RosetteInsert.create_standard(
            name="Test", well_format=24, num_ring=4, has_center=True,
        )
        x, y = rosette.get_subwell_xy(0)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 0.0)

    def test_ring_positions(self):
        rosette = RosetteInsert.create_standard(
            name="Test", well_format=24, num_ring=4, has_center=False,
            ring_radius_mm=3.0,
        )
        # 4 ring wells at 0°, 90°, 180°, 270°
        positions = rosette.get_all_subwell_positions()
        self.assertEqual(len(positions), 4)

        # First well at 0° → (0, 3) approximately
        x0, y0, _ = positions[0]
        self.assertAlmostEqual(x0, 0.0, places=3)
        self.assertAlmostEqual(y0, 3.0, places=3)

        # Second well at 90° → (3, 0) approximately
        x1, y1, _ = positions[1]
        self.assertAlmostEqual(x1, 3.0, places=3)
        self.assertAlmostEqual(y1, 0.0, places=1)

    def test_serialization(self):
        rosette = RosetteInsert.create_standard(
            name="Sort-4", well_format=24, num_ring=4,
        )
        d = rosette.to_dict()
        r2 = RosetteInsert.from_dict(d)
        self.assertEqual(r2.name, "Sort-4")
        self.assertEqual(len(r2.subwells), 5)


class TestJSONLoaders(unittest.TestCase):
    """Test JSON catalog loading."""

    def test_load_needles(self):
        catalog = load_needle_catalog("config/hardware/needles.json")
        self.assertGreater(len(catalog), 10)
        self.assertIn(22, catalog)
        n22 = catalog[22]
        self.assertEqual(n22.gauge, 22)
        self.assertAlmostEqual(n22.od_um, 718)
        self.assertAlmostEqual(n22.id_um, 413)

    def test_load_syringes(self):
        catalog = load_syringe_catalog("config/hardware/syringes.json")
        self.assertGreater(len(catalog), 4)
        self.assertIn(100, catalog)
        s100 = catalog[100]
        self.assertEqual(s100.volume_uL, 100)
        self.assertAlmostEqual(s100.barrel_id_mm, 2.060, places=3)

    def test_load_missing_file(self):
        catalog = load_needle_catalog("nonexistent.json")
        self.assertEqual(len(catalog), 0)

    def test_all_syringe_conversions(self):
        """Verify µL↔mm conversions are consistent for all syringe sizes."""
        catalog = load_syringe_catalog("config/hardware/syringes.json")
        for vol, s in catalog.items():
            # Full stroke should dispense full volume
            dispensed = s.mm_to_uL(s.stroke_length_mm)
            self.assertAlmostEqual(dispensed, vol, places=0,
                                   msg=f"Syringe {vol}µL: {s.stroke_length_mm}mm stroke != {dispensed}µL")


class TestWorkspaceConfig(unittest.TestCase):
    """Test WorkspaceConfig serialization."""

    def test_create_default(self):
        ws = WorkspaceConfig()
        self.assertEqual(len(ws.pumps), 3)
        self.assertIn("P1", ws.pumps)

    def test_round_trip_json(self):
        ws = WorkspaceConfig()
        ws.needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
        ws.plate_format = 96
        ws.ink_library["TestInk"] = InkSpec(name="TestInk", ink_type="hydrogel")

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False, mode="w") as f:
            path = f.name
        try:
            ws.save_json(path)
            ws2 = WorkspaceConfig.load_json(path)
            self.assertEqual(ws2.needle.gauge, 22)
            self.assertEqual(ws2.plate_format, 96)
            self.assertIn("TestInk", ws2.ink_library)
        finally:
            os.unlink(path)

    def test_validate(self):
        ws = WorkspaceConfig()
        issues = ws.validate()
        self.assertTrue(any("needle" in i.lower() for i in issues))


class TestWellRoleColors(unittest.TestCase):
    """Test that all well roles have colors defined."""

    def test_all_roles_have_colors(self):
        for role in WellRole:
            self.assertIn(role, ROLE_COLORS,
                         f"Missing color for WellRole.{role.name}")


# ---------------------------------------------------------------------------
# Test FlowPhysics
# ---------------------------------------------------------------------------

class TestHagenPoiseuille(unittest.TestCase):
    """Test Hagen-Poiseuille pressure calculation."""

    def test_known_values(self):
        """Water (1 cP) through 22G needle (413µm ID, 25.4mm long) at 1 µL/s."""
        mu = 0.001          # Pa·s (water)
        L = 0.0254          # m (1 inch)
        d = 413e-6          # m
        Q = 1e-9            # m³/s (1 µL/s)

        p = hagen_poiseuille_pressure(mu, L, Q, d)
        # Expected: (128 × 0.001 × 0.0254 × 1e-9) / (π × (413e-6)⁴)
        expected = (128 * 0.001 * 0.0254 * 1e-9) / (math.pi * (413e-6) ** 4)
        self.assertAlmostEqual(p, expected, places=0)
        # ~35 Pa for water at 1 µL/s through 22G — very low pressure
        self.assertGreater(p, 10)
        self.assertLess(p, 500)

    def test_zero_diameter(self):
        p = hagen_poiseuille_pressure(0.001, 0.025, 1e-9, 0.0)
        self.assertEqual(p, float("inf"))

    def test_higher_viscosity_higher_pressure(self):
        """More viscous fluid = more pressure."""
        p_water = hagen_poiseuille_pressure(0.001, 0.025, 1e-9, 400e-6)
        p_gel = hagen_poiseuille_pressure(0.050, 0.025, 1e-9, 400e-6)
        self.assertGreater(p_gel, p_water)


class TestReynoldsNumber(unittest.TestCase):
    """Test Reynolds number calculation."""

    def test_laminar_water(self):
        """Water at low flow rate through needle should be laminar."""
        rho = 1000          # kg/m³
        d = 413e-6          # m
        A = math.pi * (d / 2) ** 2
        Q = 1e-9            # m³/s (1 µL/s)
        v = Q / A           # m/s
        mu = 0.001          # Pa·s

        re = reynolds_number(rho, v, d, mu)
        self.assertLess(re, 100)  # Very laminar
        self.assertEqual(classify_flow_regime(re), FlowRegime.LAMINAR)

    def test_regime_boundaries(self):
        self.assertEqual(classify_flow_regime(100), FlowRegime.LAMINAR)
        self.assertEqual(classify_flow_regime(2100), FlowRegime.TRANSITIONAL)
        self.assertEqual(classify_flow_regime(4000), FlowRegime.TURBULENT)


class TestGranularRegime(unittest.TestCase):
    """Test granular/cell regime classification."""

    def setUp(self):
        self.needle_22g = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)

    def test_pure_liquid(self):
        ink = InkSpec(name="DPBS", viscosity_cP=1.0)
        regime, ratio = classify_granular_regime(self.needle_22g, ink)
        self.assertEqual(regime, GranularRegime.NO_PARTICLES)

    def test_free_flow_cells(self):
        ink = InkSpec(name="HEK293", cell_diameter_um=15)
        regime, ratio = classify_granular_regime(self.needle_22g, ink)
        self.assertEqual(regime, GranularRegime.FREE_FLOW)
        self.assertGreater(ratio, 4.0)

    def test_intermittent(self):
        ink = InkSpec(name="Large", cell_diameter_um=200)
        regime, ratio = classify_granular_regime(self.needle_22g, ink)
        self.assertEqual(regime, GranularRegime.INTERMITTENT)

    def test_jamming(self):
        ink = InkSpec(name="Huge", cell_diameter_um=500)
        regime, ratio = classify_granular_regime(self.needle_22g, ink)
        self.assertEqual(regime, GranularRegime.JAMMING)


class TestWallShearStress(unittest.TestCase):
    """Test cell shear stress calculation."""

    def test_positive_value(self):
        tau = wall_shear_stress(0.001, 1e-9, 413e-6)
        self.assertGreater(tau, 0)

    def test_higher_flow_higher_shear(self):
        tau1 = wall_shear_stress(0.001, 1e-9, 413e-6)
        tau2 = wall_shear_stress(0.001, 5e-9, 413e-6)
        self.assertGreater(tau2, tau1)


class TestMaxSafeFlowRate(unittest.TestCase):
    """Test max safe flow rate computation."""

    def test_wider_needle_faster_flow(self):
        n22 = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
        n30 = NeedleSpec(gauge=30, od_um=312, id_um=159, wall_um=76)
        ink = InkSpec(name="Water", viscosity_cP=1.0)

        rate_22 = max_safe_flow_rate_uL_s(n22, ink)
        rate_30 = max_safe_flow_rate_uL_s(n30, ink)
        self.assertGreater(rate_22, rate_30)

    def test_lower_viscosity_faster_flow(self):
        n = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
        water = InkSpec(name="Water", viscosity_cP=1.0)
        gel = InkSpec(name="Gel", viscosity_cP=50.0)

        self.assertGreater(
            max_safe_flow_rate_uL_s(n, water),
            max_safe_flow_rate_uL_s(n, gel),
        )


class TestExtrusionRate(unittest.TestCase):
    """Test extrusion volume model."""

    def test_basic_extrusion(self):
        """5 mm/s print speed, 22G needle (OD=0.718mm), 0.2mm layer height."""
        n = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
        rate = extrusion_flow_rate(5.0, n, 0.2)
        # Expected: 5.0 × 0.718 × 0.2 = 0.718 µL/s
        self.assertAlmostEqual(rate, 5.0 * 0.718 * 0.2, places=3)

    def test_pump_speed(self):
        n = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
        s = SyringeSpec(volume_uL=100, stroke_length_mm=30.0, barrel_id_mm=2.060)
        speed = extrusion_pump_speed(5.0, n, s, 0.2)
        self.assertGreater(speed, 0)


class TestFlowSafetyCalculation(unittest.TestCase):
    """Test integrated flow safety calculator."""

    def setUp(self):
        self.needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
        self.syringe = SyringeSpec(volume_uL=100, stroke_length_mm=30.0,
                                    barrel_id_mm=2.060)
        self.water = InkSpec(name="Water", ink_type="buffer", viscosity_cP=1.0)
        self.cells = InkSpec(name="Cells", ink_type="cells", viscosity_cP=1.2,
                             cell_diameter_um=20)

    def test_safe_water_flow(self):
        result = calculate_flow_safety(
            self.needle, self.syringe, self.water, 1.0
        )
        self.assertTrue(result.is_safe)
        self.assertEqual(result.flow_regime, FlowRegime.LAMINAR)
        self.assertGreater(result.max_safe_flow_rate_uL_s, 1.0)

    def test_unsafe_high_flow(self):
        """Very high flow rate with viscous ink should exceed pressure limit."""
        viscous_ink = InkSpec(name="Gel", ink_type="hydrogel", viscosity_cP=100.0)
        result = calculate_flow_safety(
            self.needle, self.syringe, viscous_ink, 50.0,
            pressure_limit_Pa=50_000,
        )
        self.assertFalse(result.is_safe)

    def test_cell_shear_included(self):
        result = calculate_flow_safety(
            self.needle, self.syringe, self.cells, 1.0
        )
        self.assertGreater(result.wall_shear_stress_Pa, 0)

    def test_summary_output(self):
        result = calculate_flow_safety(
            self.needle, self.syringe, self.water, 1.0
        )
        summary = result.summary()
        self.assertIn("SAFE", summary)

    def test_print_safety_convenience(self):
        result = calculate_print_safety(
            self.needle, self.syringe, self.water,
            print_speed_mm_s=5.0, layer_height_mm=0.2,
        )
        self.assertIsNotNone(result)
        self.assertGreater(result.requested_flow_rate_uL_s, 0)


class TestCompatibilityReport(unittest.TestCase):
    """Test compatibility report generator."""

    def test_empty_pump(self):
        needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
        pump = PumpLoadout(pump_id="P1")
        reports = generate_compatibility_report(needle, {"P1": pump})
        self.assertEqual(len(reports), 1)
        self.assertEqual(reports[0]["status"], "no_config")

    def test_configured_pump(self):
        needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
        ink = InkSpec(name="TestInk", viscosity_cP=1.0)
        syringe = SyringeSpec(volume_uL=100, stroke_length_mm=30.0,
                               barrel_id_mm=2.060)
        pump = PumpLoadout(pump_id="P1", syringe=syringe)
        pump.fluid_column.aspirate_ink(50.0, ink)

        reports = generate_compatibility_report(needle, {"P1": pump})
        self.assertEqual(len(reports), 1)
        self.assertGreater(reports[0]["max_rate_uL_s"], 0)


class TestPumpSpeedConversions(unittest.TestCase):
    """Test pump speed <-> flow rate conversions."""

    def test_round_trip(self):
        s = SyringeSpec(volume_uL=100, stroke_length_mm=30.0, barrel_id_mm=2.060)
        speed = flow_rate_to_pump_speed(1.0, s)
        rate = pump_speed_to_flow_rate(speed, s)
        self.assertAlmostEqual(rate, 1.0, places=6)


# ---------------------------------------------------------------------------
# Test ControllerProtocol
# ---------------------------------------------------------------------------

class TestControllerProtocol(unittest.TestCase):
    """Test JSON-based controller protocol loading."""

    def test_load_proscan_iii(self):
        proto = ControllerProtocol.load("config/controllers/proscan_iii.json")
        self.assertEqual(proto.controller_name, "Prior ProScan III")
        self.assertEqual(proto.manufacturer, "Prior Scientific")

    def test_load_proscan_ii(self):
        proto = ControllerProtocol.load("config/controllers/proscan_ii.json")
        self.assertEqual(proto.controller_name, "Prior ProScan II")

    def test_terminators(self):
        proto3 = ControllerProtocol.load("config/controllers/proscan_iii.json")
        self.assertEqual(proto3.tx_terminator, b"\r\n")

        proto2 = ControllerProtocol.load("config/controllers/proscan_ii.json")
        self.assertEqual(proto2.tx_terminator, b"\r")

    def test_format_command(self):
        proto = ControllerProtocol.load("config/controllers/proscan_iii.json")
        cmd = proto.format_command("move_absolute", x=1000, y=2000)
        self.assertEqual(cmd, "G 1000,2000")

    def test_format_relative(self):
        proto = ControllerProtocol.load("config/controllers/proscan_iii.json")
        cmd = proto.format_command("move_relative", dx=500, dy=-300)
        self.assertEqual(cmd, "GR 500,-300")

    def test_unsupported_command(self):
        proto = ControllerProtocol.load("config/controllers/proscan_ii.json")
        cmd = proto.format_command("set_jerk", jerk=50)
        self.assertIsNone(cmd)

    def test_has_command(self):
        proto3 = ControllerProtocol.load("config/controllers/proscan_iii.json")
        self.assertTrue(proto3.has_command("set_jerk"))

        proto2 = ControllerProtocol.load("config/controllers/proscan_ii.json")
        self.assertFalse(proto2.has_command("set_jerk"))

    def test_supports_jerk(self):
        proto3 = ControllerProtocol.load("config/controllers/proscan_iii.json")
        self.assertTrue(proto3.supports_jerk)

        proto2 = ControllerProtocol.load("config/controllers/proscan_ii.json")
        self.assertFalse(proto2.supports_jerk)

    def test_command_format_proscan_ii(self):
        """ProScan II uses comma after G (G,x,y) vs ProScan III (G x,y)."""
        proto2 = ControllerProtocol.load("config/controllers/proscan_ii.json")
        cmd = proto2.format_command("move_absolute", x=1000, y=2000)
        self.assertEqual(cmd, "G,1000,2000")

    def test_position_query(self):
        proto = ControllerProtocol.load("config/controllers/proscan_iii.json")
        cmd = proto.format_command("position_query")
        self.assertEqual(cmd, "P")

    def test_file_not_found(self):
        with self.assertRaises(FileNotFoundError):
            ControllerProtocol.load("nonexistent.json")

    def test_discover_files(self):
        files = discover_controller_files("config/controllers")
        self.assertGreaterEqual(len(files), 2)

    def test_load_all(self):
        protos = load_all_protocols("config/controllers")
        self.assertIn("Prior ProScan III", protos)
        self.assertIn("Prior ProScan II", protos)

    def test_detection_info(self):
        proto = ControllerProtocol.load("config/controllers/proscan_iii.json")
        det = proto.get_detection_info()
        self.assertEqual(det["wake_command"], "STAGE")
        self.assertIn("ProScan", det["identify_tokens"])


if __name__ == "__main__":
    unittest.main()
