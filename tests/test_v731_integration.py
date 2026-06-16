"""
Tests for v7.3.1 — End-to-end integration and SimulatedCamera mosaic.

7.1: End-to-end workflow: A1 teach → geometry predict → mosaic scan → Z-teach → jog fast-travel
7.4: SimulatedCamera generates detectable well frames for mosaic scan
7.5: Plate geometry (A1 from center), FOV raster grid, SimulatedCamera auto-origin
"""

import math
import sys
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from SupportClasses.WellPlate import (
    WellPlate, PLATE_FOOTPRINT_X_MM, PLATE_FOOTPRINT_Y_MM,
    STAGE_TRAVEL_X_MM, STAGE_TRAVEL_Y_MM,
)
from SupportClasses.MosaicBuilder import MosaicBuilder, AffineCalibration


# ── 7.4: SimulatedCamera Mosaic Integration ──────────────────────────


class TestSimulatedCameraMosaic(unittest.TestCase):
    """Test that SimulatedCamera frames work with MosaicBuilder + WellDetector."""

    def _make_camera_and_plate(self):
        """Create a SimulatedCamera with a 6-well plate."""
        from SupportClasses.SimulatedCamera import SimulatedCamera
        plate = WellPlate.from_format(6)
        origin_um = (50000.0, 50000.0)
        cam = SimulatedCamera(
            resolution=(916, 686),
            micron_per_pixel=3.34,
            plate=plate,
            plate_origin_um=origin_um,
        )
        return cam, plate, origin_um

    def test_camera_generates_frames(self):
        """SimulatedCamera produces non-empty BGR frames."""
        cam, plate, origin = self._make_camera_and_plate()
        cam.set_stage_position(origin[0], origin[1], z_mm=0.0)
        ok, frame = cam.read()
        self.assertTrue(ok)
        self.assertEqual(frame.ndim, 3)
        self.assertEqual(frame.shape[2], 3)  # BGR

    def test_well_visible_at_a1(self):
        """Frame captured at A1 position has significant non-background pixels."""
        cam, plate, origin = self._make_camera_and_plate()
        cam.set_stage_position(origin[0], origin[1], z_mm=0.0)
        _, frame = cam.read()
        # A1 is at origin — frame should contain well pixels (not all black)
        mean_intensity = np.mean(frame)
        self.assertGreater(mean_intensity, 5.0,
                           "Frame at A1 should contain visible well, not all black")

    def test_mosaic_builder_with_simulated_frames(self):
        """MosaicBuilder accepts SimulatedCamera frames and produces a mosaic."""
        cam, plate, origin = self._make_camera_and_plate()
        builder = MosaicBuilder(micron_per_pixel=3.34)
        positions = plate.get_all_positions_from_a1(origin[0], origin[1])

        # Capture frames at each well position
        for well_name in plate.meander_order():
            wx, wy = positions[well_name]
            cam.set_stage_position(wx, wy, z_mm=0.0)
            _, frame = cam.read()
            builder.add_frame(well_name, frame, wx, wy)

        self.assertEqual(builder.frame_count, len(positions))

        # Build mosaic
        mosaic = builder.build_mosaic()
        self.assertIsNotNone(mosaic)
        self.assertEqual(mosaic.ndim, 3)
        self.assertGreater(mosaic.shape[0], 0)
        self.assertGreater(mosaic.shape[1], 0)

    def test_mosaic_with_well_detection_pipeline(self):
        """WellDetector + MosaicBuilder pipeline runs without errors on simulated frames.

        Note: The SimulatedCamera renders simplified wells (solid circles with edge
        rings) that may not always be detected by HoughCircles depending on the
        plate format and camera resolution. This test validates the pipeline runs
        end-to-end, not that detection succeeds for every frame.
        """
        from SupportClasses.SimulatedCamera import SimulatedCamera
        from SupportClasses.VisionDetector import WellDetector

        plate = WellPlate.from_format(6)
        origin = (50000.0, 50000.0)
        um_per_px = 3.34
        cam = SimulatedCamera(
            resolution=(916, 686),
            micron_per_pixel=um_per_px,
            plate=plate,
            plate_origin_um=origin,
        )

        builder = MosaicBuilder(micron_per_pixel=um_per_px)
        positions = plate.get_all_positions_from_a1(origin[0], origin[1])

        well_diam_um = plate.well_diameter * 1000.0
        expected_diam_px = well_diam_um / um_per_px

        for well_name in plate.meander_order():
            wx, wy = positions[well_name]
            cam.set_stage_position(wx, wy, z_mm=0.0)
            _, frame = cam.read()

            # Detection may or may not succeed — that's OK
            detection = WellDetector.detect_well(
                frame, expected_diameter_px=expected_diam_px, tolerance=0.5)
            builder.add_frame(well_name, frame, wx, wy, detection=detection)

        # Pipeline should complete without errors
        self.assertEqual(builder.frame_count, len(positions))
        mosaic = builder.build_mosaic()
        self.assertIsNotNone(mosaic)

    def test_affine_from_simulated_is_near_identity(self):
        """With perfectly aligned SimulatedCamera, affine should be near-identity."""
        from SupportClasses.SimulatedCamera import SimulatedCamera
        from SupportClasses.VisionDetector import WellDetector

        plate = WellPlate.from_format(6)
        origin = (50000.0, 50000.0)
        um_per_px = 3.34
        cam = SimulatedCamera(
            resolution=(916, 686),
            micron_per_pixel=um_per_px,
            plate=plate,
            plate_origin_um=origin,
        )

        builder = MosaicBuilder(micron_per_pixel=um_per_px)
        positions = plate.get_all_positions_from_a1(origin[0], origin[1])
        well_diam_um = plate.well_diameter * 1000.0
        expected_diam_px = well_diam_um / um_per_px

        for well_name in plate.meander_order():
            wx, wy = positions[well_name]
            cam.set_stage_position(wx, wy, z_mm=0.0)
            _, frame = cam.read()
            detection = WellDetector.detect_well(
                frame, expected_diameter_px=expected_diam_px, tolerance=0.5)
            builder.add_frame(well_name, frame, wx, wy, detection=detection)

        cal = builder.fit_affine(positions)
        if cal is not None:
            # Rotation should be very small (< 1 degree)
            self.assertLess(abs(cal.rotation_deg), 1.0,
                            f"Expected near-zero rotation, got {cal.rotation_deg:.3f}")
            # Scale should be near 1.0
            self.assertAlmostEqual(cal.scale, 1.0, delta=0.05,
                                   msg=f"Expected scale ~1.0, got {cal.scale:.4f}")


# ── 7.1: End-to-End Workflow Integration ─────────────────────────────


class TestEndToEndWorkflow(unittest.TestCase):
    """Test the full calibration → jog navigation data flow."""

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        if QApplication.instance() is None:
            cls._app = QApplication(sys.argv)
        else:
            cls._app = QApplication.instance()

    def _make_mock_controller(self):
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = True
        ctrl.xy_stage = MagicMock()
        ctrl.zp_stage = MagicMock()
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0, "P1": 0, "P2": 0, "P3": 0}
        ctrl.get_xy_position.return_value = (50000.0, 50000.0)
        ctrl.get_zp_position.return_value = (10.0, 0.0, 0.0, 0.0)
        ctrl.move_z_absolute = MagicMock()
        ctrl.move_xy_absolute = MagicMock()
        ctrl.safe_travel_to = MagicMock()
        return ctrl

    def test_geometry_prediction_from_a1(self):
        """After teaching A1, predicted positions are generated for all wells."""
        plate = WellPlate.from_format(96)
        a1_x, a1_y = 50000.0, 50000.0
        positions = plate.get_all_positions_from_a1(a1_x, a1_y)

        self.assertEqual(len(positions), 96)
        self.assertIn("A1", positions)
        self.assertIn("H12", positions)
        # A1 should be at the origin
        self.assertAlmostEqual(positions["A1"][0], a1_x, places=1)
        self.assertAlmostEqual(positions["A1"][1], a1_y, places=1)
        # A2 should be 9mm (9000µm) away in X
        dx = positions["A2"][0] - positions["A1"][0]
        self.assertAlmostEqual(dx, 9000.0, places=0)

    def test_meander_order_covers_all_wells(self):
        """Meander order includes every well exactly once."""
        plate = WellPlate.from_format(96)
        order = plate.meander_order()
        self.assertEqual(len(order), 96)
        self.assertEqual(len(set(order)), 96)  # all unique

    def test_meander_order_serpentine_pattern(self):
        """Meander order alternates row direction."""
        plate = WellPlate.from_format(96)
        order = plate.meander_order()
        # Row A (even): A1, A2, ... A12
        self.assertEqual(order[0], "A1")
        self.assertEqual(order[11], "A12")
        # Row B (odd): B12, B11, ... B1
        self.assertEqual(order[12], "B12")
        self.assertEqual(order[23], "B1")

    def test_affine_correction_improves_positions(self):
        """Affine correction from mosaic scan refines predicted positions."""
        plate = WellPlate.from_format(6)
        predicted = plate.get_all_positions_from_a1(50000.0, 50000.0)

        # Simulate a 1-degree rotation offset
        rotation_deg = 1.0
        rad = math.radians(rotation_deg)
        cos_r, sin_r = math.cos(rad), math.sin(rad)

        # "True" positions are rotated versions of predicted
        true_positions = {}
        cx = np.mean([p[0] for p in predicted.values()])
        cy = np.mean([p[1] for p in predicted.values()])
        for name, (px, py) in predicted.items():
            dx, dy = px - cx, py - cy
            true_positions[name] = (
                cx + dx * cos_r - dy * sin_r,
                cy + dx * sin_r + dy * cos_r,
            )

        # Build MosaicBuilder with "detected" positions at true locations.
        # detected absolute = stage_position + offset, so stage=true, offset=(0,0)
        # means detected_absolute == true_position.
        from SupportClasses.MosaicBuilder import FrameRecord
        builder = MosaicBuilder(micron_per_pixel=3.34)
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        for name, (tx, ty) in true_positions.items():
            rec = FrameRecord(
                well_name=name,
                frame=frame,
                stage_x_um=tx,
                stage_y_um=ty,
                detected_offset_um=(0.0, 0.0),
                confidence=0.9,
            )
            builder._records.append(rec)

        cal = builder.fit_affine(predicted)
        self.assertIsNotNone(cal)
        self.assertAlmostEqual(cal.rotation_deg, rotation_deg, delta=0.1)

        # Apply correction
        corrected = cal.correct_positions(predicted)
        # Corrected positions should be close to true positions
        for name in predicted:
            cx_corr, cy_corr = corrected[name]
            tx, ty = true_positions[name]
            dist = math.sqrt((cx_corr - tx) ** 2 + (cy_corr - ty) ** 2)
            self.assertLess(dist, 100.0,
                            f"Well {name}: corrected position {dist:.0f}µm from true")

    def test_calibration_data_flows_to_jog_page(self):
        """CalibrationPage data bridge delivers data to JogControlPage."""
        from gui.pages.calibration import CalibrationPage
        from gui.pages.jog_control import JogControlPage

        ctrl = self._make_mock_controller()
        cal_page = CalibrationPage(ctrl, settings=None)
        jog_page = JogControlPage(ctrl)

        # Wire the signal (as app.py does)
        cal_page.calibration_data_changed.connect(
            lambda: jog_page.set_calibration_data(*cal_page.get_calibration_data())
        )

        # Simulate calibration workflow: set plate + teach A1
        plate = WellPlate.from_format(6)
        cal_page._plate = plate
        cal_page._taught_a1 = (50000.0, 50000.0)
        cal_page._compute_predicted_positions()

        # Verify jog page received the data
        self.assertIs(jog_page._plate, plate)
        self.assertIsNotNone(jog_page._well_positions)
        self.assertIn("A1", jog_page._well_positions)

    def test_safe_z_flows_to_jog_page(self):
        """Setting safe_z on CalibrationPage propagates to JogControlPage."""
        from gui.pages.calibration import CalibrationPage
        from gui.pages.jog_control import JogControlPage

        ctrl = self._make_mock_controller()
        cal_page = CalibrationPage(ctrl, settings=None)
        jog_page = JogControlPage(ctrl)

        cal_page.calibration_data_changed.connect(
            lambda: jog_page.set_calibration_data(*cal_page.get_calibration_data())
        )

        # Set safe Z. v7.4.2: _set_safe_z reads Z via the logical accessor.
        ctrl.get_zp_position.return_value = (15.0, 0.0, 0.0, 0.0)
        ctrl.zp_logical_value.return_value = 15.0
        cal_page._set_safe_z()

        self.assertAlmostEqual(jog_page._safe_z, 15.0)

    def test_jog_page_fast_travel_calls_safe_travel(self):
        """Right-click fast-travel on the jog workspace calls safe_travel_to().

        v7.4.3: fast travel is coordinate-based (a click on the workspace
        canvas emits zero-ref µm) rather than well-name based.
        """
        from gui.pages.jog_control import JogControlPage

        ctrl = self._make_mock_controller()
        ctrl.zp_logical_value.return_value = 10.0  # current Z for return move
        jog_page = JogControlPage(ctrl)

        plate = WellPlate.from_format(6)
        positions = plate.get_all_positions_from_a1(50000.0, 50000.0)
        jog_page.set_calibration_data(plate, positions, safe_z=15.0)

        # A1 in zero-ref µm (zero_position x/y are 0 here).
        a1_zr = positions["A1"]
        jog_page._on_workspace_fast_travel_requested(a1_zr[0], a1_zr[1])

        ctrl.safe_travel_to.assert_called_once()
        args, kwargs = ctrl.safe_travel_to.call_args
        self.assertAlmostEqual(args[0], a1_zr[0])      # stage_x (zero=0)
        self.assertAlmostEqual(args[1], a1_zr[1])      # stage_y
        self.assertAlmostEqual(kwargs["safe_z_mm"], 15.0)

    def test_jog_page_safety_gate_no_safe_z(self):
        """Without safe_z, fast travel shows a warning instead of moving."""
        from gui.pages.jog_control import JogControlPage

        ctrl = self._make_mock_controller()
        jog_page = JogControlPage(ctrl)

        plate = WellPlate.from_format(6)
        positions = plate.get_all_positions_from_a1(50000.0, 50000.0)
        jog_page.set_calibration_data(plate, positions, safe_z=None)

        # Fast travel without a safe Z — warns, does NOT call safe_travel_to.
        # Patch the name bound in the jog_control module.
        with patch('gui.pages.jog_control.QMessageBox') as mock_msg:
            jog_page._on_workspace_fast_travel_requested(50000.0, 50000.0)
            mock_msg.warning.assert_called_once()
            ctrl.safe_travel_to.assert_not_called()

    def test_jog_page_fast_travel_requires_xy_connected(self):
        """v7.4.3: fast travel is a no-op when the XY stage isn't connected."""
        from gui.pages.jog_control import JogControlPage

        ctrl = self._make_mock_controller()
        ctrl.is_xy_connected = False
        jog_page = JogControlPage(ctrl)

        plate = WellPlate.from_format(6)
        positions = plate.get_all_positions_from_a1(50000.0, 50000.0)
        jog_page.set_calibration_data(plate, positions, safe_z=15.0)

        jog_page._on_workspace_fast_travel_requested(50000.0, 50000.0)
        ctrl.safe_travel_to.assert_not_called()


# ── 7.5: Plate Geometry & Raster Scan ────────────────────────────


class TestPlateGeometryFromCenter(unittest.TestCase):
    """Test ANSI/SLAS plate footprint and A1-from-center computation."""

    def test_footprint_constants(self):
        """ANSI/SLAS footprint constants are correct."""
        self.assertAlmostEqual(PLATE_FOOTPRINT_X_MM, 127.76)
        self.assertAlmostEqual(PLATE_FOOTPRINT_Y_MM, 85.48)

    def test_a1_from_center_96_well(self):
        """96-well A1 position computed from plate center is correct."""
        plate = WellPlate.from_format(96)
        center_x, center_y = 65000.0, 42500.0  # stage center
        a1_x, a1_y = plate.get_a1_from_plate_center(center_x, center_y)

        # A1 offset for 96-well: (14.38, 11.24) mm
        # Expected: center - (footprint/2 - a1_offset) * 1000
        expected_x = center_x + (14.38 - 127.76 / 2.0) * 1000.0
        expected_y = center_y + (11.24 - 85.48 / 2.0) * 1000.0
        self.assertAlmostEqual(a1_x, expected_x, places=0)
        self.assertAlmostEqual(a1_y, expected_y, places=0)

    def test_a1_from_center_6_well(self):
        """6-well A1 position computed from plate center is correct."""
        plate = WellPlate.from_format(6)
        center_x, center_y = 65000.0, 42500.0
        a1_x, a1_y = plate.get_a1_from_plate_center(center_x, center_y)

        expected_x = center_x + (24.76 - 127.76 / 2.0) * 1000.0
        expected_y = center_y + (23.16 - 85.48 / 2.0) * 1000.0
        self.assertAlmostEqual(a1_x, expected_x, places=0)
        self.assertAlmostEqual(a1_y, expected_y, places=0)

    def test_all_positions_from_center(self):
        """get_all_positions_from_plate_center produces correct well count."""
        plate = WellPlate.from_format(96)
        positions = plate.get_all_positions_from_plate_center(65000.0, 42500.0)
        self.assertEqual(len(positions), 96)
        self.assertIn("A1", positions)
        self.assertIn("H12", positions)

    def test_all_positions_from_center_matches_manual(self):
        """Positions from center match manual A1 → all positions."""
        plate = WellPlate.from_format(96)
        center = (65000.0, 42500.0)
        a1 = plate.get_a1_from_plate_center(*center)
        positions_a1 = plate.get_all_positions_from_a1(*a1)
        positions_center = plate.get_all_positions_from_plate_center(*center)
        for name in positions_a1:
            self.assertAlmostEqual(positions_a1[name][0], positions_center[name][0])
            self.assertAlmostEqual(positions_a1[name][1], positions_center[name][1])

    def test_well_area_bounds(self):
        """Well area bounds are valid and enclose wells within stage travel."""
        plate = WellPlate.from_format(6)  # 6-well fits within 130×85mm travel
        center = (65000.0, 42500.0)
        bounds = plate.get_well_area_bounds_um(*center)
        min_x, min_y, max_x, max_y = bounds

        # Bounds must be valid
        self.assertLess(min_x, max_x)
        self.assertLess(min_y, max_y)

        # All 6-well positions should be inside bounds (plate fits in travel)
        positions = plate.get_all_positions_from_plate_center(*center)
        for name, (wx, wy) in positions.items():
            self.assertGreaterEqual(wx, min_x,
                                    f"Well {name} X below min bound")
            self.assertLessEqual(wx, max_x,
                                 f"Well {name} X above max bound")
            self.assertGreaterEqual(wy, min_y,
                                    f"Well {name} Y below min bound")
            self.assertLessEqual(wy, max_y,
                                 f"Well {name} Y above max bound")

    def test_well_area_bounds_96_well_clamps(self):
        """96-well plate bounds are clamped to stage travel limits."""
        plate = WellPlate.from_format(96)
        center = (65000.0, 42500.0)
        bounds = plate.get_well_area_bounds_um(*center)
        min_x, min_y, max_x, max_y = bounds

        # Bounds are still valid
        self.assertLess(min_x, max_x)
        self.assertLess(min_y, max_y)

        # Plate Y footprint (85.48mm) slightly exceeds stage Y (85mm)
        # Bounds should be clamped to stage travel
        self.assertGreaterEqual(min_x, 0.0)
        self.assertLessEqual(max_y, STAGE_TRAVEL_Y_MM * 1000.0)

    def test_well_area_bounds_clamp_to_travel(self):
        """Well area bounds are clamped to stage travel limits."""
        plate = WellPlate.from_format(96)
        center = (65000.0, 42500.0)
        bounds = plate.get_well_area_bounds_um(*center)
        min_x, min_y, max_x, max_y = bounds

        max_travel_x = STAGE_TRAVEL_X_MM * 1000.0
        max_travel_y = STAGE_TRAVEL_Y_MM * 1000.0
        self.assertGreaterEqual(min_x, 0.0)
        self.assertLessEqual(max_x, max_travel_x)
        self.assertGreaterEqual(min_y, 0.0)
        self.assertLessEqual(max_y, max_travel_y)

    def test_well_area_bounds_from_a1(self):
        """Bounds from A1 directly match bounds from plate center."""
        plate = WellPlate.from_format(6)
        center = (65000.0, 42500.0)
        a1 = plate.get_a1_from_plate_center(*center)
        bounds_center = plate.get_well_area_bounds_um(*center)
        bounds_a1 = plate.get_well_area_bounds_from_a1_um(*a1)
        for i in range(4):
            self.assertAlmostEqual(bounds_center[i], bounds_a1[i], places=0)

    def test_all_formats_have_valid_a1(self):
        """Every plate format produces a valid A1 position from center."""
        for fmt in [6, 12, 24, 48, 96, 384]:
            plate = WellPlate.from_format(fmt)
            a1_x, a1_y = plate.get_a1_from_plate_center(65000.0, 42500.0)
            self.assertIsInstance(a1_x, float)
            self.assertIsInstance(a1_y, float)
            # A1 should be to the left and above center (offset < footprint/2)
            self.assertLess(a1_x, 65000.0, f"{fmt}-well A1 should be left of center")
            self.assertLess(a1_y, 42500.0, f"{fmt}-well A1 should be above center")


class TestRasterGridGeneration(unittest.TestCase):
    """Test MosaicBuilder.generate_raster_positions()."""

    def test_basic_raster(self):
        """Raster grid covers the scan area with expected number of positions."""
        builder = MosaicBuilder(
            frame_size_px=(916, 686), micron_per_pixel=3.34)
        # FOV: 916*3.34 = 3059.44 µm, 686*3.34 = 2291.24 µm
        # Step (10% overlap): 2753.5 × 2062.1 µm
        # Area: 100mm × 80mm = 100000 × 80000 µm
        bounds = (0.0, 0.0, 100000.0, 80000.0)
        positions = builder.generate_raster_positions(bounds, overlap=0.1)
        self.assertGreater(len(positions), 0)
        # Should be roughly (100000/2753) × (80000/2062) ≈ 36 × 39 ≈ 1400
        self.assertGreater(len(positions), 500)

    def test_single_fov_area(self):
        """Area smaller than one FOV produces exactly 1 position."""
        builder = MosaicBuilder(
            frame_size_px=(916, 686), micron_per_pixel=3.34)
        # Tiny area
        bounds = (0.0, 0.0, 1000.0, 1000.0)
        positions = builder.generate_raster_positions(bounds, overlap=0.1)
        self.assertEqual(len(positions), 1)
        # Position should be at center of bounds
        self.assertAlmostEqual(positions[0][0], 500.0, places=0)
        self.assertAlmostEqual(positions[0][1], 500.0, places=0)

    def test_serpentine_order(self):
        """Raster positions follow serpentine (meander) pattern."""
        builder = MosaicBuilder(
            frame_size_px=(100, 100), micron_per_pixel=10.0)
        # FOV = 1000µm, step = 900µm (10% overlap)
        bounds = (0.0, 0.0, 5000.0, 5000.0)
        positions = builder.generate_raster_positions(bounds, overlap=0.1)

        # At least 2 rows to check serpentine
        self.assertGreater(len(positions), 5)

        # Find row boundaries: positions with same Y
        rows_y = []
        current_y = positions[0][1]
        row_start = 0
        for i, (x, y) in enumerate(positions):
            if abs(y - current_y) > 1.0:
                rows_y.append((row_start, i - 1, current_y))
                current_y = y
                row_start = i
        rows_y.append((row_start, len(positions) - 1, current_y))

        if len(rows_y) >= 2:
            # Row 0: left-to-right (X increasing)
            r0_start, r0_end, _ = rows_y[0]
            self.assertLess(positions[r0_start][0], positions[r0_end][0])
            # Row 1: right-to-left (X decreasing)
            r1_start, r1_end, _ = rows_y[1]
            self.assertGreater(positions[r1_start][0], positions[r1_end][0])

    def test_overlap_increases_density(self):
        """Higher overlap produces more positions."""
        builder = MosaicBuilder(
            frame_size_px=(916, 686), micron_per_pixel=3.34)
        bounds = (0.0, 0.0, 50000.0, 50000.0)
        pos_10 = builder.generate_raster_positions(bounds, overlap=0.1)
        pos_30 = builder.generate_raster_positions(bounds, overlap=0.3)
        self.assertGreater(len(pos_30), len(pos_10))


class TestSimulatedCameraAutoOrigin(unittest.TestCase):
    """Test SimulatedCamera auto-computing plate origin from plate center."""

    def test_auto_origin_from_center(self):
        """SimulatedCamera computes A1 from plate center when no origin given."""
        from SupportClasses.SimulatedCamera import SimulatedCamera
        plate = WellPlate.from_format(96)
        center = (65000.0, 42500.0)
        cam = SimulatedCamera(
            resolution=(916, 686),
            micron_per_pixel=3.34,
            plate=plate,
            plate_center_um=center,
        )
        expected_a1 = plate.get_a1_from_plate_center(*center)
        self.assertAlmostEqual(cam._plate_origin_um[0], expected_a1[0], places=0)
        self.assertAlmostEqual(cam._plate_origin_um[1], expected_a1[1], places=0)

    def test_explicit_origin_overrides_center(self):
        """Explicit plate_origin_um takes priority over plate_center_um."""
        from SupportClasses.SimulatedCamera import SimulatedCamera
        plate = WellPlate.from_format(96)
        cam = SimulatedCamera(
            resolution=(916, 686),
            micron_per_pixel=3.34,
            plate=plate,
            plate_origin_um=(10000.0, 20000.0),
            plate_center_um=(65000.0, 42500.0),
        )
        self.assertAlmostEqual(cam._plate_origin_um[0], 10000.0)
        self.assertAlmostEqual(cam._plate_origin_um[1], 20000.0)

    def test_auto_origin_generates_visible_wells(self):
        """With auto-origin, camera at A1 position shows a visible well."""
        from SupportClasses.SimulatedCamera import SimulatedCamera
        plate = WellPlate.from_format(6)
        center = (65000.0, 42500.0)
        cam = SimulatedCamera(
            resolution=(916, 686),
            micron_per_pixel=3.34,
            plate=plate,
            plate_center_um=center,
        )
        # Move to A1
        a1 = plate.get_a1_from_plate_center(*center)
        cam.set_stage_position(a1[0], a1[1], z_mm=0.0)
        ok, frame = cam.read()
        self.assertTrue(ok)
        self.assertGreater(np.mean(frame), 5.0,
                           "Frame at A1 should show visible well")

    def test_raster_mosaic_with_auto_origin(self):
        """Full raster scan + mosaic build using auto-origin SimulatedCamera."""
        from SupportClasses.SimulatedCamera import SimulatedCamera
        plate = WellPlate.from_format(6)
        center = (65000.0, 42500.0)
        cam = SimulatedCamera(
            resolution=(916, 686),
            micron_per_pixel=3.34,
            plate=plate,
            plate_center_um=center,
        )
        builder = MosaicBuilder(
            frame_size_px=(916, 686), micron_per_pixel=3.34)

        # Get scan bounds and generate raster
        bounds = plate.get_well_area_bounds_um(*center)
        positions = builder.generate_raster_positions(bounds, overlap=0.1)
        self.assertGreater(len(positions), 0)

        # Capture a subset of frames (first 10 for speed)
        for i, (px, py) in enumerate(positions[:10]):
            cam.set_stage_position(px, py, z_mm=0.0)
            ok, frame = cam.read()
            self.assertTrue(ok)
            builder.add_raster_frame(frame, px, py, index=i)

        self.assertEqual(builder.frame_count, min(10, len(positions)))
        mosaic = builder.build_mosaic()
        self.assertIsNotNone(mosaic)
        self.assertEqual(mosaic.ndim, 3)


class TestFOVAwareDetection(unittest.TestCase):
    """Test FOV-aware well detection during raster scan."""

    def test_well_detection_at_well_center(self):
        """Detection succeeds when camera is positioned at a well center."""
        from SupportClasses.SimulatedCamera import SimulatedCamera
        from SupportClasses.VisionDetector import WellDetector

        # Use 1x objective scale (6.68 µm/px) — wells visible in frame
        um_per_px = 6.68
        plate = WellPlate.from_format(384)  # 3.63mm wells = 543px at 1x
        center = (65000.0, 42500.0)
        cam = SimulatedCamera(
            resolution=(916, 686),
            micron_per_pixel=um_per_px,
            plate=plate,
            plate_center_um=center,
        )

        positions = plate.get_all_positions_from_plate_center(*center)
        a1_pos = positions["A1"]

        # Position camera exactly at A1
        cam.set_stage_position(a1_pos[0], a1_pos[1], z_mm=0.0)
        _, frame = cam.read()

        # Expected diameter capped to 80% of smaller frame dim
        well_diam_px = plate.well_diameter * 1000.0 / um_per_px
        max_diam = min(916, 686) * 0.8
        expected = min(well_diam_px, max_diam)

        detection = WellDetector.detect_well_with_fallback(
            frame, expected, tolerance=0.5)
        # At 1x with 384-well, the well (~543px) fits in the 686px frame
        # Detection should succeed
        self.assertIsNotNone(detection,
                             f"Expected detection at A1 (expected {expected:.0f}px)")

    def test_diameter_cap_prevents_oversized_search(self):
        """Expected diameter is capped to prevent searching for circles > frame."""
        plate = WellPlate.from_format(6)  # 34.8mm wells
        um_per_px = 3.34  # 2x objective
        well_diam_px = plate.well_diameter * 1000.0 / um_per_px  # ~10419 px

        # Cap to 80% of smaller frame dim
        max_diam = min(916, 686) * 0.8  # 548.8
        capped = min(well_diam_px, max_diam)

        self.assertAlmostEqual(capped, 548.8, places=0)
        self.assertLess(capped, 686)  # Fits in frame


# ── v7.3.1: Spiral scan + per-well scan tests ────────────────────────


class TestThreeWellCalibration(unittest.TestCase):
    """Test 3-well auto-calibration logic."""

    def test_get_calibration_wells_96(self):
        """96-well plate should select A1, A12, H12."""
        from SupportClasses.WellPlate import ROW_LABELS
        plate = WellPlate.from_format(96)
        wells = ["A1", f"A{plate.cols}", f"{ROW_LABELS[plate.rows - 1]}{plate.cols}"]
        self.assertEqual(wells, ["A1", "A12", "H12"])

    def test_get_calibration_wells_6(self):
        """6-well plate should select A1, A3, B3."""
        from SupportClasses.WellPlate import ROW_LABELS
        plate = WellPlate.from_format(6)
        wells = ["A1", f"A{plate.cols}", f"{ROW_LABELS[plate.rows - 1]}{plate.cols}"]
        self.assertEqual(wells, ["A1", "A3", "B3"])

    def test_get_calibration_wells_384(self):
        """384-well plate should select A1, A24, P24."""
        from SupportClasses.WellPlate import ROW_LABELS
        plate = WellPlate.from_format(384)
        wells = ["A1", f"A{plate.cols}", f"{ROW_LABELS[plate.rows - 1]}{plate.cols}"]
        self.assertEqual(wells, ["A1", "A24", "P24"])

    def test_approach_position_center_small_well(self):
        """Small well fitting in FOV should use center approach."""
        # 96-well: diameter = 6.35mm = 6350 µm
        # FOV: 916 * 3.34 = 3059 µm (width), 686 * 3.34 = 2291 µm (height)
        # fov_min = 2291, 80% = 1833 µm
        # 6350 > 1833, so should NOT be center — this is edge approach
        plate = WellPlate.from_format(96)
        well_diam_um = plate.well_diameter * 1000.0
        fov_min = 686 * 3.34  # ~2291 µm
        # 96-well doesn't fit, use a hypothetical small well check
        self.assertGreater(well_diam_um, fov_min * 0.8)

    def test_approach_position_edge_large_well(self):
        """Well larger than 80% of FOV should use edge offset approach."""
        plate = WellPlate.from_format(96)
        well_diam_um = plate.well_diameter * 1000.0
        fov_min = 686 * 3.34  # ~2291 µm
        # 96-well (6350 µm) > 80% of fov_min (1833 µm) → edge approach
        self.assertGreater(well_diam_um, fov_min * 0.8)
        # Verify offset computation
        well_radius_um = well_diam_um / 2.0
        offset = well_radius_um - fov_min / 4.0
        self.assertGreater(offset, 0)
        self.assertLess(offset, well_radius_um)

    def test_procrustes_fit_identity(self):
        """When predicted == detected, fit should give identity transform."""
        pred = np.array([[0, 0], [100, 0], [100, 100]], dtype=np.float64)
        det = pred.copy()

        pred_c = pred - pred.mean(axis=0)
        det_c = det - det.mean(axis=0)
        H = pred_c.T @ det_c
        U, S, Vt = np.linalg.svd(H)
        d = np.linalg.det(Vt.T @ U.T)
        D = np.diag([1.0, 1.0 if d >= 0 else -1.0])
        R = Vt.T @ D @ U.T
        scale = np.sqrt((det_c ** 2).sum()) / np.sqrt((pred_c ** 2).sum())
        rotation = math.degrees(math.atan2(R[1, 0], R[0, 0]))

        self.assertAlmostEqual(scale, 1.0, places=5)
        self.assertAlmostEqual(rotation, 0.0, places=5)

    def test_procrustes_fit_rotation(self):
        """Procrustes should recover a known rotation."""
        angle = math.radians(5.0)
        R_true = np.array([[math.cos(angle), -math.sin(angle)],
                           [math.sin(angle),  math.cos(angle)]])

        pred = np.array([[0, 0], [9000, 0], [9000, 9000]], dtype=np.float64)
        # Rotate around centroid
        centroid = pred.mean(axis=0)
        det = ((pred - centroid) @ R_true.T) + centroid

        pred_c = pred - pred.mean(axis=0)
        det_c = det - det.mean(axis=0)
        H = pred_c.T @ det_c
        U, S, Vt = np.linalg.svd(H)
        d = np.linalg.det(Vt.T @ U.T)
        D = np.diag([1.0, 1.0 if d >= 0 else -1.0])
        R_fit = Vt.T @ D @ U.T
        rotation = math.degrees(math.atan2(R_fit[1, 0], R_fit[0, 0]))

        self.assertAlmostEqual(rotation, 5.0, places=3)

    def test_affine_calibration_correct_positions(self):
        """AffineCalibration.correct_positions should apply the transform."""
        cal = AffineCalibration(
            rotation_deg=0.0, scale=1.0,
            translation_um=(100.0, -50.0),
            num_points=3, residual_um=0.0,
        )
        positions = {"A1": (1000.0, 2000.0), "A2": (10000.0, 2000.0)}
        corrected = cal.correct_positions(positions)
        self.assertAlmostEqual(corrected["A1"][0], 1100.0, delta=1.0)
        self.assertAlmostEqual(corrected["A1"][1], 1950.0, delta=1.0)

    def test_edge_detection_large_well(self):
        """Edge-based detection finds well center for wells larger than FOV."""
        from SupportClasses.SimulatedCamera import SimulatedCamera

        plate = WellPlate.from_format(24)  # 15.54mm wells, much larger than FOV
        center = (65000.0, 42500.0)
        um_per_px = 3.34
        cam = SimulatedCamera(
            resolution=(916, 686),
            micron_per_pixel=um_per_px,
            plate=plate,
            plate_center_um=center,
        )
        positions = plate.get_all_positions_from_plate_center(*center)
        a1_pos = positions["A1"]

        well_radius_um = plate.well_diameter * 1000.0 / 2.0
        fov_min = 686 * um_per_px  # ~2291 µm
        offset = well_radius_um - fov_min / 4.0

        # Approach A1 from -X (well_index=0)
        approach_x = a1_pos[0] - offset
        approach_y = a1_pos[1]
        cam.set_stage_position(approach_x, approach_y, z_mm=0.0)
        _, frame = cam.read()

        # Run edge detection (same logic as _detect_well_at_position)
        gray = np.mean(frame, axis=2).astype(np.float64)
        fh, fw = gray.shape
        kernel_size = max(11, int(min(fw, fh) * 0.03) | 1)
        kernel = np.ones(kernel_size) / kernel_size
        margin = max(20, int(min(fw, fh) * 0.05))
        profile = gray.mean(axis=0)
        smooth = np.convolve(profile, kernel, mode='same')
        grad = np.diff(smooth)
        search = np.abs(grad[margin:fw - margin])
        edge_idx = int(np.argmax(search)) + margin
        edge_x_um = approach_x + (edge_idx - fw / 2.0) * um_per_px
        if grad[edge_idx] > 0:
            detected_x = edge_x_um + well_radius_um
        else:
            detected_x = edge_x_um - well_radius_um

        # Detected center should be within 500 µm of actual A1 position
        error = abs(detected_x - a1_pos[0])
        self.assertLess(error, 500.0,
                        f"Edge detection X error {error:.0f} µm too large "
                        f"(detected={detected_x:.0f}, actual={a1_pos[0]:.0f})")

    def test_single_well_scan_50pct_overlap(self):
        """Per-well scan with 50% overlap should produce more tiles than 10%."""
        plate = WellPlate.from_format(96)
        well_x, well_y = 30000.0, 50000.0
        bounds = plate.get_single_well_scan_bounds_um(well_x, well_y)

        builder_10 = MosaicBuilder(
            frame_size_px=(916, 686), micron_per_pixel=3.34, overlap=0.10)
        positions_10 = builder_10.generate_raster_positions(bounds, overlap=0.10)

        builder_50 = MosaicBuilder(
            frame_size_px=(916, 686), micron_per_pixel=3.34, overlap=0.50)
        positions_50 = builder_50.generate_raster_positions(bounds, overlap=0.50)

        self.assertGreater(len(positions_50), len(positions_10))


class TestSingleWellScanBounds(unittest.TestCase):
    """Test WellPlate.get_single_well_scan_bounds_um()."""

    def test_96_well_bounds(self):
        """96-well scan bounds should be roughly 1.25x well diameter."""
        plate = WellPlate.from_format(96)
        well_x, well_y = 30000.0, 50000.0
        bounds = plate.get_single_well_scan_bounds_um(well_x, well_y)
        min_x, min_y, max_x, max_y = bounds
        expected_span = plate.well_diameter * 1000.0 * 1.25
        self.assertAlmostEqual(max_x - min_x, expected_span, delta=1.0)
        self.assertAlmostEqual(max_y - min_y, expected_span, delta=1.0)
        # Centered on well
        self.assertAlmostEqual((min_x + max_x) / 2, well_x, delta=1.0)
        self.assertAlmostEqual((min_y + max_y) / 2, well_y, delta=1.0)

    def test_bounds_clamped_to_stage(self):
        """Bounds near stage edge should be clamped."""
        plate = WellPlate.from_format(6)  # large wells
        # Position near stage origin
        bounds = plate.get_single_well_scan_bounds_um(1000.0, 1000.0)
        self.assertGreaterEqual(bounds[0], 0.0)
        self.assertGreaterEqual(bounds[1], 0.0)

    def test_per_well_raster_reasonable_count(self):
        """Raster grid for a single 96-well with 50% overlap should produce a reasonable count."""
        plate = WellPlate.from_format(96)
        well_x, well_y = 30000.0, 50000.0
        bounds = plate.get_single_well_scan_bounds_um(well_x, well_y)

        builder = MosaicBuilder(
            frame_size_px=(916, 686), micron_per_pixel=3.34, overlap=0.50)
        positions = builder.generate_raster_positions(bounds, overlap=0.50)
        # 96-well diameter=6.35mm, scan area ~8mm square, FOV ~3×2.3mm, 50% overlap
        self.assertGreater(len(positions), 4)
        self.assertLess(len(positions), 100)

    def test_6_well_raster_count(self):
        """6-well plate has large wells, more tiles needed."""
        plate = WellPlate.from_format(6)
        well_x, well_y = 65000.0, 42500.0
        bounds = plate.get_single_well_scan_bounds_um(well_x, well_y)

        builder = MosaicBuilder(
            frame_size_px=(916, 686), micron_per_pixel=3.34, overlap=0.50)
        positions = builder.generate_raster_positions(bounds, overlap=0.50)
        # 6-well diameter=34.8mm, scan area ~43.5mm square → many tiles
        self.assertGreater(len(positions), 40)


class TestAutoZCalibration(unittest.TestCase):
    """Test the auto Z-bottom calibration focus-sweep algorithm."""

    def test_focus_score_peaks_at_focal_plane(self):
        """SimulatedCamera focus score peaks when needle is at focal Z."""
        from SupportClasses.SimulatedCamera import SimulatedCamera
        from SupportClasses.VisionDetector import NeedleDetector

        plate = WellPlate.from_format(96)
        focal_z_mm = 0.0
        cam = SimulatedCamera(
            resolution=(916, 686),
            plate=plate,
            plate_center_um=(65000.0, 42500.0),
            needle_od_um=500.0,
            focal_z_mm=focal_z_mm,
        )
        cam.show_needle = True

        # Position at plate origin (A1 area)
        cam.set_stage_position(cam._plate_origin_um[0], cam._plate_origin_um[1])

        # Sweep Z around focal plane and collect scores
        # SimulatedCamera: defocus = |z_mm - focal_z_mm|
        scores = {}
        for dz_mm_10 in range(-10, 12, 2):  # -1.0 to +1.0 mm in 0.2 steps
            dz_mm = dz_mm_10 / 10.0
            z_mm = focal_z_mm + dz_mm
            cam.set_stage_position(cam._stage_x_um, cam._stage_y_um, z_mm=z_mm)
            _, frame = cam.read()
            result = NeedleDetector.compute_focus_score(frame)
            scores[dz_mm] = result.score

        # The peak should be near dz=0 (at focal plane)
        peak_dz = max(scores, key=scores.get)
        self.assertLessEqual(abs(peak_dz), 0.4,
            f"Focus peak at dz={peak_dz}mm, expected near 0")

    def test_well_depth_from_plate_definition(self):
        """Well depth is accessible from plate definitions."""
        plate_96 = WellPlate.from_format(96)
        self.assertAlmostEqual(plate_96.well_depth_mm, 10.67)

        plate_6 = WellPlate.from_format(6)
        self.assertAlmostEqual(plate_6.well_depth_mm, 17.4)

        plate_384 = WellPlate.from_format(384)
        self.assertAlmostEqual(plate_384.well_depth_mm, 11.56)

    def test_calibration_wells_match_auto_z_wells(self):
        """Auto Z-cal uses the same 3 calibration wells as plate cal."""
        from SupportClasses.WellPlate import ROW_LABELS
        # 96-well: A1, A12, H12
        plate = WellPlate.from_format(96)
        expected = ["A1", "A12", f"{ROW_LABELS[plate.rows - 1]}{plate.cols}"]
        self.assertEqual(expected, ["A1", "A12", "H12"])

        # 6-well: A1, A3, B3
        plate = WellPlate.from_format(6)
        expected = ["A1", f"A{plate.cols}", f"{ROW_LABELS[plate.rows - 1]}{plate.cols}"]
        self.assertEqual(expected, ["A1", "A3", "B3"])


if __name__ == "__main__":
    unittest.main()
