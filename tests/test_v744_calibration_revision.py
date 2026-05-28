"""
test_v744_calibration_revision.py — v7.4.4 calibration revision tests.

Covers the foundation pieces of the workflow-tab refactor:

* `CameraRole` enum + `HardwareConfig.camera_roles` persistence and
  lookup helpers.
* `TwoCameraNeedleAligner` — converting 4 edge-click pixels per
  side-camera into a stage `(dx_um, dy_um)` recenter offset.
* `EdgeFitWellLocator` — fitting a known-radius circle to a partial
  arc at multiple simulated objectives.

No GUI or hardware required.
"""

from __future__ import annotations

import math
import unittest

import cv2
import numpy as np

from SupportClasses.HardwareConfig import (
    HardwareConfig,
    CameraRole,
    MAX_LIVE_CAMERAS,
)
from SupportClasses.VisionDetector import (
    TwoCameraNeedleAligner,
    TwoCameraEdgePicks,
    EdgeFitWellLocator,
)


# ---------------------------------------------------------------------------
# CameraRole + HardwareConfig persistence
# ---------------------------------------------------------------------------


class TestCameraRolePersistence(unittest.TestCase):
    def test_defaults_to_all_unassigned(self):
        cfg = HardwareConfig()
        self.assertEqual(len(cfg.camera_roles), MAX_LIVE_CAMERAS)
        for r in cfg.camera_roles:
            self.assertEqual(r, CameraRole.UNASSIGNED)

    def test_set_camera_role_in_range(self):
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.NEEDLE_X)
        cfg.set_camera_role(1, CameraRole.NEEDLE_Y)
        cfg.set_camera_role(2, CameraRole.PLATE)
        self.assertEqual(cfg.camera_roles[0], CameraRole.NEEDLE_X)
        self.assertEqual(cfg.camera_roles[1], CameraRole.NEEDLE_Y)
        self.assertEqual(cfg.camera_roles[2], CameraRole.PLATE)

    def test_set_camera_role_out_of_range_silently_ignored(self):
        cfg = HardwareConfig()
        # Should not raise.
        cfg.set_camera_role(99, CameraRole.PLATE)
        for r in cfg.camera_roles:
            self.assertEqual(r, CameraRole.UNASSIGNED)

    def test_camera_for_role_returns_first_match(self):
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.NEEDLE_X)
        cfg.set_camera_role(2, CameraRole.PLATE)
        self.assertEqual(cfg.camera_for_role(CameraRole.NEEDLE_X), 0)
        self.assertEqual(cfg.camera_for_role(CameraRole.PLATE), 2)
        self.assertIsNone(cfg.camera_for_role(CameraRole.NEEDLE_Y))

    def test_roundtrip_to_dict_from_dict(self):
        cfg = HardwareConfig()
        cfg.set_camera_role(0, CameraRole.NEEDLE_X)
        cfg.set_camera_role(1, CameraRole.NEEDLE_Y)
        cfg.set_camera_role(2, CameraRole.PLATE)
        data = cfg.to_dict()
        self.assertEqual(
            data["camera_roles"], ["needle_x", "needle_y", "plate"]
        )
        cfg2 = HardwareConfig.from_dict(data)
        self.assertEqual(cfg2.camera_roles, cfg.camera_roles)

    def test_legacy_config_without_roles_migrates_to_unassigned(self):
        # Simulate a v7.4.3-and-earlier config dict (no `camera_roles`).
        legacy = {"config_name": "legacy"}
        cfg = HardwareConfig.from_dict(legacy)
        self.assertEqual(len(cfg.camera_roles), MAX_LIVE_CAMERAS)
        for r in cfg.camera_roles:
            self.assertEqual(r, CameraRole.UNASSIGNED)

    def test_unknown_role_string_falls_back_to_unassigned(self):
        data = {
            "config_name": "weird",
            "camera_roles": ["needle_x", "made_up", "plate"],
        }
        cfg = HardwareConfig.from_dict(data)
        self.assertEqual(cfg.camera_roles[0], CameraRole.NEEDLE_X)
        self.assertEqual(cfg.camera_roles[1], CameraRole.UNASSIGNED)
        self.assertEqual(cfg.camera_roles[2], CameraRole.PLATE)


# ---------------------------------------------------------------------------
# TwoCameraNeedleAligner — Needle Location workflow math
# ---------------------------------------------------------------------------


class TestTwoCameraNeedleAligner(unittest.TestCase):
    def test_centered_picks_yield_zero_offset(self):
        a = TwoCameraNeedleAligner(1.0, 1.0, 100, 100)
        picks = TwoCameraEdgePicks(45.0, 55.0, 45.0, 55.0)  # midpoint = 50
        dx, dy = a.offset_from_edge_clicks(picks)
        self.assertAlmostEqual(dx, 0.0, places=6)
        self.assertAlmostEqual(dy, 0.0, places=6)

    def test_symmetric_offset_uses_um_per_px_scale(self):
        # Midpoint 60 in both views = 10 px right of center; um/px = 2 →
        # expect (20, 20).
        a = TwoCameraNeedleAligner(2.0, 2.0, 100, 100)
        picks = TwoCameraEdgePicks(55.0, 65.0, 55.0, 65.0)
        dx, dy = a.offset_from_edge_clicks(picks)
        self.assertAlmostEqual(dx, 20.0, places=6)
        self.assertAlmostEqual(dy, 20.0, places=6)

    def test_asymmetric_offset_per_axis(self):
        # X-view: midpoint 35, frame 100, um/px 1.5 → x_view col offset
        # = 35 - 50 = -15 px → dy_um = -15 * 1.5 = -22.5
        # Y-view: midpoint 70, frame 100, um/px 3.0 → y_view col offset
        # = 70 - 50 = 20 px → dx_um = 20 * 3.0 = 60.0
        a = TwoCameraNeedleAligner(1.5, 3.0, 100, 100)
        picks = TwoCameraEdgePicks(30.0, 40.0, 60.0, 80.0)
        dx, dy = a.offset_from_edge_clicks(picks)
        self.assertAlmostEqual(dx, 60.0, places=6)
        self.assertAlmostEqual(dy, -22.5, places=6)

    def test_sign_flip_via_constructor(self):
        # Same picks as previous test but flipping x_sign should
        # invert dx; flipping y_sign should invert dy.
        a = TwoCameraNeedleAligner(
            1.5, 3.0, 100, 100, x_sign=-1.0, y_sign=-1.0
        )
        picks = TwoCameraEdgePicks(30.0, 40.0, 60.0, 80.0)
        dx, dy = a.offset_from_edge_clicks(picks)
        self.assertAlmostEqual(dx, -60.0, places=6)
        self.assertAlmostEqual(dy, 22.5, places=6)

    def test_missing_pick_raises(self):
        a = TwoCameraNeedleAligner(1.0, 1.0, 100, 100)
        picks = TwoCameraEdgePicks(45.0, 55.0, None, None)
        with self.assertRaises(ValueError):
            a.offset_from_edge_clicks(picks)

    def test_invalid_um_per_px_rejected(self):
        with self.assertRaises(ValueError):
            TwoCameraNeedleAligner(0.0, 1.0, 100, 100)
        with self.assertRaises(ValueError):
            TwoCameraNeedleAligner(1.0, -1.0, 100, 100)


# ---------------------------------------------------------------------------
# EdgeFitWellLocator — Plate Location workflow math
# ---------------------------------------------------------------------------


def _make_circle_frame(
    size: int, cx: int, cy: int, r: int, arc_deg: float = 360.0
) -> np.ndarray:
    frame = np.zeros((size, size, 3), dtype=np.uint8)
    if arc_deg >= 360.0:
        cv2.circle(frame, (cx, cy), r, (255, 255, 255), 2)
    else:
        # Half/quarter arc — keeps the lower half of the rim only.
        cv2.ellipse(
            frame,
            (cx, cy),
            (r, r),
            angle=0,
            startAngle=0,
            endAngle=int(arc_deg),
            color=(255, 255, 255),
            thickness=2,
        )
    return frame


class TestEdgeFitWellLocator(unittest.TestCase):
    def test_full_circle_recovers_center(self):
        frame = _make_circle_frame(400, 200, 200, 80)
        loc = EdgeFitWellLocator()
        result = loc.fit_partial_arc(
            frame, expected_radius_px=80.0, um_per_px=1.67
        )
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.center_px[0], 200.0, delta=2.0)
        self.assertAlmostEqual(result.center_px[1], 200.0, delta=2.0)
        # Confidence should be high on a clean synthetic circle.
        self.assertGreater(result.confidence, 0.8)
        # The fitter snaps the reported radius to the supplied value.
        self.assertAlmostEqual(result.radius_px, 80.0, places=6)

    def test_partial_arc_recovers_center(self):
        # ~180° arc (half circle) still recovers center within a few px.
        frame = _make_circle_frame(400, 200, 200, 80, arc_deg=200.0)
        loc = EdgeFitWellLocator()
        result = loc.fit_partial_arc(
            frame, expected_radius_px=80.0, um_per_px=1.67
        )
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.center_px[0], 200.0, delta=3.0)
        self.assertAlmostEqual(result.center_px[1], 200.0, delta=3.0)

    def test_wrong_radius_fails_band_check(self):
        # Circle radius is 40 but caller insists on 80 ± 10%.
        frame = _make_circle_frame(400, 200, 200, 40)
        loc = EdgeFitWellLocator()
        result = loc.fit_partial_arc(
            frame, expected_radius_px=80.0, um_per_px=1.67
        )
        self.assertIsNone(result)

    def test_um_conversion_uses_supplied_scale(self):
        frame = _make_circle_frame(400, 200, 200, 80)
        loc = EdgeFitWellLocator()
        result = loc.fit_partial_arc(
            frame, expected_radius_px=80.0, um_per_px=2.5
        )
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.radius_um, 80.0 * 2.5, places=6)

    def test_no_edges_returns_none(self):
        frame = np.zeros((400, 400, 3), dtype=np.uint8)
        loc = EdgeFitWellLocator()
        result = loc.fit_partial_arc(
            frame, expected_radius_px=80.0, um_per_px=1.67
        )
        self.assertIsNone(result)

    def test_invalid_inputs_return_none(self):
        frame = _make_circle_frame(400, 200, 200, 80)
        loc = EdgeFitWellLocator()
        self.assertIsNone(
            loc.fit_partial_arc(frame, expected_radius_px=0, um_per_px=1.67)
        )
        self.assertIsNone(
            loc.fit_partial_arc(frame, expected_radius_px=80, um_per_px=0)
        )


if __name__ == "__main__":
    unittest.main()
