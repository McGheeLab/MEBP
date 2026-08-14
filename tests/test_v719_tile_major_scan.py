"""
v7.19 — tile-major acquisition: every colour captured at each tile.

Operator: *"when i select multiple filter cubes at the top it should have two
modes in the mosaic full scan of each color or scan of each color per tile"*,
choosing per-tile as the default.

WHY THE SHARED REGISTRATION IS THE POINT. If each channel solved its own tile
positions and its own global shift, the channels would overlay each other no
better than when they are captured minutes apart — which is exactly what
tile-major costs a cube rotation per tile per colour to avoid. So the reference
channel is solved once and replayed.

AND WHY THE PRE-RUN GATE IS NOT OPTIONAL. A 400-tile well × 3 colours is 1200
cube changes; if one of them needs a human, the run cannot proceed unattended
and the operator must learn that BEFORE it starts, not at tile 1.
"""

from __future__ import annotations

import unittest
from types import SimpleNamespace

import numpy as np

from SupportClasses.MosaicBuilder import MosaicBuilder


def _img(h=40, w=60, v=120):
    a = np.full((h, w, 3), v, dtype=np.uint8)
    a[h // 3:2 * h // 3, w // 3:2 * w // 3] = 255      # something to register
    return a


def _builder():
    return MosaicBuilder(frame_size_px=(60, 40), micron_per_pixel=2.0,
                         retain_for_reorient=True, retain_frames=False)


class TestSharedRegistration(unittest.TestCase):
    """MosaicBuilder.apply_registration_from — the primitive tile-major needs."""

    def _pair(self):
        a, b = _builder(), _builder()
        for bld in (a, b):
            bld.generate_raster_positions((0.0, 0.0, 300.0, 200.0), overlap=0.3)
            for i, (x, y) in enumerate(
                    bld.generate_raster_positions((0.0, 0.0, 300.0, 200.0),
                                                  overlap=0.3)):
                bld.add_raster_frame(_img(), x, y, index=i)
            bld.stitch_incremental()
        return a, b

    def test_positions_and_shift_are_adopted(self):
        a, b = self._pair()
        opt = np.array([[float(i), float(i * 2)]
                        for i in range(len(a._reorient_tiles))])
        a._optimized_positions = opt
        a._global_shift_um = (12.5, -3.5)
        self.assertTrue(b.apply_registration_from(a))
        np.testing.assert_allclose(b._optimized_positions, opt)
        self.assertEqual(b._global_shift_um, (12.5, -3.5))

    def test_the_shift_travels_with_the_positions(self):
        """canvas_extent_um ADDS the global shift while tile PIXELS stay in the
        raw stage frame — so a channel that adopted the positions but not the
        shift would be offset in world coordinates despite matching pixels."""
        a, b = self._pair()
        a._optimized_positions = np.zeros((len(a._reorient_tiles), 2))
        a._global_shift_um = (40.0, 40.0)
        b.apply_registration_from(a)
        self.assertEqual(b.canvas_extent_um[0], a.canvas_extent_um[0])
        self.assertEqual(b.canvas_extent_um[1], a.canvas_extent_um[1])

    def test_a_reference_that_did_not_solve_shares_only_the_shift(self):
        """Too few confident overlaps → both keep the trusted stage placement,
        which is the correct degradation, not an error."""
        a, b = self._pair()
        a._optimized_positions = None
        a._global_shift_um = (5.0, 6.0)
        self.assertFalse(b.apply_registration_from(a))
        self.assertEqual(b._global_shift_um, (5.0, 6.0))

    def test_mismatched_tile_counts_are_REFUSED(self):
        """The only way the shared-grid assumption could be violated."""
        a, b = self._pair()
        a._optimized_positions = np.zeros((len(a._reorient_tiles) + 3, 2))
        before = list(b._reorient_tiles)
        self.assertFalse(b.apply_registration_from(a))
        self.assertEqual(len(b._reorient_tiles), len(before))

    def test_it_is_public_so_the_workflow_never_touches_the_private_one(self):
        self.assertTrue(hasattr(MosaicBuilder, "apply_registration_from"))
        import ast
        import inspect
        import textwrap
        from gui.pages.workflows import fluorescence_mosaic_workflow as m
        src = textwrap.dedent(inspect.getsource(m))
        names = {getattr(n.func, "attr", "") for n in ast.walk(ast.parse(src))
                 if isinstance(n, ast.Call)}
        self.assertNotIn("_reblend_at_positions", names)


class _FakeCam:
    def __init__(self):
        self.exposures = []
        self.gains = []

    def get_current_frame(self):
        return _img()

    def capture_fresh_frame(self, **_k):
        return _img()

    def capture_raw_average(self, n, **_k):
        return np.full((40, 60), 800, dtype=np.uint16)

    def frame_count_value(self):
        return 999

    def set_hw_exposure_us(self, v):
        self.exposures.append(v)

    def set_hw_exposure_gain(self, v):
        self.gains.append(v)


class _FakeCtrl:
    def suspend_position_poller(self):
        pass

    def resume_position_poller(self):
        pass

    def get_xy_position(self, cached=False):
        return (100.0, 200.0, 0.0)

    def wait_for_xy_arrival(self, *a, **k):
        return True

    def move_xy_absolute_um(self, *a, **k):
        return True

    def safe_travel_to(self, *a, **k):
        return True


class TestTheWorkerVisitsEachTileOnce(unittest.TestCase):
    """Drives the REAL worker's tile-major loop with fake hardware."""

    def _worker(self, channels, ensure_ok=True, why="slot empty"):
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            ChannelPlan, _SingleWellMosaicWorker)
        cam = _FakeCam()
        plans = []
        for ch in channels:
            b = _builder()
            positions = b.generate_raster_positions(
                (0.0, 0.0, 200.0, 150.0), overlap=0.3)
            plans.append(ChannelPlan(channel=ch, builder=b,
                                     exposure_us=1000.0 * (len(plans) + 1),
                                     gain_pct=10.0, avg_frames=1,
                                     levels=(0.0, 4095.0)))
        w = _SingleWellMosaicWorker(
            _FakeCtrl(), cam, plans[0].builder, positions, 10.0,
            channels=plans, settle_ms=0, fresh_frames=0)
        self.switches = []

        class _Svc:
            def __init__(self, outer):
                self._o = outer

            def ensure_filter(self, name):
                self._o.switches.append(name)
                return SimpleNamespace(ok=ensure_ok, why_not=why,
                                       simulated=False,
                                       describe=lambda: "ok")

        w._optics = lambda: _Svc(self)
        w._wait_settled = lambda: True
        return w, cam, plans

    def test_every_channel_is_captured_at_every_tile(self):
        w, _cam, plans = self._worker(["DAPI", "FITC", "Cy5"])
        n = len(w._positions)
        w._scan_tile_major()
        for p in plans:
            self.assertEqual(p.builder.frame_count, n,
                             f"{p.channel} got {p.builder.frame_count}/{n}")
        # One cube switch per channel per tile (plus the probe's, if any).
        self.assertGreaterEqual(len(self.switches), n * 3)

    def test_each_channels_own_recipe_is_applied(self):
        w, cam, _plans = self._worker(["DAPI", "FITC"])
        w._scan_tile_major()
        # Two distinct exposures, alternating — not one shared value.
        self.assertEqual(set(cam.exposures), {1000, 2000})

    def test_a_refused_cube_ABORTS_rather_than_capturing_through_the_wrong_one(self):
        """A channel silently captured through the wrong cube is a result
        nothing downstream can detect."""
        w, _cam, plans = self._worker(["DAPI"], ensure_ok=False)
        failures = []
        w.failed.connect(failures.append)
        w._scan_tile_major()
        self.assertTrue(failures)
        self.assertIn("slot empty", failures[0])
        self.assertEqual(plans[0].builder.frame_count, 0)

    def test_stop_is_honoured_mid_run(self):
        w, _cam, plans = self._worker(["DAPI", "FITC"])
        w._stop = True
        w._scan_tile_major()
        self.assertEqual(plans[0].builder.frame_count, 0)

    def test_the_reference_channel_is_solved_and_the_rest_replay_it(self):
        w, _cam, plans = self._worker(["DAPI", "FITC", "Cy5"])
        shared = []
        for p in plans[1:]:
            p.builder.apply_registration_from = (
                lambda ref, _p=p: shared.append(_p.channel) or True)
        w._scan_tile_major()
        self.assertEqual(shared, ["FITC", "Cy5"])

    def test_every_channel_is_emitted_then_finished_all(self):
        w, _cam, _plans = self._worker(["DAPI", "FITC"])
        done, ended = [], []
        w.channel_done.connect(lambda ch, *a: done.append(ch))
        w.finished_all.connect(lambda: ended.append(True))
        w._scan_tile_major()
        self.assertEqual(done, ["DAPI", "FITC"])
        self.assertEqual(ended, [True])

    def test_accumulators_are_freed_as_each_channel_is_handed_over(self):
        """Tile-major holds N composites at once (~200 MB each at 2500 px)."""
        w, _cam, plans = self._worker(["DAPI", "FITC"])
        freed = []
        for p in plans:
            p.builder.free_accumulators = (
                lambda _p=p: freed.append(_p.channel))
        w._scan_tile_major()
        self.assertEqual(sorted(freed), ["DAPI", "FITC"])

    def test_channel_major_is_untouched_when_no_channels_are_given(self):
        """The legacy single-channel path must stay byte-identical."""
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            _SingleWellMosaicWorker)
        b = _builder()
        pos = b.generate_raster_positions((0.0, 0.0, 200.0, 150.0), overlap=0.3)
        w = _SingleWellMosaicWorker(_FakeCtrl(), _FakeCam(), b, pos, 10.0)
        self.assertEqual(w._channels, [])


class TestThePreRunGate(unittest.TestCase):
    """Tile-major must refuse BY NAME rather than stall at tile 1."""

    def setUp(self):
        try:
            from PySide6.QtWidgets import QApplication
        except Exception:                                    # pragma: no cover
            self.skipTest("PySide6 not available")
        QApplication.instance() or QApplication([])

    def _page(self):
        import os
        import tempfile
        os.environ.setdefault("MEBP_WORKFLOW_SETTINGS_DIR", tempfile.mkdtemp())

        class SL:
            xy_min_x = xy_min_y = 0.0
            xy_max_x, xy_max_y = 120000.0, 80000.0

        class Ctrl:
            safety_limits = SL()
            is_zp_connected = False
            zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            FluorescenceMosaicWorkflowPage)
        return FluorescenceMosaicWorkflowPage(
            controller=Ctrl(), settings=object(), camera_manager=None)

    def test_no_cassette_blocks_tile_major(self):
        pg = self._page()
        pg._scope_state = lambda: SimpleNamespace(connected=False,
                                                  has_filter=False)
        blockers = pg._tile_major_blockers(["DAPI"])
        self.assertTrue(blockers)
        self.assertIn("cassette", blockers[0])

    def test_an_unresolvable_channel_is_named(self):
        pg = self._page()
        pg._scope_state = lambda: SimpleNamespace(connected=True,
                                                  has_filter=True)
        pg._cube_slots = lambda st: ({"DAPI": 1},
                                     {"mCherry": "no cube named mCherry"})
        blockers = pg._tile_major_blockers(["DAPI", "mCherry"])
        self.assertEqual(len(blockers), 1)
        self.assertIn("mCherry", blockers[0])
        self.assertIn("no cube named mCherry", blockers[0])

    def test_all_resolvable_channels_pass(self):
        pg = self._page()
        pg._scope_state = lambda: SimpleNamespace(connected=True,
                                                  has_filter=True)
        pg._cube_slots = lambda st: ({"DAPI": 1, "FITC": 2}, {})
        self.assertEqual(pg._tile_major_blockers(["DAPI", "FITC"]), [])

    def test_the_confirm_prices_rotations_time_and_memory(self):
        pg = self._page()
        plan = {"grid": [(0, 0)] * 400, "cols": 20, "rows": 20,
                "target_px": 2500}
        body, tile_major = pg._describe_run_cost(plan, ["DAPI", "FITC", "Cy5"],
                                                 "tile")
        self.assertTrue(tile_major)
        self.assertIn("1200 cube rotations", body)
        self.assertIn("MB", body)
        self.assertIn("min", body)

    def test_channel_major_is_priced_differently_and_says_why(self):
        pg = self._page()
        plan = {"grid": [(0, 0)] * 400, "cols": 20, "rows": 20,
                "target_px": 2500}
        body, tile_major = pg._describe_run_cost(plan, ["DAPI", "FITC"],
                                                 "channel")
        self.assertFalse(tile_major)
        self.assertNotIn("cube rotations", body)
        self.assertIn("minutes after", body)

    def test_the_default_order_is_per_tile(self):
        self.assertEqual(self._page().scan_order(), "tile")


if __name__ == "__main__":
    unittest.main()
