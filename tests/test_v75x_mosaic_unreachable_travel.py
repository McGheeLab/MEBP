"""v7.5.x — full-plate mosaic scan vs. a stage whose REAL XY travel is smaller
than the configured envelope.

Operator report: "the full plate mosaic is not able to get past a single row of
images". Root-caused from ``logs/app.log`` (2026-07-30 18:08): the whole-plate
raster region is the configured XY safety envelope (0..116327 µm in X), but the
stage physically stopped at X = 80593 µm. From that column on:

  * every ``move_xy_absolute_um`` never arrived,
  * ``wait_for_xy_arrival`` burned its full flat 10 s timeout per tile,
  * the worker ignored the result, grabbed a frame anyway and stitched a
    DUPLICATE image at the same canvas spot,
  * and the serpentine's next row started at the far (unreachable) end, so it
    ground through the same dead band again — the scan never visibly advanced.

These tests pin the new behaviour: measured-position arrival checking, a
distance-sized arrival timeout, runtime discovery of the reachable travel box,
and skipping (not re-commanding) the unreachable region.
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from SupportClasses.MosaicBuilder import MosaicBuilder  # noqa: E402
from gui.pages.calibration import _MosaicScanWorker  # noqa: E402


def _app():
    return QApplication.instance() or QApplication([])


class _Limits:
    max_xy_speed = 5945.6


class _ClampCtrl:
    """Stage stand-in whose X travel ends at ``x_limit`` µm.

    Mirrors the real failure: the move is accepted, the stage travels as far as
    it can and stops, and a fresh position read reports where it really is (Y
    still tracks the command exactly — which is how the log proved this was a
    genuine travel limit and not the known stale-``R``-ack read desync).
    """

    def __init__(self, x_limit=2200.0):
        self.x_limit = float(x_limit)
        self.moves = []
        self.arrival_timeouts = []
        self._last = (0.0, 0.0)
        self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        self.safety_limits = _Limits()
        self.poller_suspended = False

    def _apply(self, x, y):
        self._last = (min(float(x), self.x_limit), float(y))

    def safe_travel_to(self, x, y, safe_z_mm=None, target_z_mm=None,
                       apply_insert_floor=True):
        self.moves.append((x, y))
        self._apply(x, y)

    def move_xy_absolute_um(self, x, y, fast=False):
        self.moves.append((x, y))
        self._apply(x, y)

    def wait_for_xy_arrival(self, tx, ty, tolerance_mm=0.1, timeout_s=10.0):
        self.arrival_timeouts.append(timeout_s)
        return abs(self._last[0] / 1000.0 - tx) <= tolerance_mm

    def get_xy_position(self, cached=False):
        return (self._last[0], self._last[1], None)

    def suspend_position_poller(self):
        self.poller_suspended = True

    def resume_position_poller(self):
        self.poller_suspended = False


class _FakeCam:
    def __init__(self, frame):
        self._frame = frame
        self._c = 0

    def frame_count_value(self):
        self._c += 10
        return self._c

    def get_current_frame(self):
        return self._frame.copy()


def _build(bounds=(0, 0, 8000, 1500)):
    frame = np.full((60, 80, 3), 128, dtype=np.uint8)
    builder = MosaicBuilder(frame_size_px=(80, 60), micron_per_pixel=10.0,
                            overlap=0.25, target_mosaic_px=400)
    grid = builder.generate_raster_positions(bounds, overlap=0.25)
    return frame, builder, grid


def _run(ctrl, builder, grid, frame, **kw):
    worker = _MosaicScanWorker(ctrl, _FakeCam(frame), builder, grid,
                               safe_z=5.0, expected_d_px=40.0,
                               min_dist_px=20.0, **kw)
    tiles, progress, finished = [], [], []
    worker.tile.connect(lambda c, e: tiles.append(c))
    worker.progress.connect(lambda d, t: progress.append((d, t)))
    worker.finished_ok.connect(lambda *a: finished.append(a))
    worker.run()   # synchronous in-thread for the test
    return worker, tiles, progress, finished


class TestUnreachableTravelIsSkipped(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def test_scan_completes_and_skips_the_out_of_travel_columns(self):
        frame, builder, grid = _build()
        ctrl = _ClampCtrl(x_limit=2200.0)
        worker, tiles, progress, finished = _run(ctrl, builder, grid, frame)

        reachable = [(x, y) for (x, y) in grid if x <= 2200.0 + 1e-9]
        self.assertGreater(len(grid), len(reachable))   # grid over-reaches

        # It FINISHES (does not fail / hang) and reports the truncation.
        self.assertEqual(len(finished), 1)
        self.assertEqual(worker.unreachable_skipped,
                         len(grid) - len(reachable))
        self.assertTrue(worker.reach_note)

        # Only in-travel tiles are stitched — an unreached tile's frame shows
        # somewhere else, so blending it would stack a duplicate on the canvas.
        self.assertEqual(len(tiles), len(reachable))
        self.assertEqual(finished[0][3], len(reachable))   # frames

        # Progress still accounts for EVERY grid point (skips included), so the
        # operator's "n/total" counter reaches the end.
        self.assertEqual(len(progress), len(grid))
        self.assertEqual(progress[-1], (len(grid), len(grid)))

    def test_unreachable_points_are_not_re_commanded(self):
        """The whole point: 9 dead columns must not cost 9 arrival waits per
        row. After the stall threshold the region is skipped without moving."""
        frame, builder, grid = _build()
        ctrl = _ClampCtrl(x_limit=2200.0)
        worker, _tiles, _prog, _fin = _run(ctrl, builder, grid, frame)

        reachable = [(x, y) for (x, y) in grid if x <= 2200.0 + 1e-9]
        # Moves = every reachable point + at most one stall burst per row.
        self.assertLess(len(ctrl.moves), len(grid))
        self.assertGreaterEqual(len(ctrl.moves), len(reachable))
        self.assertLessEqual(
            len(ctrl.moves),
            len(reachable) + _MosaicScanWorker._MAX_CONSEC_STALL)

    def test_only_the_short_axis_is_clipped(self):
        """A stage short in +X keeps its full Y range — the reachable box is
        narrowed per axis and direction, never wholesale."""
        frame, builder, grid = _build()
        ctrl = _ClampCtrl(x_limit=2200.0)
        worker, _t, _p, _f = _run(ctrl, builder, grid, frame)

        self.assertAlmostEqual(worker._reach[2], 2200.0, places=3)  # max X
        self.assertEqual(worker._reach[0], float("-inf"))           # min X
        self.assertEqual(worker._reach[1], float("-inf"))           # min Y
        self.assertEqual(worker._reach[3], float("inf"))            # max Y
        # Every Y row still got scanned.
        self.assertEqual(len({round(y, 3) for (_x, y) in ctrl.moves}),
                         len({round(y, 3) for (_x, y) in grid}))

    def test_single_transient_miss_does_not_clip_the_grid(self):
        """One missed move (comms hiccup) must not permanently truncate the
        scan — the box is only narrowed after _MAX_CONSEC_STALL in a row."""
        frame, builder, grid = _build(bounds=(0, 0, 2000, 1500))

        class _OneMiss(_ClampCtrl):
            def __init__(self):
                super().__init__(x_limit=float("inf"))
                self._n = 0

            def _apply(self, x, y):
                self._n += 1
                if self._n == 2:          # second point lands short, once
                    self._last = (float(x) - 900.0, float(y))
                else:
                    self._last = (float(x), float(y))

        ctrl = _OneMiss()
        worker, tiles, _p, finished = _run(ctrl, builder, grid, frame)

        self.assertEqual(len(finished), 1)
        self.assertEqual(worker.unreachable_skipped, 1)   # that one tile only
        self.assertEqual(worker._reach[2], float("inf"))  # grid NOT clipped
        self.assertEqual(len(ctrl.moves), len(grid))      # all still commanded
        self.assertEqual(len(tiles), len(grid) - 1)

    def test_arrived_tile_is_stitched_even_when_the_wait_reports_timeout(self):
        """Regression guard for the stale-'R'-ack bug: wait_for_xy_arrival is
        known to time out spuriously while the stage HAS arrived, so arrival is
        judged from the measured position, never from that boolean."""
        frame, builder, grid = _build(bounds=(0, 0, 2000, 1500))

        class _LyingWait(_ClampCtrl):
            def __init__(self):
                super().__init__(x_limit=float("inf"))

            def wait_for_xy_arrival(self, *a, **k):
                return False              # always claims failure

        ctrl = _LyingWait()
        worker, tiles, _p, finished = _run(ctrl, builder, grid, frame)

        self.assertEqual(len(finished), 1)
        self.assertEqual(worker.unreachable_skipped, 0)
        self.assertEqual(len(tiles), len(grid))   # no good tile dropped


class TestArrivalTimeoutSizing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _app()

    def _worker(self, ctrl=None):
        frame, builder, grid = _build(bounds=(0, 0, 2000, 1500))
        return _MosaicScanWorker(ctrl or _ClampCtrl(), _FakeCam(frame),
                                 builder, grid, safe_z=0.0,
                                 expected_d_px=0.0, min_dist_px=1.0)

    def test_timeout_scales_with_distance_and_is_clamped(self):
        w = self._worker()
        short = w._arrival_timeout_s(0.0)
        step = w._arrival_timeout_s(3139.0)      # one real raster step
        far = w._arrival_timeout_s(10_000_000.0)
        self.assertEqual(short, _MosaicScanWorker._ARRIVE_TIMEOUT_MIN_S)
        self.assertLessEqual(step, 10.0)         # cheaper than the old flat 10 s
        self.assertGreaterEqual(step, short)
        self.assertEqual(far, _MosaicScanWorker._ARRIVE_TIMEOUT_MAX_S)

    def test_timeout_survives_a_controller_without_safety_limits(self):
        class _Bare(_ClampCtrl):
            def __init__(self):
                super().__init__()
                del self.safety_limits

        w = self._worker(_Bare())
        self.assertGreaterEqual(w._arrival_timeout_s(1000.0),
                                _MosaicScanWorker._ARRIVE_TIMEOUT_MIN_S)

    def test_worker_passes_a_sized_timeout_to_wait_for_xy_arrival(self):
        frame, builder, grid = _build(bounds=(0, 0, 2000, 1500))
        ctrl = _ClampCtrl(x_limit=float("inf"))
        _run(ctrl, builder, grid, frame)
        self.assertTrue(ctrl.arrival_timeouts)
        self.assertTrue(all(t <= _MosaicScanWorker._ARRIVE_TIMEOUT_MAX_S
                            for t in ctrl.arrival_timeouts))
        self.assertTrue(all(t >= _MosaicScanWorker._ARRIVE_TIMEOUT_MIN_S
                            for t in ctrl.arrival_timeouts))

    def test_failed_position_read_falls_back_to_the_commanded_target(self):
        """A garbled read must not be mistaken for a stall (that would skip a
        perfectly good tile) — it degrades to the legacy behaviour."""
        frame, builder, grid = _build(bounds=(0, 0, 2000, 1500))

        class _NoRead(_ClampCtrl):
            def get_xy_position(self, cached=False):
                return (None, None, None)

        ctrl = _NoRead(x_limit=float("inf"))
        worker, tiles, _p, finished = _run(ctrl, builder, grid, frame)
        self.assertEqual(len(finished), 1)
        self.assertEqual(worker.unreachable_skipped, 0)
        self.assertEqual(len(tiles), len(grid))


if __name__ == "__main__":
    unittest.main()
