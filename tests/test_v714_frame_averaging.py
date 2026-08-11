"""
test_v714_frame_averaging.py — average N frames per tile, when they agree.

Operator's request and its own condition: *"we should average the three frames
it grabs to get an even better image as long as the three frames look like
each other."*

THE HARD PART IS THE CONDITION, NOT THE AVERAGE
-----------------------------------------------
Consecutive frames NEVER look identical — shot noise guarantees a difference,
and at low signal the difference is ENTIRELY noise, which is exactly what
averaging removes. So a similarity test with an absolute threshold rejects
hardest where averaging helps most.

That is measured here, not asserted: ``test_absolute_difference_cannot_work``
reproduces the failed first design — dim noise scoring HIGHER than genuine
motion on a bright scene — so the reason for the scale-free statistic is
pinned rather than described in a comment that could rot.

The statistic that does work exploits the fact that block averaging suppresses
noise (1/B) but not structure. These tests drive it over the same range the
design was measured on: brightness 200..20000 counts, contrast 0.3..1.0,
frame sizes 64x64..2048x2048.
"""

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np

from SupportClasses import FrameAveraging as fa


# ── Synthetic imaging, matching the design measurements ───────────────

def _scene(h=256, w=256, contrast=1.0, level=2000.0, seed=0):
    """Cell-like blobs on a background — structure at a real spatial scale."""
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float64)
    img = np.zeros((h, w))
    for _ in range(12):
        cy, cx = rng.uniform(0, h), rng.uniform(0, w)
        r = rng.uniform(6, 30)
        img += np.exp(-(((yy - cy) ** 2 + (xx - cx) ** 2) / (2 * r * r)))
    img /= max(img.max(), 1e-9)
    return level * (0.15 + contrast * 0.85 * img)


def _shot(img, seed):
    """Poisson photon noise + Gaussian read noise — independent per frame."""
    rng = np.random.default_rng(seed)
    return (rng.poisson(np.clip(img, 0, None)).astype(np.float64)
            + rng.normal(0, 3.0, img.shape))


def _noise_only(base, n=3, seed0=100):
    return [_shot(base, seed0 + i) for i in range(n)]


def _with_shift(base, px, n=3, seed0=200):
    f = [_shot(base, seed0 + i) for i in range(n - 1)]
    f.append(_shot(np.roll(base, px, axis=1), seed0 + n))
    return f


# ── The statistic ─────────────────────────────────────────────────────

class TestStructureRatioIsScaleFree(unittest.TestCase):
    """The property the whole design rests on: ~1.0 for noise at ANY signal
    level and contrast, so one threshold works everywhere."""

    def test_noise_only_sits_near_one_across_the_range(self):
        worst = 1.0
        for level in (200.0, 2000.0, 20000.0):
            for contrast in (0.3, 1.0):
                base = _scene(contrast=contrast, level=level, seed=1)
                f = _noise_only(base)
                r = max(fa.structure_ratio(f[0], x) for x in f[1:])
                worst = max(worst, r)
                self.assertLess(
                    r, fa.DEFAULT_AGREEMENT_RATIO,
                    f"pure noise rejected at level={level} contrast={contrast}"
                    f" (ratio {r:.3f}) — averaging would be disabled exactly "
                    f"where it helps most")
        self.assertLess(worst, 1.3, f"noise floor drifted up to {worst:.3f}")

    def test_noise_floor_is_stable_at_every_frame_size(self):
        """Adaptive block sizing holds the statistic's SPREAD constant from a
        thumbnail to a full-resolution tile — the spread depends on how many
        blocks are averaged, so a fixed block size leaves a small frame with
        too few (measured 1.44 at 128x128/B=16, dangerously near the 1.5
        threshold).

        ⚠ Several seeds, worst case. A single seed passed even with a FIXED
        block size — the mutation that this test exists to catch — so one
        draw is not evidence about a distribution.
        """
        for (h, w) in ((512, 512), (256, 256), (128, 128), (64, 64)):
            worst = 1.0
            for k in range(8):
                base = _scene(h, w, contrast=0.6, level=2000.0, seed=2 + k)
                f = _noise_only(base, seed0=300 + 10 * k)
                worst = max(worst,
                            max(fa.structure_ratio(f[0], x) for x in f[1:]))
            self.assertLess(
                worst, 1.3,
                f"{h}x{w}: worst noise-only ratio {worst:.3f} over 8 draws — "
                f"too close to the {fa.DEFAULT_AGREEMENT_RATIO} threshold, so "
                f"real scans would randomly refuse to average")

    def test_motion_is_detected(self):
        base = _scene(contrast=1.0, level=2000.0, seed=3)
        for px in (1, 3, 8):
            f = _with_shift(base, px)
            r = max(fa.structure_ratio(f[0], x) for x in f[1:])
            self.assertGreater(r, fa.DEFAULT_AGREEMENT_RATIO,
                               f"{px} px shift not caught (ratio {r:.3f})")

    def test_brightness_step_is_detected(self):
        """A lamp flicker or an auto-exposure step — averaging across it is
        just as wrong as averaging across motion."""
        base = _scene(contrast=1.0, level=2000.0, seed=4)
        f = [_shot(base, 10), _shot(base, 11), _shot(base * 1.10, 12)]
        r = max(fa.structure_ratio(f[0], x) for x in f[1:])
        self.assertGreater(r, fa.DEFAULT_AGREEMENT_RATIO)

    def test_completely_different_scenes_peg_at_the_block_size(self):
        """The statistic's theoretical maximum is B, reached when the frames
        share no structure at all."""
        a = _shot(_scene(seed=5), 20)
        b = _shot(_scene(seed=6), 21)
        r = fa.structure_ratio(a, b)
        self.assertGreater(r, 3.0)
        self.assertLessEqual(r, fa.block_size_for(a.shape) + 0.01)

    def test_absolute_difference_cannot_work(self):
        """PINS WHY the scale-free form exists. A plain normalised difference
        scores dim NOISE higher than a real shift on a bright scene, so no
        fixed threshold can separate them. Measured, not assumed."""
        def naive(frames):
            ref = frames[0]
            b = fa.block_size_for(ref.shape)
            ds = [fa.block_mean(x, b) for x in frames]
            spread = max(np.percentile(ds[0], 95) - np.percentile(ds[0], 5),
                         1e-6)
            return max(float(np.mean(np.abs(d - ds[0]))) / spread
                       for d in ds[1:])

        dim = _scene(contrast=0.3, level=200.0, seed=7)
        bright = _scene(contrast=1.0, level=20000.0, seed=8)
        dim_noise = naive(_noise_only(dim, seed0=400))
        bright_motion = naive(_with_shift(bright, 3, seed0=500))
        self.assertGreater(
            dim_noise, bright_motion,
            "the naive metric no longer inverts — if this ever passes cleanly "
            "the scale-free statistic may be unnecessary, so re-derive rather "
            "than deleting it")
        # The real statistic gets the same two cases the right way round.
        self.assertLess(
            max(fa.structure_ratio(_noise_only(dim, seed0=400)[0], x)
                for x in _noise_only(dim, seed0=400)[1:]),
            max(fa.structure_ratio(_with_shift(bright, 3, seed0=500)[0], x)
                for x in _with_shift(bright, 3, seed0=500)[1:]))


class TestDegenerateInputs(unittest.TestCase):

    def test_flat_field_accepts_either_way(self):
        """No structure => motion is undetectable AND harmless."""
        flat = np.full((256, 256), 3000.0)
        self.assertLess(fa.structure_ratio(_shot(flat, 1), _shot(flat, 2)), 1.3)
        self.assertLess(
            fa.structure_ratio(_shot(flat, 1), _shot(np.roll(flat, 3, 1), 2)),
            1.3)

    def test_identical_buffers_do_not_divide_by_zero(self):
        a = np.zeros((32, 32))
        self.assertEqual(fa.structure_ratio(a, a), 1.0)

    def test_shape_change_is_maximal_disagreement_not_a_crash(self):
        a = np.zeros((32, 32))
        b = np.zeros((16, 16))
        self.assertGreater(fa.structure_ratio(a, b), 1.5)
        ok, _r, reason = fa.frames_agree([a, b])
        self.assertFalse(ok)
        self.assertIn("size", reason)

    def test_colour_frames_are_handled(self):
        base = _scene(seed=9)
        rgb = np.stack([_shot(base, 30)] * 3, axis=2)
        rgb2 = np.stack([_shot(base, 31)] * 3, axis=2)
        self.assertLess(fa.structure_ratio(rgb, rgb2), 1.3)


class TestFramesAgree(unittest.TestCase):

    def test_single_frame_or_empty_agrees_trivially(self):
        self.assertTrue(fa.frames_agree([])[0])
        self.assertTrue(fa.frames_agree([np.zeros((8, 8))])[0])
        self.assertTrue(fa.frames_agree(None)[0])

    def test_compares_against_the_first_not_the_previous(self):
        """A slow drift must not pass as three individually-small steps."""
        base = _scene(seed=10)
        drift = [_shot(base, 40),
                 _shot(np.roll(base, 2, axis=1), 41),
                 _shot(np.roll(base, 4, axis=1), 42)]
        ok, ratio, _r = fa.frames_agree(drift)
        self.assertFalse(ok)
        # vs the LAST frame's neighbour-step alone, which is only 2 px
        self.assertGreater(ratio, fa.structure_ratio(drift[1], drift[2]))

    def test_bad_threshold_does_not_raise(self):
        base = _scene(seed=11)
        for thr in (None, "x", float("nan")):
            fa.frames_agree(_noise_only(base), thr)


class TestAverageFrames(unittest.TestCase):

    def test_averaging_reduces_noise_by_sqrt_n(self):
        """The whole point. Measured against the noise-free scene."""
        base = _scene(contrast=1.0, level=2000.0, seed=12)
        frames = _noise_only(base, n=4, seed0=600)
        one = float(np.std(frames[0] - base))
        avg = float(np.std(fa.average_frames(frames) - base))
        self.assertLess(avg, one / 1.6,
                        f"4-frame average only improved {one/avg:.2f}x, "
                        f"expected ~2x")

    def test_dtype_and_range_preserved(self):
        u16 = [np.full((8, 8), v, np.uint16) for v in (100, 200, 300)]
        out = fa.average_frames(u16)
        self.assertEqual(out.dtype, np.uint16)
        self.assertEqual(int(out[0, 0]), 200)

    def test_uint8_accumulator_does_not_overflow(self):
        """A uint8 accumulator wraps after two bright frames; float64 does
        not. Three saturated frames must average to saturation, not to 0."""
        u8 = [np.full((8, 8), 250, np.uint8) for _ in range(3)]
        self.assertEqual(int(fa.average_frames(u8)[0, 0]), 250)

    def test_single_frame_is_returned_unchanged(self):
        a = np.zeros((4, 4), np.uint8)
        self.assertIs(fa.average_frames([a]), a)


class TestAverageIfAgreeing(unittest.TestCase):

    def test_agreeing_frames_are_averaged(self):
        base = _scene(seed=13)
        out, n, note = fa.average_if_agreeing(_noise_only(base, seed0=700))
        self.assertEqual(n, 3)
        self.assertEqual(note, "")

    def test_disagreeing_frames_fall_back_to_the_first(self):
        """NOT a refusal. One post-move frame is exactly what the scan produced
        before averaging existed, so the fallback can never be worse than the
        previous behaviour — whereas dropping the tile would make scans fail
        where they currently succeed."""
        base = _scene(seed=14)
        f = _with_shift(base, 8, seed0=800)
        out, n, note = fa.average_if_agreeing(f)
        self.assertEqual(n, 1)
        self.assertTrue(np.array_equal(out, f[0]))
        self.assertIn("disagree", note)


# ── Wiring: the raw path refuses, the plate worker averages ───────────

class TestRawAverageRequestGuard(unittest.TestCase):
    """The fluorescence path already averaged — BLINDLY. Now it refuses when
    the frames disagree, and its caller falls back to a single frame."""

    def _req(self, n=3, thr=None):
        from gui.widgets.mono_display import RawAverageRequest
        return RawAverageRequest(n, agree_threshold=thr)

    def _u16(self, a):
        return np.clip(a, 0, 65535).astype(np.uint16)

    def test_agreeing_frames_still_average(self):
        base = _scene(seed=15)
        req = self._req()
        for f in _noise_only(base, seed0=900):
            req.add(self._u16(f))
        self.assertTrue(req.done.is_set())
        self.assertIsNotNone(req.result())
        self.assertIsNone(req.error)

    def test_disagreeing_frames_are_refused_with_a_reason(self):
        base = _scene(seed=16)
        req = self._req()
        for f in _with_shift(base, 8, seed0=910):
            req.add(self._u16(f))
        self.assertIsNone(req.result())
        self.assertIn("disagree", req.error or "")

    def test_check_can_be_disabled(self):
        base = _scene(seed=17)
        req = self._req(thr=None)
        req.agree_threshold = None
        for f in _with_shift(base, 8, seed0=920):
            req.add(self._u16(f))
        self.assertIsNotNone(req.result())

    def test_shape_change_still_fails_first(self):
        req = self._req()
        req.add(np.zeros((16, 16), np.uint16))
        req.add(np.zeros((8, 8), np.uint16))
        self.assertIsNone(req.result())
        self.assertIn("shape", req.error or "")


class TestPlateWorkerAverages(unittest.TestCase):
    """The plate / rosette / single-well scan took ONE frame and ignored
    avg_frames entirely; only the fluorescence page honoured it."""

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    class _Cam:
        """Serves a scripted list of frames, one per counter tick."""

        def __init__(self, frames):
            self._frames = list(frames)
            self._i = 0
            self._n = 0

        def frame_count_value(self):
            n = self._n
            self._n += 1
            return n

        def get_hw_settings(self):
            return {"frame_rate": 30.0}

        def get_current_frame(self):
            f = self._frames[min(self._i, len(self._frames) - 1)]
            self._i += 1
            return f

    def _worker(self, cam, avg):
        from gui.pages.calibration import _MosaicScanWorker
        return _MosaicScanWorker(
            controller=object(), cam=cam, builder=object(), positions=[],
            safe_z=0.0, expected_d_px=10.0, min_dist_px=5.0,
            fresh_frames=1, fresh_timeout_s=0.5, settle_ms=0,
            avg_frames=avg)

    def test_avg_one_is_the_untouched_single_frame_path(self):
        f = [np.full((16, 16, 3), 10, np.uint8),
             np.full((16, 16, 3), 200, np.uint8)]
        got = self._worker(self._Cam(f), 1)._grab_post_move_frame()
        self.assertEqual(int(got[0, 0, 0]), 10,
                         "avg_frames=1 must return the first post-move frame "
                         "unchanged — the pre-v7.14 behaviour")

    def test_averaging_combines_frames(self):
        """⚠ The fixture matters. Three UNIFORM frames at 100/104/108 differ by
        a constant offset, which is pure structure — the guard correctly reads
        that as a brightness step and refuses (an earlier version of this test
        used exactly that and was caught by its own subject). Frames must
        differ only by spatially-uncorrelated noise, as real ones do."""
        rng = np.random.default_rng(99)
        base = np.full((64, 64, 3), 120.0)
        f = [np.clip(base + rng.normal(0, 6.0, base.shape), 0,
                     255).astype(np.uint8) for _ in range(3)]
        got = self._worker(self._Cam(f), 3)._grab_post_move_frame()
        expected = np.rint(np.mean([x.astype(np.float64) for x in f], axis=0))
        self.assertTrue(np.allclose(got.astype(np.float64), expected, atol=1),
                        "frames were not averaged")
        self.assertFalse(np.array_equal(got, f[0]),
                         "returned the first frame instead of the mean")
        # The mean must be closer to the true level than any single frame.
        self.assertLess(abs(float(got.mean()) - 120.0),
                        abs(float(f[0].mean()) - 120.0) + 1e-9)

    def test_moving_frames_fall_back_to_one(self):
        base = _scene(64, 64, seed=18)
        f = [np.stack([np.clip(x, 0, 255).astype(np.uint8)] * 3, axis=2)
             for x in _with_shift(base, 8, seed0=930)]
        got = self._worker(self._Cam(f), 3)._grab_post_move_frame()
        self.assertTrue(np.array_equal(got, f[0]),
                        "a blurred mean was stitched instead of one sharp frame")

    def test_worker_defaults_to_averaging_off(self):
        from gui.pages.calibration import _MosaicScanWorker
        w = _MosaicScanWorker(
            controller=object(), cam=object(), builder=object(), positions=[],
            safe_z=0.0, expected_d_px=1.0, min_dist_px=1.0)
        self.assertEqual(w._avg_frames, 1)

    def test_scan_passes_the_setting_through(self):
        """AST: the construction site must forward avg_frames from cfg, or the
        dialog control would be inert on these three scans."""
        import ast
        import inspect
        import gui.pages.calibration as cal
        tree = ast.parse(inspect.getsource(cal))
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                    and node.func.id == "_MosaicScanWorker"):
                if any(k.arg == "avg_frames" for k in node.keywords):
                    return
        self.fail("_MosaicScanWorker is constructed without avg_frames — the "
                  "'Average frames per tile' setting would do nothing")


if __name__ == "__main__":
    unittest.main()
