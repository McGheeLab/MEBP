"""
test_v713_signal_optimize.py — the one-shot "Optimize signal" routine.

Pure planning math (plan_auto_exposure_step, freeze_levels_from_frame) plus
the blocking driver (run_signal_optimize) against a fake manager whose signal
is linear in exposure — the physical model of a fluorescence scene. The
optimizer's contract: converge the raw P99.9 to ~70 % of full scale in a few
steps, then FREEZE the display (auto-scale off + fixed levels) so nothing
changes per frame afterwards.
"""

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np

from gui.widgets.mono_display import (
    LEVEL_MAX, freeze_levels_from_frame, plan_auto_exposure_step,
    run_signal_optimize)


CLIP = 65535


# ── Pure step function ────────────────────────────────────────────────

class TestPlanStep(unittest.TestCase):
    def _step(self, p, clipped=0.0, exp=30_000, lo=100, hi=30_000_000,
              clip=CLIP):
        return plan_auto_exposure_step(p, clipped, clip, exp, lo, hi)

    def test_converged_inside_tolerance(self):
        target = 0.70 * CLIP
        new, note = self._step(target * 1.03)
        self.assertIsNone(new)
        self.assertIn("converged", note)

    def test_scales_linearly_toward_target(self):
        # Signal at half the target → exposure doubles.
        new, _ = self._step(0.35 * CLIP, exp=30_000)
        self.assertAlmostEqual(new, 60_000, delta=600)

    def test_clipped_halves_never_scales(self):
        """A clipped frame is censored — linear scaling would undershoot and
        leave it clipped. The step must halve."""
        new, note = self._step(CLIP, clipped=0.30, exp=100_000)
        self.assertEqual(new, 50_000)
        self.assertIn("halving", note)

    def test_clipped_at_minimum_reports_reduce_illumination(self):
        new, note = self._step(CLIP, clipped=0.5, exp=100, lo=100)
        self.assertIsNone(new)
        self.assertIn("reduce illumination", note)

    def test_dim_at_max_reports_weak(self):
        new, note = self._step(0.05 * CLIP, exp=30_000_000, hi=30_000_000)
        self.assertIsNone(new)
        self.assertIn("weak", note)

    def test_dim_below_max_raises_to_max(self):
        new, note = self._step(0.001 * CLIP, exp=1_000_000, hi=30_000_000)
        self.assertEqual(new, 30_000_000)

    def test_zero_signal_quadruples(self):
        new, note = self._step(0.0, exp=10_000)
        self.assertEqual(new, 40_000)

    def test_marginal_clip_fraction_treated_as_clipped(self):
        # 0.2 % clipped — above the 0.1 % gate (mutation guard: > vs >=).
        new, note = self._step(0.9 * CLIP, clipped=0.002, exp=100_000)
        self.assertIn("halving", note)

    def test_12bit_clip_level_targets_4095_scale(self):
        # A 12-bit gain mode targets 70 % of 4095, not of 65535.
        new, _ = self._step(1433, clip=4095, exp=30_000)   # 0.35 × 4095
        self.assertAlmostEqual(new, 60_000, delta=600)


class TestFreezeLevels(unittest.TestCase):
    def test_normal_frame(self):
        rng = np.random.default_rng(1)
        frame = rng.integers(1000, 20000, size=(64, 64)).astype(np.uint16)
        lo, hi = freeze_levels_from_frame(frame)
        self.assertLess(lo, hi)
        self.assertLessEqual(hi, LEVEL_MAX)
        self.assertGreater(hi, 18000)          # headroom above P99.9

    def test_degenerate_flat_frame_widens(self):
        frame = np.full((32, 32), 500, dtype=np.uint16)
        lo, hi = freeze_levels_from_frame(frame)
        self.assertGreaterEqual(hi - lo, 200)  # never a zero-span mapping


# ── The blocking driver against a linear-scene fake manager ───────────

class _FakeMgr:
    """Signal ∝ exposure: p999 counts = k × exposure_us, clipped at clip.

    Frames: 99 % background at a tenth of the signal, top 1 % at the signal —
    so np.percentile(frame, 99.9) reads the bright population.
    """

    def __init__(self, k=1.0, exposure_us=30_000, exp_lo=100, exp_hi=200_000,
                 clip=CLIP, auto_exposure=None, no_frames=False):
        self.k = float(k)
        self.exposure_us = int(exposure_us)
        self.exp_lo, self.exp_hi = int(exp_lo), int(exp_hi)
        self.clip = int(clip)
        self.auto_exposure = auto_exposure
        self.no_frames = no_frames
        self.set_calls = []
        self.scale_calls = []

    def get_hw_settings(self, idx):
        return {"exposure_us": self.exposure_us,
                "exposure_range_us": (self.exp_lo, self.exp_hi, 30_000),
                "auto_exposure": self.auto_exposure}

    def set_hw_exposure_us(self, idx, us):
        self.exposure_us = max(self.exp_lo, min(self.exp_hi, int(us)))
        self.set_calls.append(self.exposure_us)
        return True

    def capture_raw_average(self, idx, n, timeout_s=10.0):
        if self.no_frames:
            return None
        signal = min(self.clip, self.k * self.exposure_us)
        frame = np.full((100, 100), signal / 10.0, dtype=np.float64)
        frame[:1, :] = signal                   # top 1 % = the signal
        return np.clip(np.rint(frame), 0, LEVEL_MAX).astype(np.uint16)

    def get_raw_frame_stats(self, idx):
        return {"clip_level": self.clip}

    def set_hw_andor_auto_scale(self, idx, v):
        self.scale_calls.append(("auto", v))
        return True

    def set_hw_andor_scale_lo(self, idx, v):
        self.scale_calls.append(("lo", int(v)))
        return True

    def set_hw_andor_scale_hi(self, idx, v):
        self.scale_calls.append(("hi", int(v)))
        return True


class TestRunSignalOptimize(unittest.TestCase):
    def test_converges_to_target(self):
        # k=1: target exposure ≈ 0.70×65535 ≈ 45874 µs, from 30 ms start.
        mgr = _FakeMgr(k=1.0)
        exp, note = run_signal_optimize(mgr, 0)
        self.assertIsNotNone(exp)
        signal = min(CLIP, mgr.k * mgr.exposure_us)
        frac = signal / CLIP
        self.assertGreater(frac, 0.70 * 0.90)
        self.assertLess(frac, 0.70 * 1.10)
        self.assertLessEqual(len(mgr.set_calls), 6)

    def test_saturated_start_halves_down_then_converges(self):
        # 60× over target: the frame starts clipped; the first steps must
        # HALVE (censored signal), then linear-scale in.
        mgr = _FakeMgr(k=100.0, exposure_us=60_000)
        exp, note = run_signal_optimize(mgr, 0, max_iters=12)
        self.assertIsNotNone(exp)
        # First adjustment was a halving of the start exposure.
        self.assertAlmostEqual(mgr.set_calls[1], mgr.set_calls[0] // 2,
                               delta=2)
        frac = min(CLIP, mgr.k * mgr.exposure_us) / CLIP
        self.assertLess(frac, 1.0)             # no longer clipped

    def test_dim_scene_pins_at_max_and_reports_weak(self):
        mgr = _FakeMgr(k=0.01)                  # max exposure → 2000 counts
        exp, note = run_signal_optimize(mgr, 0)
        self.assertEqual(exp, mgr.exp_hi)
        self.assertIn("weak", note)

    def test_freezes_display_after_convergence(self):
        mgr = _FakeMgr(k=1.0)
        run_signal_optimize(mgr, 0)
        kinds = [k for k, _v in mgr.scale_calls]
        self.assertEqual(kinds, ["auto", "lo", "hi"])   # auto OFF first
        self.assertEqual(mgr.scale_calls[0], ("auto", False))
        lo = dict(mgr.scale_calls)["lo"]
        hi = dict(mgr.scale_calls)["hi"]
        self.assertLess(lo, hi)
        # The white level brackets the converged signal with headroom.
        signal = min(CLIP, mgr.k * mgr.exposure_us)
        self.assertGreater(hi, signal * 0.95)

    def test_freeze_display_false_touches_no_scaling(self):
        mgr = _FakeMgr(k=1.0)
        exp, note = run_signal_optimize(mgr, 0, freeze_display=False)
        self.assertIsNotNone(exp)
        self.assertEqual(mgr.scale_calls, [])

    def test_refuses_when_hardware_auto_exposure_on(self):
        """A Tucsen with ATEXPOSURE enabled would fight the optimizer."""
        mgr = _FakeMgr(auto_exposure=True)
        exp, note = run_signal_optimize(mgr, 0)
        self.assertIsNone(exp)
        self.assertIn("auto-exposure", note)
        self.assertEqual(mgr.set_calls, [])     # nothing touched
        self.assertEqual(mgr.scale_calls, [])

    def test_no_frames_reports_cleanly(self):
        mgr = _FakeMgr(no_frames=True)
        exp, note = run_signal_optimize(mgr, 0)
        self.assertIsNone(exp)
        self.assertIn("no raw frames", note)
        self.assertEqual(mgr.scale_calls, [])   # never freezes blind

    def test_note_reports_the_numbers(self):
        mgr = _FakeMgr(k=1.0)
        _exp, note = run_signal_optimize(mgr, 0)
        self.assertIn("exposure", note)
        self.assertIn("% of clip", note)
        self.assertIn("frozen", note)

    def test_respects_12bit_clip(self):
        # 12-bit gain mode: convergence target is 70 % of 4095.
        mgr = _FakeMgr(k=0.5, clip=4095)
        exp, note = run_signal_optimize(mgr, 0)
        signal = min(4095, mgr.k * mgr.exposure_us)
        frac = signal / 4095.0
        self.assertGreater(frac, 0.60)
        self.assertLess(frac, 0.80)


# ── UI integration (offscreen) ────────────────────────────────────────

class TestDialogButton(unittest.TestCase):
    def test_optimize_button_gated_on_raw_stats(self):
        try:
            from PySide6.QtWidgets import QApplication
            _app = QApplication.instance() or QApplication(sys.argv)
            from gui.dialogs.camera_settings_dialog import CameraSettingsDialog
            from tests.test_v75x_andor_zyla_camera import _andor_widget
        except Exception as exc:  # pragma: no cover
            self.skipTest(f"camera stack unavailable: {exc}")
        mgr, _cam = _andor_widget()
        dlg = CameraSettingsDialog(mgr, 0)
        self.assertTrue(dlg._signal_group.isVisibleTo(dlg))
        self.assertTrue(dlg._opt_btn.isEnabled())

    def test_channel_prompt_auto_button_writes_spin(self):
        try:
            from PySide6.QtWidgets import QApplication
            _app = QApplication.instance() or QApplication(sys.argv)
            from gui.pages.workflows.fluorescence_mosaic_workflow import (
                _ChannelPromptDialog)
        except Exception as exc:  # pragma: no cover
            self.skipTest(f"fluorescence page unavailable: {exc}")
        applied = []
        dlg = _ChannelPromptDialog(
            "DAPI", "set filter", exposure_ms=20.0,
            on_apply_exposure=applied.append,
            camera_manager=_FakeMgr(), cam_idx=0)
        # The fake mgr has no hardware_capabilities → Auto hidden, but the
        # completion path must still write the found exposure into the spin.
        dlg._on_auto_done(45_874, "exposure 45.9 ms · converged")
        self.assertAlmostEqual(dlg.exposure_ms(), 45.874, places=2)
        self.assertIn("converged", dlg._auto_note.text())

    def test_channel_prompt_auto_visible_with_raw_stats_caps(self):
        try:
            from PySide6.QtWidgets import QApplication
            _app = QApplication.instance() or QApplication(sys.argv)
            from gui.pages.workflows.fluorescence_mosaic_workflow import (
                _ChannelPromptDialog)
        except Exception as exc:  # pragma: no cover
            self.skipTest(f"fluorescence page unavailable: {exc}")

        class _CapsMgr(_FakeMgr):
            def hardware_capabilities(self, idx):
                return {"controls": {"andor_raw_stats": True,
                                     "exposure_us": {"range": None}}}

        dlg = _ChannelPromptDialog("DAPI", "set filter",
                                   camera_manager=_CapsMgr(), cam_idx=0)
        self.assertTrue(dlg._auto_btn.isVisibleTo(dlg))
        dlg2 = _ChannelPromptDialog("DAPI", "set filter",
                                    camera_manager=None, cam_idx=None)
        self.assertFalse(dlg2._auto_btn.isVisibleTo(dlg2))


if __name__ == "__main__":
    unittest.main()
