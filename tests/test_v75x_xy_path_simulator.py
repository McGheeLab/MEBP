"""Tests for SupportClasses/XYPathSimulator.py (v7.5.x) — the shared
follow_path loop, the offline simulation, fail-region mapping, the geometry
panel, and the sketch/trajectory printability check.
"""

import math
import threading
import unittest

from SupportClasses import XYChallenge as XC
from SupportClasses import XYPathSimulator as PS
from SupportClasses.XYStageModel import StageCharacteristics, XYStageModel


ME3B = StageCharacteristics(dead_time_s=0.067, tau_s=0.027,
                            top_speed_um_s=5945.6, control_loop_ms=31.75,
                            quant_um=1.0, name="ME3B V1")

FAST = StageCharacteristics(dead_time_s=0.002, tau_s=0.001,
                            top_speed_um_s=50000.0, control_loop_ms=10.0,
                            quant_um=0.1)


def _line(length_mm=10.0, step=0.5):
    n = int(length_mm / step)
    return [(i * step, 0.0) for i in range(n + 1)]


class TestFollowPathLoop(unittest.TestCase):
    def test_near_ideal_stage_tracks_a_line_tightly(self):
        r = PS.simulate_follow(_line(), char=FAST, print_speed_mm_s=3.0,
                               tuning={"hold_speed": 1.0})
        self.assertTrue(r.completed)
        self.assertLess(r.report["rms_um"], 15.0)
        self.assertEqual(r.fail_regions, [])
        self.assertEqual(r.verdict, "pass")

    def test_always_stops_the_stage(self):
        """Every exit path must end with a (0, 0) command."""
        sent = []
        model = XYStageModel(FAST)
        io = PS.model_io(model)
        orig = io.send_um_s

        def _send(vx, vy):
            sent.append((vx, vy))
            orig(vx, vy)
        io.send_um_s = _send
        resolved = PS.resolve_for(FAST, print_speed_mm_s=3.0, tuning={})
        PS.follow_path(_line(2.0), io, print_speed_mm_s=3.0, tuning={},
                       resolved=resolved)
        self.assertEqual(sent[-1], (0.0, 0.0))

    def test_stop_event_aborts_with_reason(self):
        stop = threading.Event()
        stop.set()
        model = XYStageModel(FAST)
        resolved = PS.resolve_for(FAST, print_speed_mm_s=3.0, tuning={})
        samples, info = PS.follow_path(_line(), PS.model_io(model),
                                       print_speed_mm_s=3.0, tuning={},
                                       resolved=resolved, stop=stop)
        self.assertEqual(info["stopped_reason"], "stopped")
        self.assertFalse(info["completed"])

    def test_wall_cap_reason(self):
        samples, info = PS.follow_path(
            _line(), PS.model_io(XYStageModel(FAST, x_um=0, y_um=0)),
            print_speed_mm_s=3.0, tuning={},
            resolved=PS.resolve_for(FAST, print_speed_mm_s=3.0, tuning={}),
            max_wall_s=0.05)
        self.assertEqual(info["stopped_reason"], "wall_cap")

    def test_deterministic(self):
        a = PS.simulate_follow(XC.make_shape("Circle", 4.0, 0.5), char=ME3B,
                               print_speed_mm_s=3.0)
        b = PS.simulate_follow(XC.make_shape("Circle", 4.0, 0.5), char=ME3B,
                               print_speed_mm_s=3.0)
        self.assertEqual(a.samples, b.samples)
        self.assertEqual(a.report["rms_um"], b.report["rms_um"])

    def test_me3b_circle_completes_with_plausible_error(self):
        """The saved ME3B dynamics at the hardware-verified operating point:
        completion 1.0, no dithering, deviation in the same regime as the real
        trace (hardware measured p95 84 µm / rms 74 µm)."""
        r = PS.simulate_follow(XC.make_shape("Circle", 5.0, 0.5), char=ME3B,
                               print_speed_mm_s=3.0,
                               tuning={"lookahead": 0.9, "hold_speed": 1.0,
                                       "max_speed_frac": 0.9})
        self.assertTrue(r.completed)
        self.assertGreaterEqual(r.report["completion_frac"], 0.99)
        self.assertLess(r.report["dither_ratio"], 1.2)
        self.assertLess(r.report["p95_um"], 400.0)
        self.assertGreater(r.report["p95_um"], 5.0)   # a real stage isn't perfect

    def test_corner_cut_scales_with_lookahead(self):
        """Pure pursuit cuts corners in proportion to the lookahead — the
        dominant error term identified on hardware."""
        star = XC.make_shape("Star", 5.0, 0.5)
        small = PS.simulate_follow(star, char=ME3B, print_speed_mm_s=1.0,
                                   tuning={"lookahead": 0.3,
                                           "hold_speed": 1.0})
        large = PS.simulate_follow(star, char=ME3B, print_speed_mm_s=1.0,
                                   tuning={"lookahead": 0.5,
                                           "hold_speed": 1.0})
        self.assertTrue(small.completed)
        self.assertTrue(large.completed)
        self.assertLess(small.report["p95_um"], large.report["p95_um"])

    def test_oversized_lookahead_on_small_feature_predicted_to_fail(self):
        """A lookahead comparable to the feature size at low speed makes the
        follower orbit a star spike and stall (the projection window
        ``max(0.15, speed·dt·6)`` shrinks at low speed while the carrot pulls
        the stage deep inside the notch). The simulator must surface that as a
        FAILED run — not score the orbiting prefix as if it printed."""
        star = XC.make_shape("Star", 5.0, 0.5)
        r = PS.simulate_follow(star, char=ME3B, print_speed_mm_s=1.0,
                               tuning={"lookahead": 0.8, "hold_speed": 1.0})
        self.assertFalse(r.completed)
        self.assertEqual(r.verdict, "fail")
        self.assertFalse(r.score["pass"])
        self.assertIn("incomplete", r.score["fail_reasons"])

    def test_matches_hardware_verified_operating_point(self):
        """The exact configuration verified on ME3B V1 (2026-07-28): Star and
        Circle at 3 mm/s, lookahead 0.892, hold_speed, max_speed_frac 0.9.
        Hardware measured Star p95 364 µm / Circle p95 84 µm, both complete —
        the model must land in the same regime (loose band: same order, not a
        pinned number; the real stage adds encoder noise and comms jitter)."""
        tune = {"lookahead": 0.892, "hold_speed": 1.0, "max_speed_frac": 0.9}
        star = PS.simulate_follow(XC.make_shape("Star", 5.0, 0.5), char=ME3B,
                                  print_speed_mm_s=3.0, tuning=tune)
        circ = PS.simulate_follow(XC.make_shape("Circle", 5.0, 0.5),
                                  char=ME3B, print_speed_mm_s=3.0, tuning=tune)
        self.assertTrue(star.completed)
        self.assertTrue(circ.completed)
        self.assertLess(abs(star.report["p95_um"] - 364.0), 200.0)
        self.assertLess(abs(circ.report["p95_um"] - 84.0), 60.0)

    def test_tuning_changes_prediction(self):
        """The what-if lever: two tunings on the same saved machine give
        different predicted reports."""
        sq = XC.make_shape("Square", 5.0, 0.5)
        slow = PS.simulate_follow(sq, char=ME3B, print_speed_mm_s=3.0,
                                  tuning={"lookahead": 0.2})     # legacy cap
        fast = PS.simulate_follow(sq, char=ME3B, print_speed_mm_s=3.0,
                                  tuning={"lookahead": 0.9, "hold_speed": 1.0})
        self.assertNotEqual(slow.resolved["speed_cap_mm_s"],
                            fast.resolved["speed_cap_mm_s"])
        self.assertGreater(slow.wall_s, fast.wall_s)


class TestResolveAndTuning(unittest.TestCase):
    def test_resolve_uses_saved_characteristics(self):
        res = PS.resolve_for(ME3B, print_speed_mm_s=3.0,
                             tuning={"lookahead": 0.9})
        # cap = 0.9 / ((0.03175 + 0.067) · 2) ≈ 4.556 → print_speed binds
        self.assertEqual(res["cap_reason"], "print_speed")
        self.assertAlmostEqual(res["speed_cap_mm_s"], 3.0)
        self.assertAlmostEqual(res["max_um_s"], 5945.6)

    def test_resolve_dead_time_binds_small_lookahead(self):
        res = PS.resolve_for(ME3B, print_speed_mm_s=5.0,
                             tuning={"lookahead": 0.2})
        self.assertEqual(res["cap_reason"], "dead_time")
        self.assertLess(res["speed_cap_mm_s"], 1.1)

    def test_tuning_from_store_defaults_and_values(self):
        class FakeStore:
            def get_mode_params(self, mode):
                assert mode == "velocity"
                return {"lookahead_mm": 0.9, "pid_kp": 0.0,
                        "hold_speed": 1.0, "corner_speed_factor": 0.0}

        t = PS.tuning_from_store(FakeStore())
        self.assertEqual(t["lookahead"], 0.9)
        self.assertEqual(t["hold_speed"], 1.0)
        # zeros fall back to bench defaults for geometry params…
        self.assertEqual(t["corner_factor"], 0.4)
        self.assertEqual(t["decel"], 1.5)
        # …but stay meaningfully zero for the levers
        self.assertEqual(t["max_speed_frac"], 0.0)

    def test_tuning_from_store_broken_store(self):
        class Broken:
            def get_mode_params(self, mode):
                raise RuntimeError("no file")

        t = PS.tuning_from_store(Broken())
        self.assertEqual(t["lookahead"], PS.TUNING_DEFAULTS["lookahead"])


class TestFailRegions(unittest.TestCase):
    def test_regions_with_severity_and_world_coords(self):
        ideal = _line(10.0, 1.0)
        prof = ([(s / 10.0, 5.0) for s in range(20)]          # fine
                + [(2.0 + i / 10.0, 40.0) for i in range(5)]  # warn (>30)
                + [(3.0, 5.0)]                                # gap between them
                + [(4.0 + i / 10.0, 90.0) for i in range(5)]  # fail (>60)
                + [(6.0, 5.0)])
        regs = PS.fail_regions_from_profile(ideal, prof, resolution_um=30.0)
        self.assertEqual(len(regs), 2)
        warn, fail = regs
        self.assertEqual(warn["severity"], "warn")
        self.assertEqual(fail["severity"], "fail")
        self.assertAlmostEqual(fail["peak_um"], 90.0)
        self.assertAlmostEqual(fail["x_mm"], 4.0, delta=0.5)
        self.assertAlmostEqual(fail["y_mm"], 0.0)

    def test_no_regions_when_clean(self):
        regs = PS.fail_regions_from_profile(_line(), [(1.0, 3.0), (2.0, -8.0)],
                                            resolution_um=30.0)
        self.assertEqual(regs, [])


class TestGeometryPanel(unittest.TestCase):
    def test_panel_cells_and_callback(self):
        seen = []
        cells = PS.simulate_panel(char=FAST, print_speed_mm_s=3.0,
                                  shapes=("Square", "Circle"),
                                  sizes=(2.0, 5.0),
                                  on_cell=lambda c: seen.append(c["shape"]))
        self.assertEqual(len(cells), 4)
        self.assertEqual(seen, ["Square", "Square", "Circle", "Circle"])
        self.assertEqual(cells[0]["shape"], "Square")
        self.assertEqual(cells[1]["size_mm"], 5.0)
        self.assertTrue(all(c["result"].completed for c in cells))


class TestTrajectorySplit(unittest.TestCase):
    def _traj(self):
        """Two printed strokes joined by a lift + travel + lower."""
        rows = []
        p = 0.0

        def add(x, y, z, dp):
            nonlocal p
            p += dp
            rows.append([x, y, z, p, 0.0, 0.0, 0.0])

        add(0.0, 0.0, 0.2, 0.0)
        for i in range(1, 6):                 # print stroke 1 (pump advances)
            add(i * 1.0, 0.0, 0.2, 0.01)
        add(5.0, 0.0, 5.0, 0.0)               # Z lift (no XY) — must not break
        add(0.0, 5.0, 5.0, 0.0)               # travel (XY, pump flat) — breaks
        add(0.0, 5.0, 0.2, 0.0)               # Z lower (no XY)
        for i in range(1, 4):                 # print stroke 2
            add(i * 1.0, 5.0, 0.2, 0.01)
        return rows

    def test_splits_on_travel_not_on_z(self):
        subs = PS.printing_subpaths_from_trajectory(self._traj())
        self.assertEqual(len(subs), 2)
        self.assertEqual(subs[0][0], (0.0, 0.0))
        self.assertEqual(subs[0][-1], (5.0, 0.0))
        self.assertEqual(subs[1][0], (0.0, 5.0))
        self.assertEqual(subs[1][-1], (3.0, 5.0))

    def test_weld_node_zero_length_pump_flat_does_not_break(self):
        """The coincident weld node (zero XY distance, pump flat) that once
        broke Quick Print continuity must not split the sub-path."""
        rows = [[0.0, 0.0, 0.2, 0.00, 0, 0, 0],
                [1.0, 0.0, 0.2, 0.01, 0, 0, 0],
                [1.0, 0.0, 0.2, 0.01, 0, 0, 0],   # weld node
                [2.0, 0.0, 0.2, 0.02, 0, 0, 0]]
        subs = PS.printing_subpaths_from_trajectory(rows)
        self.assertEqual(len(subs), 1)
        self.assertEqual(subs[0][-1], (2.0, 0.0))

    def test_empty_and_tiny(self):
        self.assertEqual(PS.printing_subpaths_from_trajectory([]), [])
        self.assertEqual(PS.printing_subpaths_from_trajectory(
            [[0, 0, 0.2, 0, 0, 0, 0]]), [])


class TestCheckToolpath(unittest.TestCase):
    def test_aggregates_worst_verdict_and_regions(self):
        # A gentle line passes; a tight star at speed with a big lookahead
        # produces corner deviations.
        line = _line(8.0)
        star = XC.make_shape("Star", 2.0, 0.3)
        out = PS.check_toolpath([line, star], char=ME3B, print_speed_mm_s=3.0,
                                tuning={"lookahead": 1.5, "hold_speed": 1.0},
                                resolution_um=30.0)
        self.assertEqual(len(out["paths"]), 2)
        self.assertEqual(out["paths"][0]["result"].verdict, "pass")
        self.assertNotEqual(out["verdict"], "pass")
        self.assertTrue(any(r["path_index"] == 1
                            for r in out["fail_regions"]))
        self.assertGreater(out["worst_p95_um"], 30.0)

    def test_empty_toolpath_fails(self):
        out = PS.check_toolpath([], char=ME3B, print_speed_mm_s=3.0)
        self.assertEqual(out["verdict"], "fail")
        self.assertEqual(out["paths"], [])

    def test_check_trajectory_convenience(self):
        rows = [[0, 0, 0.2, 0.00, 0, 0, 0]]
        for i in range(1, 11):
            rows.append([i * 0.8, 0.0, 0.2, i * 0.01, 0, 0, 0])
        out = PS.check_trajectory(rows, char=FAST, print_speed_mm_s=3.0,
                                  tuning={"hold_speed": 1.0})
        self.assertEqual(out["verdict"], "pass")
        self.assertTrue(out["all_completed"])


if __name__ == "__main__":
    unittest.main()
