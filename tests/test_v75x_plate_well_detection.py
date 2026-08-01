"""v7.5.x — model-driven plate well detection (SupportClasses/PlateWellDetector).

Operator: *"detect 24 wells on the mosaic and do the best effort to ensure that
they are where they should be and they are the size they should be."*

Most tests run on a SYNTHETIC plate so they are deterministic and CI-safe. The
class at the bottom runs the detector against the operator's REAL mosaics when
they are present on disk (skipped otherwise) — that is where the accuracy
numbers quoted in the update plan come from.
"""

import json
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import cv2
    CV2 = True
except ImportError:                                    # pragma: no cover
    CV2 = False

from SupportClasses.PlateWellDetector import (       # noqa: E402
    PRESETS, WellAppearance, detect_plate_wells, find_candidates,
    fit_lattice, refine_ring, ring_response)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Synthetic plate: 4x6 wells, 19.3 mm pitch, 15.6 mm wells, 0.025 px/um
ROWS, COLS = 4, 6
PITCH_UM, DIAM_UM = 19300.0, 15600.0
PX_PER_UM = 0.025
R_PX = DIAM_UM / 2.0 * PX_PER_UM          # 195 px
PITCH_PX = PITCH_UM * PX_PER_UM           # 482.5 px


def synth_plate(rot_deg=0.0, margin=300.0, rim_bright=True,
                missing=(), noise=0.0, gradient=0.0, seed=0):
    """A mosaic that looks like the real thing.

    Mid-grey plate body; each well is a hole ringed by a brighter (or darker)
    moulded rim. The rim runs from 1.00 R to 1.19 R, matching the radial
    profile measured on the operator's real clear-plastic mosaic (bright band
    196 -> 232 px for a 196 px well) — that width matters, because a rim thin
    enough for BOTH its edges to fall inside the search band would make the
    edge-polarity tests meaningless.
    """
    th = np.radians(rot_deg)
    c, s = np.cos(th), np.sin(th)
    raw = [((j * PITCH_PX) * c - (i * PITCH_PX) * s,
            (j * PITCH_PX) * s + (i * PITCH_PX) * c, i, j)
           for i in range(ROWS) for j in range(COLS)]
    ox = margin - min(p[0] for p in raw)
    oy = margin - min(p[1] for p in raw)
    w = int(max(p[0] for p in raw) + ox + margin)
    h = int(max(p[1] for p in raw) + oy + margin)
    img = np.full((h, w), 130, np.uint8)
    centers = []
    for (rx, ry, i, j) in raw:
        x, y = rx + ox, ry + oy
        centers.append((x, y, i, j))
        if (i, j) in missing:
            continue
        rim = 165 if rim_bright else 95
        cv2.circle(img, (int(round(x)), int(round(y))),
                   int(round(R_PX * 1.19)), int(rim), -1)
        cv2.circle(img, (int(round(x)), int(round(y))),
                   int(round(R_PX)), 128, -1)
    if gradient:
        g = np.linspace(1.0 - gradient, 1.0 + gradient, w, dtype=np.float32)
        img = np.clip(img.astype(np.float32) * g[None, :], 0, 255).astype(np.uint8)
    if noise:
        rng = np.random.default_rng(seed)
        img = np.clip(img.astype(np.float32) + rng.normal(0, noise, img.shape),
                      0, 255).astype(np.uint8)
    return img, centers


def run(img, **kw):
    kw.setdefault("rows", ROWS)
    kw.setdefault("cols", COLS)
    kw.setdefault("px_per_um", PX_PER_UM)
    kw.setdefault("diameter_um", DIAM_UM)
    kw.setdefault("pitch_x_um", PITCH_UM)
    kw.setdefault("pitch_y_um", PITCH_UM)
    return detect_plate_wells(img, **kw)


@unittest.skipUnless(CV2, "OpenCV required")
class TestSyntheticPlate(unittest.TestCase):
    def test_finds_every_well_at_the_right_place_and_size(self):
        img, centers = synth_plate()
        r = run(img)
        self.assertTrue(r.ok, r.refuse_reason)
        self.assertEqual(len(r.wells), ROWS * COLS)
        self.assertEqual(r.n_measured, ROWS * COLS)
        by_rc = {(w.row, w.col): w for w in r.wells}
        for (x, y, i, j) in centers:
            w = by_rc[(i, j)]
            self.assertLess(np.hypot(w.center_px[0] - x, w.center_px[1] - y),
                            3.0, f"{i},{j} centre off")
            self.assertLess(abs(w.radius_px - R_PX), 4.0, f"{i},{j} radius off")
        self.assertLess(abs(r.measured_diameter_um - DIAM_UM), 400.0)
        self.assertLess(abs(r.rotation_deg), 0.5)

    def test_result_always_carries_the_full_grid(self):
        """A well hidden under a bubble still gets a position — downstream
        mapping needs every well, not just the visible ones."""
        img, centers = synth_plate(missing={(0, 0), (2, 3), (3, 5)})
        r = run(img)
        self.assertTrue(r.ok, r.refuse_reason)
        self.assertEqual(len(r.wells), ROWS * COLS)
        by_rc = {(w.row, w.col): w for w in r.wells}
        for rc in ((0, 0), (2, 3), (3, 5)):
            self.assertEqual(by_rc[rc].center_source, "lattice")
        # ...and the filled-in wells still land on the true grid position.
        for (x, y, i, j) in centers:
            w = by_rc[(i, j)]
            self.assertLess(np.hypot(w.center_px[0] - x, w.center_px[1] - y),
                            6.0)

    def test_missing_wells_get_the_plate_median_size(self):
        img, _ = synth_plate(missing={(1, 1)})
        r = run(img)
        w = next(w for w in r.wells if (w.row, w.col) == (1, 1))
        self.assertIn(w.radius_source, ("plate_median", "nominal"))
        self.assertLess(abs(w.radius_px - r.median_radius_px), 1e-6)

    def test_rotated_plate(self):
        for deg in (2.0, -7.0, 15.0):
            img, centers = synth_plate(rot_deg=deg)
            r = run(img)
            self.assertTrue(r.ok, f"{deg}: {r.refuse_reason}")
            self.assertEqual(r.n_measured, ROWS * COLS, f"{deg}")
            self.assertLess(abs(((r.rotation_deg - deg + 180) % 360) - 180),
                            1.0, f"{deg}")

    def test_illumination_gradient_does_not_move_the_wells(self):
        """The matched filter is a difference of local means, so a stitched
        mosaic's tile-to-tile brightness steps must not shift a centre."""
        base = run(synth_plate()[0])
        shaded = run(synth_plate(gradient=0.45)[0])
        self.assertTrue(shaded.ok, shaded.refuse_reason)
        for a, b in zip(base.wells, shaded.wells):
            self.assertLess(np.hypot(a.center_px[0] - b.center_px[0],
                                     a.center_px[1] - b.center_px[1]), 2.0)

    def test_noise(self):
        r = run(synth_plate(noise=20.0)[0])
        self.assertTrue(r.ok, r.refuse_reason)
        self.assertEqual(r.n_measured, ROWS * COLS)

    def test_dark_rim_plate_is_auto_sensed(self):
        """A plate whose wells read the other way round needs no code change —
        only the edge polarity differs, and 'auto' finds it."""
        r = run(synth_plate(rim_bright=False)[0])
        self.assertTrue(r.ok, r.refuse_reason)
        self.assertEqual(r.n_measured, ROWS * COLS)
        self.assertEqual(r.polarity, "falling")


@unittest.skipUnless(CV2, "OpenCV required")
class TestRefusals(unittest.TestCase):
    """Placing a full grid of WRONG markers is worse than refusing: the
    caller's fallback (teach 3 corners by hand) is a fine outcome."""

    def test_wrong_plate_type_is_refused(self):
        img, _ = synth_plate()
        r = detect_plate_wells(img, rows=8, cols=12, px_per_um=PX_PER_UM,
                               diameter_um=6350.0, pitch_x_um=9000.0,
                               pitch_y_um=9000.0)
        self.assertFalse(r.ok)
        self.assertIn("plate type", r.refuse_reason)

    def test_wrong_polarity_profile_is_refused(self):
        img, _ = synth_plate()               # bright rim => "rising"
        r = run(img, appearance=WellAppearance(edge_polarity="falling"))
        self.assertFalse(r.ok)
        self.assertTrue(r.refuse_reason)

    def test_unmeasurable_wells_are_refused_even_when_the_grid_matches(self):
        """The second gate on its own: with the lattice gate disabled, a run
        that locks a grid but measures no edges must still refuse (and say
        the appearance profile is the likely cause)."""
        img, _ = synth_plate()
        r = run(img, appearance=WellAppearance(edge_polarity="falling",
                                               min_lattice_frac=0.0))
        self.assertFalse(r.ok)
        self.assertIn("appearance profile", r.refuse_reason)

    def test_blank_image_is_refused(self):
        r = run(np.full((1000, 1400), 130, np.uint8))
        self.assertFalse(r.ok)

    def test_wells_too_small_to_measure(self):
        img, _ = synth_plate()
        r = run(img, px_per_um=0.0004)       # 15.6 mm well => 6 px across
        self.assertFalse(r.ok)
        self.assertIn("too small", r.refuse_reason)

    def test_unknown_geometry_is_refused_not_guessed(self):
        img, _ = synth_plate()
        self.assertFalse(run(img, diameter_um=0.0).ok)
        self.assertFalse(run(img, pitch_x_um=0.0).ok)
        self.assertFalse(run(img, px_per_um=0.0).ok)
        self.assertFalse(run(img, rows=0).ok)

    def test_none_and_empty_image(self):
        self.assertFalse(run(None).ok)
        self.assertFalse(run(np.zeros((0, 0), np.uint8)).ok)


@unittest.skipUnless(CV2, "OpenCV required")
class TestStages(unittest.TestCase):
    def test_ring_response_peaks_at_the_well_centres(self):
        img, centers = synth_plate()
        resp = ring_response(img, R_PX, downsample=4, band_frac=0.06)
        ds = 4
        for (x, y, _i, _j) in centers[:6]:
            patch = resp[int(y / ds) - 2:int(y / ds) + 3,
                         int(x / ds) - 2:int(x / ds) + 3]
            self.assertGreater(float(patch.max()), 0.4 * float(resp.max()))

    def test_candidates_separate_wells_from_everything_else(self):
        img, _ = synth_plate()
        cands = find_candidates(img, R_PX, PITCH_PX, ROWS * COLS,
                                WellAppearance(), "rising")
        self.assertGreaterEqual(len(cands), ROWS * COLS)
        top = [c[2] for c in cands[:ROWS * COLS]]
        self.assertGreater(min(top), 0.0)

    def test_lattice_fit_labels_every_node(self):
        img, centers = synth_plate()
        cands = find_candidates(img, R_PX, PITCH_PX, ROWS * COLS,
                                WellAppearance(), "rising")
        fit, assign, why = fit_lattice(cands, ROWS, COLS, PITCH_PX, PITCH_PX,
                                       WellAppearance())
        self.assertIsNotNone(fit, why)
        self.assertEqual(len(assign), ROWS * COLS)

    def test_a_returned_lattice_always_respects_the_plate_spacing(self):
        """fit_lattice's spacing gate is a guard on the LS solution, not a
        plate-type check: whenever it returns a fit, that fit's spacing is
        within tolerance of the spacing it was given."""
        img, _ = synth_plate()
        cands = find_candidates(img, R_PX, PITCH_PX, ROWS * COLS,
                                WellAppearance(), "rising")
        app = WellAppearance()
        for f in (0.5, 0.8, 1.0, 1.4, 2.5, 4.0):
            fit, _a, _why = fit_lattice(cands, ROWS, COLS, PITCH_PX * f,
                                        PITCH_PX * f, app)
            if fit is None:
                continue
            A, _t = fit
            for axis in (0, 1):
                ratio = float(np.linalg.norm(A[:, axis])) / (PITCH_PX * f)
                self.assertGreaterEqual(ratio, 1.0 - app.pitch_tol_frac, f"{f}")
                self.assertLessEqual(ratio, 1.0 + app.pitch_tol_frac, f"{f}")

    def test_diagonal_lattice_confusion_is_rejected_end_to_end(self):
        """A square grid also admits a 45-degree lattice at sqrt(2)x the pitch
        — it lands on exactly half the wells. fit_lattice's spacing gate cannot
        see that (the fitted spacing matches the inflated nominal it was
        given), so the end-to-end inlier-fraction gate has to, which is why
        that gate sits above 50 %."""
        img, _ = synth_plate()
        r = run(img, pitch_x_um=PITCH_UM * np.sqrt(2.0),
                pitch_y_um=PITCH_UM * np.sqrt(2.0))
        self.assertFalse(r.ok)

    def test_lattice_is_canonically_oriented(self):
        """A 180-degree-flipped lattice fits equally well; the fit must always
        return the near-zero-rotation one so row/col labels are deterministic."""
        img, _ = synth_plate(rot_deg=178.0)
        cands = find_candidates(img, R_PX, PITCH_PX, ROWS * COLS,
                                WellAppearance(), "rising")
        fit, _a, why = fit_lattice(cands, ROWS, COLS, PITCH_PX, PITCH_PX,
                                   WellAppearance())
        self.assertIsNotNone(fit, why)
        A, _t = fit
        self.assertLessEqual(
            abs(np.degrees(np.arctan2(A[1, 0], A[0, 0]))), 90.0)

    def test_lattice_fit_needs_candidates(self):
        fit, assign, why = fit_lattice([], ROWS, COLS, PITCH_PX, PITCH_PX,
                                       WellAppearance())
        self.assertIsNone(fit)
        self.assertEqual(assign, {})
        self.assertTrue(why)

    def test_refine_recovers_a_displaced_centre(self):
        img, centers = synth_plate()
        x, y, _i, _j = centers[7]
        cx, cy, r, q = refine_ring(img, x + 9.0, y - 7.0, R_PX,
                                   WellAppearance(), "rising")
        self.assertLess(np.hypot(cx - x, cy - y), 3.0)
        self.assertLess(abs(r - R_PX), 4.0)
        self.assertGreater(q, 0.25)

    def test_refine_is_bounded_by_the_search_band(self):
        """A wild radius can never be reported: the walk only looks inside
        nominal x (1 +/- diameter_tol)."""
        img, centers = synth_plate()
        x, y, _i, _j = centers[0]
        app = WellAppearance(diameter_tol=0.10)
        _cx, _cy, r, _q = refine_ring(img, x, y, R_PX, app, "rising")
        self.assertGreaterEqual(r, R_PX * 0.85)
        self.assertLessEqual(r, R_PX * 1.15)


class TestAppearanceProfile(unittest.TestCase):
    """The per-plate-type knobs — how a new plate product gets characterised."""

    def test_defaults_are_auto_sensing(self):
        self.assertEqual(WellAppearance().edge_polarity, "auto")

    def test_presets_exist_and_are_distinct(self):
        self.assertIn("clear_plastic_rim", PRESETS)
        self.assertEqual(PRESETS["clear_plastic_rim"].edge_polarity, "rising")
        self.assertEqual(PRESETS["bright_disc"].edge_polarity, "falling")
        self.assertEqual(PRESETS["dark_disc"].edge_polarity, "rising")

    def test_to_dict_emits_only_overrides(self):
        self.assertEqual(WellAppearance().to_dict(), {})
        d = WellAppearance(diameter_tol=0.2).to_dict()
        self.assertEqual(d, {"diameter_tol": 0.2})

    def test_from_dict_round_trip_and_preset_base(self):
        a = WellAppearance.from_dict({"key": "bright_disc",
                                      "diameter_tol": 0.2})
        self.assertEqual(a.edge_polarity, "falling")     # from the preset
        self.assertEqual(a.diameter_tol, 0.2)            # explicit override
        self.assertEqual(WellAppearance.from_dict(None), WellAppearance())
        self.assertEqual(WellAppearance.from_dict({}), WellAppearance())

    def test_from_dict_ignores_junk(self):
        a = WellAppearance.from_dict({"nonsense": 1, "diameter_tol": 0.2})
        self.assertEqual(a.diameter_tol, 0.2)

    def test_for_plate_type_reads_the_stored_profile(self):
        class _PT:
            well_detection = {"key": "bright_disc"}
        self.assertEqual(
            WellAppearance.for_plate_type(_PT()).edge_polarity, "falling")

        class _Bare:
            pass
        self.assertEqual(WellAppearance.for_plate_type(_Bare()),
                         WellAppearance())

    def test_plate_type_carries_the_profile_and_stays_byte_identical(self):
        from SupportClasses.PlateTypeStore import PlateType
        legacy = {"id": "x", "base_format": 24, "display_name": "X",
                  "manufacturer": "", "model": "", "bottom_material": "",
                  "well_depth_mm": None,
                  "z_offsets": {"top": 1.0, "bottom": 2.0, "safe": 3.0,
                                "max": 4.0},
                  "builtin": False}
        # A plate type written before this feature round-trips unchanged.
        self.assertEqual(PlateType.from_dict(legacy).to_dict(), legacy)
        pt = PlateType.from_dict(
            dict(legacy, well_detection={"key": "clear_plastic_rim"}))
        self.assertEqual(pt.well_detection, {"key": "clear_plastic_rim"})
        self.assertEqual(pt.to_dict()["well_detection"],
                         {"key": "clear_plastic_rim"})
        self.assertEqual(WellAppearance.for_plate_type(pt).edge_polarity,
                         "rising")


@unittest.skipUnless(CV2, "OpenCV required")
class TestRealMosaics(unittest.TestCase):
    """Against the operator's actual scans. Skipped when they aren't on disk.

    These pin the numbers the algorithm was tuned to: they are the reason the
    defaults are what they are.
    """

    @classmethod
    def setUpClass(cls):
        path = os.path.join(REPO, "config", "hardware", "plate_mosaics.json")
        if not os.path.exists(path):
            raise unittest.SkipTest("no plate_mosaics.json")
        with open(path, encoding="utf-8") as f:
            cls.store = (json.load(f) or {}).get("mosaics") or {}

    def _detect(self, key):
        m = self.store.get(key)
        if not m:
            self.skipTest(f"mosaic {key!r} not present")
        img_path = os.path.join(REPO, "config", "hardware", m.get("image", ""))
        if not os.path.exists(img_path):
            self.skipTest(f"mosaic image for {key!r} not on disk")
        img = cv2.imread(img_path)
        if img is None:
            self.skipTest(f"could not read the mosaic for {key!r}")
        return detect_plate_wells(
            img, rows=4, cols=6, px_per_um=float(m["mosaic_scale"]),
            diameter_um=15600.0, pitch_x_um=19300.0, pitch_y_um=19300.0)

    def test_clear_plastic_24_well_all_24(self):
        """The plate from the operator's request: every well, right size."""
        r = self._detect("nest-plastic-24")
        self.assertTrue(r.ok, r.refuse_reason)
        self.assertEqual(len(r.wells), 24)
        self.assertEqual(r.n_measured, 24)
        self.assertEqual(r.polarity, "rising")
        # Ø within 5 % of the catalogue 15.6 mm, and every well within 3 % of
        # the plate median (a moulded plate is uniform).
        self.assertAlmostEqual(r.measured_diameter_um / 1000.0, 15.6, delta=0.8)
        radii = np.array([w.radius_px for w in r.wells])
        self.assertLess(radii.std() / radii.mean(), 0.03)
        self.assertLess(abs(r.rotation_deg), 2.0)
        self.assertLess(r.lattice_rms_px, 10.0)

    def test_other_real_mosaics_still_resolve(self):
        for key in ("24", "plate-24_Rossette A1"):
            with self.subTest(key):
                r = self._detect(key)
                self.assertTrue(r.ok, f"{key}: {r.refuse_reason}")
                self.assertEqual(len(r.wells), 24)
                self.assertGreaterEqual(r.n_measured, 20)
                self.assertAlmostEqual(r.measured_diameter_um / 1000.0, 15.6,
                                       delta=1.0)


@unittest.skipUnless(CV2, "OpenCV required")
class TestMappingDialogWiring(unittest.TestCase):
    """The 'Auto-detect wells' button actually runs this detector."""

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _dialog(self, img, scale=PX_PER_UM):
        import tempfile
        from pathlib import Path
        import SupportClasses.WellTrainingStore as wts
        from SupportClasses.WellPlate import WellPlate
        from gui.dialogs.mosaic_well_mapping_dialog import (
            MosaicWellMappingDialog)
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        wts._store_singleton = wts.WellTrainingStore(Path(tmp.name))
        self.addCleanup(lambda: setattr(wts, "_store_singleton", None))
        h, w = img.shape[:2]
        return MosaicWellMappingDialog(
            WellPlate.from_format(24), img,
            (0.0, 0.0, w / scale, h / scale), scale,
            um_per_px=1.0 / scale, plate_key="24")

    def test_button_places_all_wells_at_their_measured_size(self):
        img, centers = synth_plate()
        dlg = self._dialog(np.dstack([img] * 3))
        self.assertEqual(len(dlg._well_items), ROWS * COLS)
        # The drawn radius is the MEASURED one, not the nominal fallback.
        for m in dlg._well_items.values():
            self.assertLess(abs(m.r_px - R_PX), 6.0)
        self.assertTrue(any(abs(m.r_px - R_PX) > 1e-6
                            for m in dlg._well_items.values()))

    def test_refusal_leaves_the_manual_corner_flow_armed(self):
        """A blank mosaic must not place 24 bogus markers — the operator's
        3-corner fallback has to stay available."""
        dlg = self._dialog(np.full((1200, 1700, 3), 130, np.uint8))
        self.assertEqual(len(dlg._well_items), 0)
        self.assertTrue(dlg._status.text())

    def test_pitch_falls_back_to_the_nominal_positions(self):
        """A custom plate carrying no well_spacing still yields a pitch."""
        img, _ = synth_plate()
        dlg = self._dialog(np.dstack([img] * 3))
        dlg._plate.well_spacing_x = 0.0
        dlg._plate.well_spacing_y = 0.0
        px, py = dlg._grid_pitch_mm()
        self.assertAlmostEqual(px, 19.3, places=3)
        self.assertAlmostEqual(py, 19.3, places=3)

    def test_appearance_comes_from_the_plate_type(self):
        import SupportClasses.PlateTypeStore as pts
        from SupportClasses.PlateTypeStore import PlateType
        img, _ = synth_plate()
        dlg = self._dialog(np.dstack([img] * 3))
        dlg._plate_key = "unit-test-plate"
        store = pts.get_store()
        store._types["unit-test-plate"] = PlateType(
            id="unit-test-plate", base_format=24,
            well_detection={"key": "bright_disc"})
        self.addCleanup(store._types.pop, "unit-test-plate", None)
        self.assertEqual(dlg._well_appearance().edge_polarity, "falling")


if __name__ == "__main__":
    unittest.main()
