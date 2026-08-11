"""v7.17 — LabLink node config, filter optics, and the image-job sidecar.

The contract under test is `lablink/docs/IMAGE-JOB-FORMAT.md`. Three of its
rules carry real, measured consequences and each has a test that fails if the
rule is dropped:

* **unique upload names** — same name + different bytes is a 409, which is 4xx
  and therefore never retried, so a re-scan would be permanently rejected;
* **acquisition channel order** — the array position is how channel indices
  resolve, and ND3 stores ids alphabetically;
* **never fabricate optical metadata** — supplying NA + emission moved a
  measured object count from 2855 to 2660 with no warning from any layer, so a
  plausible guess is worse than an absence the hub can name.
"""

from __future__ import annotations

import json
import os
import tempfile
import unittest
from datetime import datetime
from pathlib import Path

import numpy as np

from SupportClasses import LabLinkJob as J


# ── config store ──────────────────────────────────────────────────

class TestConfigStore(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="mebp_lablink_")
        os.environ["MEBP_LABLINK_CONFIG_DIR"] = self.dir
        os.environ.pop("LABLINK_TOKEN", None)
        from SupportClasses import LabLinkConfigStore as C
        C.reset_store()
        self.C = C
        self.store = C.get_store()

    def tearDown(self):
        self.C.reset_store()
        os.environ.pop("MEBP_LABLINK_CONFIG_DIR", None)
        os.environ.pop("LABLINK_TOKEN", None)

    def test_off_until_deliberately_enabled(self):
        """Pushing lab images off the machine must never start by itself."""
        self.assertFalse(self.store.enabled())
        for name in self.C.SOURCES:
            self.assertFalse(self.store.source(name)["enabled"])

    def test_round_trips_through_disk(self):
        self.store.set_base_url("http://hub:8765/")
        self.store.set_token("secret")
        self.store.set_source("fluorescence_well", {
            "enabled": True, "workflow": "nd2studios", "recipe": "decon",
            "knobs": {"iterations": 10}})
        self.C.reset_store()
        fresh = self.C.get_store()
        self.assertEqual(fresh.base_url(), "http://hub:8765")   # trailing / dropped
        self.assertEqual(fresh.token(), "secret")
        self.assertEqual(fresh.source("fluorescence_well")["recipe"], "decon")

    def test_environment_token_wins_over_the_file(self):
        self.store.set_token("from-file")
        os.environ["LABLINK_TOKEN"] = "from-env"
        self.assertEqual(self.store.token(), "from-env")
        self.assertEqual(self.store.token_source(), "environment")

    def test_as_dict_redacts_the_token(self):
        """`logs/` has tracked subdirectories, so a config dump must be safe."""
        self.store.set_token("super-secret")
        self.assertNotIn("super-secret", json.dumps(self.store.as_dict()))
        self.assertIn("super-secret",
                      json.dumps(self.store.as_dict(redact=False)))

    def test_a_null_knob_survives_a_save_and_reload(self):
        """omitted / null / 0 are three different instructions to the hub."""
        self.store.set_source("still", {"knobs": {"min_area_um2": None,
                                                  "level": 0}})
        self.C.reset_store()
        knobs = self.C.get_store().source("still")["knobs"]
        self.assertIn("min_area_um2", knobs)
        self.assertIsNone(knobs["min_area_um2"])
        self.assertEqual(knobs["level"], 0)

    def test_a_source_with_no_recipe_is_not_enabled(self):
        """Otherwise the operator watches a queue grow instead of being told
        to finish configuring."""
        self.store.set_base_url("http://hub:8765")
        self.store.set_token("t")
        self.store.set_enabled(True)
        self.store.set_source("still", {"enabled": True, "workflow": "nd2studios"})
        self.assertFalse(self.store.source_enabled("still"))
        self.store.set_source("still", {"recipe": "segment"})
        self.assertTrue(self.store.source_enabled("still"))

    def test_a_corrupt_file_degrades_to_off_rather_than_raising(self):
        Path(self.dir, "lablink.json").write_text("{not json", encoding="utf-8")
        self.C.reset_store()
        self.assertFalse(self.C.get_store().enabled())

    def test_unknown_source_is_refused(self):
        with self.assertRaises(ValueError):
            self.store.set_source("hologram", {"enabled": True})

    def test_no_lablink_keys_leaked_into_hardware_config(self):
        """The CAMERA_CAL_PERSIST_STORE lesson, plus: this holds a token."""
        from SupportClasses.HardwareConfig import HardwareConfig
        keys = set(HardwareConfig().to_dict())
        self.assertFalse([k for k in keys if "lablink" in k.lower()])


# ── filter optics ─────────────────────────────────────────────────

class TestFilterOptics(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="mebp_scope_")
        os.environ["MEBP_MICROSCOPE_CONFIG_DIR"] = self.dir
        from SupportClasses import MicroscopeConfigStore as M
        self.M = M
        self.store = M.MicroscopeConfigStore()

    def tearDown(self):
        os.environ.pop("MEBP_MICROSCOPE_CONFIG_DIR", None)

    def test_absent_stays_absent(self):
        """A cube never filled in is UNKNOWN — never a nominal default."""
        self.assertEqual(self.store.filter_optics_for("Cy5"), {})

    def test_matched_case_insensitively(self):
        """The channel name comes from the scan store, the cube name is typed
        by hand; whitespace or case must not cost a deconvolution its λ."""
        self.store.set_filter_optics("FITC", emission_nm=519.0)
        self.assertEqual(self.store.filter_optics_for("  fitc "),
                         {"emission_nm": 519.0})

    def test_a_micron_value_is_refused_not_clamped(self):
        """0.519 is µm. Clamping would fabricate a measured-looking number."""
        with self.assertRaises(ValueError) as ctx:
            self.store.set_filter_optics("FITC", emission_nm=0.519)
        self.assertIn("519", str(ctx.exception))

    def test_malformed_stored_entries_are_dropped_on_load(self):
        self.store.set_filter_optics("FITC", emission_nm=519.0)
        raw = json.loads(Path(self.dir, "microscope.json").read_text())
        raw["filter_optics"]["Junk"] = {"emission_nm": "green"}
        raw["filter_optics"]["Empty"] = {}
        Path(self.dir, "microscope.json").write_text(json.dumps(raw))
        fresh = self.M.MicroscopeConfigStore()
        self.assertEqual(set(fresh.filter_optics()), {"FITC"})

    def test_filter_cube_labels_are_untouched(self):
        """The new map is a SIBLING; the {str: str} label coercion still holds."""
        self.store.set_filter_label(1, "DAPI")
        self.store.set_filter_optics("DAPI", emission_nm=461.0)
        fresh = self.M.MicroscopeConfigStore()
        self.assertEqual(fresh.filter_label(1), "DAPI")


# ── naming ────────────────────────────────────────────────────────

class TestUploadNames(unittest.TestCase):

    def test_the_name_rule_matches_the_vendored_protocol(self):
        """Duplicated only so a name can be checked without importing the
        client; the two must never drift."""
        from SupportClasses.lablink import protocol
        self.assertEqual(J.NAME_RE.pattern, protocol.NAME_RE.pattern)
        self.assertEqual(J.WINDOWS_RESERVED, protocol.WINDOWS_RESERVED)

    def test_ten_thousand_same_second_stems_do_not_collide(self):
        """⚠ This test found a real defect: the first cut used 2 nonce bytes
        and produced 729 collisions here. A collision is a 409, which is 4xx
        and never retried — the exact failure a unique name exists to prevent.

        The bound is calibrated, not arbitrary. Expected collisions over 10k
        same-second names are 763 at 2 bytes, 3.0 at 3 and 0.012 at 4, so
        `>= 9995` passes essentially always at the shipped 4 bytes and fails
        immediately at 2 or 3.
        """
        seen = set()
        for _ in range(10000):
            stem = J.build_stem("decon", "A1")
            J.validate_upload_name(stem + ".nd3")
            seen.add(stem)
        self.assertGreaterEqual(len(seen), 9995)

    def test_hostile_plate_and_well_names_still_yield_a_legal_name(self):
        """These are real keys from this machine's plate library."""
        for hint in ("plate-24_Rosette in A2&A3", "plate#A1", "A1 (well 3)",
                     "", "   ", "***", "é" * 40, "x" * 200):
            stem = J.build_stem("decon", hint)
            J.validate_upload_name(stem + J.SIDECAR_SUFFIX)

    def test_the_stem_leaves_the_reply_side_room(self):
        """The hub appends a suffix and possibly the produced stem against the
        same 128-char ceiling, and silently declines to return an overlong
        output."""
        self.assertLessEqual(len(J.build_stem("decon", "A1" * 40)), J.MAX_STEM)

    def test_reserved_and_malformed_names_are_refused(self):
        for bad in ("CON.nd3", "COM1.job.json", "_leading.nd3",
                    "trailing ", "trailing.", ".leading", "a/b.nd3",
                    "a\\b.nd3", "", "x" * 129):
            with self.assertRaises(J.LabLinkJobError, msg=bad):
                J.validate_upload_name(bad)

    def test_an_interior_space_is_legal(self):
        """Spaces ARE allowed — MATLAB/ImageJ users produce them, and the URL
        percent-encodes. Only a TRAILING one is refused."""
        J.validate_upload_name("scan 001.nd3")

    def test_two_stems_a_second_apart_differ(self):
        """The 409 kill: a re-scan must not reuse a name."""
        a = J.build_stem("d", "A1", now=datetime(2026, 8, 8, 14, 25, 30))
        b = J.build_stem("d", "A1", now=datetime(2026, 8, 8, 14, 25, 31))
        self.assertNotEqual(a, b)


# ── sidecar ───────────────────────────────────────────────────────

def _write_nd3(path, channels, *, pitch=0.65, bit_depth="12-bit", na=0.45):
    from SupportClasses import ND3
    with ND3.ND3Writer(path) as w:
        for spec in channels:
            name, number = spec[0], spec[1]
            entry = {"name": name}
            if number is not None:
                entry["channel_number"] = number
            if len(spec) > 2:           # extra channel-entry fields, verbatim
                entry.update(spec[2])
            acq = {"bit_depth": bit_depth, "magnification": "10x"}
            if na is not None:
                acq["numerical_aperture"] = na
            p = pitch(name) if callable(pitch) else pitch
            w.add_image(name.replace(" ", "_"), np.zeros((4, 4), np.uint16),
                        axes="YX", pixel_format="gray16",
                        meta={"scale": {"um_per_px": p}, "acquisition": acq},
                        channels=[entry])
    return path


class TestSidecar(unittest.TestCase):

    def setUp(self):
        self.dir = Path(tempfile.mkdtemp(prefix="mebp_job_"))

    def test_channels_are_in_acquisition_order_not_alphabetical(self):
        """⚠ ND3Reader.image_ids() is sorted(). The sidecar array position is
        how the hub resolves channel indices, so alphabetical order would
        analyse the wrong channel and look entirely normal."""
        p = _write_nd3(self.dir / "w.nd3",
                       [("DAPI", 1), ("FITC", 2), ("Cy5", 4)])
        from SupportClasses import ND3
        with ND3.open_nd3(p) as r:
            self.assertEqual(r.image_ids(), ["Cy5", "DAPI", "FITC"])   # premise
        side, _ = J.build_sidecar(p)
        self.assertEqual([c["name"] for c in side["image"]["channels"]],
                         ["DAPI", "FITC", "Cy5"])

    def test_bit_depth_is_the_sensor_not_the_container(self):
        """uint16 array, 12-bit sensor. The dtype is the invented value the
        sidecar exists to override."""
        p = _write_nd3(self.dir / "w.nd3", [("FITC", 2)])
        side, _ = J.build_sidecar(p)
        self.assertEqual(side["image"]["bit_depth"], 12)

    def test_unknown_bit_depth_is_refused_never_guessed_from_dtype(self):
        p = _write_nd3(self.dir / "w.nd3", [("FITC", 2)], bit_depth="")
        with self.assertRaises(J.LabLinkJobError) as ctx:
            J.build_sidecar(p)
        self.assertIn("bit depth", str(ctx.exception).lower())
        # ...but an explicit value is accepted
        side, _ = J.build_sidecar(p, bit_depth=14)
        self.assertEqual(side["image"]["bit_depth"], 14)

    def test_mixed_pixel_sizes_are_refused(self):
        """The sidecar carries ONE pixel_size_um; silently picking a channel's
        would make every um^2 figure wrong while still looking plausible."""
        p = _write_nd3(self.dir / "w.nd3", [("DAPI", 1), ("FITC", 2)],
                       pitch=lambda n: 0.65 if n == "DAPI" else 1.30)
        with self.assertRaises(J.LabLinkJobError) as ctx:
            J.build_sidecar(p)
        self.assertIn("pixel size", str(ctx.exception).lower())

    def test_missing_wavelengths_are_reported_never_fabricated(self):
        p = _write_nd3(self.dir / "w.nd3", [("FITC", 2), ("Cy5", 4)])
        optics = {"FITC": {"emission_nm": 519.0}}
        side, missing = J.build_sidecar(p, optics_lookup=optics.get)
        names = {c["name"]: c for c in side["image"]["channels"]}
        self.assertEqual(names["FITC"]["emission_nm"], 519.0)
        self.assertNotIn("emission_nm", names["Cy5"])
        self.assertTrue(any("Cy5" in m for m in missing))
        self.assertIn("missing_metadata", J.describe_missing(missing))

    def test_file_recorded_wavelengths_pass_through(self):
        """ND3_SPEC §14: an ND3 channel entry MAY carry emission/excitation
        wavelengths, and this bridge passes them through when present — a
        future acquisition path that knows its filters must not earn a
        `missing_metadata` refusal for facts the file already records. The
        optics lookup (the microscope's own config) stays authoritative when
        both know the channel."""
        p = _write_nd3(
            self.dir / "w.nd3",
            [("DAPI", 1, {"emission_nm": 461.0, "excitation_nm": 395.0}),
             ("FITC", 2, {"emission_nm": 519.0})])
        side, missing = J.build_sidecar(p)
        names = {c["name"]: c for c in side["image"]["channels"]}
        self.assertEqual(names["DAPI"]["emission_nm"], 461.0)
        self.assertEqual(names["DAPI"]["excitation_nm"], 395.0)
        self.assertEqual(names["FITC"]["emission_nm"], 519.0)
        self.assertFalse(any("emission" in m for m in missing), missing)
        # the lookup wins over the file when both know the channel
        side, _ = J.build_sidecar(
            p, optics_lookup={"DAPI": {"emission_nm": 465.0}}.get)
        names = {c["name"]: c for c in side["image"]["channels"]}
        self.assertEqual(names["DAPI"]["emission_nm"], 465.0)
        self.assertEqual(names["DAPI"]["excitation_nm"], 395.0)

    def test_missing_na_is_reported(self):
        p = _write_nd3(self.dir / "w.nd3", [("FITC", 2)], na=None)
        _, missing = J.build_sidecar(p)
        self.assertIn("objective_na", missing)

    def test_a_null_knob_reaches_the_sidecar_as_null(self):
        p = _write_nd3(self.dir / "w.nd3", [("FITC", 2)])
        side, _ = J.build_sidecar(p, recipe="decon",
                                  knobs={"min_area_um2": None, "iterations": 0})
        self.assertIn("min_area_um2", side["job"]["knobs"])
        self.assertIsNone(side["job"]["knobs"]["min_area_um2"])
        self.assertEqual(side["job"]["knobs"]["iterations"], 0)

    def test_no_job_block_when_no_recipe_is_given(self):
        """`job` is optional and the API is authoritative; a disagreement is
        refused outright, so it is written only when the caller means it."""
        p = _write_nd3(self.dir / "w.nd3", [("FITC", 2)])
        side, _ = J.build_sidecar(p)
        self.assertNotIn("job", side)

    def test_the_true_name_survives_in_source(self):
        p = _write_nd3(self.dir / "w.nd3", [("FITC", 2)])
        side, _ = J.build_sidecar(p, original_name="A1 (well 3).nd3")
        self.assertEqual(side["source"]["original_name"], "A1 (well 3).nd3")

    def test_format_is_the_exact_declared_string(self):
        p = _write_nd3(self.dir / "w.nd3", [("FITC", 2)])
        side, _ = J.build_sidecar(p)
        self.assertEqual(side["format"], "lablink.imagejob/1")


class TestJobPair(unittest.TestCase):

    def setUp(self):
        self.dir = Path(tempfile.mkdtemp(prefix="mebp_pair_"))

    def test_the_pair_shares_a_stem_and_the_sidecar_suffix(self):
        p = _write_nd3(self.dir / "w.nd3", [("FITC", 2)])
        side, _ = J.build_sidecar(p)
        pair = J.write_job_pair(p, self.dir / "out", "decon_A1_x_7f3a", side)
        self.assertEqual(pair.image_path.name, "decon_A1_x_7f3a.nd3")
        self.assertEqual(pair.sidecar_path.name, "decon_A1_x_7f3a.job.json")
        self.assertEqual(json.loads(pair.sidecar_path.read_text())["format"],
                         "lablink.imagejob/1")

    def test_the_image_is_snapshotted_not_referenced(self):
        """MEBP's stores reuse deterministic filenames that a re-scan
        overwrites, so a queued live path can be swapped under the uploader."""
        p = _write_nd3(self.dir / "w.nd3", [("FITC", 2)])
        side, _ = J.build_sidecar(p)
        pair = J.write_job_pair(p, self.dir / "out", "a_b_c_1111", side)
        before = pair.image_path.read_bytes()
        p.write_bytes(b"a completely different acquisition")
        self.assertEqual(pair.image_path.read_bytes(), before)

    def test_an_illegal_stem_is_refused_before_anything_is_written(self):
        p = _write_nd3(self.dir / "w.nd3", [("FITC", 2)])
        side, _ = J.build_sidecar(p)
        out = self.dir / "out2"
        with self.assertRaises(J.LabLinkJobError):
            J.write_job_pair(p, out, "_illegal", side)
        self.assertFalse(list(out.glob("*")) if out.exists() else [])


if __name__ == "__main__":
    unittest.main()
