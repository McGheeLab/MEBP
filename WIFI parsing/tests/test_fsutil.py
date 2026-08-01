"""Unit tests for lablink.fsutil: atomic writes and name validation."""

import json
import os
import tempfile
import unittest
from pathlib import Path

from lablink import fsutil


class TestAtomicWrite(unittest.TestCase):
    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self.dir = Path(self._td.name)
        self.addCleanup(self._td.cleanup)

    def test_write_bytes_roundtrip_and_no_tmp_residue(self):
        p = self.dir / "out.bin"
        fsutil.atomic_write_bytes(p, b"hello")
        self.assertEqual(p.read_bytes(), b"hello")
        self.assertEqual([f.name for f in self.dir.iterdir()], ["out.bin"])

    def test_write_json_roundtrip(self):
        p = self.dir / "out.json"
        fsutil.atomic_write_json(p, {"a": 1, "b": [1, 2]})
        self.assertEqual(json.loads(p.read_text()), {"a": 1, "b": [1, 2]})

    def test_overwrite_is_atomic_replace(self):
        p = self.dir / "out.json"
        fsutil.atomic_write_json(p, {"v": 1})
        fsutil.atomic_write_json(p, {"v": 2})
        self.assertEqual(json.loads(p.read_text()), {"v": 2})
        self.assertEqual(len(list(self.dir.iterdir())), 1)

    def test_failed_serialization_leaves_no_tmp(self):
        p = self.dir / "out.json"

        class Unserializable:
            pass

        with self.assertRaises(TypeError):
            fsutil.atomic_write_json(p, {"bad": Unserializable()})
        self.assertFalse(p.exists())
        self.assertEqual(list(self.dir.glob("*.tmp")), [])

    def test_read_json_tolerates_missing_and_corrupt(self):
        self.assertIsNone(fsutil.read_json(self.dir / "nope.json"))
        self.assertEqual(fsutil.read_json(self.dir / "nope.json", default={}), {})
        bad = self.dir / "bad.json"
        bad.write_text("{not json")
        self.assertEqual(fsutil.read_json(bad, default="d"), "d")

    def test_sha256_file(self):
        p = self.dir / "x.bin"
        p.write_bytes(b"abc")
        self.assertEqual(
            fsutil.sha256_file(p),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
        )


class TestValidateName(unittest.TestCase):
    def test_accepts(self):
        for name in [
            "a",
            "mosaic_001.png",
            "results 2026-08-01.csv",
            "A1.b-scan.mat",
            "x" * 128,
            "CONSOLE.txt",  # not a reserved name (CON is, CONSOLE is not)
        ]:
            with self.subTest(name=name):
                self.assertEqual(fsutil.validate_name(name), name)

    def test_rejects(self):
        for name in [
            "",
            None,
            "..",
            "../etc",
            "a/b",
            "a\\b",
            ".hidden",          # must start alphanumeric
            "-dash-start",
            "name.",            # trailing dot (Windows strips it)
            "name ",            # trailing space
            "x" * 129,          # too long
            "CON",
            "con.txt",
            "Nul.dat",
            "COM1",
            "lpt9.log",
            "bad\tname",
            "bad\nname",
            "naïve.png",        # non-ASCII outside the allowed class
        ]:
            with self.subTest(name=name):
                with self.assertRaises((ValueError, TypeError)):
                    fsutil.validate_name(name)


if __name__ == "__main__":
    unittest.main()
