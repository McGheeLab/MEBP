"""
test_v75_print_session.py — PrintSessionManager round-trip tests.

Covers the v7.5.0 PrintSession backend: save → list → load → modify →
save → load again, plus validate_collection_refs, duplicate, delete.
"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from SupportClasses.PrintSessionManager import (
    PrintSession, PrintSessionManager, SESSION_SUFFIX, SCHEMA_VERSION,
)


class TestPrintSessionRoundTrip(unittest.TestCase):

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.mgr = PrintSessionManager(sessions_dir=self._tmp.name)

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def test_new_session_default_metadata(self) -> None:
        s = self.mgr.new_session("Smoke")
        self.assertEqual(s.name, "Smoke")
        self.assertEqual(s.schema_version, SCHEMA_VERSION)
        self.assertTrue(s.metadata.created)
        self.assertTrue(s.metadata.modified)

    def test_save_and_list(self) -> None:
        s = self.mgr.new_session("First")
        path = self.mgr.save_session(s)
        self.assertIsNotNone(path)
        self.assertTrue(path.exists())
        self.assertTrue(str(path).endswith(SESSION_SUFFIX))

        listed = self.mgr.list_sessions()
        self.assertEqual(len(listed), 1)
        self.assertEqual(listed[0]["name"], "First")

    def test_load_restores_state(self) -> None:
        s = self.mgr.new_session("With State")
        s.objects = [{
            "name": "ring",
            "object_type": "circle",
            "params": {"radius": 1.5},
            "source": "parametric",
        }]
        s.collections = [{"name": "Default", "objects": []}]
        s.execution_config = {
            "ink_swap": {"waste": True, "wash_pre": True},
            "pump_volume_overrides_uL": {"P1": 250.0},
        }
        s.wells = {
            "assignments": {
                "A1": {"role": "print", "print_collections": ["Default"]},
            },
        }
        path = self.mgr.save_session(s)
        loaded = self.mgr.load_session(path)
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded.name, "With State")
        self.assertEqual(loaded.objects[0]["params"]["radius"], 1.5)
        self.assertEqual(
            loaded.execution_config["ink_swap"]["waste"], True,
        )

    def test_validate_collection_refs(self) -> None:
        s = self.mgr.new_session("Refs")
        s.collections = [{"name": "Default"}]
        s.wells = {
            "assignments": {
                "A1": {"role": "print", "print_collections": ["Default"]},
                "B2": {"role": "print", "print_collections": ["Missing"]},
            },
        }
        missing = s.validate_collection_refs()
        self.assertIn("B2", missing)
        self.assertNotIn("A1", missing)

    def test_duplicate(self) -> None:
        s = self.mgr.new_session("Orig")
        s.execution_config = {"ink_swap": {"waste": True}}
        self.mgr.save_session(s)
        dup = self.mgr.duplicate_session("Orig", "Copy")
        self.assertIsNotNone(dup)
        self.assertEqual(dup.name, "Copy")
        self.assertEqual(len(self.mgr.list_sessions()), 2)

    def test_delete(self) -> None:
        s = self.mgr.new_session("Delete Me")
        self.mgr.save_session(s)
        self.assertEqual(len(self.mgr.list_sessions()), 1)
        ok = self.mgr.delete_session("Delete Me")
        self.assertTrue(ok)
        self.assertEqual(len(self.mgr.list_sessions()), 0)

    def test_load_missing_returns_none(self) -> None:
        result = self.mgr.load_session(Path(self._tmp.name) / "nope.print.json")
        self.assertIsNone(result)


class TestPrintSessionByteEqualRoundTrip(unittest.TestCase):

    def test_save_load_save_byte_equal(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            mgr = PrintSessionManager(sessions_dir=tmp)
            s = mgr.new_session("RoundTrip")
            s.objects = [{
                "name": "ring",
                "object_type": "circle",
                "params": {"radius": 1.0},
                "source": "parametric",
            }]
            s.execution_config = {
                "ink_swap": {"waste": True, "wash_pre": True, "wash_post": False,
                             "buffer": True, "ink_load": True, "wash_final": True},
                "pump_volume_overrides_uL": {},
            }
            path1 = mgr.save_session(s)
            loaded = mgr.load_session(path1)
            loaded.metadata.modified = s.metadata.modified  # stabilize
            path2 = mgr.save_session(loaded, path=path1)
            with open(path1) as f:
                bytes1 = f.read()
            with open(path2) as f:
                bytes2 = f.read()
            self.assertEqual(bytes1, bytes2)


if __name__ == "__main__":
    unittest.main()
