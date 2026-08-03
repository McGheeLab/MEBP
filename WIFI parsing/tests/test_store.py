"""Unit tests for lablink.store.FileStore — durability, idempotency, seq, TTL.

Pure store tests, no HTTP; these pin every crash-safety invariant before any
networking exists.
"""

import hashlib
import json
import tempfile
import time
import unittest
from pathlib import Path

from lablink import store as store_mod
from lablink.store import Conflict, DiskFull, FileStore, NotFound, ShaMismatch, TooLarge


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def put(fs: FileStore, channel: str, name: str, data: bytes, meta=None):
    with fs.begin_upload(channel, name, len(data)) as up:
        up.write(data)
        return up.commit(sha(data), meta)


class StoreCase(unittest.TestCase):
    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self.root = Path(self._td.name)
        self.addCleanup(self._td.cleanup)
        self.fs = FileStore(self.root)


class TestUploadCommit(StoreCase):
    def test_commit_visible_with_correct_record(self):
        status, rec = put(self.fs, "ch", "a.bin", b"hello", meta={"k": "v"})
        self.assertEqual(status, "created")
        self.assertEqual(rec["size"], 5)
        self.assertEqual(rec["sha256"], sha(b"hello"))
        self.assertEqual(rec["seq"], 1)
        self.assertEqual(rec["meta"], {"k": "v"})
        latest, files = self.fs.list_files("ch")
        self.assertEqual(latest, 1)
        self.assertEqual([f["name"] for f in files], ["a.bin"])
        # tmp/ is empty after a commit
        self.assertEqual(list((self.root / "ch" / "tmp").iterdir()), [])

    def test_read_back(self):
        put(self.fs, "ch", "a.bin", b"0123456789")
        fh, rec = self.fs.open_read("ch", "a.bin")
        with fh:
            self.assertEqual(fh.read(), b"0123456789")
        fh, rec = self.fs.open_read("ch", "a.bin", offset=6)
        with fh:
            self.assertEqual(fh.read(), b"6789")
        with self.assertRaises(NotFound):
            self.fs.open_read("ch", "missing.bin")

    def test_abandoned_session_invisible_and_swept(self):
        up = self.fs.begin_upload("ch", "half.bin", 100)
        up.write(b"x" * 50)
        # Simulate a crash: session dropped without commit or abort.
        up._fh.close()
        _, files = self.fs.list_files("ch")
        self.assertEqual(files, [])
        self.assertEqual(len(list((self.root / "ch" / "tmp").iterdir())), 1)
        # A new store instance (process restart) sweeps the stale part.
        fs2 = FileStore(self.root)
        fs2.open_channel("ch")
        self.assertEqual(list((self.root / "ch" / "tmp").iterdir()), [])

    def test_context_manager_abort_removes_tmp(self):
        with self.fs.begin_upload("ch", "x.bin", 10) as up:
            up.write(b"12345")
            # no commit -> __exit__ aborts
        self.assertEqual(list((self.root / "ch" / "tmp").iterdir()), [])
        _, files = self.fs.list_files("ch")
        self.assertEqual(files, [])

    def test_sha_mismatch_rejected_nothing_visible(self):
        with self.fs.begin_upload("ch", "x.bin", 5) as up:
            up.write(b"hello")
            with self.assertRaises(ShaMismatch):
                up.commit(sha(b"other"))
        _, files = self.fs.list_files("ch")
        self.assertEqual(files, [])
        self.assertEqual(list((self.root / "ch" / "tmp").iterdir()), [])

    def test_orphan_sidecar_invisible_and_overwritable(self):
        # Crash-between-meta-and-data: sidecar exists, data file does not.
        ch = self.fs.open_channel("ch")
        (ch.meta / "ghost.bin.json").write_text(
            json.dumps({"name": "ghost.bin", "size": 1, "sha256": "0" * 64, "seq": 7})
        )
        _, files = self.fs.list_files("ch")
        self.assertEqual(files, [])
        # A later upload of the same name simply succeeds.
        status, rec = put(self.fs, "ch", "ghost.bin", b"real")
        self.assertEqual(status, "created")
        _, files = self.fs.list_files("ch")
        self.assertEqual([f["name"] for f in files], ["ghost.bin"])

    def test_body_exceeding_declared_length_rejected(self):
        with self.fs.begin_upload("ch", "x.bin", 3) as up:
            with self.assertRaises(store_mod.StoreError):
                up.write(b"12345")


class TestCollisionPolicy(StoreCase):
    def test_duplicate_same_sha_is_noop(self):
        _, rec1 = put(self.fs, "ch", "a.bin", b"same")
        status, rec2 = put(self.fs, "ch", "a.bin", b"same")
        self.assertEqual(status, "duplicate")
        self.assertEqual(rec2["seq"], rec1["seq"])  # seq NOT bumped
        latest, files = self.fs.list_files("ch")
        self.assertEqual(latest, 1)
        self.assertEqual(len(files), 1)

    def test_different_sha_conflicts(self):
        put(self.fs, "ch", "a.bin", b"one")
        with self.assertRaises(Conflict):
            put(self.fs, "ch", "a.bin", b"two")
        # Original content untouched.
        fh, _ = self.fs.open_read("ch", "a.bin")
        with fh:
            self.assertEqual(fh.read(), b"one")

    def test_delete_then_reput_gets_new_seq(self):
        _, rec1 = put(self.fs, "ch", "a.bin", b"one")
        self.assertTrue(self.fs.delete("ch", "a.bin"))
        status, rec2 = put(self.fs, "ch", "a.bin", b"two")
        self.assertEqual(status, "created")
        self.assertGreater(rec2["seq"], rec1["seq"])


class TestCaseInsensitiveNames(StoreCase):
    """Two names differing only in case must never coexist in one channel.

    Windows and macOS treat them as the same file, so a client fetching the
    channel into a single folder would silently overwrite one with the other.
    Refusing at upload keeps a channel safe to download on any platform.
    """

    def test_case_variant_with_different_content_is_refused(self):
        put(self.fs, "ch", "Scan.png", b"AAA")
        with self.assertRaises(Conflict) as ctx:
            put(self.fs, "ch", "scan.png", b"BBB")
        self.assertIn("capitalisation", str(ctx.exception))
        self.assertIn("Scan.png", str(ctx.exception))
        _, files = self.fs.list_files("ch")
        self.assertEqual([f["name"] for f in files], ["Scan.png"])

    def test_case_variant_with_identical_content_stays_idempotent(self):
        """Identical bytes must be a no-op even under a different case.

        Only one file exists either way, so the channel is still safe to fetch
        on any filesystem — and refusing would break the retry-after-crash
        guarantee, which for the sync agent meant a permanent, unrecoverable
        conflict for a file it had already delivered.
        """
        _, first = put(self.fs, "ch", "Scan.png", b"AAA")
        status, rec = put(self.fs, "ch", "scan.png", b"AAA")
        self.assertEqual(status, "duplicate")
        self.assertEqual(rec["seq"], first["seq"])
        _, files = self.fs.list_files("ch")
        self.assertEqual([f["name"] for f in files], ["Scan.png"])

    def test_case_conflict_detected_on_a_case_sensitive_store(self):
        # Simulate a Linux server: the exact-name lookup misses, so the
        # explicit scan is what has to catch it.
        ch = self.fs.open_channel("ch")
        put(self.fs, "ch", "Data.csv", b"one")
        self.assertEqual(self.fs._case_conflict(ch, "data.csv"), "Data.csv")
        self.assertEqual(self.fs._case_conflict(ch, "DATA.CSV"), "Data.csv")
        self.assertIsNone(self.fs._case_conflict(ch, "Data.csv"))   # itself
        self.assertIsNone(self.fs._case_conflict(ch, "other.csv"))

    def test_exact_same_name_still_deduplicates(self):
        put(self.fs, "ch", "same.bin", b"x")
        status, _ = put(self.fs, "ch", "same.bin", b"x")
        self.assertEqual(status, "duplicate")


class TestDeleteWhileOpen(StoreCase):
    """Deleting a file a download still holds open must not raise.

    Windows refuses to unlink an open file. Removing the sidecar first makes
    the file invisible immediately (the invariant needs both halves), so the
    delete always takes effect and the bytes are reclaimed later.
    """

    def test_delete_succeeds_while_file_is_open(self):
        put(self.fs, "ch", "held.bin", b"y" * 500)
        fh, _ = self.fs.open_read("ch", "held.bin")
        try:
            self.assertTrue(self.fs.delete("ch", "held.bin"))
            _, files = self.fs.list_files("ch")
            self.assertEqual(files, [])                  # invisible at once
        finally:
            fh.close()

    def test_orphaned_data_is_reclaimed_on_restart(self):
        put(self.fs, "ch", "held.bin", b"z" * 500)
        ch = self.fs.open_channel("ch")
        fh, _ = self.fs.open_read("ch", "held.bin")
        try:
            self.fs.delete("ch", "held.bin")
        finally:
            fh.close()
        # On Windows the data file survived the delete; a fresh store sweeps it.
        FileStore(self.root).open_channel("ch")
        self.assertFalse((ch.data / "held.bin").exists())

    def test_sweep_keeps_files_that_have_a_sidecar(self):
        put(self.fs, "ch", "keep.bin", b"keep")
        ch = self.fs.open_channel("ch")
        FileStore(self.root).open_channel("ch")
        self.assertTrue((ch.data / "keep.bin").exists())
        _, files = FileStore(self.root).list_files("ch")
        self.assertEqual([f["name"] for f in files], ["keep.bin"])


class TestSeq(StoreCase):
    def test_monotonic_across_restart(self):
        for i in range(3):
            put(self.fs, "ch", f"f{i}.bin", bytes([i]))
        fs2 = FileStore(self.root)
        _, rec = put(fs2, "ch", "f3.bin", b"\x03")
        self.assertEqual(rec["seq"], 4)

    def test_monotonic_even_with_counter_file_deleted(self):
        for i in range(3):
            put(self.fs, "ch", f"f{i}.bin", bytes([i]))
        (self.root / "ch" / "_seq.json").unlink()
        fs2 = FileStore(self.root)
        _, rec = put(fs2, "ch", "f3.bin", b"\x03")
        self.assertEqual(rec["seq"], 4)  # recovered from sidecars

    def test_since_seq_filter(self):
        for i in range(5):
            put(self.fs, "ch", f"f{i}.bin", bytes([i]))
        latest, files = self.fs.list_files("ch", since_seq=3)
        self.assertEqual(latest, 5)
        self.assertEqual([f["seq"] for f in files], [4, 5])

    def test_unknown_channel_lists_empty(self):
        latest, files = self.fs.list_files("never-used")
        self.assertEqual((latest, files), (0, []))


class TestLimitsAndHousekeeping(StoreCase):
    def test_max_size_rejected_before_any_write(self):
        fs = FileStore(self.root, max_file_bytes=10)
        with self.assertRaises(TooLarge):
            fs.begin_upload("ch", "big.bin", 11)

    def test_disk_full_guard(self):
        # A declared size near the volume's free space must trip the margin
        # (max_file_bytes raised so only the free-disk guard can fire).
        fs = FileStore(self.root, max_file_bytes=2**63)
        with self.assertRaises(DiskFull):
            fs.begin_upload("ch", "huge.bin", 2**62)

    def test_delete(self):
        put(self.fs, "ch", "a.bin", b"x")
        self.assertTrue(self.fs.delete("ch", "a.bin"))
        self.assertFalse(self.fs.delete("ch", "a.bin"))
        _, files = self.fs.list_files("ch")
        self.assertEqual(files, [])
        ch = self.fs.open_channel("ch")
        self.assertFalse((ch.data / "a.bin").exists())
        self.assertFalse((ch.meta / "a.bin.json").exists())

    def test_ttl_sweep_removes_only_expired(self):
        fs = FileStore(self.root, ttl_seconds=100)
        put(fs, "ch", "old.bin", b"old")
        put(fs, "ch", "new.bin", b"new")
        # Age the first record by editing its sidecar (the sweep reads it).
        ch = fs.open_channel("ch")
        rec = json.loads((ch.meta / "old.bin.json").read_text())
        rec["uploaded"] = time.time() - 200
        (ch.meta / "old.bin.json").write_text(json.dumps(rec))
        removed = fs.cleanup_expired()
        self.assertEqual(removed, 1)
        _, files = fs.list_files("ch")
        self.assertEqual([f["name"] for f in files], ["new.bin"])

    def test_ttl_zero_never_sweeps(self):
        put(self.fs, "ch", "a.bin", b"x")
        self.assertEqual(self.fs.cleanup_expired(), 0)

    def test_bad_names_rejected_everywhere(self):
        for bad in ["../x", "a/b", "CON", "x."]:
            with self.subTest(name=bad):
                with self.assertRaises(ValueError):
                    self.fs.begin_upload("ch", bad, 1)
                with self.assertRaises(ValueError):
                    self.fs.open_channel(bad)


if __name__ == "__main__":
    unittest.main()
