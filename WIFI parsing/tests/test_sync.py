"""End-to-end tests for the folder-sync agent against a real loopback server."""

import logging
import os
import tempfile
import threading
import time
import unittest
from pathlib import Path

from lablink.client import LabLinkClient, LabLinkError, PARTIAL_DIR
from lablink.server import make_server
from lablink.store import FileStore
from lablink.sync import SyncAgent

TOKEN = "sync-token"


class SyncCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._td = tempfile.TemporaryDirectory()
        cls.store = FileStore(Path(cls._td.name) / "store")
        cls.srv = make_server(cls.store, "127.0.0.1", 0, TOKEN, label="sync-test")
        cls.url = f"http://127.0.0.1:{cls.srv.server_address[1]}"
        cls.thread = threading.Thread(target=cls.srv.serve_forever, daemon=True)
        cls.thread.start()
        logging.disable(logging.CRITICAL)   # agents log loudly by design

    @classmethod
    def tearDownClass(cls):
        logging.disable(logging.NOTSET)
        cls.srv.shutdown()
        cls.srv.server_close()
        cls.thread.join(timeout=5)
        cls._td.cleanup()

    def setUp(self):
        self._wd = tempfile.TemporaryDirectory()
        self.work = Path(self._wd.name)
        self.addCleanup(self._wd.cleanup)
        self.client = LabLinkClient(self.url, TOKEN, timeout=10)
        # Unique channel names per test so cases cannot interfere.
        self.chan_out = f"out{self.id().rsplit('.', 1)[-1][:40]}"
        self.chan_in = f"in{self.id().rsplit('.', 1)[-1][:40]}"

    def agent(self, outbox_dir=None, inbox_dir=None, state=None,
              out_channel=None, in_channel=None) -> SyncAgent:
        cfg = {"url": self.url, "token": TOKEN, "poll_seconds": 0.1,
               "state_file": str(state or (self.work / "state.json"))}
        if outbox_dir:
            cfg["outbox"] = {"dir": str(outbox_dir),
                             "channel": out_channel or self.chan_out}
        if inbox_dir:
            cfg["inbox"] = {"dir": str(inbox_dir),
                            "channel": in_channel or self.chan_in}
        return SyncAgent(cfg)

    @staticmethod
    def settle(agent: SyncAgent, cycles: int = 2) -> None:
        """Run enough cycles for the stability gate to release files."""
        for _ in range(cycles):
            agent.run_once()


class TestConfigValidation(SyncCase):
    def test_same_channel_both_directions_refused(self):
        with self.assertRaises(ValueError) as ctx:
            self.agent(self.work / "o", self.work / "i",
                       out_channel="shared", in_channel="shared")
        self.assertIn("DIFFERENT channels", str(ctx.exception))

    def test_missing_url_or_token_refused(self):
        for cfg in ({"token": TOKEN}, {"url": self.url}):
            with self.assertRaises(ValueError):
                SyncAgent({**cfg, "outbox": {"dir": "x", "channel": "y"}})

    def test_no_direction_refused(self):
        with self.assertRaises(ValueError):
            SyncAgent({"url": self.url, "token": TOKEN})

    def test_one_way_agents_allowed(self):
        self.agent(outbox_dir=self.work / "o")
        self.agent(inbox_dir=self.work / "i")


class TestOutbox(SyncCase):
    def test_stability_gate_delays_then_uploads(self):
        out = self.work / "out"
        out.mkdir()
        agent = self.agent(outbox_dir=out)
        (out / "a.bin").write_bytes(b"hello")

        agent.run_once()   # first sighting only — must NOT upload yet
        self.assertEqual(self.client.list_files(self.chan_out)["files"], [])
        agent.run_once()   # unchanged across two scans -> upload
        names = [f["name"] for f in self.client.list_files(self.chan_out)["files"]]
        self.assertEqual(names, ["a.bin"])

    def test_growing_file_is_held_back_until_stable(self):
        out = self.work / "out"
        out.mkdir()
        agent = self.agent(outbox_dir=out)
        p = out / "growing.bin"

        p.write_bytes(b"x" * 100)
        agent.run_once()
        p.write_bytes(b"x" * 200)          # changed between scans
        agent.run_once()
        self.assertEqual(self.client.list_files(self.chan_out)["files"], [])

        self.settle(agent)                  # now stable
        files = self.client.list_files(self.chan_out)["files"]
        self.assertEqual([f["name"] for f in files], ["growing.bin"])
        self.assertEqual(files[0]["size"], 200)

    def test_files_left_in_place_and_not_resent(self):
        out = self.work / "out"
        out.mkdir()
        agent = self.agent(outbox_dir=out)
        (out / "keep.bin").write_bytes(b"data")
        self.settle(agent)
        self.assertTrue((out / "keep.bin").exists())

        seq_before = self.client.list_files(self.chan_out)["seq"]
        self.settle(agent, cycles=3)
        self.assertEqual(self.client.list_files(self.chan_out)["seq"], seq_before)

    def test_invalid_and_dotted_names_skipped_without_crashing(self):
        out = self.work / "out"
        out.mkdir()
        agent = self.agent(outbox_dir=out)
        (out / "good.bin").write_bytes(b"ok")
        (out / ".hidden").write_bytes(b"skip")
        (out / "CON").write_bytes(b"skip")
        self.settle(agent)
        names = [f["name"] for f in self.client.list_files(self.chan_out)["files"]]
        self.assertEqual(names, ["good.bin"])

    def test_conflict_recorded_once_and_others_still_flow(self):
        out = self.work / "out"
        out.mkdir()
        agent = self.agent(outbox_dir=out)
        # Server already holds different content under this name.
        other = self.work / "other"
        other.mkdir()
        (other / "clash.bin").write_bytes(b"server-version")
        self.client.upload(self.chan_out, other / "clash.bin")

        (out / "clash.bin").write_bytes(b"local-version")
        (out / "fine.bin").write_bytes(b"no problem")
        self.settle(agent)

        self.assertIn("clash.bin", agent._state["failed"])
        names = sorted(f["name"] for f in self.client.list_files(self.chan_out)["files"])
        self.assertEqual(names, ["clash.bin", "fine.bin"])
        # The server's copy is untouched.
        got = self.client.download(self.chan_out, "clash.bin", self.work / "check")
        self.assertEqual(got.read_bytes(), b"server-version")

        self.settle(agent, cycles=3)   # keeps running, no retry storm, no crash
        self.assertIn("clash.bin", agent._state["failed"])


class TestHostileServer(SyncCase):
    """A listing arrives over the network; the agent must not trust its names.

    Security regression: `dest / name` accepted absolute paths, traversals and
    UNC shares, and `_have_locally` stat()ed them BEFORE any validation. A
    spoofed listing (no token needed — there is no TLS) could make Windows
    offer NTLM credentials to an attacker's SMB share, and turn the
    download/no-download decision into a filesystem existence-and-size oracle.
    """

    HOSTILE_NAMES = [
        "../../../Windows/win.ini",
        "..\\..\\evil.txt",
        "C:/Windows/win.ini",
        "//attacker-host/share/f",
        "\\\\attacker-host\\share\\f",
        "/etc/passwd",
        "CON",
        ".ssh",
        "",
        None,
        5,
    ]

    def test_hostile_names_are_refused_before_any_filesystem_access(self):
        inbox = self.work / "in"
        agent = self.agent(inbox_dir=inbox)

        touched, downloaded = [], []
        agent._have_locally = lambda local, rec, verify_sha: touched.append(local)
        agent.client.download = lambda *a, **k: downloaded.append(a)
        agent.client.list_files = lambda ch, since_seq=None: {
            "channel": ch, "seq": 1,
            "files": [{"name": n, "size": 1, "sha256": "0" * 64, "seq": 1}
                      for n in self.HOSTILE_NAMES],
        }

        agent._poll_inbox()
        self.assertEqual(touched, [], "a rejected name must never be stat()ed")
        self.assertEqual(downloaded, [])
        self.assertEqual(agent._state["since_seq"], 0)

    def test_malformed_listing_entries_do_not_crash_the_cycle(self):
        agent = self.agent(inbox_dir=self.work / "in")
        agent.client.list_files = lambda ch, since_seq=None: {
            "channel": ch, "seq": 3,
            "files": [
                "not-a-dict",
                {"name": "ok.bin"},                            # missing fields
                {"name": "ok.bin", "size": "big", "sha256": "x", "seq": 1},
                {"size": 1, "sha256": "0" * 64, "seq": 2},     # no name
            ],
        }
        agent.run_once()          # must not raise
        self.assertEqual(agent._state["since_seq"], 0)


class TestConflictRecovery(SyncCase):
    def test_reconcile_retries_a_previously_conflicting_file(self):
        """A 409 must not be permanent.

        Regression: `failed` was persisted and never re-probed, so the advice in
        the error message — delete the server copy and it will send — did
        nothing, and the file was never delivered.
        """
        out = self.work / "out"
        out.mkdir()
        agent = self.agent(outbox_dir=out)

        other = self.work / "other"
        other.mkdir()
        (other / "clash.bin").write_bytes(b"server-version")
        self.client.upload(self.chan_out, other / "clash.bin")

        (out / "clash.bin").write_bytes(b"local-version")
        agent._cycle = 1                          # not a reconcile cycle
        self.settle(agent)
        self.assertIn("clash.bin", agent._state["failed"])

        # Operator follows the advice in the error message.
        self.client.delete(self.chan_out, "clash.bin")

        agent._cycle = 0                          # reconcile clears the blacklist
        self.settle(agent)
        got = self.client.download(self.chan_out, "clash.bin", self.work / "check")
        self.assertEqual(got.read_bytes(), b"local-version")


class TestInbox(SyncCase):
    def test_downloads_new_files_atomically(self):
        inbox = self.work / "in"
        agent = self.agent(inbox_dir=inbox)
        src = self.work / "src.bin"
        data = os.urandom(80_000)
        src.write_bytes(data)
        self.client.upload(self.chan_in, src, name="result.bin")

        agent.run_once()
        self.assertEqual((inbox / "result.bin").read_bytes(), data)
        # A watcher on the inbox never sees a partial file.
        self.assertEqual(sorted(p.name for p in inbox.iterdir() if p.is_file()),
                         ["result.bin"])
        self.assertEqual(list((inbox / PARTIAL_DIR).iterdir()), [])

    def test_since_seq_advances_and_no_redownload(self):
        inbox = self.work / "in"
        agent = self.agent(inbox_dir=inbox)
        src = self.work / "s.bin"
        src.write_bytes(b"one")
        self.client.upload(self.chan_in, src, name="one.bin")
        agent.run_once()
        seq_after = agent._state["since_seq"]
        self.assertGreater(seq_after, 0)

        mtime = (inbox / "one.bin").stat().st_mtime_ns
        agent.run_once()
        self.assertEqual(agent._state["since_seq"], seq_after)
        self.assertEqual((inbox / "one.bin").stat().st_mtime_ns, mtime)

    def test_failed_download_is_retried_not_skipped(self):
        """A failure mid-batch must not let later successes skip the cursor past it.

        Regression: found on the bench when a server restart made one download
        of a 20-file batch fail; the next file's success advanced since_seq
        past it and that file was lost forever.
        """
        inbox = self.work / "in"
        src = self.work / "s.bin"
        for i in range(3):
            src.write_bytes(f"payload-{i}".encode())
            self.client.upload(self.chan_in, src, name=f"f{i}.bin")

        agent = self.agent(inbox_dir=inbox)
        real_download = agent.client.download
        failed_once = []

        def flaky(channel, name, dest, **kw):
            if name == "f1.bin" and not failed_once:
                failed_once.append(name)
                raise LabLinkError(0, "simulated network drop")
            return real_download(channel, name, dest, **kw)

        agent.client.download = flaky
        agent.run_once()

        # f0 and f2 arrived; f1 did not — and the cursor did NOT jump past it.
        self.assertTrue((inbox / "f0.bin").exists())
        self.assertFalse((inbox / "f1.bin").exists())
        self.assertTrue((inbox / "f2.bin").exists())

        agent.client.download = real_download
        agent.run_once()
        self.assertEqual((inbox / "f1.bin").read_bytes(), b"payload-1")

    def test_reconcile_recovers_a_file_the_cursor_skipped(self):
        """A cursor that has already moved past a missing file must self-heal.

        Regression: the bench run left B's since_seq at 21 with batch_18.bin
        still on the server and absent locally — unreachable forever by
        incremental polling alone.
        """
        inbox = self.work / "in"
        src = self.work / "s.bin"
        for i in range(3):
            src.write_bytes(f"data-{i}".encode())
            self.client.upload(self.chan_in, src, name=f"g{i}.bin")
        latest = self.client.list_files(self.chan_in)["seq"]

        agent = self.agent(inbox_dir=inbox)
        agent._state["since_seq"] = latest          # cursor past everything
        agent._cycle = 1                            # not a reconcile cycle
        agent.run_once()
        self.assertFalse((inbox / "g0.bin").exists())   # incremental sees nothing

        agent._cycle = 0                            # reconcile cycle
        agent.run_once()
        for i in range(3):
            self.assertEqual((inbox / f"g{i}.bin").read_bytes(), f"data-{i}".encode())

    def test_startup_always_reconciles(self):
        agent = self.agent(inbox_dir=self.work / "in")
        self.assertEqual(agent._cycle, 0)

    def test_reconcile_skips_present_files_without_hashing(self):
        """The periodic full pass must not re-read every local file."""
        inbox = self.work / "in"
        inbox.mkdir()
        src = self.work / "s.bin"
        data = os.urandom(5000)
        src.write_bytes(data)
        self.client.upload(self.chan_in, src, name="present.bin")
        (inbox / "present.bin").write_bytes(data)

        agent = self.agent(inbox_dir=inbox)
        calls = []
        agent.client.download = lambda *a, **k: calls.append(a) or None
        agent.run_once()                            # cycle 0 -> reconcile
        self.assertEqual(calls, [])

    def test_existing_matching_file_is_not_redownloaded(self):
        inbox = self.work / "in"
        inbox.mkdir()
        data = b"already here"
        src = self.work / "s.bin"
        src.write_bytes(data)
        self.client.upload(self.chan_in, src, name="dup.bin")
        (inbox / "dup.bin").write_bytes(data)      # same content, placed by hand
        mtime = (inbox / "dup.bin").stat().st_mtime_ns

        agent = self.agent(inbox_dir=inbox)         # fresh state, since_seq 0
        agent.run_once()
        self.assertEqual((inbox / "dup.bin").stat().st_mtime_ns, mtime)
        self.assertGreater(agent._state["since_seq"], 0)


class TestEndToEnd(SyncCase):
    def test_outbox_to_channel_to_other_machines_inbox(self):
        out_a = self.work / "machineA" / "outbox"
        in_b = self.work / "machineB" / "inbox"
        out_a.mkdir(parents=True)

        agent_a = self.agent(outbox_dir=out_a, state=self.work / "a.json")
        agent_b = self.agent(inbox_dir=in_b, state=self.work / "b.json",
                             in_channel=self.chan_out)   # B reads A's channel

        payloads = {f"file{i}.bin": os.urandom(20_000 + i) for i in range(5)}
        for name, data in payloads.items():
            (out_a / name).write_bytes(data)

        self.settle(agent_a)
        agent_b.run_once()

        for name, data in payloads.items():
            self.assertEqual((in_b / name).read_bytes(), data, f"{name} mismatch")

    def test_state_loss_causes_no_duplicates_or_retransfer(self):
        out_a = self.work / "out"
        in_b = self.work / "in"
        out_a.mkdir()
        state_a = self.work / "a.json"
        state_b = self.work / "b.json"

        agent_a = self.agent(outbox_dir=out_a, state=state_a)
        agent_b = self.agent(inbox_dir=in_b, state=state_b, in_channel=self.chan_out)
        (out_a / "x.bin").write_bytes(b"payload")
        self.settle(agent_a)
        agent_b.run_once()
        seq_before = self.client.list_files(self.chan_out)["seq"]
        mtime_before = (in_b / "x.bin").stat().st_mtime_ns

        # Both agents lose their memory entirely and restart.
        state_a.unlink()
        state_b.unlink()
        agent_a2 = self.agent(outbox_dir=out_a, state=state_a)
        agent_b2 = self.agent(inbox_dir=in_b, state=state_b, in_channel=self.chan_out)
        self.settle(agent_a2)
        agent_b2.run_once()

        # Nothing new on the server (re-upload was a no-op), nothing rewritten.
        self.assertEqual(self.client.list_files(self.chan_out)["seq"], seq_before)
        self.assertEqual((in_b / "x.bin").stat().st_mtime_ns, mtime_before)

    def test_server_outage_is_survived_and_recovered(self):
        out = self.work / "out"
        out.mkdir()
        # Point an agent at a dead port: cycles must not raise.
        dead = SyncAgent({
            "url": "http://127.0.0.1:9", "token": TOKEN, "poll_seconds": 0.1,
            "outbox": {"dir": str(out), "channel": self.chan_out},
            "state_file": str(self.work / "dead.json"),
            "timeout": 1, "retries": 1,
        })
        (out / "later.bin").write_bytes(b"queued while offline")
        self.settle(dead)                     # no exception despite no server

        # Same folder, working server: the file goes through on the next cycles.
        alive = self.agent(outbox_dir=out)
        self.settle(alive)
        names = [f["name"] for f in self.client.list_files(self.chan_out)["files"]]
        self.assertEqual(names, ["later.bin"])


if __name__ == "__main__":
    unittest.main()
