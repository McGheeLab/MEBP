"""Loopback integration tests: a real ThreadingHTTPServer + the real client.

Proves the wire behaviour end-to-end — round-trip fidelity, Range resume,
409/200 collision semantics, percent-encoding, concurrency and limits.
"""

import hashlib
import os
import tempfile
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from lablink.client import LabLinkClient, LabLinkError, PARTIAL_DIR
from lablink.server import make_server
from lablink.store import FileStore

TOKEN = "test-token"


class ServerCase(unittest.TestCase):
    """Starts a server on 127.0.0.1:<ephemeral> for the whole class."""

    max_file_mb = 512

    @classmethod
    def setUpClass(cls):
        cls._td = tempfile.TemporaryDirectory()
        cls.root = Path(cls._td.name) / "store"
        cls.store = FileStore(cls.root, max_file_bytes=cls.max_file_mb * 1024 * 1024)
        cls.srv = make_server(cls.store, "127.0.0.1", 0, TOKEN, label="test-server")
        cls.port = cls.srv.server_address[1]
        cls.thread = threading.Thread(target=cls.srv.serve_forever, daemon=True)
        cls.thread.start()
        cls.url = f"http://127.0.0.1:{cls.port}"

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()
        cls.thread.join(timeout=5)
        cls._td.cleanup()

    def setUp(self):
        self.client = LabLinkClient(self.url, TOKEN, timeout=10)
        self._wd = tempfile.TemporaryDirectory()
        self.work = Path(self._wd.name)
        self.addCleanup(self._wd.cleanup)

    def make_file(self, name: str, data: bytes) -> Path:
        p = self.work / name
        p.write_bytes(data)
        return p

    def raw_request(self, method: str, path: str, data: bytes | None = None,
                    headers: dict | None = None):
        """Issue a hand-built request, bypassing the client. Returns (status, body).

        Lets the tests exercise malformed requests the client would never send.
        """
        import urllib.error
        import urllib.request

        url = f"{self.url}{path}"
        req = urllib.request.Request(url, data=data, method=method)
        for k, v in (headers or {}).items():
            req.add_header(k, v)
        try:
            with urllib.request.urlopen(req, timeout=10) as resp:
                return resp.status, resp.read(), dict(resp.headers)
        except urllib.error.HTTPError as exc:
            return exc.code, exc.read(), dict(exc.headers)


class TestHelloAndAuth(ServerCase):
    def test_hello_needs_no_token(self):
        anon = LabLinkClient(self.url, "wrong-token", timeout=10)
        info = anon.hello()
        self.assertEqual(info["service"], "lablink")
        self.assertEqual(info["protocol"], 1)
        self.assertEqual(info["name"], "test-server")
        self.assertIn("time", info)

    def test_bad_token_rejected_on_list(self):
        bad = LabLinkClient(self.url, "wrong-token", timeout=10)
        with self.assertRaises(LabLinkError) as ctx:
            bad.list_files("auth")
        self.assertEqual(ctx.exception.status, 401)

    def test_bad_token_rejected_on_upload(self):
        bad = LabLinkClient(self.url, "wrong-token", timeout=10)
        p = self.make_file("x.bin", b"data")
        with self.assertRaises(LabLinkError) as ctx:
            bad.upload("auth", p, retries=1)
        self.assertEqual(ctx.exception.status, 401)

    def test_unknown_endpoint_404(self):
        with self.assertRaises(LabLinkError) as ctx:
            self.client._json("GET", f"{self.url}/nope")
        self.assertEqual(ctx.exception.status, 404)


class TestRoundTrip(ServerCase):
    def test_upload_list_download_delete(self):
        data = os.urandom(200_000)
        src = self.make_file("mosaic.png", data)
        rec = self.client.upload("rt", src, meta={"kind": "mosaic", "well": "A1"})
        self.assertEqual(rec["status"], "created")
        self.assertEqual(rec["sha256"], hashlib.sha256(data).hexdigest())

        listing = self.client.list_files("rt")
        self.assertEqual(listing["seq"], rec["seq"])
        self.assertEqual(listing["files"][0]["meta"], {"kind": "mosaic", "well": "A1"})

        dest = self.work / "dest"
        out = self.client.download("rt", "mosaic.png", dest)
        self.assertEqual(out.read_bytes(), data)
        # A watcher on dest never sees a partial: parts live in .partial/
        self.assertEqual([p.name for p in dest.iterdir() if p.is_file()], ["mosaic.png"])
        self.assertEqual(list((dest / PARTIAL_DIR).iterdir()), [])

        self.client.delete("rt", "mosaic.png")
        self.assertEqual(self.client.list_files("rt")["files"], [])

    def test_name_with_spaces_percent_encodes(self):
        src = self.make_file("mosaic 001 A1.png", b"spaced")
        self.client.upload("space", src)
        names = [f["name"] for f in self.client.list_files("space")["files"]]
        self.assertEqual(names, ["mosaic 001 A1.png"])
        out = self.client.download("space", "mosaic 001 A1.png", self.work / "d")
        self.assertEqual(out.read_bytes(), b"spaced")

    def test_empty_file_round_trip(self):
        src = self.make_file("empty.bin", b"")
        rec = self.client.upload("empty", src)
        self.assertEqual(rec["size"], 0)
        out = self.client.download("empty", "empty.bin", self.work / "d")
        self.assertEqual(out.read_bytes(), b"")

    def test_unknown_channel_lists_empty(self):
        listing = self.client.list_files("never-used-channel")
        self.assertEqual(listing["files"], [])
        self.assertEqual(listing["seq"], 0)

    def test_since_seq(self):
        for i in range(3):
            self.client.upload("seq", self.make_file(f"f{i}.bin", bytes([i])))
        listing = self.client.list_files("seq", since_seq=1)
        self.assertEqual([f["name"] for f in listing["files"]], ["f1.bin", "f2.bin"])

    def test_download_missing_404(self):
        with self.assertRaises(LabLinkError) as ctx:
            self.client.download("rt", "nope.bin", self.work / "d", retries=1)
        self.assertEqual(ctx.exception.status, 404)

    def test_delete_missing_404(self):
        with self.assertRaises(LabLinkError) as ctx:
            self.client.delete("rt", "nope.bin")
        self.assertEqual(ctx.exception.status, 404)


class TestResume(ServerCase):
    def test_resume_from_half_part_uses_range(self):
        data = os.urandom(120_000)
        self.client.upload("res", self.make_file("big.bin", data))

        dest = self.work / "dest"
        (dest / PARTIAL_DIR).mkdir(parents=True)
        part = dest / PARTIAL_DIR / "big.bin.part"
        part.write_bytes(data[:50_000])

        out = self.client.download("res", "big.bin", dest)
        self.assertEqual(out.read_bytes(), data)
        self.assertFalse(part.exists())

    def test_206_status_observed_for_range_request(self):
        from lablink.protocol import H_TOKEN

        data = os.urandom(50_000)
        self.client.upload("res", self.make_file("ranged.bin", data))
        status, body, headers = self.raw_request(
            "GET", "/c/res/ranged.bin",
            headers={H_TOKEN: TOKEN, "Range": "bytes=20000-"},
        )
        self.assertEqual(status, 206)
        self.assertEqual(headers["Content-Range"], "bytes 20000-49999/50000")
        self.assertEqual(body, data[20000:])

    def test_range_past_end_is_416(self):
        from lablink.protocol import H_TOKEN

        data = os.urandom(1_000)
        self.client.upload("res", self.make_file("tiny.bin", data))
        status, _, headers = self.raw_request(
            "GET", "/c/res/tiny.bin",
            headers={H_TOKEN: TOKEN, "Range": "bytes=5000-"},
        )
        self.assertEqual(status, 416)
        self.assertEqual(headers["Content-Range"], "bytes */1000")

    def test_overlong_part_triggers_416_then_clean_restart(self):
        data = os.urandom(30_000)
        self.client.upload("res", self.make_file("short.bin", data))

        dest = self.work / "dest416"
        (dest / PARTIAL_DIR).mkdir(parents=True)
        part = dest / PARTIAL_DIR / "short.bin.part"
        part.write_bytes(os.urandom(40_000))  # longer than the real file, wrong bytes

        # First attempt: 416 -> checksum mismatch -> part deleted -> retry from 0.
        out = self.client.download("res", "short.bin", dest, expected_sha=hashlib.sha256(data).hexdigest())
        self.assertEqual(out.read_bytes(), data)

    def test_unsupported_range_form_falls_back_to_full(self):
        from lablink.protocol import H_TOKEN

        data = os.urandom(10_000)
        self.client.upload("res", self.make_file("full.bin", data))
        for bad_range in ("bytes=0-99", "bytes=0-9,20-29", "items=0-", "garbage"):
            with self.subTest(range=bad_range):
                status, body, _ = self.raw_request(
                    "GET", "/c/res/full.bin",
                    headers={H_TOKEN: TOKEN, "Range": bad_range},
                )
                self.assertEqual(status, 200)
                self.assertEqual(len(body), 10_000)


class TestTruncatedTransfers(ServerCase):
    """A link that cuts mid-body must be resumed, never restarted from zero.

    Regression: found by running a real transfer through a proxy that severed
    every connection after 4 MB. The client hashed the PARTIAL file, called it
    corrupt, deleted it and refetched from zero — so it never converged. The
    part is now kept and resumed; only a complete-but-wrong file is discarded.
    """

    def _cutting_proxy(self, cut_after: int):
        """Start a TCP proxy to the test server that cuts each connection."""
        import socket
        import threading as th

        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", 0))
        listener.listen(8)
        port = listener.getsockname()[1]
        stop = th.Event()
        threads: list = []

        def pump(src, dst, limit):
            sent = 0
            try:
                while not stop.is_set():
                    data = src.recv(16384)
                    if not data:
                        break
                    dst.sendall(data)
                    sent += len(data)
                    if limit and sent >= limit:
                        break
            except OSError:
                pass
            finally:
                # Both pumps share the pair, so the second pass here finds
                # them already shut — harmless, but leaving them open emits
                # ResourceWarnings that drown the real test output.
                for s in (src, dst):
                    try:
                        s.shutdown(socket.SHUT_RDWR)
                    except OSError:
                        pass
                    try:
                        s.close()
                    except OSError:
                        pass

        def serve():
            while not stop.is_set():
                try:
                    client, _ = listener.accept()
                except OSError:
                    return
                try:
                    upstream = socket.create_connection(
                        ("127.0.0.1", self.port), timeout=5)
                    upstream.settimeout(None)
                    client.settimeout(None)
                except OSError:
                    client.close()
                    continue
                t_up = th.Thread(target=pump, args=(client, upstream, 0), daemon=True)
                t_down = th.Thread(target=pump, args=(upstream, client, cut_after),
                                   daemon=True)
                t_up.start()
                t_down.start()
                threads.extend((t_up, t_down))

        th.Thread(target=serve, daemon=True).start()

        def shutdown():
            stop.set()
            listener.close()
            for t in threads:
                t.join(timeout=2)

        self.addCleanup(shutdown)
        return f"http://127.0.0.1:{port}"

    def test_resumes_through_repeated_cuts(self):
        data = os.urandom(300_000)
        self.client.upload("cut", self.make_file("cut.bin", data))

        proxy_url = self._cutting_proxy(cut_after=64_000)   # ~5 cuts needed
        flaky = LabLinkClient(proxy_url, TOKEN, timeout=10)
        out = flaky.download("cut", "cut.bin", self.work / "dest")
        self.assertEqual(out.read_bytes(), data)
        self.assertEqual(list((self.work / "dest" / PARTIAL_DIR).iterdir()), [])

    def test_partial_is_kept_not_deleted_after_a_cut(self):
        data = os.urandom(300_000)
        self.client.upload("cut", self.make_file("keep.bin", data))

        proxy_url = self._cutting_proxy(cut_after=50_000)
        flaky = LabLinkClient(proxy_url, TOKEN, timeout=10)
        # One pass only: prove bytes survive on disk rather than being binned.
        part = self.work / "d2" / PARTIAL_DIR / "keep.bin.part"
        part.parent.mkdir(parents=True)
        _, total = flaky._fetch_to_part(
            flaky._url("c", "cut", "keep.bin"), part)
        self.assertEqual(total, len(data))
        self.assertGreater(part.stat().st_size, 0)
        self.assertLess(part.stat().st_size, len(data))
        self.assertEqual(part.read_bytes(), data[:part.stat().st_size])

    def test_hopeless_link_gives_up_instead_of_looping_forever(self):
        """A link that never delivers anything must raise, not spin."""
        import socket
        import threading as th

        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.bind(("127.0.0.1", 0))
        listener.listen(8)
        stop = th.Event()

        def accept_and_hang_up():
            while not stop.is_set():
                try:
                    conn, _ = listener.accept()
                except OSError:
                    return
                try:
                    conn.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                conn.close()          # no response at all

        th.Thread(target=accept_and_hang_up, daemon=True).start()
        self.addCleanup(lambda: (stop.set(), listener.close()))

        dead = LabLinkClient(f"http://127.0.0.1:{listener.getsockname()[1]}",
                             TOKEN, timeout=5)
        started = time.monotonic()
        with self.assertRaises(LabLinkError):
            dead.download("cut", "nothing.bin", self.work / "d3", retries=2)
        self.assertLess(time.monotonic() - started, 30, "should fail fast, not spin")


class TestCollisions(ServerCase):
    def test_identical_reupload_is_duplicate(self):
        src = self.make_file("same.bin", b"identical")
        first = self.client.upload("coll", src)
        second = self.client.upload("coll", src)
        self.assertEqual(first["status"], "created")
        self.assertEqual(second["status"], "duplicate")
        self.assertEqual(first["seq"], second["seq"])

    def test_different_content_same_name_conflicts(self):
        self.client.upload("coll", self.make_file("clash.bin", b"one"))
        other = self.work / "other"
        other.mkdir()
        changed = other / "clash.bin"
        changed.write_bytes(b"two")
        with self.assertRaises(LabLinkError) as ctx:
            self.client.upload("coll", changed, retries=1)
        self.assertEqual(ctx.exception.status, 409)


class TestValidationAndLimits(ServerCase):
    """Malformed requests the real client would never send."""

    def _put(self, name: str, body: bytes, sha: str | None = "auto",
             meta: str | None = None, token: str = TOKEN):
        from lablink.protocol import H_META, H_SHA, H_TOKEN

        headers = {H_TOKEN: token}
        if sha == "auto":
            headers[H_SHA] = hashlib.sha256(body).hexdigest()
        elif sha is not None:
            headers[H_SHA] = sha
        if meta is not None:
            headers[H_META] = meta
        return self.raw_request("PUT", f"/c/val/{urllib_quote(name)}", body, headers)

    def test_bad_sha_rejected_and_nothing_stored(self):
        status, _, _ = self._put("bad.bin", b"payload", sha="0" * 64)
        self.assertEqual(status, 400)
        self.assertEqual(self.client.list_files("val")["files"], [])

    def test_missing_sha_header_rejected(self):
        status, _, _ = self._put("nosha.bin", b"x", sha=None)
        self.assertEqual(status, 400)

    def test_reserved_windows_name_rejected(self):
        status, _, _ = self._put("CON", b"x")
        self.assertEqual(status, 400)

    def test_path_traversal_rejected(self):
        status, _, _ = self._put("../escape.bin", b"x")
        self.assertEqual(status, 400)
        self.assertFalse((self.root.parent / "escape.bin").exists())

    def test_non_object_meta_rejected(self):
        status, _, _ = self._put("meta.bin", b"x", meta="[1,2,3]")
        self.assertEqual(status, 400)

    def test_invalid_json_meta_rejected(self):
        status, _, _ = self._put("meta2.bin", b"x", meta="{not json")
        self.assertEqual(status, 400)

    def test_oversize_meta_rejected(self):
        from lablink.protocol import MAX_META_BYTES

        huge = '{"k":"' + "x" * (MAX_META_BYTES + 100) + '"}'
        status, _, _ = self._put("meta3.bin", b"x", meta=huge)
        self.assertEqual(status, 400)

    def test_bad_since_seq_rejected(self):
        from lablink.protocol import H_TOKEN

        status, _, _ = self.raw_request(
            "GET", "/c/val?since_seq=abc", headers={H_TOKEN: TOKEN}
        )
        self.assertEqual(status, 400)


class TestOversize(ServerCase):
    max_file_mb = 1  # class-level server limit for this case only

    def test_hello_publishes_limit(self):
        self.assertEqual(self.client.hello()["max_file_bytes"], 1024 * 1024)

    def test_client_refuses_locally_without_uploading(self):
        big = self.make_file("big.bin", os.urandom(2 * 1024 * 1024))
        with self.assertRaises(LabLinkError) as ctx:
            self.client.upload("size", big, retries=1)
        self.assertEqual(ctx.exception.status, 413)
        self.assertFalse(ctx.exception.retryable)
        self.assertEqual(self.client.list_files("size")["files"], [])

    def test_server_enforces_limit_even_without_the_client_precheck(self):
        # Defence in depth: a client that never asked /hello (or any other
        # language's client) still gets a clean 413 with a drainable body.
        from lablink.protocol import H_SHA, H_TOKEN

        body = os.urandom(1024 * 1024 + 1)
        status, _, _ = self.raw_request(
            "PUT", "/c/size/raw.bin", body,
            {H_TOKEN: TOKEN, H_SHA: hashlib.sha256(body).hexdigest()},
        )
        self.assertEqual(status, 413)
        self.assertEqual(self.client.list_files("size")["files"], [])


class TestConcurrency(ServerCase):
    def test_parallel_distinct_uploads_get_unique_seqs(self):
        paths = [self.make_file(f"p{i}.bin", os.urandom(100_000)) for i in range(8)]
        with ThreadPoolExecutor(max_workers=8) as pool:
            recs = list(pool.map(lambda p: self.client.upload("par", p), paths))
        seqs = sorted(r["seq"] for r in recs)
        self.assertEqual(len(set(seqs)), 8)
        listing = self.client.list_files("par")
        self.assertEqual(len(listing["files"]), 8)

    def test_parallel_identical_uploads_create_exactly_one(self):
        data = os.urandom(50_000)
        srcs = []
        for i in range(4):
            d = self.work / f"racer{i}"
            d.mkdir()
            p = d / "race.bin"
            p.write_bytes(data)
            srcs.append(p)
        with ThreadPoolExecutor(max_workers=4) as pool:
            recs = list(pool.map(lambda p: self.client.upload("race", p), srcs))
        statuses = [r["status"] for r in recs]
        self.assertEqual(statuses.count("created"), 1)
        self.assertEqual(statuses.count("duplicate"), 3)
        self.assertEqual(len(self.client.list_files("race")["files"]), 1)


def urllib_quote(s: str) -> str:
    import urllib.parse

    return urllib.parse.quote(s, safe="")


if __name__ == "__main__":
    unittest.main()
