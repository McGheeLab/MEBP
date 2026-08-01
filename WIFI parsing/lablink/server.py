"""LabLink HTTP server — stdlib ThreadingHTTPServer over a FileStore.

Endpoints
    GET    /hello                 identity + clock (NO token: lets a client
                                  distinguish "unreachable" from "wrong token")
    GET    /c/{channel}?since_seq=N   list files (unknown channel -> 200 empty)
    PUT    /c/{channel}/{name}    upload; needs Content-Length + X-Lablink-Sha256
    GET    /c/{channel}/{name}    download; supports Range: bytes=N-
    DELETE /c/{channel}/{name}    delete

The shared token is an ANTI-MISDIRECTION guard, not security: traffic is
plain HTTP and anyone on the same network segment who learns the token can
read everything. Do not put sensitive data through this link.
"""

from __future__ import annotations

import argparse
import hmac
import json
import logging
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse, parse_qs

from . import PROTOCOL_VERSION, __version__
from .protocol import (
    CHUNK,
    DEFAULT_MAX_FILE_MB,
    DEFAULT_PORT,
    DEFAULT_TIMEOUT_S,
    H_META,
    H_SHA,
    H_TOKEN,
    MAX_META_BYTES,
    SERVICE_NAME,
)
from .store import (
    Conflict,
    DiskFull,
    FileStore,
    NotFound,
    ShaMismatch,
    StoreError,
    TooLarge,
)

log = logging.getLogger("lablink.server")

# Largest unwanted request body we will read-and-discard so the client can
# finish writing and then read our error response. See _reject_with_body.
DRAIN_LIMIT = 8 * 1024 * 1024


class Handler(BaseHTTPRequestHandler):
    """One request. Every response carries an exact Content-Length (HTTP/1.1)."""

    protocol_version = "HTTP/1.1"
    timeout = DEFAULT_TIMEOUT_S          # reclaims threads parked on a dead peer
    server_version = f"LabLink/{__version__}"
    sys_version = ""

    # ------------------------------------------------------------------ helpers
    @property
    def store(self) -> FileStore:
        return self.server.store           # type: ignore[attr-defined]

    def _send_json(self, status: int, obj: dict, extra_headers: dict | None = None) -> None:
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        for k, v in (extra_headers or {}).items():
            self.send_header(k, v)
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionError):
            pass

    def _error(self, status: int, message: str) -> None:
        self._send_json(status, {"error": message})

    def _check_token(self) -> bool:
        """Constant-time token comparison; sends 401 and returns False on failure."""
        expected = self.server.token       # type: ignore[attr-defined]
        got = self.headers.get(H_TOKEN, "")
        if hmac.compare_digest(str(got), str(expected)):
            return True
        self._error(401, "bad or missing token")
        return False

    def _route(self):
        """(kind, channel, name, query) where kind in {hello, channel, file, none}."""
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        parts = [unquote(p) for p in parsed.path.split("/") if p]
        if parts == ["hello"]:
            return "hello", None, None, query
        if len(parts) == 2 and parts[0] == "c":
            return "channel", parts[1], None, query
        if len(parts) == 3 and parts[0] == "c":
            return "file", parts[1], parts[2], query
        return "none", None, None, query

    def _reject_with_body(self, status: int, message: str) -> None:
        """Reject a request that has an unsent/unread body.

        The client is still writing when we decide to refuse, so replying
        immediately makes its socket write fail with a reset and it never
        sees our status. Draining first lets the client finish its write and
        then read a proper HTTP error — but only up to DRAIN_LIMIT, beyond
        which discarding the bytes costs more than closing the connection.
        (The client also pre-checks the size limit from /hello, so the
        close-connection path is rare.)
        """
        try:
            remaining = int(self.headers.get("Content-Length", 0) or 0)
        except ValueError:
            remaining = 0
        if remaining > DRAIN_LIMIT:
            self.close_connection = True
        else:
            while remaining > 0:
                chunk = self.rfile.read(min(CHUNK, remaining))
                if not chunk:
                    break
                remaining -= len(chunk)
        self._error(status, message)

    # ------------------------------------------------------------------ verbs
    def do_GET(self):
        kind, channel, name, query = self._route()
        if kind == "hello":
            self._send_json(
                200,
                {
                    "service": SERVICE_NAME,
                    "protocol": PROTOCOL_VERSION,
                    "version": __version__,
                    "time": time.time(),
                    "name": self.server.server_name_label,  # type: ignore[attr-defined]
                    # Published so clients can refuse an oversize file locally
                    # instead of uploading it only to be rejected.
                    "max_file_bytes": self.store.max_file_bytes,
                },
            )
            return
        if not self._check_token():
            return
        if kind == "channel":
            self._handle_list(channel, query)
        elif kind == "file":
            self._handle_download(channel, name)
        else:
            self._error(404, f"no such endpoint: {self.path}")

    def do_PUT(self):
        kind, channel, name, _ = self._route()
        expected = self.server.token       # type: ignore[attr-defined]
        if not hmac.compare_digest(str(self.headers.get(H_TOKEN, "")), str(expected)):
            self._reject_with_body(401, "bad or missing token")
            return
        if kind != "file":
            self._reject_with_body(404, "PUT requires /c/{channel}/{name}")
            return
        self._handle_put(channel, name)

    def do_DELETE(self):
        kind, channel, name, _ = self._route()
        if not self._check_token():
            return
        if kind != "file":
            self._error(404, "DELETE requires /c/{channel}/{name}")
            return
        try:
            removed = self.store.delete(channel, name)
        except ValueError as exc:
            self._error(400, str(exc))
            return
        if removed:
            self._send_json(200, {"deleted": name})
        else:
            self._error(404, f"{channel}/{name} not found")

    # ------------------------------------------------------------------ handlers
    def _handle_list(self, channel: str, query: dict) -> None:
        try:
            since = int((query.get("since_seq") or ["0"])[0])
        except ValueError:
            self._error(400, "since_seq must be an integer")
            return
        try:
            latest, files = self.store.list_files(channel, since_seq=since)
        except ValueError as exc:
            self._error(400, str(exc))
            return
        self._send_json(200, {"channel": channel, "seq": latest, "files": files})

    def _handle_download(self, channel: str, name: str) -> None:
        try:
            fh, rec = self.store.open_read(channel, name)
        except ValueError as exc:
            self._error(400, str(exc))
            return
        except NotFound:
            self._error(404, f"{channel}/{name} not found")
            return

        size = rec["size"]
        start = self._parse_range(size)
        if start == "unsatisfiable":
            fh.close()
            self._send_json(
                416,
                {"error": f"range start beyond size {size}"},
                {"Content-Range": f"bytes */{size}"},
            )
            return

        with fh:
            if start:
                fh.seek(start)
                self.send_response(206)
                self.send_header("Content-Range", f"bytes {start}-{size - 1}/{size}")
                length = size - start
            else:
                self.send_response(200)
                length = size
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(length))
            self.send_header("Accept-Ranges", "bytes")
            self.send_header(H_SHA, rec["sha256"])
            self.end_headers()
            try:
                remaining = length
                while remaining > 0:
                    chunk = fh.read(min(CHUNK, remaining))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    remaining -= len(chunk)
            except (BrokenPipeError, ConnectionError):
                # Client vanished mid-download — normal for intermittent clients.
                self.close_connection = True

    def _parse_range(self, size: int):
        """Return 0 (whole file), a start offset, or "unsatisfiable".

        Only open-ended single ranges (``bytes=N-``) are honoured; anything
        else falls back to a full 200 response, which RFC 7233 permits and
        every client here copes with.
        """
        raw = self.headers.get("Range")
        if not raw:
            return 0
        raw = raw.strip()
        if not raw.startswith("bytes=") or "," in raw:
            return 0
        spec = raw[len("bytes="):].strip()
        if not spec.endswith("-"):
            return 0
        try:
            start = int(spec[:-1])
        except ValueError:
            return 0
        if start < 0:
            return 0
        if start >= size:
            return "unsatisfiable"
        return start

    def _handle_put(self, channel: str, name: str) -> None:
        # --- header validation, before touching the body -----------------
        raw_len = self.headers.get("Content-Length")
        if raw_len is None:
            self._error(400, "Content-Length is required")
            return
        try:
            declared = int(raw_len)
            if declared < 0:
                raise ValueError
        except ValueError:
            self._error(400, "Content-Length must be a non-negative integer")
            return

        claimed_sha = (self.headers.get(H_SHA) or "").strip().lower()
        if not claimed_sha:
            self._reject_with_body(400, f"{H_SHA} is required")
            return

        meta = {}
        raw_meta = self.headers.get(H_META)
        if raw_meta:
            if len(raw_meta) > MAX_META_BYTES:
                self._reject_with_body(400, f"{H_META} exceeds {MAX_META_BYTES} bytes")
                return
            try:
                meta = json.loads(raw_meta)
            except ValueError:
                self._reject_with_body(400, f"{H_META} is not valid JSON")
                return
            if not isinstance(meta, dict):
                self._reject_with_body(400, f"{H_META} must be a JSON object")
                return

        # --- open the session (name / size / disk pre-checks) ------------
        try:
            session = self.store.begin_upload(channel, name, declared)
        except ValueError as exc:
            self._reject_with_body(400, str(exc))
            return
        except TooLarge as exc:
            self._reject_with_body(413, str(exc))
            return
        except DiskFull as exc:
            self._reject_with_body(507, str(exc))
            return

        # --- stream the body with strict remaining-byte accounting -------
        try:
            with session:
                remaining = declared
                while remaining > 0:
                    chunk = self.rfile.read(min(CHUNK, remaining))
                    if not chunk:
                        raise StoreError(
                            f"connection closed with {remaining} bytes outstanding"
                        )
                    session.write(chunk)
                    remaining -= len(chunk)
                status, record = session.commit(claimed_sha, meta)
        except ShaMismatch as exc:
            self._error(400, str(exc))
            return
        except Conflict as exc:
            self._error(409, str(exc))
            return
        except (StoreError, ConnectionError) as exc:
            self.close_connection = True
            self._error(400, str(exc))
            return

        self._send_json(201 if status == "created" else 200, record)

    # ------------------------------------------------------------------ logging
    def log_message(self, fmt, *args):  # noqa: A003 - stdlib signature
        log.info("%s %s", self.address_string(), fmt % args)

    def log_error(self, fmt, *args):
        log.warning("%s %s", self.address_string(), fmt % args)


def make_server(store: FileStore, host: str, port: int, token: str,
                label: str = "") -> ThreadingHTTPServer:
    """Build a ThreadingHTTPServer with the store and token attached."""
    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    srv.store = store                                    # type: ignore[attr-defined]
    srv.token = token                                    # type: ignore[attr-defined]
    srv.server_name_label = label or socket.gethostname()  # type: ignore[attr-defined]
    return srv


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def local_ipv4_addresses() -> list[str]:
    """Best-effort list of this machine's IPv4 addresses, primary first."""
    addrs: list[str] = []
    # The UDP-connect trick reveals the address of the default route without
    # sending a packet.
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("10.255.255.255", 1))
        addrs.append(probe.getsockname()[0])
    except OSError:
        pass
    finally:
        probe.close()
    try:
        for ip in socket.gethostbyname_ex(socket.gethostname())[2]:
            if ip not in addrs:
                addrs.append(ip)
    except OSError:
        pass
    return [a for a in addrs if not a.startswith("127.")]


def _describe_address(ip: str) -> str:
    if ip.startswith("192.168.137."):
        return "  (Windows Mobile Hotspot)"
    if ip.startswith("100."):
        return "  (Tailscale)"
    return ""


def print_banner(host: str, port: int, token: str, root: Path,
                 max_file_mb: int, ttl_hours: float) -> None:
    line = "=" * 68
    print(line)
    print(f" LabLink server {__version__}   protocol {PROTOCOL_VERSION}")
    print(f" store root : {root}")
    print(f" limits     : max file {max_file_mb} MB" +
          (f", TTL {ttl_hours} h" if ttl_hours else ", no TTL (files kept forever)"))
    print(f" listening  : {host}:{port}")
    print(line)
    print(" Clients connect with:")
    for ip in local_ipv4_addresses():
        print(f"   --url http://{ip}:{port} --token {token}{_describe_address(ip)}")
    print(f"   --url http://127.0.0.1:{port} --token {token}  (this machine only)")
    print(line)
    print(" If clients cannot connect, allow the port through the firewall")
    print(" (run once, in an ADMIN PowerShell, on THIS machine):")
    print(f'   netsh advfirewall firewall add rule name="LabLink {port}" '
          f"dir=in action=allow protocol=TCP localport={port}")
    print(" To remove it later:")
    print(f'   netsh advfirewall firewall delete rule name="LabLink {port}"')
    print(line)
    print(" The token is an anti-misdirection guard, NOT security.")
    print(" Traffic is plain HTTP — do not send sensitive data.")
    print(line, flush=True)


def _ttl_thread(store: FileStore, interval_s: float = 600) -> threading.Thread:
    def loop():
        while True:
            time.sleep(interval_s)
            try:
                store.cleanup_expired()
            except Exception:                     # never let housekeeping kill the server
                log.exception("TTL sweep failed")

    t = threading.Thread(target=loop, name="lablink-ttl", daemon=True)
    t.start()
    return t


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="lablink_server",
        description="LabLink file-exchange server (stdlib only).",
    )
    ap.add_argument("--root", default=None,
                    help="store directory (default: ./lablink_data next to this script)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--bind", default="0.0.0.0",
                    help="bind address (default: all interfaces)")
    ap.add_argument("--token", default=os.environ.get("LABLINK_TOKEN"),
                    help="shared token (or set LABLINK_TOKEN)")
    ap.add_argument("--max-file-mb", type=int, default=DEFAULT_MAX_FILE_MB)
    ap.add_argument("--ttl-hours", type=float, default=0,
                    help="delete files older than this (0 = keep forever)")
    ap.add_argument("--label", default="", help="friendly server name shown by /hello")
    ap.add_argument("--quiet", action="store_true", help="only log warnings")
    args = ap.parse_args(argv)

    if not args.token:
        ap.error("a token is required: pass --token or set LABLINK_TOKEN")

    logging.basicConfig(
        level=logging.WARNING if args.quiet else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    root = Path(args.root) if args.root else Path(__file__).resolve().parent.parent / "lablink_data"
    store = FileStore(
        root,
        max_file_bytes=args.max_file_mb * 1024 * 1024,
        ttl_seconds=args.ttl_hours * 3600,
    )
    if args.ttl_hours:
        store.cleanup_expired()
        _ttl_thread(store)

    srv = make_server(store, args.bind, args.port, args.token, args.label)
    print_banner(args.bind, args.port, args.token, root, args.max_file_mb, args.ttl_hours)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down…", flush=True)
    finally:
        srv.shutdown()
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
