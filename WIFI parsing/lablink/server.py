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
import re
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

# Much smaller allowance before a token has been presented: enough that a
# mistyped token still yields a readable 401, too small to be an amplifier.
UNAUTH_DRAIN_LIMIT = 64 * 1024

# Absolute ceiling on one connection's lifetime. Handler.timeout is per-read
# and resets on every byte, so without this a peer dribbling one byte a minute
# pins a thread indefinitely.
MAX_CONNECTION_SECONDS = 3600.0

# Cap on simultaneous connections. Each one costs an OS thread, and accept
# happens before any authentication, so an unauthenticated peer could otherwise
# spawn threads until the process dies.
MAX_CONNECTIONS = 64


def _reject_json_constant(token: str):
    """Refuse NaN / Infinity / -Infinity, which are not valid JSON."""
    raise ValueError(f"{token} is not valid JSON")


class Handler(BaseHTTPRequestHandler):
    """One request. Every response carries an exact Content-Length (HTTP/1.1)."""

    protocol_version = "HTTP/1.1"
    timeout = DEFAULT_TIMEOUT_S          # reclaims threads parked on a dead peer
    server_version = f"LabLink/{__version__}"
    sys_version = ""

    # ------------------------------------------------------------------ helpers
    def setup(self):
        super().setup()
        self._deadline = time.monotonic() + MAX_CONNECTION_SECONDS

    def _past_deadline(self) -> bool:
        """True once this connection has outlived MAX_CONNECTION_SECONDS.

        Checked inside every byte-consuming loop, because the socket timeout is
        per-operation and resets on each successful read: a peer sending one
        byte per 29 s would otherwise hold a thread for as long as it liked.
        """
        return time.monotonic() > getattr(self, "_deadline", float("inf"))

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

    def _token_ok(self) -> bool:
        """Constant-time token comparison.

        Compares BYTES: headers are decoded as latin-1, so any byte 0x80-0xFF
        in the token header yields a non-ASCII str, and hmac.compare_digest
        raises TypeError on those — an unauthenticated caller could crash the
        handler and dump a traceback with one malformed header.
        """
        expected = str(self.server.token)          # type: ignore[attr-defined]
        got = self.headers.get(H_TOKEN, "") or ""
        return hmac.compare_digest(got.encode("utf-8", "surrogateescape"),
                                   expected.encode("utf-8", "surrogateescape"))

    def _check_token(self) -> bool:
        """As _token_ok, but sends 401 and returns False on failure."""
        if self._token_ok():
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

    def _reject_with_body(self, status: int, message: str,
                          max_drain: int = DRAIN_LIMIT) -> None:
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
        if remaining > max_drain:
            self.close_connection = True
        else:
            while remaining > 0:
                if self._past_deadline():
                    self.close_connection = True
                    break
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
        if not self._token_ok():
            # Drain only a SMALL body for an unauthenticated caller, so a
            # legitimate client with a mistyped token still reads a clean 401
            # instead of a connection reset — while a large body cannot be used
            # to make us do megabytes of read syscalls for free.
            self._reject_with_body(401, "bad or missing token",
                                   max_drain=UNAUTH_DRAIN_LIMIT)
            return
        if self.headers.get("Transfer-Encoding"):
            # Not supported, and silently mis-framing it is how request
            # smuggling starts if anyone ever puts a proxy in front.
            self.close_connection = True
            self._error(400, "Transfer-Encoding is not supported; "
                             "send the body with an explicit Content-Length")
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
        except OSError as exc:
            log.exception("deleting %s/%s failed", channel, name)
            self._error(500, f"could not delete {name}: {exc}")
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
        except OSError as exc:
            log.exception("opening %s/%s failed", channel, name)
            self._error(500, f"could not read {name}: {exc}")
            return

        # Validate the record BEFORE anything that could raise, because the file
        # handle is not yet owned by a `with` block — a KeyError here would leak
        # it, and on Windows a leaked read handle makes the file undeletable for
        # the life of the process.
        try:
            size = int(rec["size"])
            sha = str(rec["sha256"])
        except (KeyError, TypeError, ValueError):
            fh.close()
            log.error("corrupt sidecar for %s/%s: %r", channel, name, rec)
            self._error(500, f"metadata for {name} is corrupt")
            return

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
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header(H_SHA, sha)
            self.end_headers()
            try:
                remaining = length
                while remaining > 0:
                    chunk = fh.read(min(CHUNK, remaining))
                    if not chunk:
                        # Fewer bytes on disk than advertised: we have already
                        # committed to a Content-Length, so the only honest way
                        # out is to close and let the client retry.
                        log.error("%s/%s is shorter than its record claims",
                                  channel, name)
                        self.close_connection = True
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
            self.close_connection = True
            self._error(400, "Content-Length is required")
            return
        # Strict: int() would accept "1_0" (=10), " 5 ", "+7" and non-Latin
        # digits like "٥" (=5). A proxy in front of us would read those
        # differently, which is the seed of a request-smuggling desync.
        if not re.fullmatch(r"[0-9]+", raw_len.strip()):
            self.close_connection = True
            self._error(400, "Content-Length must be a plain non-negative integer")
            return
        declared = int(raw_len.strip())

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
                # parse_constant rejects NaN/Infinity/-Infinity. Python accepts
                # them and re-emits them verbatim, but they are NOT legal JSON:
                # one such value would be stored in the sidecar and then appear
                # in EVERY listing of that channel, permanently breaking MATLAB
                # jsondecode, JavaScript JSON.parse, Go, jq and every other
                # strict parser — while Python clients noticed nothing.
                meta = json.loads(raw_meta, parse_constant=_reject_json_constant)
            except ValueError as exc:
                self._reject_with_body(400, f"{H_META} is not valid JSON: {exc}")
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
                    if self._past_deadline():
                        raise StoreError(
                            f"upload exceeded the {MAX_CONNECTION_SECONDS:.0f}s "
                            f"connection limit with {remaining} bytes outstanding"
                        )
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
        except (OSError, KeyError) as exc:
            # A filesystem failure (path too long, disk error, a file held open
            # by a concurrent reader) or a corrupt sidecar. Without this the
            # exception escapes the handler, the socket closes with NO response
            # at all, and the client misreports it as "server unreachable" and
            # retries four times.
            log.exception("storing %s/%s failed", channel, name)
            self.close_connection = True
            self._error(500, f"could not store {name}: {exc}")
            return

        self._send_json(201 if status == "created" else 200, record)

    # ------------------------------------------------------------------ logging
    def log_message(self, fmt, *args):  # noqa: A003 - stdlib signature
        log.info("%s %s", self.address_string(), fmt % args)

    def log_error(self, fmt, *args):
        log.warning("%s %s", self.address_string(), fmt % args)


class BoundedThreadingHTTPServer(ThreadingHTTPServer):
    """ThreadingHTTPServer with a hard cap on simultaneous connections.

    Plain ThreadingHTTPServer spawns a thread per connection with no limit, and
    it does so BEFORE any token is checked — so an unauthenticated peer on the
    same network could open sockets until the process ran out of memory. Over
    the cap we close immediately rather than queueing, because a lab tool
    failing fast and visibly beats one that degrades mysteriously.
    """

    max_connections = MAX_CONNECTIONS

    def __init__(self, *args, **kwargs):
        self._conn_lock = threading.Lock()
        self._conn_count = 0
        self.rejected_connections = 0
        super().__init__(*args, **kwargs)

    def process_request(self, request, client_address):
        with self._conn_lock:
            if self._conn_count >= self.max_connections:
                self.rejected_connections += 1
                log.warning("refusing connection from %s: %d already open "
                            "(cap %d)", client_address[0], self._conn_count,
                            self.max_connections)
                try:
                    request.close()
                except OSError:
                    pass
                return
            self._conn_count += 1
        super().process_request(request, client_address)

    def shutdown_request(self, request):
        try:
            super().shutdown_request(request)
        finally:
            with self._conn_lock:
                self._conn_count = max(0, self._conn_count - 1)


def make_server(store: FileStore, host: str, port: int, token: str,
                label: str = "") -> ThreadingHTTPServer:
    """Build the HTTP server with the store and token attached."""
    srv = BoundedThreadingHTTPServer((host, port), Handler)
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
    # The token is NOT printed: this output is normally redirected to a log
    # file with default ACLs, which would hand the shared secret to every local
    # account and to log collection.
    masked = f"{token[:3]}…{token[-2:]}" if len(token) > 6 else "…"
    print(f" Clients connect with  --url <address below>  --token {masked}")
    print(" (token shown masked; use the value you configured)")
    for ip in local_ipv4_addresses():
        print(f"   http://{ip}:{port}{_describe_address(ip)}")
    print(f"   http://127.0.0.1:{port}  (this machine only)")
    print(line)
    print(" If clients cannot connect, allow the port through the firewall")
    print(" (run once, in an ADMIN PowerShell, on THIS machine).")
    print(" Over Tailscale, scope the rule to the tailnet so the port is NOT")
    print(" exposed to the local Wi-Fi:")
    print(f'   netsh advfirewall firewall add rule name="LabLink {port}" '
          f"dir=in action=allow protocol=TCP localport={port} "
          f"remoteip=100.64.0.0/10")
    print(" Only if you need plain-LAN access (opens the port to the whole")
    print(" network segment):")
    print(f'   netsh advfirewall firewall add rule name="LabLink {port} LAN" '
          f"dir=in action=allow protocol=TCP localport={port}")
    print(" To remove:")
    print(f'   netsh advfirewall firewall delete rule name="LabLink {port}"')
    print(line)
    print(" The token is an anti-misdirection guard, NOT security.")
    print(" Traffic is plain HTTP -- do not send sensitive data.")
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
    ap.add_argument("--token", default=None,
                    help="shared token. PREFER the LABLINK_TOKEN environment "
                         "variable: a command line is visible to every local "
                         "user via the process table")
    ap.add_argument("--max-file-mb", type=int, default=DEFAULT_MAX_FILE_MB)
    ap.add_argument("--ttl-hours", type=float, default=0,
                    help="delete files older than this (0 = keep forever)")
    ap.add_argument("--label", default="", help="friendly server name shown by /hello")
    ap.add_argument("--quiet", action="store_true", help="only log warnings")
    args = ap.parse_args(argv)

    token = args.token or os.environ.get("LABLINK_TOKEN")
    if not token:
        ap.error("a token is required: set LABLINK_TOKEN (preferred) or pass --token")
    if args.token:
        print("note: --token is visible in the process table to other local "
              "users; prefer the LABLINK_TOKEN environment variable.",
              file=sys.stderr)

    # stdout, not the default stderr: the usual way to run this is
    #   Start-Process ... -RedirectStandardOutput server.log
    # and sending the request log to stderr would silently drop exactly the
    # lines you want when diagnosing a client that cannot connect.
    logging.basicConfig(
        level=logging.WARNING if args.quiet else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        stream=sys.stdout,
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

    srv = make_server(store, args.bind, args.port, token, args.label)
    print_banner(args.bind, args.port, token, root, args.max_file_mb, args.ttl_hours)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down...", flush=True)
    finally:
        srv.shutdown()
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
