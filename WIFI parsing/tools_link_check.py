#!/usr/bin/env python3
"""Decide whether two lab machines can talk, and how fast — before installing anything.

This file is DELIBERATELY SELF-CONTAINED: it imports nothing from lablink and
nothing from MEBP, so you can copy this single file to the other machine on a
USB stick and run the reachability test on day one.

Usage::

    # On machine A (the would-be server), if no LabLink server is running yet:
    python tools_link_check.py --listen

    # On machine B:
    python tools_link_check.py --check http://<machine-A-ip>:8765

    # Against a real LabLink server (adds a genuine upload/download test):
    python tools_link_check.py --check http://192.168.137.1:8765 --token SECRET --mb 8

Why not just use ping? ICMP and TCP are filtered independently on campus
networks: ping can succeed where the port is blocked, and vice versa. This
tool tests the actual TCP port the file exchange will use.
"""

import argparse
import hashlib
import json
import os
import socket
import statistics
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

DEFAULT_PORT = 8765
CHUNK = 64 * 1024
H_TOKEN = "X-Lablink-Token"
H_SHA = "X-Lablink-Sha256"

PASS, FAIL, WARN, INFO = "[PASS]", "[FAIL]", "[WARN]", "[ .. ]"


def say(tag: str, message: str) -> None:
    print(f" {tag}  {message}", flush=True)


# ---------------------------------------------------------------------------
# --listen : a minimal probe responder
# ---------------------------------------------------------------------------
class ProbeHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    timeout = 30
    server_version = "LabLinkProbe/1.0"
    sys_version = ""

    def _json(self, status: int, obj: dict) -> None:
        body = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.split("?")[0] == "/hello":
            self._json(200, {"service": "lablink-probe", "probe": True,
                             "time": time.time(), "name": socket.gethostname()})
        else:
            self._json(404, {"error": "probe serves /hello and PUT /sink only"})

    def do_PUT(self):
        """Read and discard the body so upload throughput can be measured."""
        try:
            remaining = int(self.headers.get("Content-Length", 0) or 0)
        except ValueError:
            remaining = 0
        received = 0
        while remaining > 0:
            chunk = self.rfile.read(min(CHUNK, remaining))
            if not chunk:
                break
            received += len(chunk)
            remaining -= len(chunk)
        self._json(200, {"received": received})

    def log_message(self, fmt, *args):
        print(f"   probe: {self.address_string()} {fmt % args}", flush=True)


def local_ipv4_addresses() -> list:
    addrs = []
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


def describe(ip: str) -> str:
    if ip.startswith("192.168.137."):
        return "   (Windows Mobile Hotspot)"
    if ip.startswith("100."):
        return "   (Tailscale)"
    return ""


def run_listen(port: int) -> int:
    srv = ThreadingHTTPServer(("0.0.0.0", port), ProbeHandler)
    srv.daemon_threads = True
    line = "=" * 68
    print(line)
    print(f" LabLink probe listening on port {port}")
    print(line)
    print(" From the OTHER machine, run:")
    for ip in local_ipv4_addresses():
        print(f"   python tools_link_check.py --check http://{ip}:{port}{describe(ip)}")
    print(line)
    print(" If the check times out, allow the port (ADMIN PowerShell, this machine):")
    print(f'   netsh advfirewall firewall add rule name="LabLink {port}" '
          f"dir=in action=allow protocol=TCP localport={port}")
    print(line, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nprobe stopped")
    finally:
        srv.shutdown()
        srv.server_close()
    return 0


# ---------------------------------------------------------------------------
# --check : the reachability + throughput test
# ---------------------------------------------------------------------------
def _request(method: str, url: str, data=None, headers=None, timeout=30):
    req = urllib.request.Request(url, data=data, method=method)
    for k, v in (headers or {}).items():
        req.add_header(k, v)
    return urllib.request.urlopen(req, timeout=timeout)


def check_hello(url: str, timeout: float):
    """(info, median_rtt_ms) or (None, None). Prints its own PASS/FAIL."""
    rtts, info = [], None
    for _ in range(5):
        t0 = time.time()
        try:
            with _request("GET", f"{url}/hello", timeout=timeout) as resp:
                payload = json.loads(resp.read().decode())
        except urllib.error.HTTPError as exc:
            say(FAIL, f"GET /hello returned HTTP {exc.code} — is this a LabLink server?")
            return None, None
        except (urllib.error.URLError, OSError) as exc:
            reason = getattr(exc, "reason", exc)
            say(FAIL, f"cannot reach {url}: {reason}")
            return None, None
        rtts.append((time.time() - t0) * 1000)
        info = payload
    rtt = statistics.median(rtts)
    kind = "probe" if info.get("probe") else f"lablink {info.get('version', '?')}"
    say(PASS, f"reachable: {info.get('name', '?')} ({kind}), median RTT {rtt:.1f} ms")
    return info, rtt


def check_clock(info: dict) -> bool:
    skew = info.get("time", 0) - time.time()
    if abs(skew) > 2:
        say(WARN, f"clocks differ by {abs(skew):.1f} s — file timestamps across "
                  f"machines will look wrong (transfers still work: LabLink "
                  f"orders files by seq, not by clock)")
        return False
    say(PASS, f"clocks agree within {abs(skew):.1f} s")
    return True


def check_transfer(url: str, token: str, megabytes: int, is_probe: bool,
                   timeout: float) -> bool:
    """Upload then download a synthetic blob; print MB/s each way."""
    size = megabytes * 1024 * 1024
    blob = os.urandom(size)
    digest = hashlib.sha256(blob).hexdigest()
    name = f"linkcheck-{int(time.time())}.bin"

    if is_probe:
        target, headers = f"{url}/sink", {}
    else:
        target = f"{url}/c/linkcheck/{urllib.parse.quote(name, safe='')}"
        headers = {H_TOKEN: token, H_SHA: digest,
                   "Content-Type": "application/octet-stream"}

    # ---- upload -------------------------------------------------------
    t0 = time.time()
    try:
        with _request("PUT", target, data=blob, headers=headers, timeout=timeout) as resp:
            resp.read()
    except urllib.error.HTTPError as exc:
        if exc.code == 401:
            say(FAIL, "upload rejected: HTTP 401 — the token does not match the server's")
        else:
            body = exc.read()[:200].decode("utf-8", "replace")
            say(FAIL, f"upload failed: HTTP {exc.code} {body}")
        return False
    except (urllib.error.URLError, OSError) as exc:
        say(FAIL, f"upload failed: {getattr(exc, 'reason', exc)}")
        return False
    up_s = max(time.time() - t0, 1e-6)
    say(PASS, f"upload   {megabytes} MB in {up_s:.1f} s  =  {size / up_s / 1e6:.1f} MB/s "
              f"({size * 8 / up_s / 1e6:.0f} Mbit/s)")

    if is_probe:
        say(INFO, "download not tested against a probe (start a real server for that)")
        return True

    # ---- download -----------------------------------------------------
    t0 = time.time()
    try:
        with _request("GET", target, headers={H_TOKEN: token}, timeout=timeout) as resp:
            got = resp.read()
    except (urllib.error.HTTPError, urllib.error.URLError, OSError) as exc:
        say(FAIL, f"download failed: {exc}")
        return False
    down_s = max(time.time() - t0, 1e-6)
    if hashlib.sha256(got).hexdigest() != digest:
        say(FAIL, "downloaded bytes do not match what was uploaded (corruption!)")
        return False
    say(PASS, f"download {megabytes} MB in {down_s:.1f} s  =  "
              f"{len(got) / down_s / 1e6:.1f} MB/s "
              f"({len(got) * 8 / down_s / 1e6:.0f} Mbit/s), checksum verified")

    try:
        with _request("DELETE", target, headers={H_TOKEN: token}, timeout=timeout) as resp:
            resp.read()
        say(PASS, "test file deleted from the server")
    except Exception:
        say(WARN, f"could not delete the test file {name} — remove it manually")
    return True


HINTS = """
 What to do next
 ---------------
 connection refused    Nothing is listening on that port. Start the server (or
                       the --listen probe) on the other machine, check the port.
 timed out / no route  Most likely campus WiFi client isolation: the access
                       point drops machine-to-machine traffic. Try, in order:
                         1. Windows Mobile Hotspot on the server machine
                            (clients then use http://192.168.137.1:8765)
                         2. Tailscale on both machines (use the 100.x address)
                       Also confirm the firewall rule on the SERVER machine.
 HTTP 401              The token does not match. Both sides must use the same
                       --token (or LABLINK_TOKEN).
 slow (<2 MB/s)        If using Tailscale, run `tailscale status` — a "relay"
                       peer routes through the internet. A direct connection or
                       a Mobile Hotspot will be much faster.
"""


def run_check(url: str, token: str, megabytes: int, timeout: float) -> int:
    url = url.rstrip("/")
    line = "=" * 68
    print(line)
    print(f" LabLink link check -> {url}")
    print(line)

    info, _ = check_hello(url, timeout)
    if info is None:
        print(line)
        print(" LINK CHECK: FAIL (not reachable)")
        print(HINTS)
        return 1

    check_clock(info)  # advisory only

    is_probe = bool(info.get("probe"))
    if not is_probe and not token:
        say(WARN, "no --token given: skipping the transfer test "
                  "(reachability is confirmed)")
        print(line)
        print(" LINK CHECK: PASS (reachable; rerun with --token to measure throughput)")
        return 0

    ok = check_transfer(url, token or "", megabytes, is_probe, timeout)
    print(line)
    if ok:
        print(" LINK CHECK: PASS")
        return 0
    print(" LINK CHECK: FAIL (transfer)")
    print(HINTS)
    return 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="tools_link_check",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--listen", action="store_true",
                      help="run a minimal probe responder on this machine")
    mode.add_argument("--check", metavar="URL",
                      help="test the link to this server or probe")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="port for --listen")
    ap.add_argument("--token", default=os.environ.get("LABLINK_TOKEN"),
                    help="shared token (needed for the transfer test)")
    ap.add_argument("--mb", type=int, default=8, help="transfer test size in MB")
    ap.add_argument("--timeout", type=float, default=60)
    args = ap.parse_args(argv)

    if args.listen:
        return run_listen(args.port)
    return run_check(args.check, args.token, args.mb, args.timeout)


if __name__ == "__main__":
    sys.exit(main())
