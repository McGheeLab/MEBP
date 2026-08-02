#!/usr/bin/env python3
"""One command to connect this machine to the lab's LabLink server.

Run it with no arguments and it checks everything, in order, and tells you
exactly what to fix if something is wrong:

    python3 lablink_connect.py            (macOS / Linux)
    python  lablink_connect.py            (Windows)

Then, once the check passes:

    python3 lablink_connect.py send report.csv     send file(s) to the lab
    python3 lablink_connect.py get                 fetch anything new
    python3 lablink_connect.py list                see what is on the server

Settings are remembered in lablink_config.json beside this file, so you only
ever type the server address and token once (and the defaults below are
already filled in for this lab).

Deliberately forgiving: it can be run from any folder, works on Python 3.8+,
never needs arguments, and turns every failure into a sentence telling you
what to do next.
"""

import json
import os
import platform
import socket
import sys
import time
from pathlib import Path

# --- defaults for this lab; override in lablink_config.json or on the CLI ----
DEFAULT_URL = "http://10.134.186.83:8765"
DEFAULT_TOKEN = "k7ouj2tny6d0br3l5xsp"
DEFAULT_SEND_CHANNEL = "results"     # this machine -> lab
DEFAULT_GET_CHANNEL = "mebp-out"     # lab -> this machine

HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "lablink_config.json"
INBOX = HERE / "inbox"

MIN_PYTHON = (3, 8)
BAR = "=" * 68


# ---------------------------------------------------------------------------
# output helpers
# ---------------------------------------------------------------------------
def ok(msg):
    print(f" [OK]    {msg}", flush=True)


def bad(msg):
    print(f" [FAIL]  {msg}", flush=True)


def warn(msg):
    print(f" [WARN]  {msg}", flush=True)


def info(msg):
    print(f"         {msg}", flush=True)


def die(msg, hint=None):
    bad(msg)
    if hint:
        print()
        print(hint.rstrip())
    print(BAR)
    sys.exit(1)


# ---------------------------------------------------------------------------
# environment
# ---------------------------------------------------------------------------
def check_python():
    v = sys.version_info
    if v[:2] < MIN_PYTHON:
        die(
            f"Python {v.major}.{v.minor} is too old (need "
            f"{MIN_PYTHON[0]}.{MIN_PYTHON[1]}+).",
            "Install a newer Python from https://www.python.org/downloads/\n"
            "then run this script with that interpreter.",
        )
    ok(f"Python {v.major}.{v.minor}.{v.micro} on {platform.system()} "
       f"({socket.gethostname()})")


def load_lablink():
    """Import the lablink package that lives beside this script."""
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    try:
        from lablink.client import LabLinkClient, LabLinkError  # noqa: E402
        return LabLinkClient, LabLinkError
    except ImportError as exc:
        die(
            f"cannot import the lablink package: {exc}",
            f"This script must sit next to the 'lablink' folder.\n"
            f"It is currently in:\n    {HERE}\n"
            f"which contains: {', '.join(sorted(p.name for p in HERE.iterdir())[:8])}\n\n"
            f"Fix: copy the whole 'WIFI parsing' folder over, or pull the repo\n"
            f"branch Version-7.9.0 and run this from inside that folder.",
        )


# ---------------------------------------------------------------------------
# config
# ---------------------------------------------------------------------------
def load_config(args):
    cfg = {"url": DEFAULT_URL, "token": DEFAULT_TOKEN,
           "send_channel": DEFAULT_SEND_CHANNEL,
           "get_channel": DEFAULT_GET_CHANNEL}
    if CONFIG_PATH.exists():
        try:
            stored = json.loads(CONFIG_PATH.read_text())
            if isinstance(stored, dict):
                cfg.update({k: v for k, v in stored.items() if v})
        except (OSError, ValueError) as exc:
            warn(f"ignoring unreadable {CONFIG_PATH.name}: {exc}")
    if os.environ.get("LABLINK_URL"):
        cfg["url"] = os.environ["LABLINK_URL"]
    if os.environ.get("LABLINK_TOKEN"):
        cfg["token"] = os.environ["LABLINK_TOKEN"]
    if args.url:
        cfg["url"] = args.url
    if args.token:
        cfg["token"] = args.token
    cfg["url"] = cfg["url"].rstrip("/")
    return cfg


def save_config(cfg):
    try:
        CONFIG_PATH.write_text(json.dumps(cfg, indent=2))
    except OSError as exc:
        warn(f"could not save settings: {exc}")


# ---------------------------------------------------------------------------
# diagnosis
# ---------------------------------------------------------------------------
def my_addresses():
    addrs = []
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("10.255.255.255", 1))
        addrs.append(probe.getsockname()[0])
    except OSError:
        pass
    finally:
        probe.close()
    return addrs


def same_machine(url_host):
    """True if url_host is one of THIS machine's own addresses.

    Worth catching loudly: talking to your own IP succeeds instantly and
    looks like a perfect result, while proving nothing about the network.
    """
    try:
        if url_host in ("127.0.0.1", "localhost", socket.gethostname()):
            return True
        return url_host in my_addresses()
    except OSError:
        return False


def tcp_probe(host, port, timeout=6):
    """(reachable, detail) for a raw TCP connect — before any HTTP."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    t0 = time.time()
    try:
        s.connect((host, port))
        return True, (time.time() - t0) * 1000
    except socket.timeout:
        return False, "timeout"
    except ConnectionRefusedError:
        return False, "refused"
    except OSError as exc:
        return False, str(exc)
    finally:
        s.close()


NETWORK_HELP = """ What to try, in order
 ---------------------
 1. Is the server running on the lab machine? Its window should say
    "listening 0.0.0.0:8765".

 2. Is the firewall rule in place? On the LAB machine, in an Administrator
    PowerShell, once:
      netsh advfirewall firewall add rule name="LabLink 8765" dir=in \\
        action=allow protocol=TCP localport=8765

 3. Are both machines on the same network? University Wi-Fi usually blocks
    machine-to-machine traffic, which looks exactly like this timeout.
    Best fix, needs no installation:
      - on the LAB (Windows) machine: Settings > Network & Internet >
        Mobile hotspot > On
      - on THIS machine: join that hotspot from the Wi-Fi menu
      - then re-run:  python3 lablink_connect.py --url http://192.168.137.1:8765

 4. If the machines must stay on different networks, install Tailscale
    (https://tailscale.com/download) on both, sign in with the same account,
    and use the lab machine's 100.x.y.z address as the --url."""


def run_check(cfg, LabLinkClient, LabLinkError, megabytes):
    from urllib.parse import urlparse

    parsed = urlparse(cfg["url"])
    host, port = parsed.hostname, parsed.port or 80
    print(BAR)
    print(f" LabLink connection check  ->  {cfg['url']}")
    print(BAR)
    check_python()

    if same_machine(host):
        warn(f"{host} is THIS machine's own address.")
        info("A connection to yourself always succeeds without touching the")
        info("network, so this proves nothing about reaching the lab machine.")
        info("Use the LAB machine's address instead.")
        print(BAR)
        return 1

    # --- raw TCP first: separates "no route" from "HTTP problem" -----------
    reachable, detail = tcp_probe(host, port)
    if not reachable:
        if detail == "refused":
            bad(f"nothing is listening on {host}:{port} (connection refused)")
        elif detail == "timeout":
            bad(f"no reply from {host}:{port} (timed out)")
        else:
            bad(f"cannot reach {host}:{port} — {detail}")
        print()
        print(NETWORK_HELP)
        print(BAR)
        return 1
    ok(f"TCP connect to {host}:{port} in {detail:.0f} ms")

    client = LabLinkClient(cfg["url"], cfg["token"], timeout=30)

    # --- identity + clock --------------------------------------------------
    try:
        rtts = []
        for _ in range(5):
            t0 = time.time()
            hello = client.hello()
            rtts.append((time.time() - t0) * 1000)
    except LabLinkError as exc:
        die(f"the server did not answer /hello: {exc}",
            "Something is listening on that port but it is not a LabLink\n"
            "server. Check the port number.")
    rtt = sorted(rtts)[len(rtts) // 2]
    ok(f"server '{hello.get('name')}' (lablink {hello.get('version')}), "
       f"RTT {rtt:.0f} ms")
    if rtt < 2:
        warn("that round-trip is suspiciously fast for a real network hop —")
        info("check you are not pointing at this same machine.")

    skew = hello.get("time", 0) - time.time()
    if abs(skew) > 2:
        warn(f"clocks differ by {abs(skew):.0f} s. Transfers still work "
             f"(ordering uses sequence numbers), but timestamps will look odd.")
    else:
        ok(f"clocks agree within {abs(skew):.1f} s")

    # --- token + real transfer --------------------------------------------
    try:
        client.list_files(cfg["get_channel"])
    except LabLinkError as exc:
        if exc.status == 401:
            die("the token was rejected (HTTP 401).",
                "Both machines must use the same token. The lab machine prints\n"
                "it when the server starts. Pass it here with:\n"
                "    python3 lablink_connect.py --token THE-TOKEN")
        die(f"could not list files: {exc}")
    ok("token accepted")

    import hashlib
    import tempfile

    blob = os.urandom(megabytes * 1024 * 1024)
    name = f"connectcheck-{int(time.time())}.bin"
    with tempfile.TemporaryDirectory() as td:
        src = Path(td) / name
        src.write_bytes(blob)
        try:
            t0 = time.time()
            client.upload("linkcheck", src, retries=1)
            up = max(time.time() - t0, 1e-6)
            ok(f"upload   {megabytes} MB in {up:.1f} s = {len(blob)/up/1e6:.1f} MB/s "
               f"({len(blob)*8/up/1e6:.0f} Mbit/s)")

            t0 = time.time()
            got = client.download("linkcheck", name, Path(td) / "back", retries=1)
            down = max(time.time() - t0, 1e-6)
            if hashlib.sha256(got.read_bytes()).hexdigest() != \
                    hashlib.sha256(blob).hexdigest():
                die("the file came back CORRUPTED — do not trust this link.")
            ok(f"download {megabytes} MB in {down:.1f} s = "
               f"{got.stat().st_size/down/1e6:.1f} MB/s, checksum verified")
        except LabLinkError as exc:
            die(f"transfer failed: {exc}", NETWORK_HELP)
        finally:
            try:
                client.delete("linkcheck", name)
            except LabLinkError:
                pass

    save_config(cfg)
    print(BAR)
    print(" CONNECTION CHECK: PASS")
    print(BAR)
    print(" Settings saved — from now on you can just run:")
    print(f"   python3 {Path(__file__).name} send <file>     send to the lab")
    print(f"   python3 {Path(__file__).name} get             fetch new files")
    print(f"   python3 {Path(__file__).name} list            see what is there")
    print(BAR)
    return 0


# ---------------------------------------------------------------------------
# everyday actions
# ---------------------------------------------------------------------------
def run_send(cfg, client, LabLinkError, files, channel):
    if not files:
        die("give at least one file to send, e.g.\n"
            "    python3 lablink_connect.py send results.csv")
    failed = 0
    for raw in files:
        p = Path(raw).expanduser()
        if not p.is_file():
            bad(f"{raw}: not a file")
            failed += 1
            continue
        try:
            t0 = time.time()
            rec = client.upload(channel, p)
            secs = max(time.time() - t0, 1e-6)
            ok(f"{rec['status']:<9} {rec['name']}  "
               f"({rec['size']/1e6:.1f} MB, {rec['size']/secs/1e6:.1f} MB/s)")
        except (LabLinkError, ValueError) as exc:
            bad(f"{p.name}: {exc}")
            failed += 1
    return 1 if failed else 0


def run_get(cfg, client, LabLinkError, channel):
    INBOX.mkdir(exist_ok=True)
    try:
        listing = client.list_files(channel)
    except LabLinkError as exc:
        die(f"could not list '{channel}': {exc}")
    if not listing["files"]:
        info(f"nothing on the server in '{channel}' yet")
        return 0
    new = 0
    for rec in listing["files"]:
        local = INBOX / rec["name"]
        if local.exists() and local.stat().st_size == rec["size"]:
            continue
        try:
            client.download(channel, rec["name"], INBOX,
                            expected_sha=rec["sha256"])
            ok(f"got {rec['name']} ({rec['size']/1e6:.1f} MB)")
            new += 1
        except (LabLinkError, ValueError) as exc:
            bad(f"{rec['name']}: {exc}")
    info(f"{new} new file(s) in {INBOX}" if new else "already up to date")
    return 0


def run_list(cfg, client, LabLinkError, channel):
    try:
        listing = client.list_files(channel)
    except LabLinkError as exc:
        die(f"could not list '{channel}': {exc}")
    files = listing["files"]
    if not files:
        info(f"channel '{channel}' is empty")
        return 0
    print(f"{'SIZE':>10}  {'UPLOADED':19}  NAME")
    for rec in files:
        stamp = time.strftime("%Y-%m-%d %H:%M:%S",
                              time.localtime(rec.get("uploaded", 0)))
        print(f"{rec['size']/1e6:>7.1f} MB  {stamp:19}  {rec['name']}")
    return 0


# ---------------------------------------------------------------------------
def main(argv=None):
    import argparse

    ap = argparse.ArgumentParser(
        prog="lablink_connect",
        description="Connect this machine to the lab's LabLink file server.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Run with no arguments to check the connection.",
    )
    ap.add_argument("action", nargs="?", default="check",
                    choices=["check", "send", "get", "list"])
    ap.add_argument("files", nargs="*", help="files to send")
    ap.add_argument("--url", help="server address, e.g. http://192.168.137.1:8765")
    ap.add_argument("--token", help="shared token")
    ap.add_argument("--channel", help="override the channel for this action")
    ap.add_argument("--mb", type=int, default=8, help="check transfer size (MB)")
    args = ap.parse_args(argv)

    LabLinkClient, LabLinkError = load_lablink()
    cfg = load_config(args)

    if args.action == "check":
        return run_check(cfg, LabLinkClient, LabLinkError, args.mb)

    client = LabLinkClient(cfg["url"], cfg["token"], timeout=60)
    try:
        if args.action == "send":
            return run_send(cfg, client, LabLinkError, args.files,
                            args.channel or cfg["send_channel"])
        if args.action == "get":
            return run_get(cfg, client, LabLinkError,
                           args.channel or cfg["get_channel"])
        return run_list(cfg, client, LabLinkError,
                        args.channel or cfg["get_channel"])
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except Exception as exc:                      # never dump a raw traceback
        print(f"\n [FAIL]  unexpected error: {type(exc).__name__}: {exc}")
        print("         Re-run with LABLINK_DEBUG=1 for the full traceback.")
        if os.environ.get("LABLINK_DEBUG"):
            raise
        sys.exit(1)
