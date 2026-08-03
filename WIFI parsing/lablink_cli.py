#!/usr/bin/env python3
"""LabLink command-line client — send, get, list, delete and watch files.

Usage::

    set LABLINK_URL=http://192.168.137.1:8765
    set LABLINK_TOKEN=secret

    python lablink_cli.py hello
    python lablink_cli.py send mebp-out "mosaic 001.png" --meta "{\\"well\\":\\"A1\\"}"
    python lablink_cli.py list mebp-out
    python lablink_cli.py get results analysis.csv --dest .\\inbox
    python lablink_cli.py get results --all --dest .\\inbox
    python lablink_cli.py delete mebp-out "mosaic 001.png" --yes
    python lablink_cli.py watch results --dest .\\inbox

--url/--token may be given on the command line, in LABLINK_URL/LABLINK_TOKEN,
or in a JSON config file passed with --config.

Exit codes: 0 ok, 1 error, 2 usage.
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lablink.client import LabLinkClient, LabLinkError  # noqa: E402
from lablink.fsutil import human_size, read_json  # noqa: E402

EXIT_OK, EXIT_ERROR, EXIT_USAGE = 0, 1, 2


def human_time(epoch: float) -> str:
    try:
        return datetime.fromtimestamp(epoch).strftime("%Y-%m-%d %H:%M:%S")
    except (OverflowError, OSError, ValueError):
        return "?"


def build_client(args) -> LabLinkClient:
    cfg = read_json(Path(args.config), default={}) if args.config else {}
    url = args.url or os.environ.get("LABLINK_URL") or cfg.get("url")
    token = args.token or os.environ.get("LABLINK_TOKEN") or cfg.get("token")
    if not url:
        raise SystemExit("no server URL: pass --url, set LABLINK_URL, or use --config")
    if not token:
        raise SystemExit("no token: pass --token, set LABLINK_TOKEN, or use --config")
    return LabLinkClient(url, token, timeout=args.timeout)


# ---------------------------------------------------------------------------
# verbs
# ---------------------------------------------------------------------------
def cmd_hello(client: LabLinkClient, args) -> int:
    t0 = time.time()
    info = client.hello()
    rtt_ms = (time.time() - t0) * 1000
    skew = info.get("time", 0) - time.time()
    print(f"server   : {info.get('name')} (lablink {info.get('version')}, "
          f"protocol {info.get('protocol')})")
    print(f"url      : {client.base_url}")
    print(f"rtt      : {rtt_ms:.1f} ms")
    print(f"clock    : {abs(skew):.1f} s {'ahead' if skew > 0 else 'behind'} this machine"
          + ("   ** more than 2 s — check the clocks **" if abs(skew) > 2 else ""))
    limit = info.get("max_file_bytes")
    if limit:
        print(f"max file : {human_size(limit)}")
    return EXIT_OK


def cmd_list(client: LabLinkClient, args) -> int:
    listing = client.list_files(args.channel, since_seq=args.since_seq)
    if args.json:
        print(json.dumps(listing, indent=2))
        return EXIT_OK
    files = listing["files"]
    if not files:
        print(f"channel '{args.channel}' is empty"
              + (f" beyond seq {args.since_seq}" if args.since_seq else ""))
        return EXIT_OK
    print(f"{'SEQ':>5}  {'SIZE':>10}  {'SHA256':12}  {'UPLOADED':19}  NAME")
    for rec in files:
        line = (f"{rec['seq']:>5}  {human_size(rec['size']):>10}  "
                f"{rec['sha256'][:12]}  {human_time(rec.get('uploaded', 0)):19}  "
                f"{rec['name']}")
        if rec.get("meta"):
            line += f"   {json.dumps(rec['meta'])}"
        print(line)
    print(f"\n{len(files)} file(s); channel seq {listing['seq']}")
    return EXIT_OK


def cmd_send(client: LabLinkClient, args) -> int:
    meta = None
    if args.meta:
        try:
            meta = json.loads(args.meta)
        except ValueError as exc:
            raise SystemExit(f"--meta is not valid JSON: {exc}")
        if not isinstance(meta, dict):
            raise SystemExit("--meta must be a JSON object")
    if args.name and len(args.files) != 1:
        raise SystemExit("--name can only be used with a single file")

    failures = 0
    for path in args.files:
        p = Path(path)
        if not p.is_file():
            print(f"ERROR  {p}: not a file", file=sys.stderr)
            failures += 1
            continue
        t0 = time.time()
        try:
            rec = client.upload(args.channel, p, name=args.name, meta=meta)
        except (LabLinkError, ValueError) as exc:
            print(f"ERROR  {p.name}: {exc}", file=sys.stderr)
            failures += 1
            continue
        secs = max(time.time() - t0, 1e-6)
        rate = rec["size"] / secs / (1024 * 1024)
        print(f"{rec['status']:<9} {rec['name']}  ({human_size(rec['size'])}, "
              f"seq {rec['seq']}, {rate:.1f} MB/s)")
    return EXIT_ERROR if failures else EXIT_OK


def cmd_get(client: LabLinkClient, args) -> int:
    dest = Path(args.dest)
    if args.all:
        names = [r["name"] for r in client.list_files(args.channel)["files"]]
        if not names:
            print(f"channel '{args.channel}' is empty")
            return EXIT_OK
    else:
        if not args.name:
            raise SystemExit("give a file name, or --all")
        names = [args.name]

    # Pin each file's hash from the listing so downloads are verified against
    # what the listing promised, not merely against the server's own header.
    shas = {r["name"]: r.get("sha256")
            for r in client.list_files(args.channel).get("files", [])}
    failures = 0
    for name in names:
        t0 = time.time()
        try:
            out = client.download(args.channel, name, dest,
                                  expected_sha=shas.get(name))
        except (LabLinkError, ValueError, OSError) as exc:
            print(f"ERROR  {name}: {exc}", file=sys.stderr)
            failures += 1
            continue
        secs = max(time.time() - t0, 1e-6)
        size = out.stat().st_size
        print(f"got       {name}  ({human_size(size)}, "
              f"{size / secs / (1024 * 1024):.1f} MB/s) -> {out}")
    return EXIT_ERROR if failures else EXIT_OK


def cmd_delete(client: LabLinkClient, args) -> int:
    if not args.yes:
        reply = input(f"delete {args.channel}/{args.name}? [y/N] ").strip().lower()
        if reply not in ("y", "yes"):
            print("cancelled")
            return EXIT_OK
    client.delete(args.channel, args.name)
    print(f"deleted   {args.name}")
    return EXIT_OK


def cmd_watch(client: LabLinkClient, args) -> int:
    """Poll a channel, printing (and optionally downloading) each new file."""
    since = args.since_seq or 0
    if since == 0 and not args.from_start:
        since = client.list_files(args.channel)["seq"]
        print(f"watching '{args.channel}' from seq {since} "
              f"(use --from-start for existing files); Ctrl+C to stop")
    else:
        print(f"watching '{args.channel}' from seq {since}; Ctrl+C to stop")

    try:
        while True:
            try:
                listing = client.list_files(args.channel, since_seq=since)
            except LabLinkError as exc:
                print(f"  (server unreachable: {exc.message}) retrying...", file=sys.stderr)
                time.sleep(args.interval)
                continue
            # Advance only over an unbroken run of successes: bumping the cursor
            # after a later file succeeded would skip the failed one forever.
            may_advance = True
            for rec in listing["files"]:
                stamp = human_time(rec.get("uploaded", 0))
                print(f"[{stamp}] seq {rec['seq']:<5} {rec['name']} "
                      f"({human_size(rec['size'])})")
                if args.dest:
                    try:
                        out = client.download(args.channel, rec["name"],
                                              Path(args.dest),
                                              expected_sha=rec.get("sha256"))
                        print(f"           -> {out}")
                    except (LabLinkError, ValueError, OSError) as exc:
                        print(f"           ERROR downloading: {exc}", file=sys.stderr)
                        may_advance = False
                        continue
                if may_advance:
                    since = rec["seq"]
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nstopped")
        return EXIT_OK


# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="lablink_cli", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", help="server URL (or LABLINK_URL)")
    ap.add_argument("--token", help="shared token (or LABLINK_TOKEN)")
    ap.add_argument("--config", help="JSON file with {\"url\":…, \"token\":…}")
    ap.add_argument("--timeout", type=float, default=30)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("hello", help="check the server is reachable")

    p = sub.add_parser("list", help="list files in a channel")
    p.add_argument("channel")
    p.add_argument("--since-seq", type=int, default=None)
    p.add_argument("--json", action="store_true")

    p = sub.add_parser("send", help="upload one or more files")
    p.add_argument("channel")
    p.add_argument("files", nargs="+")
    p.add_argument("--name", help="store under this name (single file only)")
    p.add_argument("--meta", help="JSON object attached to the file")

    p = sub.add_parser("get", help="download a file (or all files)")
    p.add_argument("channel")
    p.add_argument("name", nargs="?")
    p.add_argument("--all", action="store_true")
    p.add_argument("--dest", default=".", help="destination folder (default: .)")

    p = sub.add_parser("delete", help="delete a file from a channel")
    p.add_argument("channel")
    p.add_argument("name")
    p.add_argument("--yes", action="store_true", help="skip the confirmation")

    p = sub.add_parser("watch", help="poll a channel and report (or fetch) new files")
    p.add_argument("channel")
    p.add_argument("--interval", type=float, default=5)
    p.add_argument("--dest", help="download new files into this folder")
    p.add_argument("--since-seq", type=int, default=0)
    p.add_argument("--from-start", action="store_true",
                   help="include files already on the server")

    args = ap.parse_args(argv)
    handlers = {
        "hello": cmd_hello, "list": cmd_list, "send": cmd_send,
        "get": cmd_get, "delete": cmd_delete, "watch": cmd_watch,
    }
    try:
        client = build_client(args)
        return handlers[args.cmd](client, args)
    except SystemExit as exc:
        if isinstance(exc.code, str):
            print(f"error: {exc.code}", file=sys.stderr)
            return EXIT_USAGE
        raise
    except LabLinkError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return EXIT_ERROR
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return EXIT_ERROR


if __name__ == "__main__":
    sys.exit(main())
