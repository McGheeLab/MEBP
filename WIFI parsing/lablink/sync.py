"""SyncAgent — mirror a local outbox folder to a channel, and a channel to an inbox.

This is the piece that makes LabLink "the machine watches a folder": drop a
file in the outbox and it appears on the other machine's inbox, with no code
on either side of the workflow. MATLAB, ImageJ, MEBP and plain scripts all
participate by reading and writing ordinary files.

Two robustness properties matter most:

STABILITY GATE — a file is uploaded only once its (size, mtime) has been
identical across two consecutive scans. Without it the agent would happily
upload the first half of a mosaic that MEBP or MATLAB is still writing.

IDEMPOTENCY — the state file is an optimization, not a correctness
requirement. Delete it and the agent re-hashes the outbox (uploads become
server-side no-ops) and re-lists the inbox channel (downloads are skipped
where the local file already matches). Correctness lives in checksums and
names, never in remembered state.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from pathlib import Path

from .client import LabLinkClient, LabLinkError, PARTIAL_DIR
from .fsutil import atomic_write_json, read_json, sha256_file, validate_name

log = logging.getLogger("lablink.sync")

CONFLICT_RELOG_S = 600          # re-report an unresolved 409 at most this often

CONFIG_TEMPLATE = {
    "url": "http://192.168.137.1:8765",
    "token": "CHANGE-ME",
    "outbox": {"dir": "C:/lablink/outbox", "channel": "mebp-out"},
    "inbox": {"dir": "C:/lablink/inbox", "channel": "results"},
    "poll_seconds": 5,
    "state_file": "C:/lablink/sync_state.json",
}


class SyncAgent:
    """Bidirectional folder<->channel mirror. Restartable at any moment."""

    def __init__(self, cfg: dict):
        self.cfg = cfg
        url = cfg.get("url")
        token = cfg.get("token")
        if not url or not token:
            raise ValueError("config needs both 'url' and 'token'")

        self.outbox = cfg.get("outbox") or None
        self.inbox = cfg.get("inbox") or None
        if not self.outbox and not self.inbox:
            raise ValueError("config needs an 'outbox', an 'inbox', or both")

        # Enforced, not merely documented: one agent using the same channel in
        # both directions would download its own uploads forever.
        if (self.outbox and self.inbox
                and self.outbox.get("channel") == self.inbox.get("channel")):
            raise ValueError(
                f"outbox and inbox must use DIFFERENT channels "
                f"(both are '{self.outbox.get('channel')}'). Use one channel per "
                f"direction, e.g. outbox 'mebp-out' and inbox 'results'."
            )
        for box, label in ((self.outbox, "outbox"), (self.inbox, "inbox")):
            if box:
                if not box.get("dir") or not box.get("channel"):
                    raise ValueError(f"{label} needs both 'dir' and 'channel'")
                validate_name(box["channel"])

        self.poll_seconds = float(cfg.get("poll_seconds", 5))
        self.client = LabLinkClient(url, token, timeout=cfg.get("timeout", 60))
        # The polling loop IS the retry mechanism, so a long in-call retry
        # chain only delays the other direction. Keep per-call retries low.
        self.retries = max(1, int(cfg.get("retries", 2)))

        state_file = cfg.get("state_file")
        self.state_path = Path(state_file) if state_file else None
        self._state = self._load_state()
        # Cycle 0 reconciles, so every start-up does a full pass.
        self.reconcile_cycles = max(1, int(cfg.get("reconcile_cycles", 60)))
        self._cycle = 0
        self._pending: dict[str, tuple[int, int]] = {}   # name -> (size, mtime_ns)
        self._skipped_names: set[str] = set()
        self._conflict_logged: dict[str, float] = {}
        self.running = False

    # ------------------------------------------------------------------ state
    def _load_state(self) -> dict:
        base = {"uploaded": {}, "failed": {}, "since_seq": 0}
        if not self.state_path:
            return base
        stored = read_json(self.state_path, default=None)
        if isinstance(stored, dict):
            base.update({k: stored.get(k, v) for k, v in base.items()})
        return base

    def _save_state(self) -> None:
        if not self.state_path:
            return
        try:
            self.state_path.parent.mkdir(parents=True, exist_ok=True)
            atomic_write_json(self.state_path, self._state)
        except OSError as exc:
            log.warning("could not save state to %s: %s", self.state_path, exc)

    # ------------------------------------------------------------------ outbox
    def _scan_outbox(self) -> None:
        box = self.outbox
        if not box:
            return
        d = Path(box["dir"])
        try:
            d.mkdir(parents=True, exist_ok=True)
            entries = [p for p in d.iterdir() if p.is_file()]
        except OSError as exc:
            log.warning("cannot read outbox %s: %s", d, exc)
            return

        seen_now: dict[str, tuple[int, int]] = {}
        for path in entries:
            name = path.name
            if name.startswith("."):
                continue
            try:
                validate_name(name)
            except ValueError as exc:
                if name not in self._skipped_names:
                    self._skipped_names.add(name)
                    log.warning("skipping %s: %s", name, exc)
                continue
            try:
                st = path.stat()
            except OSError:
                continue                       # vanished between listing and stat
            sig = (st.st_size, st.st_mtime_ns)
            seen_now[name] = sig

            # Stability gate: identical signature required on two scans running.
            if self._pending.get(name) != sig:
                continue

            try:
                digest = sha256_file(path)
            except OSError as exc:
                log.warning("cannot read %s: %s", name, exc)
                continue
            if self._state["uploaded"].get(name) == digest:
                continue                        # already sent, unchanged
            if self._state["failed"].get(name) == digest:
                self._maybe_relog_conflict(name)
                continue                        # known-conflicting, don't retry-storm

            try:
                rec = self.client.upload(box["channel"], path, name=name,
                                         retries=self.retries)
            except LabLinkError as exc:
                if exc.status == 409:
                    self._state["failed"][name] = digest
                    self._save_state()
                    self._log_conflict(name, exc)
                else:
                    log.warning("upload %s failed: %s", name, exc)
                continue

            self._state["uploaded"][name] = digest
            self._state["failed"].pop(name, None)
            self._conflict_logged.pop(name, None)
            self._save_state()
            log.info("sent %s (%s, %d bytes, seq %s)",
                     name, rec["status"], rec["size"], rec["seq"])

        self._pending = seen_now

    def _log_conflict(self, name: str, exc: LabLinkError) -> None:
        self._conflict_logged[name] = time.time()
        log.error(
            "CONFLICT: %s exists on the server with different content and will "
            "NOT be sent. %s Rename the local file (timestamps work well), or "
            "delete the server copy: lablink_cli.py delete %s \"%s\" --yes",
            name, exc.message, self.outbox["channel"], name,
        )

    def _maybe_relog_conflict(self, name: str) -> None:
        last = self._conflict_logged.get(name, 0)
        if time.time() - last > CONFLICT_RELOG_S:
            self._conflict_logged[name] = time.time()
            log.error("CONFLICT (still unresolved): %s is not being sent", name)

    # ------------------------------------------------------------------ inbox
    def _have_locally(self, local: Path, rec: dict, verify_sha: bool) -> bool:
        """Is this record already on disk? (What makes a lost state file harmless.)

        A reconcile pass compares size only: hashing every local file would
        mean re-reading the whole inbox, which is minutes of I/O once the
        folder holds a few hundred mosaics. Real downloads always verify the
        checksum, so a size match here only ever skips a file this agent (or
        the operator) already put in place.
        """
        try:
            if not local.exists() or local.stat().st_size != rec["size"]:
                return False
            return sha256_file(local) == rec["sha256"] if verify_sha else True
        except OSError:
            return False

    def _poll_inbox(self, full: bool = False) -> None:
        """Fetch new files. *full* re-lists from seq 0 instead of the cursor.

        The periodic full pass is a self-healing measure: if the cursor ever
        moves past a file that never landed — a bug, a hand-edited state file,
        a file restored on the server — an incremental poll would never look
        back and that file would be lost silently. The pass is cheap: files
        already on disk are skipped on a stat().
        """
        box = self.inbox
        if not box:
            return
        dest = Path(box["dir"])
        try:
            dest.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            log.warning("cannot create inbox %s: %s", dest, exc)
            return

        since = 0 if full else self._state["since_seq"]
        try:
            listing = self.client.list_files(box["channel"], since_seq=since)
        except LabLinkError as exc:
            log.warning("cannot list %s: %s", box["channel"], exc)
            return

        # since_seq may only advance over an UNBROKEN run of successes from the
        # start of the batch. Once one file fails, later successes must not
        # move the cursor past it, or that file would never be retried — it
        # would be silently lost. Later files are still downloaded (progress is
        # never blocked by one bad file); they are simply skipped cheaply next
        # cycle by the "already have it" check below.
        may_advance = True
        for rec in listing["files"]:
            name = rec["name"]
            if self._have_locally(dest / name, rec, verify_sha=not full):
                if may_advance:
                    self._advance_seq(rec["seq"])
                continue
            try:
                self.client.download(box["channel"], name, dest,
                                     expected_sha=rec["sha256"],
                                     retries=self.retries)
            except (LabLinkError, ValueError) as exc:
                log.warning("download %s failed (will retry): %s", name, exc)
                may_advance = False
                continue
            log.info("received %s%s (%d bytes, seq %s)",
                     name, " [recovered by reconcile]" if full else "",
                     rec["size"], rec["seq"])
            if may_advance:
                self._advance_seq(rec["seq"])

        # A clean full pass proves nothing is outstanding below the channel's
        # current seq, even where deletions left gaps.
        if full and may_advance:
            self._advance_seq(listing["seq"])

    def _advance_seq(self, seq: int) -> None:
        """Record progress per file, so a crash mid-batch resumes correctly."""
        if seq > self._state["since_seq"]:
            self._state["since_seq"] = seq
            self._save_state()

    # ------------------------------------------------------------------ loop
    def run_once(self) -> None:
        """One scan/poll cycle. Never raises for network or filesystem trouble.

        Every reconcile_cycles-th cycle (and always the first, i.e. at
        startup) the inbox is reconciled against the full channel listing
        rather than the cursor — see _poll_inbox.
        """
        full = (self._cycle % self.reconcile_cycles) == 0
        self._cycle += 1
        try:
            self._scan_outbox()
        except Exception:
            log.exception("outbox scan failed")
        try:
            self._poll_inbox(full=full)
        except Exception:
            log.exception("inbox poll failed")

    def run(self) -> None:
        self.running = True
        directions = []
        if self.outbox:
            directions.append(f"{self.outbox['dir']} -> [{self.outbox['channel']}]")
        if self.inbox:
            directions.append(f"[{self.inbox['channel']}] -> {self.inbox['dir']}")
        log.info("sync started: %s (every %.0fs)", "; ".join(directions), self.poll_seconds)
        try:
            while self.running:
                self.run_once()
                time.sleep(self.poll_seconds)
        except KeyboardInterrupt:
            log.info("stopping")
        finally:
            self.running = False
            self._save_state()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="lablink_sync",
        description="Mirror a local folder to a LabLink channel and back.",
    )
    ap.add_argument("--config", help="path to the sync config JSON")
    ap.add_argument("--make-config", metavar="PATH",
                    help="write a template config to PATH and exit")
    ap.add_argument("--once", action="store_true",
                    help="run a single cycle and exit (for scripts and tests)")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
    )

    if args.make_config:
        path = Path(args.make_config)
        if path.exists():
            print(f"refusing to overwrite {path}", file=sys.stderr)
            return 1
        path.parent.mkdir(parents=True, exist_ok=True)
        atomic_write_json(path, CONFIG_TEMPLATE)
        print(f"wrote a template config to {path}\n"
              f"Edit url, token and the folder paths, then run:\n"
              f'  python lablink_sync.py --config "{path}"')
        return 0

    if not args.config:
        ap.error("--config is required (or use --make-config to create one)")

    cfg = read_json(Path(args.config), default=None)
    if cfg is None:
        print(f"cannot read config: {args.config}", file=sys.stderr)
        return 1
    try:
        agent = SyncAgent(cfg)
    except ValueError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 1

    if args.once:
        agent.run_once()
    else:
        agent.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
