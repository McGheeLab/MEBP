"""FileStore — the server-side storage engine. Every durability invariant lives here.

On-disk layout (per channel, all under one store root, same volume so
os.replace is atomic on NTFS):

    <root>/<channel>/
        _seq.json           {"seq": N}  atomic-rewrite counter
        data/<name>         the bytes       (visible <=> committed)
        meta/<name>.json    {name, size, sha256, seq, uploaded, meta}
        tmp/upload-<uuid>.part   in-flight uploads; swept at startup

VISIBILITY INVARIANT: a file exists <=> BOTH data/<name> and meta/<name>.json
exist. Upload commits the meta sidecar first, THEN os.replace()s the data
file into place; delete removes data first, THEN meta. A crash at any point
therefore leaves either a complete file or an invisible orphan sidecar
(harmlessly overwritten by the next upload attempt) — never a half-visible one.

COLLISION POLICY: re-upload of the same name with the same sha256 is an
idempotent no-op (the existing record is returned); same name with different
content is a Conflict. This is what lets clients crash and retry — or lose
their state files entirely — without duplicating or corrupting anything.

SEQ: a per-channel monotonic counter. Recovered on open as
max(_seq.json, max(sidecar seqs)) so deleting the counter file can never
reissue a seq. Clients poll "what's new since N" with it.
"""

from __future__ import annotations

import hashlib
import logging
import os
import shutil
import threading
import time
import uuid
from pathlib import Path

from .fsutil import atomic_write_json, read_json, validate_name
from .protocol import DEFAULT_MAX_FILE_MB, DISK_FREE_MARGIN_MB

log = logging.getLogger("lablink.store")


# ---------------------------------------------------------------------------
# Exceptions — the server maps these onto HTTP statuses.
# ---------------------------------------------------------------------------
class StoreError(Exception):
    """Base class for store failures."""


class ShaMismatch(StoreError):
    """Uploaded bytes do not hash to the claimed sha256 (-> 400)."""


class Conflict(StoreError):
    """Name already exists with different content (-> 409)."""


class TooLarge(StoreError):
    """Declared size exceeds the store's per-file limit (-> 413)."""


class DiskFull(StoreError):
    """Accepting the upload would leave too little free disk (-> 507)."""


class NotFound(StoreError):
    """No such file in the channel (-> 404)."""


class _Channel:
    """Per-channel state: directory paths, commit lock, seq counter."""

    def __init__(self, root: Path, name: str):
        self.name = name
        self.dir = root / name
        self.data = self.dir / "data"
        self.meta = self.dir / "meta"
        self.tmp = self.dir / "tmp"
        self.lock = threading.Lock()
        self.seq = 0


class _UploadSession:
    """One in-flight upload. Context manager: abandoning it removes the tmp part.

    write() streams body chunks to tmp while feeding the sha256 hash;
    commit() verifies the claimed sha and performs the atomic, ordered
    commit under the channel lock.
    """

    def __init__(self, store: "FileStore", ch: _Channel, name: str, declared_len: int):
        self._store = store
        self._ch = ch
        self._name = name
        self._declared_len = declared_len
        self._part = ch.tmp / f"upload-{uuid.uuid4().hex}.part"
        self._fh = open(self._part, "wb")
        self._hasher = hashlib.sha256()
        self._size = 0
        self._done = False

    # -- context manager ----------------------------------------------------
    def __enter__(self) -> "_UploadSession":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.abort()

    # -- streaming ----------------------------------------------------------
    def write(self, chunk: bytes) -> None:
        if self._done:
            raise StoreError("upload session already finished")
        self._size += len(chunk)
        if self._size > self._declared_len:
            raise StoreError(
                f"body exceeds declared Content-Length ({self._size} > {self._declared_len})"
            )
        self._hasher.update(chunk)
        self._fh.write(chunk)

    # -- terminal states ----------------------------------------------------
    def commit(self, claimed_sha: str, meta: dict | None = None) -> tuple[str, dict]:
        """Finish the upload. Returns ("created"|"duplicate", record).

        Raises ShaMismatch / Conflict. Commit order (the invariant):
        meta sidecar first, then os.replace of the data file, then the seq
        counter — see module docstring for the crash analysis.
        """
        if self._done:
            raise StoreError("upload session already finished")
        self._fh.close()
        digest = self._hasher.hexdigest()
        claimed = (claimed_sha or "").strip().lower()
        if digest != claimed:
            self.abort()
            raise ShaMismatch(f"sha256 mismatch: got {digest}, claimed {claimed or '<missing>'}")

        ch = self._ch
        with ch.lock:
            existing = self._store._record_if_visible(ch, self._name)
            if existing is None:
                # On a case-SENSITIVE server the lookup above cannot see a
                # collision, so look explicitly.
                clash_name = self._store._case_conflict(ch, self._name)
                existing = (self._store._record_if_visible(ch, clash_name)
                            if clash_name else None)

            if existing is not None:
                self.abort()
                # Identical content is always an idempotent no-op, even when
                # the stored name differs in case — only ONE file exists, so a
                # client can still fetch the channel safely, and refusing here
                # would break the retry-after-crash guarantee the whole design
                # rests on.
                if existing.get("sha256") == digest:
                    return "duplicate", existing
                # Different content. A name differing only in capitalisation
                # cannot be allowed alongside the existing one: Windows and
                # macOS treat them as the same file, so a client downloading
                # the channel into one folder would silently lose one of them.
                if existing.get("name") != self._name:
                    raise Conflict(
                        f"{ch.name} already holds '{existing.get('name')}', which "
                        f"differs from '{self._name}' only in capitalisation, and "
                        f"has different content. Windows and macOS treat those as "
                        f"one file, so a client fetching this channel could not "
                        f"keep both. Use a distinct name."
                    )
                raise Conflict(
                    f"{ch.name}/{self._name} already exists with different content "
                    f"(existing sha {existing.get('sha256', '?')[:12]}, "
                    f"new {digest[:12]}). Delete it first, or use a unique name."
                )

            seq = ch.seq + 1
            record = {
                "name": self._name,
                "size": self._size,
                "sha256": digest,
                "seq": seq,
                "uploaded": time.time(),
                "meta": meta or {},
            }
            atomic_write_json(ch.meta / f"{self._name}.json", record)
            # The moment of visibility: data file appears complete, atomically.
            os.replace(self._part, ch.data / self._name)
            ch.seq = seq
            atomic_write_json(ch.dir / "_seq.json", {"seq": seq})
        self._done = True
        return "created", record

    def abort(self) -> None:
        """Remove the tmp part; safe to call twice / after commit."""
        if self._done:
            return
        self._done = True
        try:
            self._fh.close()
        except OSError:
            pass
        try:
            self._part.unlink(missing_ok=True)
        except OSError:
            pass


class FileStore:
    """Channelized, crash-safe file store. Thread-safe.

    Only the commit critical section is serialized (per channel); body
    streaming of concurrent uploads runs in parallel.
    """

    def __init__(
        self,
        root: Path | str,
        max_file_bytes: int = DEFAULT_MAX_FILE_MB * 1024 * 1024,
        ttl_seconds: float = 0,
    ):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.max_file_bytes = int(max_file_bytes)
        self.ttl_seconds = float(ttl_seconds)
        self._channels: dict[str, _Channel] = {}
        self._store_lock = threading.Lock()

    # ------------------------------------------------------------------ channels
    def open_channel(self, channel: str) -> _Channel:
        """Validate, create dirs, sweep stale tmp parts, recover seq. Cached."""
        validate_name(channel)
        with self._store_lock:
            ch = self._channels.get(channel)
            if ch is not None:
                return ch
            ch = _Channel(self.root, channel)
            for d in (ch.data, ch.meta, ch.tmp):
                d.mkdir(parents=True, exist_ok=True)
            # Sweep in-flight parts from a previous process life.
            for stale in ch.tmp.glob("*"):
                try:
                    stale.unlink()
                except OSError:
                    pass
            # Sweep data files with no sidecar: invisible either way, so this
            # only reclaims disk. Safe because an upload writes the sidecar
            # BEFORE the data file appears, so a commit in flight never looks
            # like an orphan.
            for orphan in ch.data.glob("*"):
                if not (ch.meta / f"{orphan.name}.json").exists():
                    try:
                        orphan.unlink()
                        log.info("reclaimed orphaned data file %s/%s",
                                 channel, orphan.name)
                    except OSError:
                        pass
            # Seq recovery: the counter file can be lost/corrupt without ever
            # reissuing a seq, because committed sidecars carry their seqs.
            counter = read_json(ch.dir / "_seq.json", default={}) or {}
            max_sidecar = 0
            for mp in ch.meta.glob("*.json"):
                rec = read_json(mp)
                if isinstance(rec, dict) and isinstance(rec.get("seq"), int):
                    max_sidecar = max(max_sidecar, rec["seq"])
            ch.seq = max(int(counter.get("seq", 0) or 0), max_sidecar)
            self._channels[channel] = ch
            return ch

    # ------------------------------------------------------------------ read side
    def _record_if_visible(self, ch: _Channel, name: str) -> dict | None:
        """The visibility invariant, as code: record iff data AND sidecar exist."""
        data_path = ch.data / name
        rec = read_json(ch.meta / f"{name}.json")
        if rec is None or not data_path.exists():
            return None
        return rec

    def _case_conflict(self, ch: _Channel, name: str) -> str | None:
        """An existing visible name differing from *name* only in capitalisation.

        Needed because the server may run on a case-SENSITIVE filesystem
        (Linux) while its clients do not (Windows, macOS). Without this a
        channel could hold both "Scan.png" and "scan.png", and a client
        fetching the channel into one folder would silently overwrite one with
        the other. Costs one directory listing per new upload, against a
        transfer measured in seconds.
        """
        lowered = name.lower()
        try:
            entries = ch.meta.glob("*.json")
        except OSError:
            return None
        for mp in entries:
            other = mp.name[:-len(".json")]
            if other != name and other.lower() == lowered:
                if (ch.data / other).exists():
                    return other
        return None

    def list_files(self, channel: str, since_seq: int = 0) -> tuple[int, list[dict]]:
        """(latest_seq, records with seq > since_seq, sorted by seq).

        Listing a channel that has never been used returns (0, []) WITHOUT
        creating it, so a poll of a misspelled or not-yet-written channel
        cannot litter the store with directories.
        """
        validate_name(channel)
        if not (self.root / channel).is_dir():
            return 0, []
        ch = self.open_channel(channel)
        # Snapshot the counter BEFORE listing. Read afterwards it could include
        # a commit that landed while the directory scan was already past that
        # entry, and a client trusting the returned seq as "nothing older is
        # outstanding" would step over a file it never saw.
        latest = ch.seq
        records = []
        for mp in ch.meta.glob("*.json"):
            rec = read_json(mp)
            if not isinstance(rec, dict) or "name" not in rec:
                continue
            if not (ch.data / rec["name"]).exists():
                continue  # orphan sidecar — invisible by invariant
            if rec.get("seq", 0) > since_seq:
                records.append(rec)
        records.sort(key=lambda r: r.get("seq", 0))
        return max(latest, records[-1]["seq"] if records else 0), records

    def open_read(self, channel: str, name: str, offset: int = 0):
        """(open binary file object positioned at offset, record). Raises NotFound."""
        validate_name(name)
        ch = self.open_channel(channel)
        rec = self._record_if_visible(ch, name)
        if rec is None:
            raise NotFound(f"{channel}/{name}")
        fh = open(ch.data / name, "rb")
        if offset:
            fh.seek(offset)
        return fh, rec

    # ------------------------------------------------------------------ write side
    def begin_upload(self, channel: str, name: str, declared_len: int) -> _UploadSession:
        """Pre-checks (name, size, free disk) then opens an upload session."""
        validate_name(name)
        ch = self.open_channel(channel)
        if declared_len > self.max_file_bytes:
            raise TooLarge(
                f"{declared_len} bytes exceeds the {self.max_file_bytes}-byte limit"
            )
        free = shutil.disk_usage(self.root).free
        margin = DISK_FREE_MARGIN_MB * 1024 * 1024
        if free < declared_len + margin:
            raise DiskFull(
                f"only {free // (1024 * 1024)} MB free on the store volume"
            )
        return _UploadSession(self, ch, name, declared_len)

    def delete(self, channel: str, name: str, *,
               only_if_uploaded_before: float | None = None) -> bool:
        """Remove a file. Returns False if it was not there.

        Order: SIDECAR FIRST. Removing either half makes the file invisible
        (the invariant needs both), so both orders are crash-safe — but only
        this one survives Windows refusing to unlink a file that a concurrent
        download still has open. A data file that cannot be removed yet is left
        as an invisible orphan and reclaimed when the server next starts (the
        sweep runs on a channel's FIRST open in a process, so it is a restart
        sweep, not a per-delete one).

        only_if_uploaded_before: delete only if the record is older than this
        timestamp — used by the TTL sweep so it cannot destroy a file that was
        replaced after the sweep listed it.
        """
        validate_name(name)
        ch = self.open_channel(channel)
        with ch.lock:
            rec = self._record_if_visible(ch, name)
            if rec is None:
                return False
            if only_if_uploaded_before is not None:
                # Re-check the timestamp under the lock. The TTL sweep decides
                # what is expired from a listing taken earlier; without this,
                # a file deleted and re-uploaded in between would be destroyed
                # by the sweep while the sender believed it was delivered.
                if rec.get("uploaded", 0) >= only_if_uploaded_before:
                    return False
            try:
                (ch.meta / f"{name}.json").unlink(missing_ok=True)
            except OSError as exc:
                # A concurrent reader can hold the sidecar open on Windows.
                # Nothing was removed, so report "not deleted" rather than
                # letting the error escape to the HTTP layer.
                log.info("could not remove sidecar for %s/%s: %s",
                         channel, name, exc)
                return False
            try:
                (ch.data / name).unlink(missing_ok=True)
            except OSError as exc:
                # In use (Windows) or permissions. Already invisible; the
                # bytes get reclaimed at the next process start.
                log.info("deferred removal of %s/%s: %s", channel, name, exc)
            return True

    # ------------------------------------------------------------------ housekeeping
    def cleanup_expired(self) -> int:
        """Delete records older than ttl_seconds (0 = keep forever). Returns count."""
        if self.ttl_seconds <= 0:
            return 0
        cutoff = time.time() - self.ttl_seconds
        removed = 0
        # Walk channels on DISK, not just the in-memory cache, so a restarted
        # server still expires channels it hasn't served yet this life.
        for chdir in self.root.iterdir():
            if not chdir.is_dir():
                continue
            try:
                ch = self.open_channel(chdir.name)
            except ValueError:
                continue  # a foreign directory in the root — leave it alone
            _, records = self.list_files(ch.name)
            for rec in records:
                if rec.get("uploaded", 0) < cutoff:
                    if self.delete(ch.name, rec["name"],
                                   only_if_uploaded_before=cutoff):
                        removed += 1
        if removed:
            log.info("TTL sweep removed %d expired file(s)", removed)
        return removed
