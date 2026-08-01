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
            if existing is not None:
                self.abort()
                if existing["sha256"] == digest:
                    return "duplicate", existing
                raise Conflict(
                    f"{ch.name}/{self._name} already exists with different content "
                    f"(existing sha {existing['sha256'][:12]}, new {digest[:12]}). "
                    f"Delete it first, or use a unique name."
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

    def list_files(self, channel: str, since_seq: int = 0) -> tuple[int, list[dict]]:
        """(latest_seq, records with seq > since_seq, sorted by seq)."""
        ch = self.open_channel(channel)
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
        return ch.seq, records

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

    def delete(self, channel: str, name: str) -> bool:
        """Remove a file. Order: data first, then meta (invariant holds mid-crash)."""
        validate_name(name)
        ch = self.open_channel(channel)
        with ch.lock:
            if self._record_if_visible(ch, name) is None:
                return False
            (ch.data / name).unlink(missing_ok=True)
            (ch.meta / f"{name}.json").unlink(missing_ok=True)
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
                    if self.delete(ch.name, rec["name"]):
                        removed += 1
        if removed:
            log.info("TTL sweep removed %d expired file(s)", removed)
        return removed
