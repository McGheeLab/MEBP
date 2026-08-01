"""LabLinkClient — stdlib urllib client for the LabLink file exchange.

Robustness model
    upload   whole-file retry with exponential backoff; the server's
             same-name+same-sha idempotency makes a retry after a crash a
             no-op rather than a duplicate.
    download resume via ``Range: bytes=N-`` into ``<dest>/.partial/<name>.part``
             then sha256-verify then os.replace into <dest>. Because parts
             live in a subdirectory, a naive folder watcher pointed at <dest>
             can never observe a partial file.
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

from .fsutil import sha256_file, validate_name
from .protocol import CHUNK, DEFAULT_TIMEOUT_S, H_META, H_SHA, H_TOKEN

log = logging.getLogger("lablink.client")

PARTIAL_DIR = ".partial"

# Backstop against a pathological loop; real transfers need one pass per
# network interruption, and each pass must move bytes to keep going.
MAX_DOWNLOAD_PASSES = 500


class LabLinkError(Exception):
    """A failed request. ``status`` is the HTTP code, or 0 for a network error."""

    def __init__(self, status: int, message: str):
        super().__init__(f"[{status}] {message}" if status else message)
        self.status = status
        self.message = message

    @property
    def retryable(self) -> bool:
        """Network errors and 5xx are worth retrying; 4xx are not."""
        return self.status == 0 or self.status >= 500


class LabLinkClient:
    def __init__(self, base_url: str, token: str, timeout: float = DEFAULT_TIMEOUT_S):
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.timeout = timeout
        self._max_file_bytes: int | None = None   # cached from /hello, lazily

    # ------------------------------------------------------------------ plumbing
    def _url(self, *segments: str, query: dict | None = None) -> str:
        path = "/".join(urllib.parse.quote(s, safe="") for s in segments)
        url = f"{self.base_url}/{path}"
        if query:
            url += "?" + urllib.parse.urlencode(query)
        return url

    def _open(self, method: str, url: str, data=None, headers: dict | None = None):
        """Perform a request; returns the open response. Raises LabLinkError."""
        req = urllib.request.Request(url, data=data, method=method)
        req.add_header(H_TOKEN, self.token)
        for k, v in (headers or {}).items():
            req.add_header(k, v)
        try:
            return urllib.request.urlopen(req, timeout=self.timeout)
        except urllib.error.HTTPError as exc:
            try:
                payload = json.loads(exc.read().decode("utf-8", "replace"))
                message = payload.get("error", str(exc))
            except Exception:
                message = str(exc)
            raise LabLinkError(exc.code, message) from None
        except (urllib.error.URLError, OSError) as exc:
            raise LabLinkError(0, f"cannot reach {self.base_url}: {exc}") from None

    def _json(self, method: str, url: str, data=None, headers: dict | None = None) -> dict:
        with self._open(method, url, data=data, headers=headers) as resp:
            body = resp.read()
        return json.loads(body.decode("utf-8")) if body else {}

    # ------------------------------------------------------------------ API
    def hello(self) -> dict:
        """Server identity + clock. Requires no token (reachability probe)."""
        info = self._json("GET", f"{self.base_url}/hello")
        limit = info.get("max_file_bytes")
        if isinstance(limit, int) and limit > 0:
            self._max_file_bytes = limit
        return info

    def _check_size_limit(self, size: int, name: str) -> None:
        """Refuse an oversize file locally, before spending the bandwidth.

        The server enforces the limit too, but it can only reject after the
        client has begun writing — at which point the client's socket write
        fails with a reset and it never reads the 413. Asking /hello once
        turns that into an instant, clearly-worded, non-retryable error.
        """
        if self._max_file_bytes is None:
            try:
                self.hello()
            except LabLinkError:
                return          # server unreachable/old: let the upload try anyway
        if self._max_file_bytes and size > self._max_file_bytes:
            raise LabLinkError(
                413,
                f"{name} is {size} bytes; the server's limit is "
                f"{self._max_file_bytes} bytes "
                f"({self._max_file_bytes // (1024 * 1024)} MB). "
                f"Raise it with --max-file-mb on the server.",
            )

    def list_files(self, channel: str, since_seq: int | None = None) -> dict:
        query = {"since_seq": since_seq} if since_seq else None
        return self._json("GET", self._url("c", channel, query=query))

    def delete(self, channel: str, name: str) -> None:
        self._json("DELETE", self._url("c", channel, name))

    def upload(self, channel: str, path: Path | str, name: str | None = None,
               meta: dict | None = None, retries: int = 4) -> dict:
        """Upload a file. Returns the server record with an added "status" key
        of "created" or "duplicate". Retries transient failures with backoff."""
        path = Path(path)
        name = name or path.name
        validate_name(name)
        size = path.stat().st_size
        self._check_size_limit(size, name)
        digest = sha256_file(path)

        headers = {
            H_SHA: digest,
            "Content-Type": "application/octet-stream",
            "Content-Length": str(size),
        }
        if meta:
            encoded = json.dumps(meta, ensure_ascii=True)
            headers[H_META] = encoded

        url = self._url("c", channel, name)
        last: LabLinkError | None = None
        for attempt in range(retries):
            try:
                with open(path, "rb") as fh:
                    with self._open("PUT", url, data=fh, headers=headers) as resp:
                        record = json.loads(resp.read().decode("utf-8"))
                        record["status"] = "created" if resp.status == 201 else "duplicate"
                if record.get("sha256") != digest:
                    raise LabLinkError(0, f"server echoed a different sha256 for {name}")
                return record
            except LabLinkError as exc:
                last = exc
                if not exc.retryable or attempt == retries - 1:
                    raise
                delay = 2 ** attempt
                log.warning("upload %s failed (%s); retrying in %ds", name, exc, delay)
                time.sleep(delay)
        raise last  # pragma: no cover - loop always returns or raises

    def download(self, channel: str, name: str, dest_dir: Path | str,
                 expected_sha: str | None = None, retries: int = 4) -> Path:
        """Download into dest_dir with resume + verification. Returns the path.

        A transfer cut short by the network leaves a SHORTER-than-expected
        part, which is resumed on the next pass — it is emphatically not
        treated as corruption, because deleting it would throw away the very
        progress the resume exists to keep. Only a part that is complete (or
        overlong) and hashes wrong is discarded and refetched from zero.

        Passes that move bytes do not consume the retry budget, so an
        arbitrarily large file still completes over a link that drops every
        few megabytes; *retries* bounds consecutive passes that achieve
        nothing.
        """
        validate_name(name)
        dest_dir = Path(dest_dir)
        dest_dir.mkdir(parents=True, exist_ok=True)
        partial_dir = dest_dir / PARTIAL_DIR
        partial_dir.mkdir(exist_ok=True)
        part = partial_dir / f"{name}.part"
        final = dest_dir / name
        url = self._url("c", channel, name)

        def part_size() -> int:
            return part.stat().st_size if part.exists() else 0

        stalled = 0
        for _ in range(MAX_DOWNLOAD_PASSES):
            before = part_size()
            try:
                server_sha, total = self._fetch_to_part(url, part)
            except LabLinkError as exc:
                if not exc.retryable:
                    raise
                stalled += 1
                if stalled >= retries:
                    raise
                log.warning("download %s failed (%s); retrying in %ds",
                            name, exc, 2 ** (stalled - 1))
                time.sleep(2 ** (stalled - 1))
                continue

            after = part_size()
            if total is not None and after < total:
                # Truncated. Keep the bytes; resume from here.
                if after > before:
                    stalled = 0            # progress: don't burn the budget
                    log.info("resuming %s at %d/%d bytes", name, after, total)
                    continue
                stalled += 1
                if stalled >= retries:
                    raise LabLinkError(
                        0, f"download of {name} stalled at {after}/{total} bytes")
                time.sleep(2 ** (stalled - 1))
                continue

            sha = server_sha or expected_sha
            if sha and sha256_file(part) != sha:
                log.warning("checksum mismatch for %s; refetching from zero", name)
                part.unlink(missing_ok=True)
                stalled += 1
                if stalled >= retries:
                    raise LabLinkError(0, f"checksum mismatch for {name}")
                continue

            os.replace(part, final)
            return final

        raise LabLinkError(0, f"download of {name} did not converge")

    def _fetch_to_part(self, url: str, part: Path):
        """Fetch url into *part*, resuming if a partial exists.

        Returns (server_sha256 or None, total_file_size or None). A connection
        broken mid-body is NOT an error here: whatever arrived is on disk and
        the caller compares the size against *total* to decide what to do.
        """
        have = part.stat().st_size if part.exists() else 0
        headers = {"Range": f"bytes={have}-"} if have else {}
        try:
            resp = self._open("GET", url, headers=headers)
        except LabLinkError as exc:
            if exc.status == 416 and have:
                # The part is at or past the full length: complete, or corrupt
                # and overlong. Let the caller's checksum decide.
                return None, None
            raise

        with resp:
            mode = "ab" if (have and resp.status == 206) else "wb"
            if mode == "wb" and have:
                log.info("server ignored Range; restarting %s from zero", part.name)
            server_sha = resp.headers.get(H_SHA)
            total = self._total_size(resp, mode == "ab", have)
            try:
                with open(part, mode) as fh:
                    while True:
                        chunk = resp.read(CHUNK)
                        if not chunk:
                            break
                        fh.write(chunk)
            except (OSError, urllib.error.URLError) as exc:
                # Cut mid-body: partial bytes are safely on disk.
                log.info("transfer of %s interrupted (%s)", part.name, exc)
        return server_sha, total

    @staticmethod
    def _total_size(resp, resumed: bool, have: int) -> int | None:
        """Full file size from Content-Range (206) or Content-Length (200)."""
        rng = resp.headers.get("Content-Range")
        if rng and "/" in rng:
            try:
                return int(rng.rsplit("/", 1)[1])
            except ValueError:
                pass
        try:
            length = int(resp.headers["Content-Length"])
        except (KeyError, TypeError, ValueError):
            return None
        return have + length if resumed else length
