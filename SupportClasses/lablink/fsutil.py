"""Filesystem helpers: atomic writes, checksums, name validation.

The atomic-write idiom (tempfile.mkstemp sibling + os.replace + finally
cleanup) mirrors the one used throughout the host repo's stores
(e.g. SupportClasses/WorkflowSettingsStore.py) so a crash at any point
never leaves a half-written file at the destination path.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import tempfile
from pathlib import Path

from .protocol import NAME_RE, WINDOWS_RESERVED


def atomic_write_bytes(path: Path, data: bytes) -> None:
    """Write *data* to *path* atomically (tmp sibling + os.replace)."""
    path = Path(path)
    fd, tmp = tempfile.mkstemp(dir=str(path.parent), suffix=".tmp")
    try:
        with os.fdopen(fd, "wb") as fh:
            fh.write(data)
        os.replace(tmp, path)
        tmp = None
    finally:
        if tmp is not None:
            try:
                os.unlink(tmp)
            except OSError:
                pass


def atomic_write_json(path: Path, obj) -> None:
    """Serialize *obj* as JSON and write it atomically to *path*."""
    atomic_write_bytes(Path(path), json.dumps(obj, indent=2).encode("utf-8"))


def read_json(path: Path, default=None):
    """Tolerant JSON loader: missing or corrupt file returns *default*."""
    try:
        with open(path, "rb") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return default


def sha256_file(path: Path) -> str:
    """Hex sha256 of a file's contents (streaming, constant memory).

    hashlib.file_digest arrived in Python 3.11; the manual loop keeps this
    working on the 3.8-3.10 interpreters that ship with older macOS and Linux
    installs, so a lab machine does not need a Python upgrade to join.
    """
    with open(path, "rb") as fh:
        if hasattr(hashlib, "file_digest"):
            return hashlib.file_digest(fh, "sha256").hexdigest()
        digest = hashlib.sha256()
        for block in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(block)
        return digest.hexdigest()


def human_size(n: float) -> str:
    """Format a byte count for display.

    Lives here so every tool agrees: the CLI and the connect script used to
    disagree about MB (1024- vs 1000-based) and reported the same file as
    8.7 MB and 9.2 MB, which reads like a failed transfer.
    """
    n = float(n)
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.0f} {unit}" if unit == "B" else f"{n:.1f} {unit}"
        n /= 1024.0
    return f"{n:.1f} GB"


def validate_name(name: str) -> str:
    """Validate a channel or file name; returns it unchanged or raises ValueError.

    Rules (see protocol.py): single path segment, NAME_RE shape, no trailing
    dot/space, not a Windows reserved device name. Raising here — the
    earliest layer — is what makes path traversal impossible everywhere else.
    """
    if not isinstance(name, str) or not name:
        raise ValueError("name must be a non-empty string")
    if "/" in name or "\\" in name:
        raise ValueError(f"name must be a single path segment: {name!r}")
    if not NAME_RE.match(name):
        raise ValueError(
            f"invalid name {name!r} (allowed: letters/digits/dot/underscore/"
            f"space/hyphen, must start alphanumeric, max 128 chars)"
        )
    if name[-1] in (".", " "):
        raise ValueError(f"name must not end with a dot or space: {name!r}")
    if name.split(".")[0].upper() in WINDOWS_RESERVED:
        raise ValueError(f"name is a reserved Windows device name: {name!r}")
    return name


def safe_name(name: str) -> str:
    """validate_name's inverse: coerce any string into a legal exchange name.

    Lives beside validate_name deliberately — one raises, one repairs, and having
    them apart is how a caller ends up writing a third, subtly different version.

    Needed at BOTH ends, for the same underlying reason. Real microscopy filenames
    routinely carry characters the name rules refuse: `2026-08-03 A1 (well 3)
    60x.nd2` has parentheses, and `+`, `#`, `,`, `[`, `]` and non-ASCII are all
    common. A sender that does not repair the name gets a 400 it has to understand;
    a hub returning a produced file under such a name would, before this existed,
    log a warning and move on — so from the far side the result SILENTLY NEVER
    ARRIVED. Substituting and reporting the substitution is strictly better than
    either.

    Callers should keep the original: it is what tells a person which file this
    actually is. The session layer records it in the artifact's metadata.
    """
    cleaned = re.sub(r"[^A-Za-z0-9._ -]", "_", str(name))
    cleaned = re.sub(r"_{2,}", "_", cleaned).strip()
    # NAME_RE wants an alphanumeric first character, and Windows silently strips a
    # trailing dot or space — which would make the stored name differ from the
    # requested one on one platform and not another.
    cleaned = cleaned.lstrip("._- ").rstrip(". ")
    if not cleaned:
        return "file"
    if not cleaned[0].isalnum():
        cleaned = f"f{cleaned}"
    if cleaned.split(".")[0].upper() in WINDOWS_RESERVED:
        cleaned = f"file_{cleaned}"
    if len(cleaned) > 128:
        # Keep the extension: it is what tells a person what the file is.
        head, dot, ext = cleaned.rpartition(".")
        cleaned = (head[: 127 - len(ext)] + "." + ext
                   if dot and len(ext) < 16 else cleaned[:128])
    try:
        return validate_name(cleaned)
    except ValueError:
        return "file"
