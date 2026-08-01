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
    """Hex sha256 of a file's contents (streaming, constant memory)."""
    with open(path, "rb") as fh:
        return hashlib.file_digest(fh, "sha256").hexdigest()


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
