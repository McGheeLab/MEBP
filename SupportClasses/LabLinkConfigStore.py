"""
LabLinkConfigStore.py — per-machine LabLink hub connection + job settings.

v7.17. MEBP is a LabLink **node**: it opens a session on a hub, uploads an
image plus its ``.job.json`` sidecar, runs a recipe the hub declares, and
collects the results. This store holds the small amount of that which is a
property of THIS machine.

⚠ NOT in ``HardwareConfig``, for two independent reasons. The first is the
recorded CAMERA_CAL_PERSIST_STORE lesson: a hardware setup file is *meant* to
travel between rigs, and loading one replaces the in-memory config wholesale,
so anything else living in it is wiped on the next save. The second is
narrower and sharper — **this file holds a bearer token**, and the setup files
are precisely the ones operators copy machine to machine.

⚠ THE TOKEN. `docs/INTEGRATING-WITH-LABLINK.md` §5a: *"Neither belongs in your
source, your repository, or a log line."* So:

* ``LABLINK_TOKEN`` in the environment wins over the stored value, which is how
  a site can keep the secret out of a file entirely;
* ``config/hardware/lablink.json`` is in ``.gitignore`` **in the same commit
  that introduced this module** — the older vendored `lablink_connect.py`
  shipped a live token to GitHub precisely because that ordering slipped;
* ``as_dict()`` redacts the token. Use ``token()`` when you actually need it.

**No channel names are stored.** `in_channel`/`out_channel` come from the
`POST /s` response and are never derived — §3 of the integration doc lists
that among the things a client must read rather than construct. Nor are recipe
names, knob bounds, size limits or timeouts cached here: *"Discover, do not
hardcode."* What IS stored is the operator's CHOICE of workflow + recipe +
knob values per output kind, which only they can supply.
"""

from __future__ import annotations

import json
import logging
import os
import threading
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_DEFAULT_FILENAME = "lablink.json"
ENV_CONFIG_DIR = "MEBP_LABLINK_CONFIG_DIR"
ENV_TOKEN = "LABLINK_TOKEN"

SCHEMA_VERSION = "1.0"

#: The four things MEBP can push. Keys of the ``sources`` map.
SOURCES = ("fluorescence_well", "plate_mosaic", "still", "video")

SOURCE_LABELS = {
    "fluorescence_well": "Fluorescence well scan",
    "plate_mosaic": "Plate mosaic",
    "still": "Captured image",
    "video": "Captured video",
}

#: Refuse a per-source ceiling above this. The hub's own `max_file_bytes` is
#: the real limit and is read at run time; this only bounds the operator's
#: local override so a typo cannot disable the check entirely.
MAX_UPLOAD_CEILING_MB = 65536


def _default_path() -> Path:
    d = os.environ.get(ENV_CONFIG_DIR)
    if d:
        return Path(d) / _DEFAULT_FILENAME
    return Path(__file__).resolve().parent.parent / "config" / "hardware" / _DEFAULT_FILENAME


def _blank_source() -> dict:
    return {
        "enabled": False,
        "workflow": "",
        "recipe": "",
        # Knob values the operator pinned. Empty = send nothing and let every
        # knob take its recipe default. ⚠ null is NOT the same as absent: it
        # means "derive from the file". Both are legal and they are kept
        # distinct all the way to the wire.
        "knobs": {},
        # 0 = no local ceiling; the hub's max_file_bytes still applies.
        "max_upload_mb": 0,
    }


class LabLinkConfigStore:
    """Load/save the per-machine LabLink node configuration."""

    def __init__(self, path: Optional[Path] = None):
        self._path = Path(path) if path is not None else _default_path()
        self._lock = threading.RLock()
        self._data: dict = self._blank()
        self._listeners: list = []
        self._load()

    # ── Defaults / load / save ────────────────────────────────────

    @staticmethod
    def _blank() -> dict:
        return {
            "version": SCHEMA_VERSION,
            # Master switch. OFF until the operator turns it on: pushing lab
            # images off the machine is not something to start doing silently.
            "enabled": False,
            "base_url": "",
            "token": "",
            # From POST /enroll, minted ONCE per machine. Enrolling on every
            # launch fills the operator's node list with junk they cannot tell
            # apart (integration doc §5a).
            "node_id": "",
            "sources": {name: _blank_source() for name in SOURCES},
        }

    def _load(self) -> None:
        data = self._blank()
        try:
            if self._path.is_file():
                raw = json.loads(self._path.read_text(encoding="utf-8"))
                if isinstance(raw, dict):
                    for key in ("base_url", "token", "node_id"):
                        if isinstance(raw.get(key), str):
                            data[key] = raw[key]
                    if isinstance(raw.get("enabled"), bool):
                        data["enabled"] = raw["enabled"]
                    src = raw.get("sources")
                    if isinstance(src, dict):
                        for name in SOURCES:
                            data["sources"][name] = _merge_source(src.get(name))
        except (OSError, ValueError) as exc:
            # A hand-edited or truncated file must not stop the app starting;
            # it degrades to defaults, which are "off".
            logger.error(f"LabLinkConfigStore: cannot read {self._path}: {exc}")
        self._data = data

    def _write(self) -> None:
        """Atomic write. Caller holds ``_lock``."""
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._path.with_name(self._path.name + ".tmp")
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
        except OSError as exc:
            logger.error(f"LabLinkConfigStore: failed to save: {exc}")

    def save(self) -> None:
        with self._lock:
            self._write()
        self._notify()

    @property
    def path(self) -> Path:
        return self._path

    # ── Listeners ─────────────────────────────────────────────────

    def add_listener(self, cb) -> None:
        with self._lock:
            if cb not in self._listeners:
                self._listeners.append(cb)

    def remove_listener(self, cb) -> None:
        with self._lock:
            if cb in self._listeners:
                self._listeners.remove(cb)

    def _notify(self) -> None:
        for cb in list(self._listeners):
            try:
                cb(self)
            except Exception:       # a bad listener must not break a mutation
                logger.exception("LabLinkConfigStore listener failed")

    # ── Connection ────────────────────────────────────────────────

    def base_url(self) -> str:
        with self._lock:
            return str(self._data.get("base_url", "")).rstrip("/")

    def set_base_url(self, url: str, *, save: bool = True) -> None:
        with self._lock:
            self._data["base_url"] = str(url or "").strip().rstrip("/")
            if save:
                self._write()
        self._notify()

    def token(self) -> str:
        """The site token. The environment wins over the stored value.

        Returned, never logged — ``as_dict()`` redacts it on purpose.
        """
        env = os.environ.get(ENV_TOKEN)
        if env:
            return env.strip()
        with self._lock:
            return str(self._data.get("token", ""))

    def token_source(self) -> str:
        """``"environment"``, ``"file"`` or ``"unset"`` — for the UI to say
        where the secret came from without ever showing it."""
        if os.environ.get(ENV_TOKEN):
            return "environment"
        with self._lock:
            return "file" if self._data.get("token") else "unset"

    def set_token(self, token: str, *, save: bool = True) -> None:
        with self._lock:
            self._data["token"] = str(token or "").strip()
            if save:
                self._write()
        self._notify()

    def node_id(self) -> str:
        with self._lock:
            return str(self._data.get("node_id", ""))

    def set_node_id(self, node_id: str, *, save: bool = True) -> None:
        """Persist the identity minted by ``POST /enroll``.

        Called once. The integration doc is explicit that re-enrolling every
        launch fills the operator's node list with entries they cannot tell
        apart, so the service must check this before enrolling.
        """
        with self._lock:
            self._data["node_id"] = str(node_id or "").strip()
            if save:
                self._write()
        self._notify()

    def is_configured(self) -> bool:
        return bool(self.base_url() and self.token())

    # ── Master switch ─────────────────────────────────────────────

    def enabled(self) -> bool:
        with self._lock:
            return bool(self._data.get("enabled", False))

    def set_enabled(self, on: bool, *, save: bool = True) -> None:
        with self._lock:
            self._data["enabled"] = bool(on)
            if save:
                self._write()
        self._notify()

    # ── Per-source job settings ───────────────────────────────────

    def source(self, name: str) -> dict:
        """A COPY of one source's settings (never the live dict)."""
        with self._lock:
            return json.loads(json.dumps(
                self._data["sources"].get(name) or _blank_source()))

    def sources(self) -> dict:
        with self._lock:
            return json.loads(json.dumps(self._data["sources"]))

    def set_source(self, name: str, values: dict, *, save: bool = True) -> None:
        if name not in SOURCES:
            raise ValueError(
                f"unknown source {name!r}; expected one of {', '.join(SOURCES)}")
        with self._lock:
            merged = _merge_source(
                {**self._data["sources"].get(name, {}), **(values or {})})
            self._data["sources"][name] = merged
            if save:
                self._write()
        self._notify()

    def source_enabled(self, name: str) -> bool:
        """True only when the master switch is on AND this source is armed AND
        a recipe has actually been chosen.

        The recipe check is not belt-and-braces: a source enabled with no
        recipe would queue work that can never run, and the operator would see
        a growing queue rather than a configuration prompt.
        """
        if not self.enabled() or not self.is_configured():
            return False
        s = self.source(name)
        return bool(s.get("enabled") and s.get("workflow") and s.get("recipe"))

    def max_upload_bytes(self, name: str) -> int:
        """The operator's local ceiling in bytes, or 0 for "no local limit".

        This is *in addition to* the hub's ``max_file_bytes``, never instead of
        it — the hub's figure is authoritative and read at run time, because
        the operator intends to raise it later.
        """
        mb = self.source(name).get("max_upload_mb", 0)
        try:
            mb = int(mb)
        except (TypeError, ValueError):
            return 0
        return max(0, min(MAX_UPLOAD_CEILING_MB, mb)) * 1024 * 1024

    # ── Diagnostics ───────────────────────────────────────────────

    def as_dict(self, *, redact: bool = True) -> dict:
        """Deep copy for display or logging.

        ⚠ ``redact`` defaults True and callers should leave it that way: this
        is what makes it safe to drop the whole config into a log line, and a
        log directory in this repo is partly tracked.
        """
        with self._lock:
            data = json.loads(json.dumps(self._data))
        if redact and data.get("token"):
            data["token"] = "***"
        return data


def _merge_source(raw) -> dict:
    """One source's settings over the blank template, ignoring unknown keys.

    ⚠ ``knobs`` is copied VERBATIM, including ``null`` values. Omitted, null
    and 0 are three different instructions to the hub (default / derive /
    a pinned zero), so this must not "tidy" nulls away.
    """
    out = _blank_source()
    if not isinstance(raw, dict):
        return out
    if isinstance(raw.get("enabled"), bool):
        out["enabled"] = raw["enabled"]
    for key in ("workflow", "recipe"):
        if isinstance(raw.get(key), str):
            out[key] = raw[key]
    if isinstance(raw.get("knobs"), dict):
        out["knobs"] = dict(raw["knobs"])
    try:
        out["max_upload_mb"] = max(
            0, min(MAX_UPLOAD_CEILING_MB, int(raw.get("max_upload_mb", 0))))
    except (TypeError, ValueError):
        out["max_upload_mb"] = 0
    return out


# ── Module-level singleton ────────────────────────────────────────

_store: Optional[LabLinkConfigStore] = None


def get_store(path: Optional[Path] = None) -> LabLinkConfigStore:
    global _store
    if _store is None:
        _store = LabLinkConfigStore(path)
    return _store


def reset_store() -> None:
    """Drop the singleton — for tests that change the env override."""
    global _store
    _store = None
