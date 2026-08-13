"""
device_config.py — read-only port hints from the app's device profiles.

The main MEBP app caches the last COM port it used for the ZP board in
``config/hardware/devices/*.json`` under ``zp_stage.last_port``. Since this tool
talks to the SAME physical board, that is a useful default for the port combo.

Strictly read-only, and deliberately dependency-free: a plain ``json.load``
rather than importing ``HardwareConfig``/``DeviceProfile``, so this standalone
tool cannot be broken by a schema change in the app, and cannot accidentally
write to the app's configuration.

Note the key is NOT reliably present — ``ME3B V2.json`` has a ``zp_stage`` block
with no ``last_port`` — so every lookup goes through ``.get()`` chains.
"""

from __future__ import annotations

import json
import logging
import re
import time
from dataclasses import dataclass
from pathlib import Path

logger = logging.getLogger(__name__)


def _repo_root() -> Path:
    here = Path(__file__).resolve()
    for cand in (here.parent, *here.parents):
        if (cand / "SupportClasses").is_dir():
            return cand
    return here.parent.parent.parent


DEFAULT_DEVICES_DIR = _repo_root() / "config" / "hardware" / "devices"

#: Baud is NOT stored in the device profiles — it is a code constant in
#: ``SupportClasses/ZPStage.py`` (``DEFAULT_BAUDRATE = 38400``). Same board, so
#: the same value is the right default here.
DEFAULT_BAUD = 38400

COMMON_BAUDS = (250000, 115200, 57600, 38400, 19200, 9600)


@dataclass(frozen=True)
class PortHint:
    profile_name: str
    port: str
    source_file: str

    @property
    def label(self) -> str:
        return f"{self.port}  (last used by {self.profile_name})"


def discover_last_ports(devices_dir: Path | None = None) -> list[PortHint]:
    """
    Scan device profiles for cached ZP ports.

    Never raises: a missing directory, unreadable file or unexpected schema just
    yields fewer hints. Results are de-duplicated by port, preserving the
    first-seen profile name.
    """
    root = Path(devices_dir) if devices_dir else DEFAULT_DEVICES_DIR
    hints: list[PortHint] = []
    seen: set[str] = set()

    try:
        files = sorted(root.glob("*.json"))
    except Exception:
        return hints

    for path in files:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            logger.debug("skipping unreadable device profile %s", path)
            continue
        if not isinstance(data, dict):
            continue

        port = (data.get("zp_stage") or {}).get("last_port")
        if not port or not isinstance(port, str):
            continue
        port = port.strip()
        if not port or port in seen:
            continue
        seen.add(port)
        hints.append(
            PortHint(
                profile_name=str(data.get("profile_name") or path.stem),
                port=port,
                source_file=str(path.name),
            )
        )

    return hints


def available_ports() -> list[dict]:
    """
    Live serial ports.

    Goes through :mod:`.serial_helpers`, which reuses the app's ``SerialUtils``
    logic without importing the ``SupportClasses`` package (whose ``__init__``
    would pull in the whole hardware layer). Returns an empty list if
    enumeration is unavailable.
    """
    from .serial_helpers import list_serial_ports
    try:
        return list_serial_ports()
    except Exception as e:
        logger.debug("port enumeration unavailable: %s", e)
        return []


# ═══════════════════════════════════════════════════════════════════
# Port ranking
# ═══════════════════════════════════════════════════════════════════

#: USB VID:PID prefixes of controllers that commonly run Marlin.
#: 0483:5740 = STMicroelectronics virtual COM — the SKR Mini E3 V3's native USB
#: (STM32G0B1). 1a86 = CH340/CH341. 10c4:ea60 = SiLabs CP210x.
_MARLIN_HWID_HINTS = ("0483:5740", "1a86:7523", "1a86:5523", "10c4:ea60",
                      "2341:", "1eaf:0004")

#: FTDI. In THIS rig an FTDI adapter is the Prior ProScan XY stage, not the
#: heater board, so it is actively pushed down the list rather than merely
#: not boosted — offering it first invites connecting to the wrong device.
_FOREIGN_HWID_HINTS = ("0403:6001", "0403:")

_DESC_HINTS = ("stm32", "usb serial device", "marlin", "ch340", "cp210",
               "virtual com")


def _score_port(info: dict, cached: set[str]) -> int:
    hwid = (info.get("hwid") or "").upper()
    desc = (info.get("description") or "").lower()
    dev = info.get("device") or ""

    score = 0
    for hint in _MARLIN_HWID_HINTS:
        if hint.upper() in hwid:
            score += 100
            break
    for hint in _FOREIGN_HWID_HINTS:
        if hint.upper() in hwid:
            score -= 60
            break
    if any(h in desc for h in _DESC_HINTS):
        score += 20
    # A cached hint only helps if the port actually exists right now.
    if dev in cached:
        score += 25
    return score


def _norm_exclusions(exclude_ports) -> set[str]:
    """Uppercased device names to skip (``None``/empty → nothing skipped)."""
    return {str(p).upper() for p in (exclude_ports or []) if p}


def ranked_ports(*, exclude_ports=None) -> list[tuple[str, int, str]]:
    """
    Present ports ordered by how likely each is to be the Marlin heater board.

    Returns ``(device, score, description)``. Only ports that CURRENTLY EXIST are
    included — a cached hint for a port that has since disappeared must never be
    offered as the default, which is exactly how "it won't connect" happens.

    ``exclude_ports`` removes ports owned by OTHER devices (the app's live ZP
    and XY connections) so they are never even offered — opening a port asserts
    DTR and auto-resets an Arduino/Marlin board, the exact hazard
    ``XYStageManager(exclude_ports=…)`` exists for.
    """
    cached = {h.port for h in discover_last_ports()}
    excl = _norm_exclusions(exclude_ports)
    ports = available_ports()
    scored = [
        (p.get("device") or "", _score_port(p, cached), p.get("description") or "")
        for p in ports
        if p.get("device") and str(p.get("device")).upper() not in excl
    ]
    scored.sort(key=lambda t: (-t[1], t[0]))
    return scored


def suggest_ports(*, exclude_ports=None) -> list[str]:
    """
    Ordered port suggestions for the UI combo, best guess first.

    Deliberately excludes cached ports that are not currently present, and any
    port in ``exclude_ports`` (reserved for another device).
    """
    return [dev for dev, _score, _desc in ranked_ports(exclude_ports=exclude_ports)]


def stale_hints() -> list[str]:
    """Cached ports from the device profiles that are no longer present."""
    present = {p.get("device") for p in available_ports()}
    return [h.port for h in discover_last_ports() if h.port not in present]


# ═══════════════════════════════════════════════════════════════════
# Marlin auto-detect
# ═══════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class DetectedBoard:
    port: str
    baud: int
    firmware: str
    description: str = ""


def detect_marlin(
    *,
    bauds: tuple[int, ...] = (38400, 115200, 250000),
    ports: list[str] | None = None,
    probe_timeout_s: float = 2.5,
    on_progress=None,
    exclude_ports=None,
) -> DetectedBoard | None:
    """
    Find a Marlin board by asking each candidate port ``M115``.

    Read-only and safe: ``M115`` reports firmware identity and changes nothing.
    Ports are tried best-guess first, and each is opened once per baud rate until
    a ``FIRMWARE_NAME`` reply appears.

    ``exclude_ports`` is enforced even over an explicit ``ports`` list: those
    ports belong to the app's live ZP / XY connections, and merely OPENING one
    DTR-resets the board behind it. In-app, the ZP board is a Marlin board too
    — without the exclusion this scan would happily "detect" the printing
    board and (worse) reset it on the way.

    Note a board using the MCU's native USB (like the SKR Mini E3 V3) ignores the
    baud rate entirely, so for those the first attempt succeeds regardless.
    """
    try:
        import serial
    except ImportError:
        return None

    excl = _norm_exclusions(exclude_ports)
    candidates = (
        ports if ports is not None
        else suggest_ports(exclude_ports=exclude_ports)
    )
    candidates = [d for d in candidates if str(d).upper() not in excl]
    descs = {dev: desc for dev, _s, desc in ranked_ports()}

    for dev in candidates:
        for baud in bauds:
            if on_progress:
                try:
                    on_progress(f"Probing {dev} at {baud}…")
                except Exception:
                    pass
            sp = None
            try:
                sp = serial.Serial(dev, baud, timeout=0.4, write_timeout=2.0)
            except Exception as e:
                logger.debug("cannot open %s @%d: %s", dev, baud, e)
                break  # port is busy or gone; other bauds will fail too
            try:
                time.sleep(0.6)          # allow a boot banner to arrive
                try:
                    sp.reset_input_buffer()
                except Exception:
                    pass
                sp.write(b"M115\n")
                sp.flush()
                deadline = time.monotonic() + probe_timeout_s
                fw = None
                while time.monotonic() < deadline:
                    try:
                        raw = sp.readline()
                    except Exception:
                        break
                    if not raw:
                        continue
                    line = raw.decode("ascii", errors="replace").strip()
                    if "FIRMWARE_NAME" in line.upper():
                        fw = line
                    if line == "ok" or line.startswith("ok "):
                        break
                if fw:
                    name = fw
                    up = fw.upper()
                    idx = up.find("FIRMWARE_NAME")
                    if idx >= 0:
                        name = fw[idx + len("FIRMWARE_NAME"):].lstrip(": ")
                        cut = re.split(r"\s+[A-Z_]{3,}:", name, maxsplit=1)
                        name = cut[0].strip()
                    return DetectedBoard(port=dev, baud=baud, firmware=name,
                                         description=descs.get(dev, ""))
            finally:
                try:
                    sp.close()
                except Exception:
                    pass
    return None
