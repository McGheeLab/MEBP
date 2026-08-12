"""
_serial_helpers.py — reuse ``SupportClasses/SerialUtils.py`` WITHOUT importing
the ``SupportClasses`` package.

Why this exists
---------------
``SerialUtils`` itself is exactly what we want: hardware-agnostic port
enumeration and friendly error messages, already battle-tested against this
machine's USB-serial quirks. But a normal ``from SupportClasses.SerialUtils
import ...`` executes ``SupportClasses/__init__.py`` first, and that eagerly
imports the ENTIRE hardware layer — ``ZPStageManager``, ``StageController``,
``PrintManager``, the multiprocessing ``XboxController``, and the XY simulator,
which even reads a profile off disk as an import side effect.

For a standalone diagnostic tool that is wrong three times over: it contradicts
the point of being decoupled from the app, it makes startup slow and noisy, and
it means an unrelated breakage (or a missing optional dependency such as pygame)
would stop this tool from launching for no good reason.

So we load the single module directly from its file path, which skips the package
``__init__`` entirely, and fall back to plain pyserial if anything about that
fails. Same code, none of the baggage.
"""

from __future__ import annotations

import importlib.util
import logging
import sys
from pathlib import Path

logger = logging.getLogger(__name__)

_MODULE_NAME = "_incubator_serialutils"
_serialutils = None
_tried = False


def _repo_root() -> Path:
    here = Path(__file__).resolve()
    for cand in (here.parent, *here.parents):
        if (cand / "SupportClasses").is_dir():
            return cand
    return here.parent.parent.parent


def _load():
    """Load SerialUtils.py by path, once. Returns the module or ``None``."""
    global _serialutils, _tried
    if _tried:
        return _serialutils
    _tried = True

    if _MODULE_NAME in sys.modules:
        _serialutils = sys.modules[_MODULE_NAME]
        return _serialutils

    path = _repo_root() / "SupportClasses" / "SerialUtils.py"
    if not path.is_file():
        logger.debug("SerialUtils.py not found at %s", path)
        return None
    try:
        spec = importlib.util.spec_from_file_location(_MODULE_NAME, path)
        if spec is None or spec.loader is None:
            return None
        mod = importlib.util.module_from_spec(spec)
        # Register before exec so the module can refer to itself if it ever needs to.
        sys.modules[_MODULE_NAME] = mod
        spec.loader.exec_module(mod)
        _serialutils = mod
    except Exception as e:
        logger.debug("could not load SerialUtils standalone: %s", e)
        sys.modules.pop(_MODULE_NAME, None)
        _serialutils = None
    return _serialutils


# ═══════════════════════════════════════════════════════════════════
# Public helpers (each degrades gracefully on its own)
# ═══════════════════════════════════════════════════════════════════

def list_serial_ports() -> list[dict]:
    """
    Enumerate serial ports as dicts with ``device``/``description``/``hwid``/
    ``manufacturer``. Empty list if enumeration is unavailable.
    """
    mod = _load()
    if mod is not None:
        try:
            return mod.list_serial_ports()
        except Exception as e:
            logger.debug("SerialUtils.list_serial_ports failed: %s", e)

    try:
        import serial.tools.list_ports
        return [
            {
                "device": p.device,
                "description": p.description,
                "hwid": p.hwid,
                "manufacturer": getattr(p, "manufacturer", ""),
            }
            for p in serial.tools.list_ports.comports()
        ]
    except Exception:
        return []


def friendly_error_message(error: Exception) -> str:
    """Human-readable explanation for a serial failure."""
    mod = _load()
    if mod is not None:
        try:
            return mod.friendly_error_message(error)
        except Exception:
            pass

    msg = str(error).lower()
    if "access is denied" in msg or "permission" in msg:
        return "Port access denied — is another program using it?"
    if "no such file" in msg or "filenotfound" in msg:
        return "Port not found — is the device plugged in?"
    if "timeout" in msg:
        return "Communication timed out — device may be busy."
    return f"Serial error: {error}"


def port_is_healthy(port) -> bool:
    """
    Cheap liveness check on an open port object.

    Mirrors ``SerialUtils.check_port_health``: reading ``in_waiting`` is enough to
    surface a vanished USB device on Windows.
    """
    mod = _load()
    if mod is not None:
        try:
            return bool(mod.check_port_health(port))
        except Exception:
            pass
    if port is None:
        return False
    try:
        if hasattr(port, "is_open") and not port.is_open:
            return False
        _ = port.in_waiting
        return True
    except Exception:
        return False


def using_shared_serialutils() -> bool:
    """True when the app's SerialUtils was loaded (diagnostic / self-test aid)."""
    return _load() is not None
