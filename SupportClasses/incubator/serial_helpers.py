"""
serial_helpers.py — thin façade over ``SupportClasses/SerialUtils.py``.

History: the standalone tool loaded ``SerialUtils.py`` by file path to avoid
executing ``SupportClasses/__init__.py`` (which eagerly imports the whole
hardware layer). Vendored INTO the app that cost is already paid — the package
is loaded long before this module is — so the plain import is preferred: it
reuses the one module instance the rest of the app holds instead of creating a
private duplicate. The path-load fallback is kept only for the odd context
where the package import fails (e.g. this subpackage exercised standalone);
plain pyserial is the last resort so every helper degrades rather than raises.
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
    """Resolve the SerialUtils module, once. Returns the module or ``None``."""
    global _serialutils, _tried
    if _tried:
        return _serialutils
    _tried = True

    # Preferred: the app's own module instance.
    try:
        from SupportClasses import SerialUtils as mod  # noqa: N813
        _serialutils = mod
        return _serialutils
    except Exception as e:
        logger.debug("package SerialUtils import failed (%s); trying by path", e)

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

    Mirrors ``SerialUtils.check_port_health``: reading ``in_waiting`` is enough
    to surface a vanished USB device on Windows.
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
    """True when the app's SerialUtils was resolved (diagnostic aid)."""
    return _load() is not None
