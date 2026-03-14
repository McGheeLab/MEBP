"""
Serial Utilities — Resilient serial communication helpers.

Provides:
    - Custom exception hierarchy for serial errors
    - Retry decorator for transient failures
    - Connection health checking
    - Safe read/write wrappers with timeout
    - Serial port scanning
    - ConnectionWatchdog for background disconnect detection
"""

from __future__ import annotations

import functools
import logging
import threading
import time
from typing import Callable, Optional

logger = logging.getLogger(__name__)

try:
    import serial
    import serial.tools.list_ports

    _HAS_SERIAL = True
except ImportError:
    serial = None  # type: ignore[assignment]
    _HAS_SERIAL = False


# ═══════════════════════════════════════════════════════════════════
# Exception Hierarchy
# ═══════════════════════════════════════════════════════════════════

class SerialError(Exception):
    """Base class for friendly serial communication errors."""


class SerialDisconnectedError(SerialError):
    """The serial device has been physically disconnected."""


class SerialTimeoutError(SerialError):
    """A serial operation timed out."""


class SerialAccessError(SerialError):
    """Port access was denied (another process using it)."""


# ═══════════════════════════════════════════════════════════════════
# Retry Decorator
# ═══════════════════════════════════════════════════════════════════

def retry_serial(
    max_retries: int = 2,
    delay: float = 0.1,
    on_failure: Optional[Callable[[Exception], None]] = None,
):
    """
    Decorator that retries serial operations on transient failures.

    Args:
        max_retries: Number of additional attempts after the first.
        delay:       Seconds to wait between retries.
        on_failure:  Optional callback invoked on final failure.
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            last_error: Optional[Exception] = None
            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except (serial.SerialException, OSError) as e:
                    last_error = e
                    if attempt < max_retries:
                        logger.debug(
                            f"Serial retry {attempt + 1}/{max_retries} "
                            f"for {func.__name__}: {e}"
                        )
                        time.sleep(delay)
                    else:
                        msg = friendly_error_message(e)
                        logger.error(f"{func.__name__} failed: {msg}")
                        if on_failure:
                            on_failure(e)
                        raise SerialError(msg) from e
                except Exception:
                    raise  # Non-serial exceptions propagate immediately
            return None
        return wrapper
    return decorator


# ═══════════════════════════════════════════════════════════════════
# Port Health & Safe I/O
# ═══════════════════════════════════════════════════════════════════

def check_port_health(port) -> bool:
    """
    Non-destructive check that a serial port is still responsive.

    Returns True if the port appears healthy.
    """
    if port is None:
        return False
    try:
        if hasattr(port, "is_open") and not port.is_open:
            return False
        _ = port.in_waiting  # lightweight liveness check
        return True
    except (serial.SerialException, OSError, AttributeError):
        return False


def safe_write(port, data: bytes) -> bool:
    """
    Write *data* to a serial port with health check and error wrapping.

    Raises:
        SerialDisconnectedError: If port is not connected.
        SerialTimeoutError:       If write times out.
    """
    if not check_port_health(port):
        raise SerialDisconnectedError("Serial port is not connected")
    try:
        port.write(data)
        port.flush()
        return True
    except serial.SerialTimeoutException:
        raise SerialTimeoutError("Write timed out")
    except (serial.SerialException, OSError) as e:
        raise SerialDisconnectedError(f"Write failed: {friendly_error_message(e)}")


def safe_readline(port, timeout: float = 1.0) -> str:
    """
    Read a line from a serial port with explicit timeout.

    Returns:
        Decoded and stripped response string.

    Raises:
        SerialDisconnectedError: If port is not connected.
        SerialTimeoutError:       If no data received.
    """
    if not check_port_health(port):
        raise SerialDisconnectedError("Serial port is not connected")

    old_timeout = port.timeout
    try:
        port.timeout = timeout
        line = port.readline()
        if not line:
            raise SerialTimeoutError("Read timed out (no data)")
        return line.decode("ascii", errors="replace").strip()
    except serial.SerialTimeoutException:
        raise SerialTimeoutError("Read timed out")
    except (serial.SerialException, OSError) as e:
        raise SerialDisconnectedError(f"Read failed: {friendly_error_message(e)}")
    finally:
        try:
            port.timeout = old_timeout
        except Exception:
            pass


def safe_read_all(port, pre_delay: float = 0.01) -> str:
    """
    Read all available data from a serial port.

    Returns:
        Decoded and stripped response string.
    """
    if not check_port_health(port):
        raise SerialDisconnectedError("Serial port is not connected")

    time.sleep(pre_delay)
    try:
        data = port.read_all()
        return data.decode("ascii", errors="replace").strip()
    except (serial.SerialException, OSError) as e:
        raise SerialDisconnectedError(f"Read failed: {friendly_error_message(e)}")


# ═══════════════════════════════════════════════════════════════════
# Port Discovery
# ═══════════════════════════════════════════════════════════════════

def list_serial_ports() -> list[dict]:
    """
    List available serial ports with metadata.

    Returns:
        List of dicts with 'device', 'description', 'hwid', 'manufacturer'.
    """
    if not _HAS_SERIAL:
        return []
    try:
        return [
            {
                "device": p.device,
                "description": p.description,
                "hwid": p.hwid,
                "manufacturer": getattr(p, "manufacturer", ""),
            }
            for p in serial.tools.list_ports.comports()
        ]
    except Exception as e:
        logger.error(f"Failed to list ports: {e}")
        return []


# ═══════════════════════════════════════════════════════════════════
# Friendly Error Messages
# ═══════════════════════════════════════════════════════════════════

def friendly_error_message(error: Exception) -> str:
    """Convert low-level serial exceptions to user-friendly strings."""
    msg = str(error).lower()

    if "access is denied" in msg or "permission" in msg:
        return "Port access denied — is another program using it?"
    if "filenotfounderror" in msg or "no such file" in msg:
        return "Port not found — is the device plugged in?"
    if "device disconnected" in msg or "ioerror" in msg:
        return "Device disconnected — check USB cable."
    if "timeout" in msg:
        return "Communication timed out — device may be busy."
    if "break condition" in msg:
        return "Serial break condition — try reconnecting."

    return f"Serial error: {error}"


# ═══════════════════════════════════════════════════════════════════
# Connection Watchdog
# ═══════════════════════════════════════════════════════════════════

class ConnectionWatchdog:
    """
    Background thread that monitors serial connections.

    Calls a callback when a previously-connected port is detected as
    disconnected.

    Usage::

        wd = ConnectionWatchdog(check_interval=3.0)
        wd.watch("XY", lambda: my_serial_port, on_xy_disconnect)
        wd.start()
        ...
        wd.stop()
    """

    def __init__(self, check_interval: float = 2.0):
        self.check_interval = check_interval
        self._watches: dict[str, dict] = {}
        self._periodic_callbacks: list[Callable] = []  # v7.3.5
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    def watch(
        self,
        name: str,
        port_getter: Callable,
        on_disconnect: Callable,
    ) -> None:
        """
        Register a port to monitor.

        Args:
            name:           Identifier for this connection (e.g. "XY").
            port_getter:    Callable that returns the serial port object.
            on_disconnect:  Called (once) when a disconnect is detected.
        """
        with self._lock:
            self._watches[name] = {
                "port_getter": port_getter,
                "on_disconnect": on_disconnect,
                "was_connected": False,
            }

    def unwatch(self, name: str) -> None:
        """Stop watching a named connection."""
        with self._lock:
            self._watches.pop(name, None)

    def add_periodic(self, callback: Callable) -> None:
        """v7.3.5: Register a callback to run on every watchdog cycle."""
        with self._lock:
            self._periodic_callbacks.append(callback)

    def remove_periodic(self, callback: Callable) -> None:
        """v7.3.5: Remove a periodic callback."""
        with self._lock:
            try:
                self._periodic_callbacks.remove(callback)
            except ValueError:
                pass

    def start(self) -> None:
        """Start the watchdog thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._check_loop, daemon=True, name="Watchdog"
        )
        self._thread.start()
        logger.debug("ConnectionWatchdog started")

    def stop(self) -> None:
        """Stop the watchdog thread."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        logger.debug("ConnectionWatchdog stopped")

    def _check_loop(self) -> None:
        while self._running:
            with self._lock:
                watches = list(self._watches.items())

            for name, info in watches:
                try:
                    port = info["port_getter"]()
                    is_connected = check_port_health(port)

                    if info["was_connected"] and not is_connected:
                        logger.warning(f"[Watchdog] {name} disconnected!")
                        try:
                            info["on_disconnect"]()
                        except Exception as e:
                            logger.error(f"[Watchdog] Disconnect callback error for {name}: {e}")

                    info["was_connected"] = is_connected
                except Exception:
                    pass  # Don't crash the watchdog

            # v7.3.5: Run periodic callbacks (e.g. position save)
            with self._lock:
                callbacks = list(self._periodic_callbacks)
            for cb in callbacks:
                try:
                    cb()
                except Exception as e:
                    logger.debug(f"[Watchdog] Periodic callback error: {e}")

            time.sleep(self.check_interval)
