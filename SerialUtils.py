"""
Serial Utilities - Resilient serial communication helpers.

Provides:
  - Retry decorator for serial operations
  - Connection health checking
  - Friendly error messages
  - Serial port scanning utilities
"""

import time
import logging
import functools
from typing import Optional, Callable

logger = logging.getLogger(__name__)

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


class SerialError(Exception):
    """Friendly serial communication error."""
    pass


class SerialDisconnectedError(SerialError):
    """The serial device has been disconnected."""
    pass


class SerialTimeoutError(SerialError):
    """A serial operation timed out."""
    pass


def retry_serial(max_retries: int = 2, delay: float = 0.1,
                 on_failure: Optional[Callable] = None):
    """
    Decorator that retries serial operations on transient failures.
    
    Args:
        max_retries: Number of retry attempts
        delay: Delay between retries in seconds
        on_failure: Optional callback on final failure (receives exception)
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            last_error = None
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
                        error_msg = _friendly_serial_error(e)
                        logger.error(f"{func.__name__} failed: {error_msg}")
                        if on_failure:
                            on_failure(e)
                        raise SerialError(error_msg) from e
                except Exception:
                    raise
            return None
        return wrapper
    return decorator


def check_port_health(port) -> bool:
    """
    Check if a serial port is still alive and responsive.
    
    Args:
        port: pyserial Serial object
        
    Returns:
        True if port appears healthy
    """
    if port is None:
        return False

    try:
        if hasattr(port, 'is_open') and not port.is_open:
            return False
        # Try a non-destructive check
        _ = port.in_waiting
        return True
    except (serial.SerialException, OSError):
        return False


def safe_serial_write(port, data: bytes, timeout: float = 2.0) -> bool:
    """
    Write data to serial port with timeout and error handling.
    
    Returns:
        True if write succeeded
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
        raise SerialDisconnectedError(f"Write failed: {_friendly_serial_error(e)}")


def safe_serial_readline(port, timeout: float = 1.0) -> str:
    """
    Read a line from serial port with timeout and error handling.
    
    Returns:
        Decoded string (stripped)
    """
    if not check_port_health(port):
        raise SerialDisconnectedError("Serial port is not connected")

    old_timeout = port.timeout
    try:
        port.timeout = timeout
        line = port.readline()
        if not line:
            raise SerialTimeoutError("Read timed out (no data)")
        return line.decode('ascii', errors='replace').strip()
    except serial.SerialTimeoutException:
        raise SerialTimeoutError("Read timed out")
    except (serial.SerialException, OSError) as e:
        raise SerialDisconnectedError(f"Read failed: {_friendly_serial_error(e)}")
    finally:
        try:
            port.timeout = old_timeout
        except Exception:
            pass


def safe_serial_read_all(port, pre_delay: float = 0.01) -> str:
    """
    Read all available data from serial port.
    
    Returns:
        Decoded string (stripped)
    """
    if not check_port_health(port):
        raise SerialDisconnectedError("Serial port is not connected")

    time.sleep(pre_delay)
    try:
        data = port.read_all()
        return data.decode('ascii', errors='replace').strip()
    except (serial.SerialException, OSError) as e:
        raise SerialDisconnectedError(f"Read failed: {_friendly_serial_error(e)}")


def list_serial_ports() -> list[dict]:
    """
    List available serial ports with details.
    
    Returns:
        List of dicts with 'device', 'description', 'hwid' keys
    """
    if serial is None:
        return []

    try:
        ports = serial.tools.list_ports.comports()
        return [
            {
                "device": p.device,
                "description": p.description,
                "hwid": p.hwid,
                "manufacturer": getattr(p, 'manufacturer', ''),
            }
            for p in ports
        ]
    except Exception as e:
        logger.error(f"Failed to list ports: {e}")
        return []


def _friendly_serial_error(error: Exception) -> str:
    """Convert serial exceptions to user-friendly messages."""
    msg = str(error).lower()

    if "access is denied" in msg or "permission" in msg:
        return "Port access denied. Is another program using it?"
    elif "filenotfounderror" in msg or "no such file" in msg:
        return "Port not found. Is the device plugged in?"
    elif "device disconnected" in msg or "ioerror" in msg:
        return "Device disconnected. Check USB cable."
    elif "timeout" in msg:
        return "Communication timed out. Device may be busy or unresponsive."
    elif "break condition" in msg:
        return "Serial break condition. Try reconnecting."
    else:
        return f"Serial error: {error}"


class ConnectionWatchdog:
    """
    Monitors serial connections and calls a callback when disconnect is detected.
    
    Usage:
        watchdog = ConnectionWatchdog(check_interval=2.0)
        watchdog.watch("XY", serial_port, on_disconnect_callback)
        watchdog.start()
    """

    def __init__(self, check_interval: float = 2.0):
        self.check_interval = check_interval
        self._watches: dict[str, dict] = {}
        self._running = False
        self._thread = None

    def watch(self, name: str, port_getter: Callable, on_disconnect: Callable):
        """
        Register a serial port to watch.
        
        Args:
            name: Identifier for this connection
            port_getter: Callable that returns the serial port object (or None)
            on_disconnect: Callback when disconnect is detected
        """
        self._watches[name] = {
            "port_getter": port_getter,
            "on_disconnect": on_disconnect,
            "was_connected": False,
        }

    def unwatch(self, name: str):
        """Stop watching a connection."""
        self._watches.pop(name, None)

    def start(self):
        """Start the watchdog thread."""
        import threading
        self._running = True
        self._thread = threading.Thread(target=self._check_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the watchdog."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)

    def _check_loop(self):
        while self._running:
            for name, info in list(self._watches.items()):
                try:
                    port = info["port_getter"]()
                    is_connected = check_port_health(port)

                    if info["was_connected"] and not is_connected:
                        logger.warning(f"[Watchdog] {name} disconnected!")
                        try:
                            info["on_disconnect"]()
                        except Exception as e:
                            logger.error(f"[Watchdog] Disconnect callback error: {e}")

                    info["was_connected"] = is_connected
                except Exception:
                    pass

            time.sleep(self.check_interval)
