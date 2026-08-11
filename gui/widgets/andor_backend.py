"""
Andor Backend — pylablib wrapper for ANDOR SDK3 scientific cameras.

v7.5.x: Self-contained ANDOR Zyla (SDK3) camera interface that provides the
same OpenCV-VideoCapture-like API the CameraWidget dual/triple-backend system
expects from ``ToupCamBackend`` (see gui/widgets/toupcam_backend.py). Targets
the ANDOR ZYLA-4.2P-USB3, a 2048x2048 mono 16-bit sCMOS camera.

Key differences from the ToupTek path:
  * Driven by the Andor SDK3 runtime (atcore.dll + companion DLLs) via the
    ``pylablib`` package (``Andor.AndorSDK3Camera``) — NOT UVC/DirectShow, NOT
    ToupTek. It never appears in the OpenCV or ToupCam enumerations.
  * The sensor is mono 16-bit; the rest of the app expects 8-bit BGR. This
    backend converts every frame to BGR8 inside ``read()`` (per-frame
    percentile auto-scale + GRAY2BGR), so the CameraWidget pipeline is unchanged.
  * The live feed defaults to a binned readout (2x2 -> 1024x1024) for a smooth
    USB3 preview; full 2048x2048 is selectable via the resolution list.

Everything is lazy + guarded: importing this module never requires pylablib or
loads the SDK DLLs. ``ANDOR_AVAILABLE`` only becomes truthy on first access
when numpy + pylablib import AND the atcore DLL directory is found. When the SDK
is absent the app simply behaves as if no Andor camera exists.

Usage::

    from gui.widgets.andor_backend import AndorBackend, ANDOR_AVAILABLE

    if ANDOR_AVAILABLE:
        devices = AndorBackend.enumerate()
        cam = AndorBackend()
        if cam.open(devices[0]['id']):
            ok, frame = cam.read()   # BGR uint8 numpy array
            cam.release()
"""

from __future__ import annotations

import os
import sys
import logging
import threading
import time
from pathlib import Path

logger = logging.getLogger(__name__)

try:
    import numpy as np
    _NP_AVAILABLE = True
except ImportError:
    _NP_AVAILABLE = False


# ════════════════════════════════════════════════════════════════════
#  Resolution / binning presets
# ════════════════════════════════════════════════════════════════════
# The Zyla exposes resolutions through on-sensor binning of the full
# 2048x2048 array. Each preset is (width, height, hbin, vbin). The live
# preview auto-picks a preset <= 1280 wide (1024x1024, 2x2) for a smooth
# USB3 stream; full-res 1x1 is available via the resolution combo.
_ZYLA_FULL = (2048, 2048)
RESOLUTION_PRESETS = [
    (1024, 1024, 2, 2),   # index 0 — default preview (auto-picked)
    (512, 512, 4, 4),     # index 1
    (256, 256, 8, 8),     # index 2
    (2048, 2048, 1, 1),   # index 3 — full resolution
]

# Default exposure applied on open (seconds) when the camera comes up with an
# unusable value. A middling exposure so the operator sees *something*.
_DEFAULT_EXPOSURE_S = 0.03

# Longest exposure the UI advertises (the Zyla's spec'd maximum is 30 s). The
# setter always clamps against the SDK's own live range, so an optimistic
# ceiling here is safe — an artificially tiny one was the v7.13 bench bug.
_MAX_EXPOSURE_S = 30.0

# Fraction of the USB link's MaxInterfaceTransferRate the frame rate is capped
# to. Running AT the link ceiling overflows the camera's internal buffer under
# any jitter (dropped frames / stalled acquisition at full resolution).
_LINK_RATE_MARGIN = 0.97

# Headroom multiplier when lowering the frame period to fit a requested
# exposure (rolling-shutter readout overhead).
_EXPOSURE_PERIOD_HEADROOM = 1.02

# Display-level range for the mono-16 sensor (black/white points used when the
# per-frame auto-scale is turned OFF).
_LEVEL_MAX = 65535


# ════════════════════════════════════════════════════════════════════
#  Sensor feature table (v7.13)
# ════════════════════════════════════════════════════════════════════
# The Zyla's biggest signal-to-noise levers are plain SDK3 features that were
# never wired: sensor cooling (dark current), pixel readout rate (slow readout
# = lower read noise), the pre-amp gain mode (16-bit low-noise), and the
# on-camera spurious-noise / static-blemish filters. One table drives the
# probe, the open-time defaults, the capability advertisement, the settings
# dialog and persistence, so a feature can never be half-wired.
#
# Rows: (settings_key, sdk_feature_name, kind, default)
#   kind "bool" — default is the bool to apply.
#   kind "enum" — default is a tuple of MATCH TOKENS, not a literal SDK
#     string: the applied value is whichever runtime-enumerated allowed value
#     contains all tokens (case-insensitive). Hardcoding an SDK enum string a
#     different camera/SDK build might spell differently is exactly the class
#     of semantic mismatch the Tucsen bring-up documented — never do it.
ANDOR_SENSOR_FEATURES = (
    ("andor_sensor_cooling",     "SensorCooling",           "bool", True),
    ("andor_readout_rate",       "PixelReadoutRate",        "enum", ("216",)),
    ("andor_gain_mode",          "SimplePreAmpGainControl", "enum", ("16-bit", "low noise")),
    ("andor_noise_filter",       "SpuriousNoiseFilter",     "bool", True),
    ("andor_blemish_correction", "StaticBlemishCorrection", "bool", True),
)

# Keys (in table order) — convenience for capability/persistence consumers.
ANDOR_SENSOR_FEATURE_KEYS = tuple(row[0] for row in ANDOR_SENSOR_FEATURES)


def _match_enum_value(values, tokens) -> "str | None":
    """First allowed enum value containing ALL tokens (case-insensitive).

    ``values`` is the runtime-enumerated list from the SDK; ``tokens`` the
    match-token tuple from ANDOR_SENSOR_FEATURES. Returns None when nothing
    matches — the caller must then SKIP the default rather than guess.
    """
    if not values:
        return None
    toks = [str(t).lower() for t in (tokens or ())]
    if not toks:
        return None
    for v in values:
        s = str(v).lower()
        if all(t in s for t in toks):
            return str(v)
    return None


def _parse_bit_depth(text) -> "int | None":
    """Full-scale count from a bit-depth-ish string ("12 Bit", "16-bit (...)").

    Returns 2**n − 1 for the first plausible integer (8..32) found, else None.
    """
    if text is None:
        return None
    digits = ""
    for ch in str(text):
        if ch.isdigit():
            digits += ch
        elif digits:
            break
    if not digits:
        return None
    try:
        n = int(digits)
    except ValueError:
        return None
    if 8 <= n <= 32:
        return (1 << n) - 1
    return None


# ════════════════════════════════════════════════════════════════════
#  DLL Discovery
# ════════════════════════════════════════════════════════════════════

def _find_andor_dll_dir() -> str | None:
    """Locate the directory containing ``atcore.dll`` (Andor SDK3 runtime).

    Search order:
      1. Repo-bundled ``DLLs/zyla dlls/`` (operator-provided, checked in).
      2. Andor SDK3 developer install (Program Files\\Andor SDK3).
      3. Andor Solis install (bundles the SDK3 DLLs).
      4. System PATH.
    Returns the directory path string, or None if atcore.dll is not found.
    """
    import platform

    core = "atcore.dll" if platform.system() == "Windows" else "libatcore.so"

    candidates: list[Path] = []

    # 1. Repo-bundled DLLs (project_root/DLLs/zyla dlls). Note the space in the
    #    folder name — kept verbatim to match what the operator pasted in.
    project_root = Path(__file__).resolve().parent.parent.parent
    candidates.append(project_root / "DLLs" / "zyla dlls")

    # 2/3. Program Files installs
    if platform.system() == "Windows":
        prog_dirs = [
            os.environ.get("PROGRAMFILES", r"C:\Program Files"),
            os.environ.get("PROGRAMFILES(X86)", r"C:\Program Files (x86)"),
        ]
        app_names = ["Andor SDK3", "Andor SOLIS", "Andor Solis", "Andor"]
        for prog in prog_dirs:
            if not prog:
                continue
            for app in app_names:
                candidates.append(Path(prog) / app)

    # 4. System PATH
    for p in os.environ.get("PATH", "").split(os.pathsep):
        if p:
            candidates.append(Path(p))

    for d in candidates:
        try:
            if (d / core).exists():
                logger.info(f"Andor SDK3 DLLs found: {d}")
                return str(d)
        except OSError:
            continue

    # Fall back to a recursive scan of the Program Files app dirs (SDK3 may
    # nest the DLLs a level deep).
    if platform.system() == "Windows":
        for prog in [os.environ.get("PROGRAMFILES", r"C:\Program Files"),
                     os.environ.get("PROGRAMFILES(X86)", r"C:\Program Files (x86)")]:
            if not prog:
                continue
            for app in ["Andor SDK3", "Andor SOLIS", "Andor Solis"]:
                base = Path(prog) / app
                if not base.exists():
                    continue
                try:
                    for hit in base.rglob(core):
                        logger.info(f"Andor SDK3 DLLs found (nested): {hit.parent}")
                        return str(hit.parent)
                except OSError:
                    continue

    return None


# ════════════════════════════════════════════════════════════════════
#  pylablib loading (lazy, cached)
# ════════════════════════════════════════════════════════════════════

_andor_mod = None      # pylablib.devices.Andor module, once loaded
_andor_error = None    # cached failure reason


def _load_andor():
    """Import pylablib's Andor module with the SDK3 DLL path configured.

    Returns the ``pylablib.devices.Andor`` module, or None if unavailable.
    Result is cached (success or failure) so the import/DLL probe runs once.
    """
    global _andor_mod, _andor_error

    if _andor_mod is not None:
        return _andor_mod
    if _andor_error is not None:
        return None

    if not _NP_AVAILABLE:
        _andor_error = "numpy not available"
        return None

    dll_dir = _find_andor_dll_dir()
    if dll_dir is None:
        _andor_error = "atcore.dll (Andor SDK3) not found"
        logger.info(f"Andor: {_andor_error}")
        return None

    try:
        # Help Windows resolve the SDK3 companion DLLs.
        if hasattr(os, "add_dll_directory"):
            try:
                os.add_dll_directory(dll_dir)
            except OSError:
                pass

        import pylablib as pll
        pll.par["devices/dlls/andor_sdk3"] = dll_dir
        from pylablib.devices import Andor  # noqa: N814

        _andor_mod = Andor
        logger.info(f"Andor (pylablib) loaded; SDK3 dlls at {dll_dir}")
        return Andor
    except Exception as exc:  # ImportError, OSError, pylablib errors
        _andor_error = f"pylablib/Andor import failed: {exc}"
        logger.info(f"Andor: {_andor_error}")
        return None


def _check_available() -> bool:
    """True if numpy + pylablib + SDK3 DLLs are all present."""
    if not _NP_AVAILABLE:
        return False
    return _load_andor() is not None


class _LazyAvailable:
    """Descriptor that resolves availability on first bool() access.

    Mirrors ToupCamBackend's pattern so merely importing this module never
    touches the SDK.
    """
    _value = None

    def __bool__(self):
        if self._value is None:
            self._value = _check_available()
        return self._value

    def __repr__(self):
        return str(bool(self))


ANDOR_AVAILABLE = _LazyAvailable()


# v7.13: the raw averaged-capture request lives in gui/widgets/mono_display.py
# (RawAverageRequest) so the Tucsen backend services the IDENTICAL contract —
# re-exported below as _RawAverageRequest for existing references.


# ════════════════════════════════════════════════════════════════════
#  AndorBackend — OpenCV/ToupCam-like Camera Interface
# ════════════════════════════════════════════════════════════════════

class AndorBackend:
    """ANDOR SDK3 camera (Zyla) with the ToupCam-compatible backend surface.

    Threading: pylablib runs its own SDK acquisition; a daemon reader thread
    pulls the newest mono-16 frame into a lock-protected buffer, and ``read()``
    returns the latest frame converted to BGR8. This keeps the GUI-thread grab
    timer non-blocking (mirrors ToupCamBackend's callback -> buffer -> read()).
    """

    # The Zyla has no ToupTek-style ISP brightness/contrast/gamma; only
    # exposure is exposed as a hardware control (see hardware_capabilities()).
    HW_RANGES: dict = {}

    # ── Discovery ─────────────────────────────────────────────────
    @staticmethod
    def enumerate() -> list[dict]:
        """Enumerate connected Andor SDK3 cameras.

        Returns a list of dicts with keys ``id`` (stable serial string),
        ``displayname`` (model), ``resolutions`` (binning presets), and
        ``preview_count``. Empty list when the SDK is unavailable.
        """
        Andor = _load_andor()
        if Andor is None:
            return []

        try:
            count = int(_andor_cameras_number(Andor))
        except Exception as exc:
            logger.warning(f"Andor enumerate failed: {exc}")
            return []

        if count <= 0:
            # The SDK3 runtime loaded but no camera was found. The usual cause is
            # a missing/failed USB DRIVER (Device Manager shows the camera with a
            # yellow bang / Code 28) — copying the SDK3 DLLs does NOT install the
            # camera's USB driver; run the Andor SDK3 / Solis installer for that.
            # Other causes: camera unpowered / cable, or bound by another process.
            logger.info(
                "Andor SDK3 loaded but 0 cameras enumerated — check the camera "
                "is powered + connected and that the Andor USB driver is "
                "installed (Device Manager must not show it with an error/Code 28)")
            return []

        devices = []
        res = [(w, h) for (w, h, _hb, _vb) in RESOLUTION_PRESETS]
        for idx in range(count):
            serial, model = _read_identity(Andor, idx)
            devices.append({
                "id": serial or f"andor#{idx}",
                "displayname": model or f"Andor Zyla #{idx}",
                "resolutions": res,
                "preview_count": len(res),
                "index": idx,
            })
            logger.info(f"Andor found: {model} (serial {serial}) idx={idx}")
        return devices

    def __init__(self):
        self._Andor = None
        self._cam = None
        self._frame: "np.ndarray | None" = None       # latest BGR8
        self._lock = threading.Lock()
        self._frame_ready = threading.Event()
        # v7.14: monotonic count of GENUINELY-NEW frames delivered by the
        # reader thread. read() is non-blocking and hands back the same cached
        # frame as often as it is asked, so a caller polling read() cannot tell
        # a new frame from a repeat — which is how a mosaic tile ended up
        # stitching the frame exposed DURING the stage move. See
        # SupportClasses/CaptureTiming.py.
        self._frames_acquired = 0
        self._reader: "threading.Thread | None" = None
        self._running = False
        self._w = 0
        self._h = 0
        self._eSize = 0
        self._device_id = ""
        # Display scaling (mono-16 -> 8-bit conversion for the live view).
        # auto=True reproduces the historical per-frame percentile auto-scale
        # (the display brightness "chases" the scene); auto=False freezes the
        # mapping at fixed black/white levels in sensor counts.
        self._auto_scale = True
        self._scale_lo = 0
        self._scale_hi = _LEVEL_MAX
        self._last_auto_levels: "tuple[float, float] | None" = None
        # v7.13 — sensor features probed at open (key -> spec dict); the true
        # full-scale clip level for the CURRENT gain mode (12-bit modes clip at
        # ~4095, far below the uint16 container); latest raw-frame statistics;
        # the pending averaged-capture request serviced by the reader thread.
        self._features: dict = {}
        self._clip_level = _LEVEL_MAX
        self._raw_stats: "dict | None" = None
        self._avg_request: "_RawAverageRequest | None" = None
        self._temp_read_ts = 0.0
        self._temp_cache: tuple = (None, None)

    # ── Lifecycle ─────────────────────────────────────────────────
    def open(self, device_id: str, resolution_index: int | None = None) -> bool:
        """Open the Andor camera identified by ``device_id`` (serial string).

        Falls back to opening index 0 if the serial can't be matched. Sets a
        default binning preset (auto-picks <=1280 wide unless resolution_index
        is given), configures Mono16 + a sane exposure, and starts acquisition
        + the reader thread.
        """
        if self._cam is not None:
            self.release()

        Andor = _load_andor()
        if Andor is None:
            return False
        self._Andor = Andor

        idx = _index_for_id(Andor, device_id)
        try:
            cam = Andor.AndorSDK3Camera(idx=idx)
        except Exception as exc:
            logger.warning(f"Andor: failed to open idx={idx}: {exc}")
            return False
        self._cam = cam
        self._device_id = device_id

        # Deliver mono-16 frames.
        try:
            cam.set_attribute_value("PixelEncoding", "Mono16")
        except Exception:
            pass

        # v7.13 — probe the sensor-quality features and apply the low-noise
        # defaults BEFORE acquisition starts (readout rate / gain mode may be
        # NOTWRITABLE mid-acquisition). Persisted per-identity hw_controls are
        # restored AFTER start via the camera_started signal and simply
        # overwrite these — persisted wins, absent key = default stands.
        self._probe_sensor_features()
        self._apply_sensor_defaults()
        self._clip_level = self._read_clip_level()

        # Resolution / binning
        if resolution_index is None:
            resolution_index = self._auto_pick_resolution_index()
        self._apply_resolution_index(resolution_index)

        # A sane default exposure so a fresh camera shows something.
        try:
            cur = cam.get_exposure()
            if not cur or cur <= 0:
                cam.set_exposure(_DEFAULT_EXPOSURE_S)
        except Exception:
            pass

        # v7.13.x — cap FrameRate to the USB link's sustainable rate BEFORE
        # acquisition starts. At full 2048x2048 Mono16 the sensor-max frame
        # rate exceeds MaxInterfaceTransferRate, overflowing the camera's
        # internal buffer (dropped frames, stalled acquisition).
        self._sync_frame_rate()

        if not self._start_stream():
            self.release()
            return False

        logger.info(f"Andor opened: {self._w}x{self._h} (eSize {self._eSize})")
        return True

    def _buffer_nframes(self) -> int:
        """Ring-buffer depth sized by BYTES, not a fixed frame count.

        pylablib defaults to 100 frames; at full 2048x2048 mono-16 that is
        ~840 MB (8.4 MB/frame) — a huge allocation that can thrash RAM and stall
        the GUI. Cap the ring to ~256 MB (min 10 frames) so a full-res stream
        stays light."""
        frame_bytes = max(1, int(self._w) * int(self._h) * 2)
        return int(min(100, max(10, (256 * 1024 * 1024) // frame_bytes)))

    def _start_stream(self) -> bool:
        """Start continuous acquisition + the daemon reader thread."""
        cam = self._cam
        if cam is None:
            return False
        try:
            cam.start_acquisition(mode="sequence", nframes=self._buffer_nframes())
        except Exception as exc:
            logger.warning(f"Andor: start_acquisition failed: {exc}")
            return False
        self._running = True
        self._frame_ready.clear()
        self._reader = threading.Thread(
            target=self._reader_loop, daemon=True, name="AndorReader")
        self._reader.start()
        return True

    def _rearm_acquisition(self):
        """Re-arm a stopped acquisition in place (reader-thread self-heal).

        Called only from the reader thread when the SDK acquisition has stopped
        (e.g. a transient buffer overflow) so the live feed recovers instead of
        silently freezing."""
        cam = self._cam
        if cam is None:
            return
        try:
            cam.stop_acquisition()
        except Exception:
            pass
        try:
            cam.start_acquisition(mode="sequence", nframes=self._buffer_nframes())
        except Exception as exc:
            logger.debug(f"Andor re-arm failed: {exc}")

    def _stall_threshold_fails(self) -> int:
        """Consecutive wait_for_frame failures before a FORCED re-arm.

        Exposure-aware: at 0.5 s per wait timeout a long exposure produces
        several timeouts between perfectly healthy frames, so the stall window
        is max(10 s, 4x the current exposure). A wedged camera whose exposure
        can't even be read falls back to the 10 s floor — the case that most
        needs the forced re-arm.
        """
        exp_s = 0.0
        try:
            e = self.get_exposure_time()
            if e:
                exp_s = float(e) / 1e6
        except Exception:
            pass
        stall_s = max(10.0, 4.0 * exp_s)
        return max(20, int(stall_s / 0.5) + 1)

    def _reader_loop(self):
        """Pull the newest frame from pylablib into the BGR8 buffer.

        Resilient: logs (throttled) instead of silently swallowing errors, and
        if the SDK acquisition has actually stopped it re-arms it — so a
        transient stall self-heals rather than freezing the feed forever."""
        cam = self._cam
        fails = 0
        backoff = 1
        while self._running and cam is not None:
            try:
                cam.wait_for_frame(timeout=0.5)
            except Exception as exc:
                fails += 1
                if fails == 1 or fails % 50 == 0:
                    logger.info(
                        f"Andor reader: wait_for_frame failed x{fails}: {exc}")
                # If acquisition died (e.g. buffer overflow), re-arm it rather
                # than spin silently. Check occasionally to avoid hammering.
                if fails % 10 == 0:
                    rearmed = False
                    try:
                        if not cam.acquisition_in_progress():
                            logger.info("Andor reader: acquisition stopped — re-arming")
                            self._rearm_acquisition()
                            rearmed = True
                    except Exception:
                        pass
                    # v7.13.x — a wedged USB stream can keep CLAIMING the
                    # acquisition is in progress while no frame ever arrives
                    # (bench: full-res feed "goes dead"). Force a re-arm after
                    # a stall window sized to the exposure (a 5 s exposure
                    # legitimately yields many wait timeouts per frame), with
                    # exponential backoff so a truly dead camera doesn't
                    # re-arm-spin.
                    if not rearmed and fails >= self._stall_threshold_fails() * backoff:
                        logger.info(
                            f"Andor reader: no frame for ~{fails * 0.5:.0f}s while "
                            "acquisition claims to be in progress — forcing a re-arm")
                        self._rearm_acquisition()
                        rearmed = True
                        backoff = min(backoff * 2, 16)
                    if rearmed:
                        fails = 0
                continue
            try:
                raw = cam.read_newest_image()
            except Exception as exc:
                logger.debug(f"Andor read_newest_image error: {exc}")
                continue
            if raw is None:
                continue

            # Reduce to ONE 2-D plane — raw statistics, the averaged-capture
            # accumulator, the auto-levels and the display conversion must all
            # consult the same pixels.
            plane = None
            if _NP_AVAILABLE:
                try:
                    arr = np.asarray(raw)
                    if arr.ndim == 3:
                        arr = arr[..., 0]
                    if arr.ndim == 2:
                        plane = arr
                except Exception:
                    plane = None

            # v7.13 — raw 16-bit statistics (saturation/histogram). Guarded so
            # statistics can never take the feed down.
            if plane is not None and plane.dtype != np.uint8:
                try:
                    stats = compute_raw_frame_stats(plane, self._clip_level)
                except Exception:
                    stats = None
                if stats is not None:
                    now = time.monotonic()
                    if now - self._temp_read_ts > 2.0:
                        # The reader thread owns the camera, so the periodic
                        # temperature read happens here — never on a GUI timer.
                        self._temp_read_ts = now
                        self._temp_cache = (self.get_sensor_temperature(),
                                            self.get_temperature_status())
                    stats["temperature_c"] = self._temp_cache[0]
                    stats["temperature_status"] = self._temp_cache[1]
                    with self._lock:
                        self._raw_stats = stats

            # v7.13 — service a pending averaged-capture request.
            if plane is not None:
                with self._lock:
                    req = self._avg_request
                if req is not None:
                    try:
                        req.add(plane)
                    except Exception as exc:
                        req.fail(f"accumulate error: {exc}")
                    if req.done.is_set():
                        with self._lock:
                            if self._avg_request is req:
                                self._avg_request = None

            with self._lock:
                auto = self._auto_scale
                manual_levels = (self._scale_lo, self._scale_hi)
            levels = None
            if auto:
                # Per-frame percentile auto-scale; remember the levels so a
                # switch to manual freezes the CURRENT look instead of jumping.
                if plane is not None and plane.dtype != np.uint8:
                    levels = _auto_levels(plane)
                    with self._lock:
                        self._last_auto_levels = levels
            else:
                levels = manual_levels
            bgr = _mono_to_bgr8(raw, levels=levels)
            if bgr is None:
                continue
            with self._lock:
                self._frame = bgr
                self._frames_acquired += 1
            self._frame_ready.set()
            fails = 0
            backoff = 1

    def isOpened(self) -> bool:
        return self._cam is not None and self._running

    def frames_acquired(self) -> int:
        """v7.14: monotonic count of distinct frames the sensor has delivered.

        Advances ONLY when the reader thread receives a new frame — unlike
        ``read()``, which returns the cached frame on demand. A caller that
        needs a post-move frame must wait on THIS, not on read() call count.
        """
        with self._lock:
            return self._frames_acquired

    def read(self) -> tuple[bool, "np.ndarray | None"]:
        """Return (ok, latest BGR8 frame). Non-blocking after the first frame."""
        if not self.isOpened():
            return False, None
        if self._frame is None:
            self._frame_ready.wait(timeout=1.0)
        with self._lock:
            if self._frame is not None:
                return True, self._frame.copy()
        return False, None

    def release(self):
        """Stop acquisition + close the camera, serialized against the reader."""
        self._fail_pending_average("camera released")
        self._running = False
        reader = self._reader
        self._reader = None
        if reader is not None and reader.is_alive() and reader is not threading.current_thread():
            try:
                reader.join(timeout=1.5)
            except Exception:
                pass
        cam = self._cam
        with self._lock:
            self._cam = None
            self._frame = None
            self._raw_stats = None
        if cam is not None:
            try:
                cam.stop_acquisition()
            except Exception:
                pass
            try:
                cam.close()
            except Exception:
                pass
        self._frame_ready.clear()
        if sys.meta_path is not None:
            logger.info("Andor released")

    def __del__(self):
        try:
            self.release()
        except Exception:
            pass

    # ── Resolution / binning ──────────────────────────────────────
    def _auto_pick_resolution_index(self) -> int:
        """Pick the first preset <= 1280 wide (else full res)."""
        for i, (w, _h, _hb, _vb) in enumerate(RESOLUTION_PRESETS):
            if 0 < w <= 1280:
                return i
        return len(RESOLUTION_PRESETS) - 1

    def _apply_resolution_index(self, index: int) -> bool:
        """Set full-frame ROI at the preset's binning."""
        cam = self._cam
        if cam is None:
            return False
        index = max(0, min(int(index), len(RESOLUTION_PRESETS) - 1))
        w, h, hbin, vbin = RESOLUTION_PRESETS[index]
        ok = True
        try:
            # Full sensor AOI at the requested binning; pylablib clamps to the
            # detector and reports the achieved ROI.
            cam.set_roi(0, _ZYLA_FULL[0], 0, _ZYLA_FULL[1], hbin, vbin)
        except Exception as exc:
            logger.debug(f"Andor set_roi failed: {exc}")
            ok = False
        # Read back the achieved size.
        aw, ah = self._read_current_size(fallback=(w, h))
        self._w, self._h = aw, ah
        self._eSize = index
        return ok

    def _read_current_size(self, fallback: tuple[int, int]) -> tuple[int, int]:
        cam = self._cam
        if cam is None:
            return fallback
        try:
            roi = cam.get_roi()
            # pylablib get_roi -> (hstart, hend, vstart, vend, hbin, vbin)
            if roi and len(roi) >= 6:
                hstart, hend, vstart, vend, hbin, vbin = roi[:6]
                w = max(1, int((hend - hstart) // max(1, int(hbin))))
                h = max(1, int((vend - vstart) // max(1, int(vbin))))
                return (w, h)
        except Exception:
            pass
        # Fall back to a raw frame's shape if acquisition is live.
        with self._lock:
            f = self._frame
        if f is not None and f.ndim >= 2:
            return (int(f.shape[1]), int(f.shape[0]))
        return fallback

    def get_resolution(self) -> tuple[int, int]:
        return (self._w, self._h)

    def get_eSize(self):
        return self._eSize

    def get_resolution_list(self) -> list:
        return [(w, h) for (w, h, _hb, _vb) in RESOLUTION_PRESETS]

    def set_resolution_index(self, index: int) -> bool:
        """Switch binning preset: stop, re-ROI, restart, all under the lock."""
        cam = self._cam
        if cam is None:
            return False
        self._fail_pending_average("resolution changed")
        self._running = False
        reader = self._reader
        self._reader = None
        if reader is not None and reader.is_alive() and reader is not threading.current_thread():
            try:
                reader.join(timeout=1.5)
            except Exception:
                pass
        try:
            cam.stop_acquisition()
        except Exception:
            pass
        with self._lock:
            self._frame = None
        ok = self._apply_resolution_index(index)
        # The ROI change moves the sensor-max frame rate; re-cap it to the USB
        # link's sustainable rate (acquisition is stopped here — always
        # writable). Full resolution is exactly where the overflow bites.
        self._sync_frame_rate()
        started = self._start_stream()
        logger.info(f"Andor resolution -> {self._w}x{self._h} (eSize {self._eSize})"
                    f"{'' if (ok and started) else ' [partial]'}")
        return ok and started

    # ── Hardware controls (exposure + display scaling) ────────────
    def get_exposure_time(self):
        """Current exposure time in microseconds, or None."""
        cam = self._cam
        if cam is None:
            return None
        try:
            s = cam.get_exposure()
            return int(round(float(s) * 1e6)) if s else None
        except Exception:
            return None

    def _sdk_attr(self, name: str):
        """SDK attribute object with LIVE min/max when the pylablib build
        supports ``update_properties`` (without it the limits are the values
        cached at construction — stale relative to any binning / gain-mode /
        frame-rate change made since). None when absent."""
        cam = self._cam
        if cam is None:
            return None
        try:
            return cam.get_attribute(name, update_properties=True)
        except TypeError:
            pass
        except Exception:
            return None
        try:
            return cam.get_attribute(name)
        except Exception:
            return None

    def get_frame_rate(self) -> "float | None":
        cam = self._cam
        if cam is None:
            return None
        try:
            return float(cam.get_attribute_value("FrameRate"))
        except Exception:
            return None

    def get_max_interface_transfer_rate(self) -> "float | None":
        """The USB link's sustainable frame rate (fps), or None if the SDK
        doesn't expose it on this model."""
        cam = self._cam
        if cam is None:
            return None
        try:
            v = float(cam.get_attribute_value("MaxInterfaceTransferRate"))
            return v if v > 0 else None
        except Exception:
            return None

    def _sync_frame_rate(self):
        """Cap FrameRate to min(FrameRate.max, link rate x margin).

        FrameRate.max is dynamic — the SDK already folds the current exposure,
        ROI and readout rate into it — so raising FrameRate to (at most) that
        max can never clamp the exposure back down. Capping below the link's
        MaxInterfaceTransferRate is Andor's own guidance for USB Zylas: run
        faster and the camera's INTERNAL buffer overflows (dropped frames,
        stalled acquisition — the full-resolution "feed goes dead" bench bug).
        Missing attributes → silent no-op."""
        cam = self._cam
        if cam is None:
            return
        attr = self._sdk_attr("FrameRate")
        if attr is None:
            return
        hi = getattr(attr, "max", None)
        if not hi or float(hi) <= 0:
            return
        target = float(hi)
        mitr = self.get_max_interface_transfer_rate()
        if mitr:
            target = min(target, mitr * _LINK_RATE_MARGIN)
        lo = getattr(attr, "min", None)
        if lo:
            target = max(target, float(lo))
        try:
            cam.set_attribute_value("FrameRate", target)
            logger.debug(f"Andor: FrameRate -> {target:.3f} fps"
                         + (f" (link max {mitr:.3f})" if mitr else ""))
        except Exception as exc:
            logger.debug(f"Andor: FrameRate -> {target:.3f} fps refused: {exc}")

    def _apply_exposure_s(self, req_s: float) -> bool:
        """FrameRate-aware exposure write (see put_exposure_time)."""
        cam = self._cam
        if cam is None:
            return False
        # 1) Lower the frame rate first when the requested exposure does not
        #    fit the current frame period — the OPPOSITE of pylablib's
        #    set_exposure(), which pins FrameRate at max and truncates the
        #    exposure to ~one maximum-rate frame.
        try:
            fr = float(cam.get_attribute_value("FrameRate"))
            period = 1.0 / fr if fr > 0 else None
        except Exception:
            period = None
        if period is not None and req_s > period * 0.98:
            try:
                cam.set_frame_period(req_s * _EXPOSURE_PERIOD_HEADROOM)
            except Exception as exc:
                logger.debug(f"Andor: frame-period lowering failed: {exc}")
        # 2) Clamp into the LIVE ExposureTime range (limits reflect the frame
        #    rate we just set) and write the attribute directly — never
        #    through pylablib's set_exposure.
        t = req_s
        attr = self._sdk_attr("ExposureTime")
        if attr is not None:
            lo = getattr(attr, "min", None)
            hi = getattr(attr, "max", None)
            if lo is not None:
                t = max(t, float(lo))
            if hi is not None:
                t = min(t, float(hi))
        if abs(t - req_s) > max(1e-6, 0.02 * req_s):
            logger.info(f"Andor: exposure clamped {req_s * 1e3:.3f} -> "
                        f"{t * 1e3:.3f} ms by the SDK's own range")
        try:
            cam.set_attribute_value("ExposureTime", t)
        except Exception as exc:
            logger.debug(f"Andor set ExposureTime failed: {exc}")
            return False
        # 3) Re-raise FrameRate to min(its new max, link rate) so short
        #    exposures keep a live-feeling feed. Its max now accounts for the
        #    exposure just set, so this cannot clamp the exposure back down.
        self._sync_frame_rate()
        return True

    def put_exposure_time(self, microseconds) -> bool:
        """Set exposure, managing FrameRate so long exposures actually stick.

        pylablib's own ``set_exposure()`` first pins FrameRate at its MAXIMUM
        (``set_frame_period(0)``) and then truncates the request against
        ``ExposureTime.max ~= 1/FrameRate`` — so any exposure longer than one
        maximum-rate frame silently collapsed to tens of milliseconds (the
        "exposure resets to a small number" bench bug). This bypasses it:
        lower FrameRate to fit the exposure, write ExposureTime directly, then
        raise FrameRate back to min(max, link rate).

        Returns True only when the achieved exposure is within ~2 % of the
        request; the achieved value is always what get_exposure_time reads.
        """
        cam = self._cam
        if cam is None:
            return False
        req_s = max(0.0, float(microseconds) / 1e6)
        applied = self._apply_exposure_s(req_s)
        if not applied and self._running:
            # Live write refused — pause acquisition, apply, restart (the
            # proven set_sensor_feature recovery shape; the stream is ALWAYS
            # restarted so a refused setting never costs the feed).
            self._running = False
            reader = self._reader
            self._reader = None
            if reader is not None and reader.is_alive() \
                    and reader is not threading.current_thread():
                try:
                    reader.join(timeout=1.5)
                except Exception:
                    pass
            try:
                cam.stop_acquisition()
            except Exception:
                pass
            with self._lock:
                self._frame = None
            applied = self._apply_exposure_s(req_s)
            self._start_stream()
        if not applied:
            return False
        got = self.get_exposure_time()
        if got is not None:
            got_s = float(got) / 1e6
            if abs(got_s - req_s) > max(2e-4, 0.02 * req_s):
                logger.info(f"Andor: exposure requested {req_s * 1e3:.3f} ms, "
                            f"achieved {got_s * 1e3:.3f} ms")
                return False
        return True

    def get_exposure_time_range(self):
        """(min_us, max_us, default_us) or None.

        The hi bound is the exposure ACHIEVABLE — put_exposure_time lowers the
        frame rate to fit — not ExposureTime.max at the CURRENT frame rate,
        which is ~one frame period and was the tiny ceiling the dialog spin
        clamped every keystroke against. Capped at the Zyla's spec'd 30 s.
        """
        cam = self._cam
        if cam is None:
            return None
        attr = self._sdk_attr("ExposureTime")
        if attr is None:
            return None
        lo = getattr(attr, "min", None)
        hi = getattr(attr, "max", None)
        if lo is None or hi is None:
            return None
        lo, hi = float(lo), float(hi)
        fr = self._sdk_attr("FrameRate")
        fr_min = getattr(fr, "min", None) if fr is not None else None
        if fr_min and float(fr_min) > 0:
            hi = max(hi, min(_MAX_EXPOSURE_S, 1.0 / float(fr_min)))
        try:
            cur = cam.get_exposure() or lo
        except Exception:
            cur = lo
        return (int(round(lo * 1e6)), int(round(hi * 1e6)),
                int(round(float(cur) * 1e6)))

    # ── Sensor-quality features (v7.13) ───────────────────────────
    # Cooling / readout rate / gain mode / noise + blemish filters. All access
    # is defensive: a feature that fails to probe is simply absent (its dialog
    # control auto-hides), and enum values are always the SDK's own
    # runtime-enumerated strings — never hardcoded literals.

    def _probe_sensor_features(self):
        """Discover which ANDOR_SENSOR_FEATURES this camera implements."""
        cam = self._cam
        feats: dict = {}
        if cam is None:
            self._features = feats
            return
        for key, sdk_name, kind, default in ANDOR_SENSOR_FEATURES:
            try:
                attr = cam.get_attribute(sdk_name)
            except Exception:
                continue
            if attr is None:
                continue
            values = None
            if kind == "enum":
                try:
                    vals = getattr(attr, "values", None)
                    if vals:
                        values = [str(v) for v in vals]
                except Exception:
                    values = None
                if not values:
                    # An enum whose allowed values can't be enumerated can't be
                    # offered safely — skip it entirely.
                    logger.info(f"Andor: {sdk_name} present but its values "
                                "could not be enumerated — feature hidden")
                    continue
            feats[key] = {"sdk_name": sdk_name, "kind": kind,
                          "values": values, "default": default}
        self._features = feats
        if feats:
            logger.info(f"Andor sensor features available: {sorted(feats)}")

    def _resolved_default(self, spec):
        """The concrete value a spec's default resolves to, or None to skip."""
        if spec["kind"] == "enum":
            return _match_enum_value(spec.get("values") or [], spec.get("default"))
        return bool(spec.get("default"))

    def _apply_sensor_defaults(self):
        """Apply the low-noise defaults at open (pre-acquisition, direct sets)."""
        cam = self._cam
        if cam is None:
            return
        for key, spec in self._features.items():
            value = self._resolved_default(spec)
            if value is None:
                logger.info(
                    f"Andor sensor default SKIPPED: {spec['sdk_name']} has no "
                    f"value matching {spec.get('default')} in {spec.get('values')}")
                continue
            try:
                cam.set_attribute_value(spec["sdk_name"], value)
                logger.info(f"Andor sensor default applied: "
                            f"{spec['sdk_name']} = {value!r}")
            except Exception as exc:
                logger.info(f"Andor sensor default FAILED: "
                            f"{spec['sdk_name']} = {value!r}: {exc}")

    def apply_sensor_defaults(self) -> bool:
        """Re-apply the table defaults on a LIVE camera (dialog Defaults button)."""
        ok = True
        for key, spec in self._features.items():
            value = self._resolved_default(spec)
            if value is None:
                continue
            ok = self.set_sensor_feature(key, value) and ok
        return ok

    def sensor_feature_specs(self) -> dict:
        """{key: {"kind", "values"}} for the AVAILABLE features (capabilities)."""
        return {k: {"kind": v["kind"],
                    "values": (list(v["values"]) if v.get("values") else None)}
                for k, v in self._features.items()}

    def sensor_feature_values(self, key: str) -> "list[str] | None":
        spec = self._features.get(key)
        if not spec or not spec.get("values"):
            return None
        return list(spec["values"])

    def get_sensor_feature(self, key: str):
        """Current value of a probed feature (bool or SDK enum string), or None."""
        spec = self._features.get(key)
        cam = self._cam
        if spec is None or cam is None:
            return None
        try:
            v = cam.get_attribute_value(spec["sdk_name"])
        except Exception:
            return None
        if spec["kind"] == "bool":
            return bool(v)
        return str(v)

    def set_sensor_feature(self, key: str, value) -> bool:
        """Set a probed feature; falls back to stop→apply→restart when the SDK
        refuses a live write (readout rate / gain mode are typically
        NOTWRITABLE mid-acquisition — same recovery as a resolution switch).

        Enum values are validated against the probed allowed list: a stale
        persisted string from a different camera/SDK build is REFUSED with a
        log line, never guessed at.
        """
        spec = self._features.get(key)
        cam = self._cam
        if spec is None or cam is None:
            return False
        if spec["kind"] == "enum":
            sval = str(value)
            values = spec.get("values") or []
            if sval not in values:
                logger.info(f"Andor: refusing {key} = {value!r} — not one of "
                            f"the camera's allowed values {values}")
                return False
            value = sval
        else:
            value = bool(value)

        applied = False
        try:
            cam.set_attribute_value(spec["sdk_name"], value)
            applied = True
        except Exception:
            applied = False

        if not applied and self._running:
            # Live write refused — pause acquisition, apply, restart (the
            # proven set_resolution_index recovery shape).
            self._running = False
            reader = self._reader
            self._reader = None
            if reader is not None and reader.is_alive() \
                    and reader is not threading.current_thread():
                try:
                    reader.join(timeout=1.5)
                except Exception:
                    pass
            try:
                cam.stop_acquisition()
            except Exception:
                pass
            with self._lock:
                self._frame = None
            try:
                cam.set_attribute_value(spec["sdk_name"], value)
                applied = True
            except Exception as exc:
                logger.warning(f"Andor: {spec['sdk_name']} = {value!r} failed "
                               f"even with acquisition stopped: {exc}")
            # ALWAYS restart the stream, even when the set failed — a dead
            # feed must never be the price of a refused setting.
            self._start_stream()

        if applied:
            logger.info(f"Andor sensor feature: {spec['sdk_name']} = {value!r}")
            if key in ("andor_gain_mode", "andor_readout_rate"):
                # BitDepth follows the gain mode; the clip level must track it.
                self._clip_level = self._read_clip_level()
                # The readout/gain change moves the sensor-max frame rate —
                # re-cap to the USB link's sustainable rate (v7.13.x).
                self._sync_frame_rate()
        return applied

    def get_sensor_temperature(self) -> "float | None":
        cam = self._cam
        if cam is None:
            return None
        try:
            return float(cam.get_attribute_value("SensorTemperature"))
        except Exception:
            return None

    def get_temperature_status(self) -> "str | None":
        cam = self._cam
        if cam is None:
            return None
        try:
            v = cam.get_attribute_value("TemperatureStatus")
            return str(v) if v is not None else None
        except Exception:
            return None

    def _read_bit_depth(self) -> "str | None":
        cam = self._cam
        if cam is None:
            return None
        try:
            v = cam.get_attribute_value("BitDepth")
            return str(v) if v is not None else None
        except Exception:
            return None

    def _read_clip_level(self) -> int:
        """True full-scale for the CURRENT gain mode.

        A Zyla in a 12-bit gain mode clips at ~4095, far below the uint16
        container's 65535 — judging saturation against 65535 there would never
        fire. Preference: the SDK's own BitDepth → the gain-mode string →
        full 16-bit. The observed frame max is carried in the stats dict, so a
        wrong clip level is diagnosable on the bench.
        """
        lvl = _parse_bit_depth(self._read_bit_depth())
        if lvl:
            return int(lvl)
        lvl = _parse_bit_depth(self.get_sensor_feature("andor_gain_mode"))
        if lvl:
            return int(lvl)
        return _LEVEL_MAX

    def get_raw_clip_level(self) -> int:
        return int(self._clip_level)

    # ── Raw frame statistics + averaged capture (v7.13) ───────────

    def get_raw_frame_stats(self) -> "dict | None":
        """Latest raw-frame statistics snapshot (see compute_raw_frame_stats).

        Lock-guarded copy; the histogram array is copied so callers can hold
        it across reader updates. Returns None before the first mono frame.
        """
        with self._lock:
            st = self._raw_stats
        if st is None:
            return None
        out = dict(st)
        hist = out.get("hist")
        if hist is not None:
            try:
                out["hist"] = hist.copy()
            except Exception:
                out["hist"] = list(hist)
        return out

    def capture_raw_average(self, n: int, timeout_s: float = 10.0) -> "np.ndarray | None":
        """Per-pixel mean of ``n`` consecutive NEW raw frames, as uint16.

        BLOCKING — must be called off the GUI thread (mosaic workers already
        sample frames from their own thread; same rule). Serviced by the
        reader thread so there is never a second ``wait_for_frame`` waiter.
        Returns None on: closed camera, n < 1, timeout, a concurrent request,
        a resolution change or release mid-capture.
        """
        if not _NP_AVAILABLE:
            return None
        try:
            n = int(n)
        except (TypeError, ValueError):
            return None
        if n < 1 or not self.isOpened():
            return None
        req = _RawAverageRequest(n)
        with self._lock:
            if self._avg_request is not None:
                return None
            self._avg_request = req
        try:
            if not req.done.wait(timeout=max(0.1, float(timeout_s))):
                req.fail("timeout")
                logger.info(f"Andor capture_raw_average({n}) timed out")
                return None
            if req.error:
                logger.info(f"Andor capture_raw_average({n}): {req.error}")
            return req.result()
        finally:
            with self._lock:
                if self._avg_request is req:
                    self._avg_request = None

    def _fail_pending_average(self, reason: str):
        with self._lock:
            req = self._avg_request
            self._avg_request = None
        if req is not None:
            req.fail(reason)

    # ── Display scaling (mono-16 → 8-bit display conversion) ──────
    # The Zyla has NO ISP auto-gain; what LOOKS like the camera auto-adjusting
    # is this backend's per-frame percentile auto-scale in the mono16→BGR8
    # display conversion. These controls make that behaviour an option: auto
    # ON = per-frame normalize (historical default), auto OFF = fixed
    # black/white levels in raw sensor counts (0..65535).

    def get_display_auto_scale(self) -> bool:
        with self._lock:
            return bool(self._auto_scale)

    def set_display_auto_scale(self, enabled: bool) -> bool:
        """Toggle per-frame auto-scaling of the displayed image.

        Turning auto OFF seeds the manual black/white levels from the levels
        the last auto-scaled frame used, so the image freezes at its current
        appearance rather than jumping to an arbitrary mapping.
        """
        enabled = bool(enabled)
        with self._lock:
            if not enabled and self._auto_scale and self._last_auto_levels:
                lo, hi = self._last_auto_levels
                self._scale_lo = max(0, min(_LEVEL_MAX - 1, int(round(lo))))
                self._scale_hi = max(self._scale_lo + 1,
                                     min(_LEVEL_MAX, int(round(hi))))
            self._auto_scale = enabled
        logger.info(f"Andor display auto-scale -> {enabled}"
                    + ("" if enabled else
                       f" (levels {self._scale_lo}..{self._scale_hi})"))
        return True

    def get_display_levels(self) -> tuple:
        """(black_level, white_level) in raw sensor counts."""
        with self._lock:
            return (int(self._scale_lo), int(self._scale_hi))

    def put_display_black(self, counts) -> bool:
        """Set the manual black level (counts mapping to display 0)."""
        try:
            v = int(round(float(counts)))
        except (TypeError, ValueError):
            return False
        with self._lock:
            self._scale_lo = max(0, min(_LEVEL_MAX - 1, v))
            if self._scale_hi <= self._scale_lo:
                self._scale_hi = self._scale_lo + 1
        return True

    def put_display_white(self, counts) -> bool:
        """Set the manual white level (counts mapping to display 255)."""
        try:
            v = int(round(float(counts)))
        except (TypeError, ValueError):
            return False
        with self._lock:
            self._scale_hi = max(1, min(_LEVEL_MAX, v))
            if self._scale_lo >= self._scale_hi:
                self._scale_lo = self._scale_hi - 1
        return True

    def get_display_level_range(self) -> tuple:
        """(min, max, default_white) for the level sliders."""
        return (0, _LEVEL_MAX, _LEVEL_MAX)

    # The Zyla exposes none of these ToupTek ISP controls. Report None so the
    # settings dialog hides them; software correction still works display-side.
    def get_exposure_gain(self):
        return None

    def put_exposure_gain(self, percent) -> bool:
        return False

    def get_exposure_gain_range(self):
        return None

    def get_auto_exposure(self):
        return None

    def set_auto_exposure(self, enabled: bool) -> bool:
        return False

    def get_brightness(self):
        return None

    def put_brightness(self, v) -> bool:
        return False

    def get_contrast(self):
        return None

    def put_contrast(self, v) -> bool:
        return False

    def get_gamma(self):
        return None

    def put_gamma(self, v) -> bool:
        return False

    def get_settings(self) -> dict:
        """Read every hardware setting back from the device (for the readout).

        Same shape as ToupCamBackend.get_settings(); unsupported controls are
        None so the dialog hides them.
        """
        lo, hi = self.get_display_levels()
        d = {
            "brightness": None,
            "contrast": None,
            "gamma": None,
            "exposure_us": self.get_exposure_time(),
            "exposure_gain_pct": None,
            "auto_exposure": None,
            "exposure_range_us": self.get_exposure_time_range(),
            "gain_range_pct": None,
            "andor_auto_scale": self.get_display_auto_scale(),
            "andor_scale_lo": lo,
            "andor_scale_hi": hi,
            "resolution": self.get_resolution(),
            "eSize": self.get_eSize(),
            "resolutions": self.get_resolution_list(),
            "device_id": self._device_id,
        }
        # v7.13 — sensor-quality features (None when the camera lacks one) and
        # read-only sensor info for the readout pane.
        for key in ANDOR_SENSOR_FEATURE_KEYS:
            d[key] = self.get_sensor_feature(key)
        d["andor_readout_rate_values"] = self.sensor_feature_values("andor_readout_rate")
        d["andor_gain_mode_values"] = self.sensor_feature_values("andor_gain_mode")
        d["temperature_c"] = self.get_sensor_temperature()
        d["temperature_status"] = self.get_temperature_status()
        d["bit_depth"] = self._read_bit_depth()
        d["raw_clip_level"] = self.get_raw_clip_level()
        # v7.13.x — make the frame-rate constraint visible at the bench (the
        # [camera start] readback + dialog readout).
        d["frame_rate"] = self.get_frame_rate()
        d["max_interface_transfer_rate"] = self.get_max_interface_transfer_rate()
        return d


# ════════════════════════════════════════════════════════════════════
#  Helpers
# ════════════════════════════════════════════════════════════════════

# v7.9.x: the mono→BGR8 display conversion moved to gui/widgets/mono_display.py
# so the Tucsen backend renders through the IDENTICAL math (the operator A/B's a
# Libra against this Zyla in the same MICROSCOPE role — two copies of this
# conversion would make that comparison measure our display code, not the
# sensors). Re-exported under the original names: behaviour is unchanged and
# existing `andor_backend._mono_to_bgr8` / `._auto_levels` references still
# resolve here.
from gui.widgets.mono_display import (  # noqa: E402,F401
    _auto_levels, _mono_to_bgr8, compute_raw_frame_stats, RawAverageRequest)

# Historical private name (v7.13 shipped it here first).
_RawAverageRequest = RawAverageRequest


def _andor_cameras_number(Andor) -> int:
    """Number of SDK3 cameras, tolerating pylablib API-name variation."""
    for name in ("get_cameras_number_SDK3", "get_cameras_number"):
        fn = getattr(Andor, name, None)
        if callable(fn):
            try:
                return int(fn())
            except Exception:
                continue
    return 0


def _read_identity(Andor, idx: int) -> tuple[str, str]:
    """Open camera ``idx`` briefly to read (serial, model); ('','') on failure."""
    cam = None
    try:
        cam = Andor.AndorSDK3Camera(idx=idx)
        info = cam.get_device_info()
        serial = str(getattr(info, "serial_number", "") or "")
        model = str(getattr(info, "camera_model", "")
                    or getattr(info, "camera_name", "") or "")
        return serial, model
    except Exception as exc:
        logger.debug(f"Andor identity read failed for idx={idx}: {exc}")
        return "", ""
    finally:
        if cam is not None:
            try:
                cam.close()
            except Exception:
                pass


def _index_for_id(Andor, device_id: str) -> int:
    """Resolve a serial-string device_id back to a pylablib camera index."""
    if device_id:
        # Direct integer index (e.g. "andor#0" or a bare number).
        digits = "".join(ch for ch in str(device_id) if ch.isdigit())
        try:
            count = _andor_cameras_number(Andor)
        except Exception:
            count = 0
        for idx in range(count):
            serial, _model = _read_identity(Andor, idx)
            if serial and serial == device_id:
                return idx
        if str(device_id).startswith("andor#") and digits:
            return int(digits)
    return 0
