"""
camera_manager.py — Centralized camera ownership and management.

v7.3.3: Provides a single source of truth for camera widgets so that
any page in the application can display live camera feeds without
duplicating detection, creation, or control logic.

Usage::

    # In app.py (create once)
    manager = CameraManager()  # slot count defaults to MAX_LIVE_CAMERAS

    # In any page
    manager.detect_cameras()
    cameras = manager.cameras          # list[CameraWidget]
    sources = manager.available_sources  # list[(text, data)]
    manager.set_source(0, source_data)  # assign source to camera 0
    manager.start(0)                    # start camera 0
    manager.stop(0)                     # stop camera 0

CameraWidgets are created with show_controls=False so that each page
can build its own control UI in its context panel.  The widgets can be
freely reparented into any page's layout — the manager keeps the
authoritative reference list.
"""

from __future__ import annotations

import logging
import math
import threading
from typing import Any, Optional

from PySide6.QtCore import QObject, Signal

logger = logging.getLogger(__name__)

# Import camera support (graceful fallback)
try:
    from gui.widgets.camera_widget import (
        CameraWidget, CAMERA_AVAILABLE, detect_cameras,
    )
except ImportError:
    CameraWidget = None
    CAMERA_AVAILABLE = False

    def detect_cameras(max_index=8):
        return []

try:
    from gui.widgets.camera_widget import detect_toupcam_cameras
except ImportError:
    def detect_toupcam_cameras():
        return []

# v7.5.x: default slot count follows the app-wide constant (3 → 4 for the
# MONITOR overview camera) instead of a locally-hardcoded 3.
try:
    from SupportClasses.HardwareConfig import MAX_LIVE_CAMERAS
except ImportError:
    MAX_LIVE_CAMERAS = 4


class CameraManager(QObject):
    """Central owner of all CameraWidget instances.

    Signals:
        cameras_detected: Emitted after detection completes with the
            number of available sources found.
        camera_started(int): Camera index started.
        camera_stopped(int): Camera index stopped.
    """

    cameras_detected = Signal(int)   # num_sources
    camera_started = Signal(int)     # camera index
    camera_stopped = Signal(int)     # camera index

    def __init__(self, max_cameras: int = MAX_LIVE_CAMERAS, parent=None):
        super().__init__(parent)
        self._max_cameras = max_cameras
        self._cameras: list[CameraWidget] = []
        self._available_sources: list[tuple[str, object]] = []
        # v7.5.x: cached DirectShow enumeration (name + device path per
        # index), refreshed on detect_cameras(). Used to resolve a slot's
        # stable per-device identity for the calibration store.
        self._ds_cameras: list[dict] = []

        # Per-camera calibration data
        self._um_per_px: list[float] = [1.67] * max_cameras
        self._magnification: list[float] = [2.0] * max_cameras
        # v7.5.x: Track whether each slot's µm/px was *explicitly* set by a
        # calibration vs. still the 1.67 seed default. Consumers that must
        # refuse to run on an uncalibrated camera (needle-zero / plate edge
        # fit) check this instead of `get_um_per_px(...) > 0` — the seed
        # default is > 0 and would otherwise mask the uncalibrated case.
        self._um_per_px_set: list[bool] = [False] * max_cameras
        # v7.5.x: the frame resolution (w, h) each µm/px was MEASURED at,
        # paired with the stored value. µm/px scales inversely with frame
        # width (the same optical FOV sampled across more pixels means each
        # pixel spans fewer microns), so the live transform rescales the
        # stored value from this resolution to the actual live frame size —
        # see ``effective_um_per_px`` / ``pixel_to_stage_offset``. ``None`` =
        # unknown (no rescale; the stored value is used as-is).
        self._um_per_px_res: list[Optional[tuple[int, int]]] = (
            [None] * max_cameras)
        # v7.5.x: per-slot in-plane rotation (deg) — the camera's DISPLAY
        # orientation correction. For the needle side cams this is the small
        # sensor roll (deviation from parallel), NOT the ±45° mount direction.
        # None = not measured.
        self._rotation_deg: list[Optional[float]] = [None] * max_cameras
        # v7.5.x (rotated rig): per-slot column→stage MOUNT direction (deg CCW
        # from stage +X) — the needle cameras sit symmetric about +X at ±45°.
        # Consumed ONLY by the two-camera needle aligner; never by the display.
        # None = not measured (the aligner refuses rather than guess).
        self._column_dir_deg: list[Optional[float]] = [None] * max_cameras
        # v7.5.x: per-slot mirrored-view flag (horizontal flip). A mirror
        # reverses image handedness, which rotation alone cannot express;
        # flip-x + arbitrary rotation together span every camera orientation.
        # Default False = identity click→stage mapping (legacy behaviour).
        self._mirrored: list[bool] = [False] * max_cameras
        # v7.5.x: per-slot VERTICAL flip (flip Y). Independent of the horizontal
        # flip (``_mirrored`` = flip X); together with rotation they span every
        # orientation as R(θ)·diag(sx, sy). Applied to the live display, the
        # mosaic tiles, and the click→stage mapping — ONE unified system.
        self._flip_y: list[bool] = [False] * max_cameras
        # v7.16: per-slot LIVE-VIEW orientation — ``(flip_x, flip_y, rot)`` or
        # None. Deliberately NOT the same numbers as the three above, which are
        # the MEASURED camera→stage matrix that orients mosaic tiles and maps
        # every click. This one is only ever read by the live feeds, so the
        # operator can set up the view they want to look at without moving any
        # geometry — and, crucially, without a later measurement resetting it.
        # None = no preference ⇒ fall back to the measured orientation.
        self._view_orient: list[Optional[tuple[bool, bool, float]]] = (
            [None] * max_cameras)
        # v7.16: per-slot centred crop. Unlike the flips above (declarations
        # each consumer applies), this one really removes pixels, at the frame
        # source, so every surface inherits it. The widget owns it once running;
        # this list is the cache for slots with no widget yet.
        self._crop: list[Any] = [None] * max_cameras
        # v7.16: last completed Tucsen enumeration. The probe is bounded (see
        # _probe_tucam_bounded) and a late answer lands here rather than being
        # discarded, so a slow SDK costs one detection round, not the camera.
        self._tucam_cache: Optional[list] = None

        # Create camera widgets (headless — no built-in controls)
        if CAMERA_AVAILABLE and CameraWidget is not None:
            for i in range(max_cameras):
                cam = CameraWidget(
                    camera_label=f"Camera {i + 1}",
                    compact=True,
                    show_controls=False,
                    parent=None,  # no parent — pages will reparent
                )
                self._cameras.append(cam)
            logger.info(f"CameraManager created {max_cameras} camera widgets")
        else:
            logger.warning("CameraManager: camera support not available")

    # ── Properties ────────────────────────────────────────────────

    @property
    def cameras(self) -> list:
        """All CameraWidget instances (may be empty if no camera support)."""
        return list(self._cameras)

    @property
    def max_cameras(self) -> int:
        return self._max_cameras

    @property
    def available_sources(self) -> list[tuple[str, object]]:
        """List of (display_text, source_data) from last detection."""
        return list(self._available_sources)

    @property
    def num_sources(self) -> int:
        return len(self._available_sources)

    @property
    def is_available(self) -> bool:
        return CAMERA_AVAILABLE and len(self._cameras) > 0

    # ── Detection ─────────────────────────────────────────────────

    def detect_cameras(self, opencv_indices=None):
        """Detect all available camera sources (OpenCV + ToupCam + Sim).

        Refreshes internal source list and updates each CameraWidget's
        hidden combo. Emits cameras_detected(num_sources) when done.

        v7.5.x: builds the source inventory ONCE and shares it with every
        widget (previously each widget re-ran the slow OpenCV device probe).
        ``opencv_indices`` lets the caller supply a pre-probed index list (from
        ``detect_cameras_async``, run off the GUI thread at startup) so the
        blocking device opens don't freeze the UI. When None, the OpenCV probe
        runs here synchronously (the manual "Detect Cameras" button). The
        DirectShow/COM + ToupCam enumerations always run here (GUI thread) —
        they're fast and COM is happiest on the main thread.
        """
        if not self._cameras:
            self.cameras_detected.emit(0)
            return

        from gui.widgets.camera_widget import (
            detect_cameras as _probe_opencv, detect_toupcam_cameras,
        )
        try:
            from gui.widgets.camera_widget import detect_andor_cameras
        except ImportError:
            def detect_andor_cameras():
                return []
        try:
            from gui.widgets.camera_widget import detect_tucam_cameras
        except ImportError:
            def detect_tucam_cameras():
                return []
        # DirectShow identity map (Windows; [] elsewhere) — also the labels.
        try:
            from gui.widgets.camera_identity import enumerate_directshow_cameras
            ds_cams = enumerate_directshow_cameras()
        except Exception as exc:
            logger.debug(f"DirectShow identity enumeration skipped: {exc}")
            ds_cams = []
        if opencv_indices is None:
            opencv_indices = _probe_opencv()
        try:
            toupcam = detect_toupcam_cameras()
        except Exception as exc:
            logger.debug(f"ToupCam enumeration skipped: {exc}")
            toupcam = []
        try:
            andor = detect_andor_cameras()
        except Exception as exc:
            logger.debug(f"Andor enumeration skipped: {exc}")
            andor = []
        tucam = self._probe_tucam_bounded(detect_tucam_cameras)
        probe = {"opencv": opencv_indices, "dshow": ds_cams,
                 "toupcam": toupcam, "andor": andor, "tucam": tucam}

        # Use the first camera widget's refresh to populate from the inventory
        first = self._cameras[0]
        if hasattr(first, 'refresh_cameras'):
            first.refresh_cameras(probe)

        # Read detected sources from its combo
        self._available_sources = []
        if hasattr(first, 'camera_combo'):
            combo = first.camera_combo
            for i in range(combo.count()):
                data = combo.itemData(i)
                text = combo.itemText(i)
                if data and data != -1:
                    self._available_sources.append((text, data))

        # Refresh all other camera widget combos from the SAME inventory.
        for cam in self._cameras[1:]:
            if hasattr(cam, 'refresh_cameras'):
                cam.refresh_cameras(probe)

        self._ds_cameras = ds_cams

        n = len(self._available_sources)
        logger.info(f"CameraManager detected {n} sources")
        self.cameras_detected.emit(n)

    #: How long camera detection may wait for the Tucsen SDK before giving up
    #: on it for this round.
    #:
    #: Chosen from measurement, not taste. Seven ``TUCAM_Api_Init`` calls in
    #: ME3B V1's own log, timed from the "library loaded" line beside them:
    #:
    #:   healthy   2.07 / 2.11 / 2.07 s  → "1 camera(s)"
    #:   wedged   29.13 / 29.13 s        → "0 camera(s)"
    #:
    #: So a healthy probe finishes in ~2 s and a wedged one takes ~29 s with the
    #: GUI thread inside it. 4 s sits between the two with ~2x headroom over
    #: healthy, and the cache below means overshooting it costs one detection
    #: round rather than the camera.
    TUCAM_PROBE_TIMEOUT_S = 4.0

    def _probe_tucam_bounded(self, probe) -> list:
        """Enumerate Tucsen cameras without letting a wedged one freeze the UI.

        ``TUCAM_Api_Init`` is a blocking C call with no cancellation: when the
        device is present but cannot be claimed it scans for ~30 seconds before
        answering "0 cameras". Detection runs on the GUI thread (DirectShow/COM
        wants the main thread, and the rest of the enumeration is fast), so that
        is a 30-second freeze of the whole application — on startup AND on every
        press of Detect. It is also the freeze an operator sees as "python
        freezes", because nothing in the UI says a camera probe is in progress.

        So the call is made on a daemon thread and waited on for a bounded time.
        A late answer is NOT thrown away — it lands in the cache and the next
        detection uses it — so a merely slow SDK costs one round, not the
        camera. A wedged one costs 4 seconds instead of 30.
        """
        result: dict = {}

        def _worker():
            try:
                result["cams"] = probe()
            except Exception as exc:
                logger.debug(f"TUCam enumeration failed: {exc}")
                result["cams"] = []
            # Cache whenever it finishes, however late — the next detection is
            # then instant and correct.
            self._tucam_cache = result.get("cams") or []

        t = threading.Thread(target=_worker, name="TucamProbe", daemon=True)
        t.start()
        t.join(self.TUCAM_PROBE_TIMEOUT_S)
        if t.is_alive():
            cached = getattr(self, "_tucam_cache", None)
            logger.warning(
                f"Tucsen enumeration did not answer in "
                f"{self.TUCAM_PROBE_TIMEOUT_S:.0f}s — the camera is present to "
                f"Windows but the SDK cannot claim it (TUCAM_Api_Init blocks "
                f"~30s in that state). Continuing without it; unplug and "
                f"replug the camera to re-enumerate it."
                + ("" if cached is None else " Using the previous result."))
            return list(cached or [])
        return list(result.get("cams") or [])

    def detect_cameras_async(self, on_complete):
        """Probe OpenCV camera indices (the slow device opens) on a BACKGROUND
        thread, then call ``on_complete(indices)`` from that thread.

        The caller MUST marshal back to the GUI thread before calling
        ``detect_cameras(indices)`` (which touches Qt widgets + COM). Passing
        the result through a queued Qt signal does this. ``indices`` is None if
        the probe raised (caller can fall back to a synchronous detect).
        """
        import threading
        from gui.widgets.camera_widget import detect_cameras as _probe_opencv

        def _worker():
            indices = None
            try:
                indices = _probe_opencv()
            except Exception as exc:
                logger.debug(f"background OpenCV probe failed: {exc}")
                indices = None
            try:
                on_complete(indices)
            except Exception as exc:
                logger.debug(f"detect_cameras_async on_complete failed: {exc}")

        threading.Thread(
            target=_worker, daemon=True, name="CameraDetectBG").start()

    def camera_identity(self, cam_idx: int) -> Optional[tuple[str, str]]:
        """Stable ``(identity_key, friendly_name)`` for a slot's source.

        Resolves the slot's currently-assigned source against the cached
        DirectShow enumeration. Returns None if the slot has no source.
        Used to key the per-device µm/px calibration store.
        """
        source = self.get_source(cam_idx)
        if source is None:
            return None
        try:
            from gui.widgets.camera_identity import identity_for_source
            return identity_for_source(source, self._ds_cameras)
        except Exception as exc:
            logger.debug(f"camera_identity({cam_idx}) failed: {exc}")
            return None

    # ── Source assignment ─────────────────────────────────────────

    def get_source(self, cam_idx: int) -> Optional[tuple]:
        """Get the currently assigned source for a camera, or None."""
        if cam_idx < 0 or cam_idx >= len(self._cameras):
            return None
        cam = self._cameras[cam_idx]
        if hasattr(cam, 'camera_combo'):
            data = cam.camera_combo.currentData()
            if data and data != -1:
                return data
        return None

    def set_source(self, cam_idx: int, source_data) -> bool:
        """Assign a source to a camera widget by matching combo data.

        Returns True if the source was found and set.
        """
        if cam_idx < 0 or cam_idx >= len(self._cameras):
            return False
        cam = self._cameras[cam_idx]
        if not hasattr(cam, 'camera_combo'):
            return False
        for i in range(cam.camera_combo.count()):
            if cam.camera_combo.itemData(i) == source_data:
                cam.camera_combo.setCurrentIndex(i)
                return True
        return False

    # ── Start / Stop ──────────────────────────────────────────────

    def start(self, cam_idx: int):
        """Start camera at given index (synchronous — blocks until the device is
        open). Callers that need a frame immediately after (e.g. the mosaic
        scan's ``get_current_frame`` / ``capture_fresh_frame``) rely on this.
        For the startup auto-start, use ``start_async`` to avoid freezing the UI.
        """
        if cam_idx < 0 or cam_idx >= len(self._cameras):
            return
        cam = self._cameras[cam_idx]
        if getattr(cam, '_running', False) or getattr(cam, '_opening', False):
            return
        cam.start()
        if getattr(cam, '_running', False):
            # v7.5.x: adopt this physical camera's persisted calibration as soon
            # as it is running, so µm/px + orientation are available to EVERY
            # consumer without first visiting the Hardware Setup page.
            self.restore_calibration_from_store(cam_idx)
            self.camera_started.emit(cam_idx)

    def start_async(self, cam_idx: int):
        """Start camera at given index WITHOUT blocking the GUI thread.

        v7.5.x: the multi-second device open runs on a worker thread (see
        ``CameraWidget.start_async``); ``camera_started`` is emitted from the
        completion callback once the camera is actually running, so consumers
        (preview state, hw-control restore) see the same signal as the sync path.
        Used by the startup auto-start of saved cameras — the "camera boot-up
        freezes the UI" bug. Falls back to the synchronous ``start`` on an older
        widget without ``start_async``.
        """
        if cam_idx < 0 or cam_idx >= len(self._cameras):
            return
        cam = self._cameras[cam_idx]
        if getattr(cam, '_running', False) or getattr(cam, '_opening', False):
            return
        if hasattr(cam, 'start_async'):
            cam.start_async(
                on_done=lambda ok, i=cam_idx: (
                    (self.restore_calibration_from_store(i),
                     self.camera_started.emit(i)) if ok else None))
        else:
            cam.start()
            if getattr(cam, '_running', False):
                self.restore_calibration_from_store(cam_idx)
                self.camera_started.emit(cam_idx)

    def stop(self, cam_idx: int):
        """Stop camera at given index."""
        if cam_idx < 0 or cam_idx >= len(self._cameras):
            return
        cam = self._cameras[cam_idx]
        if getattr(cam, '_running', False):
            cam.stop()
            self.camera_stopped.emit(cam_idx)

    def toggle(self, cam_idx: int):
        """Toggle camera on/off."""
        if cam_idx < 0 or cam_idx >= len(self._cameras):
            return
        cam = self._cameras[cam_idx]
        if getattr(cam, '_running', False):
            self.stop(cam_idx)
        else:
            self.start(cam_idx)

    def is_running(self, cam_idx: int) -> bool:
        """Check if a camera is currently running."""
        if cam_idx < 0 or cam_idx >= len(self._cameras):
            return False
        return getattr(self._cameras[cam_idx], '_running', False)

    def stop_all(self):
        """Stop all cameras."""
        for i in range(len(self._cameras)):
            self.stop(i)

    # ── Frame access ──────────────────────────────────────────────

    def get_current_frame(self, cam_idx: int):
        """Get latest BGR numpy frame from a camera, or None."""
        if cam_idx < 0 or cam_idx >= len(self._cameras):
            return None
        cam = self._cameras[cam_idx]
        if not getattr(cam, '_running', False):
            return None
        if hasattr(cam, 'get_current_frame'):
            return cam.get_current_frame()
        return None

    # ── Per-camera calibration ────────────────────────────────────

    def get_um_per_px(self, cam_idx: int) -> float:
        """Get microns per pixel for a camera."""
        if 0 <= cam_idx < self._max_cameras:
            return self._um_per_px[cam_idx]
        return 1.67

    def set_um_per_px(self, cam_idx: int, value: float,
                      resolution: Optional[tuple[int, int]] = None):
        """Set microns per pixel for a camera, measured at ``resolution``.

        Marks the slot as explicitly calibrated (see ``is_um_per_px_calibrated``).

        ``resolution`` is the (w, h) frame size the value was measured at; it
        is stored alongside the value so ``pixel_to_stage_offset`` /
        ``effective_um_per_px`` can rescale to whatever resolution the live
        feed is actually running at. Pass ``None`` when the resolution is
        unknown — no rescale is applied (the value is treated as valid at the
        live resolution). The resolution is always paired with the value just
        set (``None`` clears any previously-stored resolution).
        """
        if 0 <= cam_idx < self._max_cameras:
            self._um_per_px[cam_idx] = value
            self._um_per_px_set[cam_idx] = True
            self._um_per_px_res[cam_idx] = (
                (int(resolution[0]), int(resolution[1]))
                if resolution and len(resolution) >= 2
                and resolution[0] and resolution[1]
                else None
            )

    def effective_um_per_px(self, cam_idx: int, live_width: float) -> float:
        """µm/px rescaled from its calibration resolution to ``live_width``.

        µm/px is measured at a specific frame resolution; the same optical FOV
        sampled across more pixels means each pixel spans fewer microns, so
        µm/px ∝ 1/width. Returns ``stored_um_per_px × calib_width /
        live_width`` when the calibration resolution is known, else the stored
        value unchanged. This keeps the pixel→stage transform correct even
        when the live feed runs at a different resolution than the calibration.

        v7.16: the denominator is the CAPTURE width, not the delivered one. A
        centred crop shrinks the frame without changing what a pixel spans, so
        dividing by a cropped width would rescale µm/px by the crop fraction.
        With no crop the two are identical and this is byte-identical to before
        — see :meth:`capture_width`.
        """
        base = self.get_um_per_px(cam_idx)
        cal_res = None
        if 0 <= cam_idx < self._max_cameras:
            cal_res = self._um_per_px_res[cam_idx]
        if cal_res and cal_res[0] and live_width and live_width > 0:
            denom = self.capture_width(cam_idx, live_width)
            if denom > 0:
                return base * float(cal_res[0]) / denom
        return base

    def get_um_per_px_resolution(self, cam_idx: int) -> Optional[tuple[int, int]]:
        """Resolution (w, h) the stored µm/px was measured at, or None."""
        if 0 <= cam_idx < self._max_cameras:
            return self._um_per_px_res[cam_idx]
        return None

    def is_um_per_px_calibrated(self, cam_idx: int) -> bool:
        """True once ``set_um_per_px`` has supplied a real value for the slot.

        Distinguishes a calibrated camera from one still carrying the 1.67
        seed default, so callers can refuse to run on uncalibrated cameras.
        """
        if 0 <= cam_idx < self._max_cameras:
            return self._um_per_px_set[cam_idx]
        return False

    def get_rotation_deg(self, cam_idx: int) -> Optional[float]:
        """In-plane rotation (deg) measured for a slot, or None if unmeasured.

        ``getattr``-guarded so lightweight ``__new__`` test doubles that omit
        ``_rotation_deg`` never raise (mirrors ``get_mirrored`` / ``_widget``)."""
        rot = getattr(self, "_rotation_deg", None)
        if rot is not None and 0 <= cam_idx < len(rot):
            return rot[cam_idx]
        return None

    def set_rotation_deg(self, cam_idx: int, value: Optional[float]):
        """Set (or clear, with None) the in-plane rotation for a slot."""
        if 0 <= cam_idx < self._max_cameras:
            self._rotation_deg[cam_idx] = (
                None if value is None else float(value))

    def get_column_dir_deg(self, cam_idx: int) -> Optional[float]:
        """Column→stage mount direction (deg CCW from stage +X) for a slot,
        or None if unmeasured.

        v7.5.x (rotated rig): the needle side cameras' mount direction (±45°
        about +X, measured by the stage-motion µm/px calibration) — consumed
        ONLY by the two-camera needle aligner, never by the display (that is
        ``get_rotation_deg``, the sensor roll). ``getattr``-guarded for
        lightweight ``__new__`` test doubles."""
        cd = getattr(self, "_column_dir_deg", None)
        if cd is not None and 0 <= cam_idx < len(cd):
            return cd[cam_idx]
        return None

    def set_column_dir_deg(self, cam_idx: int, value: Optional[float]):
        """Set (or clear, with None) the column→stage mount direction."""
        cd = getattr(self, "_column_dir_deg", None)
        if cd is not None and 0 <= cam_idx < len(cd):
            cd[cam_idx] = None if value is None else float(value)

    def get_mirrored(self, cam_idx: int) -> bool:
        """Whether the slot's view is mirrored (horizontal flip). Default False.

        v7.5.x: delegates to the owning ``CameraWidget`` (which persists the
        flag across stop/start and applies the flip at the frame source),
        falling back to the local ``_mirrored`` cache for slots without a
        widget / lightweight test doubles."""
        cam = self._widget(cam_idx)
        if cam is not None:
            return bool(getattr(cam, "mirrored", False))
        mir = getattr(self, "_mirrored", None)
        if mir is not None and 0 <= cam_idx < len(mir):
            return bool(mir[cam_idx])
        return False

    def set_mirrored(self, cam_idx: int, value: bool):
        """Set the mirrored-view flag for a slot.

        Delegates to the ``CameraWidget`` (which holds the flag; the frame is
        NOT flipped there — orientation is applied per consumer), and keeps the
        local cache in sync for no-widget slots / test doubles."""
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "set_mirrored"):
            cam.set_mirrored(bool(value))
        mir = getattr(self, "_mirrored", None)
        if mir is not None and 0 <= cam_idx < len(mir):
            mir[cam_idx] = bool(value)

    def get_flip_y(self, cam_idx: int) -> bool:
        """Whether the slot's view is flipped vertically (flip Y). Default False.
        ``getattr``-guarded for lightweight ``__new__`` test doubles."""
        fy = getattr(self, "_flip_y", None)
        if fy is not None and 0 <= cam_idx < len(fy):
            return bool(fy[cam_idx])
        return False

    def set_flip_y(self, cam_idx: int, value: bool):
        """Set the vertical-flip (flip Y) flag for a slot."""
        fy = getattr(self, "_flip_y", None)
        if fy is not None and 0 <= cam_idx < len(fy):
            fy[cam_idx] = bool(value)

    # ── v7.16: centred crop ───────────────────────────────────────

    def get_crop(self, cam_idx: int):
        """The slot's centred crop (``CameraCrop``); a neutral one by default.

        Delegates to the owning ``CameraWidget`` — which actually applies it to
        the frames — falling back to the local cache for slots without a widget
        and for lightweight ``__new__`` test doubles."""
        from SupportClasses.CameraCrop import CameraCrop
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "crop"):
            try:
                return cam.crop()
            except Exception:
                pass
        cache = getattr(self, "_crop", None)
        if cache is not None and 0 <= cam_idx < len(cache):
            return cache[cam_idx] or CameraCrop()
        return CameraCrop()

    def set_crop(self, cam_idx: int, crop) -> None:
        """Set the slot's centred crop. Applied at the frame source, so it
        reaches the live view, the mosaic, captures and recordings alike."""
        from SupportClasses.CameraCrop import CameraCrop
        crop = crop if isinstance(crop, CameraCrop) else CameraCrop.from_dict(crop)
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "set_crop"):
            try:
                cam.set_crop(crop)
            except Exception as exc:
                logger.debug(f"set_crop slot {cam_idx}: {exc}")
        cache = getattr(self, "_crop", None)
        if cache is not None and 0 <= cam_idx < len(cache):
            cache[cam_idx] = crop

    def capture_resolution(self, cam_idx: int):
        """The slot's PRE-crop frame size ``(w, h)``, or None if unknown.

        The delivered frame may be a crop of this. µm/px calibrations are
        stamped with, and rescaled against, the CAPTURE size — see
        :meth:`capture_width`.
        """
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "capture_size"):
            try:
                got = cam.capture_size()
                if got and got[0] and got[1]:
                    return (int(got[0]), int(got[1]))
            except Exception:
                pass
        return None

    def crop_center_offset_px(self, cam_idx: int) -> tuple[float, float]:
        """Raw-pixel displacement of the delivered frame's centre from the
        sensor's centre — ``(0, 0)`` unless the crop has been moved off-centre.

        The pixel↔stage conversions add this back, so an offset crop maps to
        exactly the same stage coordinates a full frame would. Without it,
        moving the crop would move the pixel that means "the stage is here" and
        drag every click, overlay and mosaic tile with it — up to ~1 mm at a
        small crop on this rig, i.e. a silently relocated plate calibration.
        """
        try:
            crop = self.get_crop(cam_idx)
            if crop is None or not crop.enabled:
                return (0.0, 0.0)
            cap = self.capture_resolution(cam_idx)
            if not cap or not cap[0] or not cap[1]:
                return (0.0, 0.0)
            return crop.center_offset_px(cap[0], cap[1])
        except Exception as exc:
            logger.debug(f"crop_center_offset_px slot {cam_idx}: {exc}")
            return (0.0, 0.0)

    def capture_width(self, cam_idx: int, fallback: float = 0.0) -> float:
        """Width in CAPTURE pixels, falling back to a delivered width.

        THE reason this exists: ``µm/px ∝ 1/width`` holds when the pixel count
        changes because the sensor is sampled differently (binning, a
        resolution switch) — the same optical field over more pixels. A CROP
        changes the pixel count WITHOUT changing what a pixel spans, so feeding
        a cropped width into that ratio rescales µm/px by the crop fraction:
        27 % on this rig's Tucsen at a square crop, in the direction that makes
        the mosaic think its field is wider than it is and leave gaps.

        Every rescale site therefore resolves its denominator through here, and
        with no crop configured this returns ``fallback`` unchanged.
        """
        got = self.capture_resolution(cam_idx)
        if got and got[0] > 0:
            return float(got[0])
        return float(fallback or 0.0)

    def view_orientation(self, cam_idx: int) -> tuple[bool, float]:
        """v7.5.x: ``(mirrored, rotation_deg)`` for a slot — the camera's
        calibrated orientation, for correcting the DISPLAY (CameraFeedView) and
        the mosaic. ``rotation_deg`` defaults to 0.0 when unmeasured.

        NOTE: this 2-tuple is kept for back-compat; ``flip Y`` is a separate
        field (``get_flip_y``) so callers must fetch it too. Prefer
        ``full_orientation`` for all three at once."""
        rot = self.get_rotation_deg(cam_idx)
        return (bool(self.get_mirrored(cam_idx)),
                float(rot) if rot is not None else 0.0)

    def full_orientation(self, cam_idx: int) -> tuple[bool, bool, float]:
        """v7.5.x: ``(flip_x, flip_y, rotation_deg)`` — the camera's MEASURED
        orientation vs the stage (flip_x == mirrored).

        This is GEOMETRY. ``MosaicBuilder._orient_tile`` orients every tile by
        it and ``pixel_to_stage_offset`` maps every click through it, so it must
        only ever come from a measurement.

        ⚠ v7.16: this is no longer what the live view shows — use
        :meth:`display_orientation` for that. They were one value until the
        objective calibration was found resetting the operator's view each time
        it ran.
        """
        rot = self.get_rotation_deg(cam_idx)
        return (bool(self.get_mirrored(cam_idx)),
                bool(self.get_flip_y(cam_idx)),
                float(rot) if rot is not None else 0.0)

    # ── v7.16: live-view orientation (display only) ────────────────

    def display_orientation(self, cam_idx: int) -> tuple[bool, bool, float]:
        """``(flip_x, flip_y, rotation_deg)`` to apply to the LIVE VIEW.

        Same field order as :meth:`full_orientation` on purpose — flips first —
        so a caller that swaps between them cannot silently trade a mirror for
        an angle.

        Falls back to the measured orientation when the operator has expressed
        no view preference, which is byte-identical to the pre-v7.16 behaviour.
        """
        vo = getattr(self, "_view_orient", None)
        if vo is not None and 0 <= cam_idx < len(vo) and vo[cam_idx] is not None:
            flip_x, flip_y, rot = vo[cam_idx]
            return (bool(flip_x), bool(flip_y), float(rot))
        return self.full_orientation(cam_idx)

    def has_display_orientation(self, cam_idx: int) -> bool:
        """True when this slot has an explicit live-view preference (so the
        measured orientation is NOT what the feed is showing)."""
        vo = getattr(self, "_view_orient", None)
        return bool(vo is not None and 0 <= cam_idx < len(vo)
                    and vo[cam_idx] is not None)

    def set_display_orientation(self, cam_idx: int, flip_x: bool, flip_y: bool,
                                rotation_deg: float) -> None:
        """Set the slot's live-view orientation. Touches NO geometry — the
        mosaic and the click→stage mapping are unaffected."""
        vo = getattr(self, "_view_orient", None)
        if vo is not None and 0 <= cam_idx < len(vo):
            vo[cam_idx] = (bool(flip_x), bool(flip_y), float(rotation_deg or 0.0))

    def clear_display_orientation(self, cam_idx: int) -> None:
        """Drop the slot's view preference so the feed follows the measurement."""
        vo = getattr(self, "_view_orient", None)
        if vo is not None and 0 <= cam_idx < len(vo):
            vo[cam_idx] = None

    def restore_calibration_from_store(self, cam_idx: int) -> bool:
        """Push the persisted per-identity calibration into this slot.

        v7.5.x. Previously the ONLY thing that did this was
        ``HardwareSetupPage._restore_calibration_for_slot`` — which runs on the
        Hardware Setup page. Until the operator visited that page,
        ``get_rotation_deg`` / ``get_flip_y`` returned ``None`` / ``False``, so a
        mosaic built from the Calibration page or a workflow was placed with NO
        calibrated orientation. (The full-plate scan papered over this by reading
        the store directly; the calibration dialog and the fluorescence scan did
        not, which is a large part of why the three mosaics disagreed.)

        Living on the manager means every consumer sees calibrated values
        regardless of navigation order. Safe to call repeatedly; returns True if
        an entry was found. Fields absent from the store are left untouched, so
        this never downgrades a freshly-measured live value to a stale one.
        """
        if not (0 <= cam_idx < self._max_cameras):
            return False
        try:
            identity = self.camera_identity(cam_idx)
        except Exception:
            identity = None
        if not identity or not identity[0]:
            return False
        try:
            from SupportClasses.CameraCalibrationStore import get_store
            store = get_store()
            entry = store.get_calibration(identity[0])
        except Exception as exc:
            logger.debug(f"restore_calibration_from_store({cam_idx}): {exc}")
            return False
        if not entry:
            return False
        try:
            if entry.get("um_per_px") is not None:
                self.set_um_per_px(
                    cam_idx, float(entry["um_per_px"]),
                    resolution=store.get_um_per_px_resolution(identity[0]))
            if entry.get("rotation_deg") is not None:
                self.set_rotation_deg(cam_idx, float(entry["rotation_deg"]))
            if entry.get("column_dir_deg") is not None:
                self.set_column_dir_deg(
                    cam_idx, float(entry["column_dir_deg"]))
            if "mirrored" in entry:
                self.set_mirrored(cam_idx, bool(entry["mirrored"]))
            if "flip_y" in entry:
                self.set_flip_y(cam_idx, bool(entry["flip_y"]))
            # v7.16: the live-view preference, restored ALONGSIDE (never
            # instead of) the measured orientation.
            #
            # ⚠ This CLEARS as well as sets. A slot's CameraWidget outlives the
            # source assigned to it, so reassigning a slot from a camera with a
            # view preference to one without must hand the view back to the
            # measurement — otherwise the new camera inherits the old one's
            # rotation. (Same hazard as the v7.16 crop restore.)
            try:
                vo = store.get_view_orientation(identity[0])
            except Exception:
                vo = None
            if vo is None:
                self.clear_display_orientation(cam_idx)
            else:
                self.set_display_orientation(
                    cam_idx, bool(vo.get("flip_x", False)),
                    bool(vo.get("flip_y", False)),
                    float(vo.get("rotation_deg", 0.0) or 0.0))
        except Exception as exc:
            logger.debug(f"restore_calibration_from_store({cam_idx}): {exc}")
            return False
        return True

    def get_magnification(self, cam_idx: int) -> float:
        """Get objective magnification for a camera."""
        if 0 <= cam_idx < self._max_cameras:
            return self._magnification[cam_idx]
        return 2.0

    def set_magnification(self, cam_idx: int, value: float):
        """Set objective magnification for a camera."""
        if 0 <= cam_idx < self._max_cameras:
            self._magnification[cam_idx] = value

    # ── Per-camera image correction (display-only) ────────────────
    # Brightness / contrast / gamma are software corrections applied to the
    # *displayed* frame (and the frame_captured QImage that drives every
    # CameraFeedView) — NOT the raw frame used for detection/calibration. The
    # owning CameraWidget holds the live values (it persists across stop/start
    # /source-change), so these just delegate to it.

    def _widget(self, cam_idx: int):
        cams = getattr(self, "_cameras", None)
        if cams and 0 <= cam_idx < len(cams):
            return cams[cam_idx]
        return None

    def get_brightness(self, cam_idx: int) -> int:
        cam = self._widget(cam_idx)
        return int(getattr(cam, "brightness", 0)) if cam is not None else 0

    def set_brightness(self, cam_idx: int, value: int):
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "set_brightness"):
            cam.set_brightness(value)

    def get_contrast(self, cam_idx: int) -> float:
        cam = self._widget(cam_idx)
        return float(getattr(cam, "contrast", 1.0)) if cam is not None else 1.0

    def set_contrast(self, cam_idx: int, value: float):
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "set_contrast"):
            cam.set_contrast(value)

    def get_gamma(self, cam_idx: int) -> float:
        cam = self._widget(cam_idx)
        return float(getattr(cam, "gamma", 1.0)) if cam is not None else 1.0

    def set_gamma(self, cam_idx: int, value: float):
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "set_gamma"):
            cam.set_gamma(value)

    def image_correction(self, cam_idx: int) -> dict:
        """Current ``{brightness, contrast, gamma}`` for a slot (neutral if N/A)."""
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "image_correction"):
            return cam.image_correction()
        return {"brightness": 0, "contrast": 1.0, "gamma": 1.0}

    def set_image_correction(self, cam_idx: int, brightness=None,
                             contrast=None, gamma=None):
        """Apply any provided correction component(s) to a slot. None = leave."""
        if brightness is not None:
            self.set_brightness(cam_idx, brightness)
        if contrast is not None:
            self.set_contrast(cam_idx, contrast)
        if gamma is not None:
            self.set_gamma(cam_idx, gamma)

    def reset_image_correction(self, cam_idx: int):
        """Restore neutral brightness/contrast/gamma for a slot."""
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "reset_image_correction"):
            cam.reset_image_correction()

    # ── Per-camera HARDWARE (camera-side) controls ────────────────
    # Drive the camera firmware (ToupCam SDK / OpenCV CAP_PROP). Distinct from
    # the software correction above. Getters read back FROM the device.

    def hardware_capabilities(self, cam_idx: int) -> dict:
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "hardware_capabilities"):
            return cam.hardware_capabilities()
        return {"source": "none", "controllable": False,
                "resolution": False, "controls": {}, "device_name": ""}

    def get_hw_settings(self, cam_idx: int) -> dict:
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "get_hw_settings"):
            return cam.get_hw_settings()
        return {"source": "none"}

    def log_hw_settings(self, cam_idx: int, prefix: str = ""):
        """Read + log a slot's hardware settings to the terminal."""
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "log_hw_settings"):
            return cam.log_hw_settings(prefix)
        return {"source": "none"}, ""

    def set_hw_auto_exposure(self, cam_idx: int, enabled: bool) -> bool:
        cam = self._widget(cam_idx)
        return bool(cam and cam.set_hw_auto_exposure(enabled))

    def set_hw_exposure_us(self, cam_idx: int, microseconds) -> bool:
        cam = self._widget(cam_idx)
        return bool(cam and cam.set_hw_exposure_us(microseconds))

    def set_hw_exposure_gain(self, cam_idx: int, percent) -> bool:
        cam = self._widget(cam_idx)
        return bool(cam and cam.set_hw_exposure_gain(percent))

    def set_hw_gamma(self, cam_idx: int, value) -> bool:
        cam = self._widget(cam_idx)
        return bool(cam and cam.set_hw_gamma(value))

    def set_hw_brightness(self, cam_idx: int, value) -> bool:
        cam = self._widget(cam_idx)
        return bool(cam and cam.set_hw_brightness(value))

    def set_hw_contrast(self, cam_idx: int, value) -> bool:
        cam = self._widget(cam_idx)
        return bool(cam and cam.set_hw_contrast(value))

    # Andor (Zyla) mono16→8-bit display scaling — no-ops on other backends.
    def set_hw_andor_auto_scale(self, cam_idx: int, enabled: bool) -> bool:
        cam = self._widget(cam_idx)
        return bool(cam and hasattr(cam, "set_hw_andor_auto_scale")
                    and cam.set_hw_andor_auto_scale(enabled))

    def set_hw_andor_scale_lo(self, cam_idx: int, counts) -> bool:
        cam = self._widget(cam_idx)
        return bool(cam and hasattr(cam, "set_hw_andor_scale_lo")
                    and cam.set_hw_andor_scale_lo(counts))

    def set_hw_andor_scale_hi(self, cam_idx: int, counts) -> bool:
        cam = self._widget(cam_idx)
        return bool(cam and hasattr(cam, "set_hw_andor_scale_hi")
                    and cam.set_hw_andor_scale_hi(counts))

    # v7.13 — Andor sensor-quality features + raw-frame statistics + averaged
    # capture. All hasattr-guarded no-ops on other backends.
    def set_hw_andor_feature(self, cam_idx: int, key: str, value) -> bool:
        cam = self._widget(cam_idx)
        return bool(cam and hasattr(cam, "set_hw_andor_feature")
                    and cam.set_hw_andor_feature(key, value))

    def reset_andor_sensor_defaults(self, cam_idx: int) -> bool:
        cam = self._widget(cam_idx)
        return bool(cam and hasattr(cam, "reset_andor_sensor_defaults")
                    and cam.reset_andor_sensor_defaults())

    def get_raw_frame_stats(self, cam_idx: int):
        """Latest raw 16-bit frame stats (lock snapshot, GUI-timer-safe)."""
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "get_raw_frame_stats"):
            return cam.get_raw_frame_stats()
        return None

    def capture_raw_average(self, cam_idx: int, n: int,
                            timeout_s: float = 10.0):
        """Blocking averaged raw capture — call from worker threads ONLY."""
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "capture_raw_average"):
            return cam.capture_raw_average(n, timeout_s=timeout_s)
        return None

    def set_capture_resolution(self, cam_idx: int, width: int, height: int):
        cam = self._widget(cam_idx)
        if cam is not None and hasattr(cam, "set_capture_resolution"):
            return cam.set_capture_resolution(width, height)
        return None

    def pixel_to_stage_offset(self, cam_idx: int,
                              px_x: float, px_y: float,
                              image_w: int, image_h: int
                              ) -> tuple[float, float]:
        """Convert pixel offset from image center to stage offset in µm.

        Args:
            cam_idx: Camera index
            px_x, px_y: Pixel coordinates in the image
            image_w, image_h: Image dimensions in pixels

        Returns:
            (dx_um, dy_um) offset from stage center position

        v7.5.x: if the camera has a calibrated rotation vs the stage axes
        (``get_rotation_deg``, measured by the stage-motion PixelCalibrationDialog
        / "Calibrate orientation"), the pixel offset is rotated into the stage
        frame so a live-view click maps to the correct XY direction even when the
        camera is mounted rotated (e.g. ~180°). With ``rotation_deg`` None/0 this
        is byte-identical to the legacy identity mapping (``dx_px·µm/px``), so
        uncalibrated cameras are unaffected. The DISPLAYED frame is intentionally
        left un-rotated — only the click→stage mapping is corrected.

        v7.5.x: frames are RAW (orientation is applied per consumer — the
        display via ``CameraFeedView``, the mosaic via ``MosaicBuilder``), so
        this maps a raw-frame click: mirror (``dx→−dx``) then ``R(θ)``. The
        ``CameraFeedView`` click handler reports raw-frame pixel coords even
        when its display is flipped/rotated, so the two stay consistent.

        Sign convention: ``θ = get_rotation_deg`` follows PixelCalibrationDialog's
        ``plus_column_direction_deg`` (the stage-plane angle, CCW from +X, that a
        stage move traces to +image-column). Inverting that measurement gives the
        click→stage map ``(dx_um, dy_um) = µm/px · R(θ) · (dx_px, dy_px)`` with the
        standard CCW rotation ``R(θ)``; it reduces to the identity mapping at θ=0.
        """
        # v7.5.x: resolve µm/px against the LIVE frame size (image_w) so a
        # resolution mismatch between calibration and the live feed doesn't
        # mis-scale the offset. ``getattr`` keeps this safe for lightweight
        # test doubles that expose only ``get_um_per_px``.
        eff = getattr(self, "effective_um_per_px", None)
        um_per_px = (eff(cam_idx, image_w) if callable(eff)
                     else self.get_um_per_px(cam_idx))
        # Pixel offset from center
        cx = image_w / 2.0
        cy = image_h / 2.0
        dx_px = px_x - cx
        dy_px = px_y - cy
        # v7.16: an OFF-CENTRE crop moves the pixel that means "the stage is
        # here", so add the crop's displacement back in the RAW frame (before
        # the flips and rotation, which is the frame it is measured in). A
        # centred or absent crop contributes (0, 0) — byte-identical.
        get_coff = getattr(self, "crop_center_offset_px", None)
        if callable(get_coff):
            off_x, off_y = get_coff(cam_idx)
            dx_px += off_x
            dy_px += off_y
        # Mirrored view → horizontal parity flip BEFORE scaling/rotation
        # (getattr-guarded for test doubles; None/False → no-op).
        get_mir = getattr(self, "get_mirrored", None)
        if callable(get_mir) and get_mir(cam_idx):
            dx_px = -dx_px
        # v7.5.x: vertical flip (flip Y) → parity flip on dy, BEFORE scale/rotate
        # (getattr-guarded; False → no-op). Same diag(sx, sy) the display + mosaic
        # apply, so a click on the live view maps to the correct stage XY.
        get_fy = getattr(self, "get_flip_y", None)
        if callable(get_fy) and get_fy(cam_idx):
            dy_px = -dy_px
        dx_um = dx_px * um_per_px
        dy_um = dy_px * um_per_px
        # Apply the calibrated camera→stage rotation (getattr-guarded for test
        # doubles). None/0 → identity, so behaviour is unchanged when unmeasured.
        get_rot = getattr(self, "get_rotation_deg", None)
        theta = get_rot(cam_idx) if callable(get_rot) else None
        if not theta:
            return (dx_um, dy_um)
        t = math.radians(float(theta))
        c, s = math.cos(t), math.sin(t)
        return (dx_um * c - dy_um * s,
                dx_um * s + dy_um * c)

    def stage_offset_to_pixel(self, cam_idx: int, dx_um: float, dy_um: float,
                              image_w: int, image_h: int) -> tuple:
        """EXACT inverse of :meth:`pixel_to_stage_offset`.

        A stage offset from the camera centre (µm) → the frame pixel that shows
        it. Lives here, immediately beside the forward map, so an overlay can
        project a target back onto the live image through the same mirror / flip
        / rotation the click path applies — drawing a marker with the naive
        identity ``dx_um / µm_per_px + w/2`` puts it in the wrong place on any
        rotated or mirrored camera, which stops being cosmetic the moment the
        operator has to click or resize the thing they see.

        Reduces to the identity mapping at θ=0 with no flips, exactly as the
        forward direction does.
        """
        eff = getattr(self, "effective_um_per_px", None)
        um_per_px = (eff(cam_idx, image_w) if callable(eff)
                     else self.get_um_per_px(cam_idx))
        if not um_per_px:
            return (image_w / 2.0, image_h / 2.0)

        dx, dy = float(dx_um), float(dy_um)
        # Undo the calibrated rotation first: R(-θ).
        get_rot = getattr(self, "get_rotation_deg", None)
        theta = get_rot(cam_idx) if callable(get_rot) else None
        if theta:
            t = math.radians(float(theta))
            c, s = math.cos(t), math.sin(t)
            dx, dy = (dx * c + dy * s, -dx * s + dy * c)

        dx_px = dx / um_per_px
        dy_px = dy / um_per_px
        # Parity flips are self-inverse and commute with the scaling above.
        get_mir = getattr(self, "get_mirrored", None)
        if callable(get_mir) and get_mir(cam_idx):
            dx_px = -dx_px
        get_fy = getattr(self, "get_flip_y", None)
        if callable(get_fy) and get_fy(cam_idx):
            dy_px = -dy_px
        # v7.16: undo the crop displacement the forward map added, in the same
        # raw frame and at the mirroring point in the chain.
        get_coff = getattr(self, "crop_center_offset_px", None)
        if callable(get_coff):
            off_x, off_y = get_coff(cam_idx)
            dx_px -= off_x
            dy_px -= off_y
        return (dx_px + image_w / 2.0, dy_px + image_h / 2.0)

    # ── Cleanup ───────────────────────────────────────────────────

    def shutdown(self):
        """Stop all cameras and clean up resources."""
        # v7.14 — finalize any live recording BEFORE the cameras close: a
        # recorder whose camera vanishes mid-write leaves a truncated file.
        try:
            from SupportClasses.CaptureContext import finalize_all_recordings
            n = finalize_all_recordings()
            if n:
                logger.info(f"Finalized {n} recording(s) on shutdown")
        except Exception as exc:
            logger.warning(f"could not finalize recordings on shutdown: {exc}")
        self.stop_all()
        logger.info("CameraManager shutdown complete")
