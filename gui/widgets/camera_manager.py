"""
camera_manager.py — Centralized camera ownership and management.

v7.3.3: Provides a single source of truth for camera widgets so that
any page in the application can display live camera feeds without
duplicating detection, creation, or control logic.

Usage::

    # In app.py (create once)
    manager = CameraManager(max_cameras=3)

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
import threading
from typing import Optional

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

    def __init__(self, max_cameras: int = 3, parent=None):
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
        # v7.5.x: per-slot in-plane rotation (deg) — the stage direction that
        # maps to the camera's lateral image axis, from the µm/px calibration.
        # None = not measured (needle aligner falls back to nominal mounting).
        self._rotation_deg: list[Optional[float]] = [None] * max_cameras

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

    def detect_cameras(self):
        """Detect all available camera sources (OpenCV + ToupCam + Sim).

        Refreshes internal source list and updates each CameraWidget's
        hidden combo. Emits cameras_detected(num_sources) when done.
        """
        if not self._cameras:
            self.cameras_detected.emit(0)
            return

        # Use the first camera widget's refresh to detect hardware
        first = self._cameras[0]
        if hasattr(first, 'refresh_cameras'):
            first.refresh_cameras()

        # Read detected sources from its combo
        self._available_sources = []
        if hasattr(first, 'camera_combo'):
            combo = first.camera_combo
            for i in range(combo.count()):
                data = combo.itemData(i)
                text = combo.itemText(i)
                if data and data != -1:
                    self._available_sources.append((text, data))

        # Refresh all other camera widget combos too
        for cam in self._cameras[1:]:
            if hasattr(cam, 'refresh_cameras'):
                cam.refresh_cameras()

        # v7.5.x: refresh the DirectShow identity map (Windows; [] elsewhere).
        try:
            from gui.widgets.camera_identity import enumerate_directshow_cameras
            self._ds_cameras = enumerate_directshow_cameras()
        except Exception as exc:
            logger.debug(f"DirectShow identity enumeration skipped: {exc}")
            self._ds_cameras = []

        n = len(self._available_sources)
        logger.info(f"CameraManager detected {n} sources")
        self.cameras_detected.emit(n)

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
        """Start camera at given index."""
        if cam_idx < 0 or cam_idx >= len(self._cameras):
            return
        cam = self._cameras[cam_idx]
        if not getattr(cam, '_running', False):
            cam.start()
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

    def set_um_per_px(self, cam_idx: int, value: float):
        """Set microns per pixel for a camera.

        Marks the slot as explicitly calibrated (see ``is_um_per_px_calibrated``).
        """
        if 0 <= cam_idx < self._max_cameras:
            self._um_per_px[cam_idx] = value
            self._um_per_px_set[cam_idx] = True

    def is_um_per_px_calibrated(self, cam_idx: int) -> bool:
        """True once ``set_um_per_px`` has supplied a real value for the slot.

        Distinguishes a calibrated camera from one still carrying the 1.67
        seed default, so callers can refuse to run on uncalibrated cameras.
        """
        if 0 <= cam_idx < self._max_cameras:
            return self._um_per_px_set[cam_idx]
        return False

    def get_rotation_deg(self, cam_idx: int) -> Optional[float]:
        """In-plane rotation (deg) measured for a slot, or None if unmeasured."""
        if 0 <= cam_idx < self._max_cameras:
            return self._rotation_deg[cam_idx]
        return None

    def set_rotation_deg(self, cam_idx: int, value: Optional[float]):
        """Set (or clear, with None) the in-plane rotation for a slot."""
        if 0 <= cam_idx < self._max_cameras:
            self._rotation_deg[cam_idx] = (
                None if value is None else float(value))

    def get_magnification(self, cam_idx: int) -> float:
        """Get objective magnification for a camera."""
        if 0 <= cam_idx < self._max_cameras:
            return self._magnification[cam_idx]
        return 2.0

    def set_magnification(self, cam_idx: int, value: float):
        """Set objective magnification for a camera."""
        if 0 <= cam_idx < self._max_cameras:
            self._magnification[cam_idx] = value

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
        """
        um_per_px = self.get_um_per_px(cam_idx)
        # Pixel offset from center
        cx = image_w / 2.0
        cy = image_h / 2.0
        dx_px = px_x - cx
        dy_px = px_y - cy
        return (dx_px * um_per_px, dy_px * um_per_px)

    # ── Cleanup ───────────────────────────────────────────────────

    def shutdown(self):
        """Stop all cameras and clean up resources."""
        self.stop_all()
        logger.info("CameraManager shutdown complete")
