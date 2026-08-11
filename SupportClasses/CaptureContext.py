"""
CaptureContext.py — process-wide handles the capture path needs.

v7.14. `CameraFeedView` is constructed at ~18 sites with only
``(camera_manager, cam_idx)``. A capture wants to stamp the stage position and
read the capture settings, and threading a `StageController` plus `Settings`
through eighteen constructors would be a large, fragile change for a small
need. The app registers them here once at startup and the capture path resolves
them lazily.

Everything degrades to ``None``: a headless test, a disconnected stage or an
app that never registered simply produces a capture record with holes — which
is the rule the metadata layer already follows (absent beats wrong).

Pure: no Qt, no hardware imports.
"""

from __future__ import annotations

import logging
import threading
import weakref

logger = logging.getLogger(__name__)

_settings = None
_controller = None
_camera_manager = None
_recorders: "weakref.WeakSet" = weakref.WeakSet()
_resolution_lock = threading.RLock()


def register(*, settings=None, controller=None, camera_manager=None) -> None:
    """Called once from ``MainWindow`` after the camera manager exists."""
    global _settings, _controller, _camera_manager
    if settings is not None:
        _settings = settings
    if controller is not None:
        _controller = controller
    if camera_manager is not None:
        _camera_manager = camera_manager


def get_settings():
    return _settings


def get_stage_controller():
    return _controller


def get_camera_manager():
    return _camera_manager


def capture_resolution_lock() -> threading.RLock:
    """Serialises momentary capture-resolution switches.

    A full-resolution still and a mosaic scan both switch the camera and
    restore it; interleaved, one's restore would undo the other's switch.
    """
    return _resolution_lock


def register_recorder(rec) -> None:
    """Track a live recording so app shutdown can finalize it.

    Weak: a recorder that is garbage-collected simply leaves the set.
    """
    try:
        _recorders.add(rec)
    except TypeError:      # pragma: no cover — non-weakrefable stand-in
        pass


def finalize_all_recordings() -> int:
    """Stop every live recording, returning how many were finalized.

    Called from camera-manager shutdown, BEFORE the cameras themselves close:
    a recorder whose camera vanished mid-write leaves a truncated file.
    """
    n = 0
    for rec in list(_recorders):
        try:
            if getattr(rec, "is_recording", False):
                rec.stop_recording()
                n += 1
        except Exception as exc:
            logger.warning(f"could not finalize a recording on shutdown: {exc}")
    return n


def reset_for_tests() -> None:
    global _settings, _controller, _camera_manager
    _settings = _controller = _camera_manager = None
    _recorders.clear()
