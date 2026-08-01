"""
spheroid_crop_worker.py — grab a fresh microscope frame off the GUI thread.

v7.8: banking a training crop needs a frame the stage has actually settled on.
Getting one means sleeping and then waiting for new frames to arrive, so it
cannot happen in a button handler — that would stall the Qt event loop and freeze
every camera feed on the page.

Two things this deliberately does NOT do:

* It does not call ``CameraWidget.capture_fresh_frame``. That method's own
  docstring forbids concurrent use with the display grab timer; instead this
  follows the pattern the mosaic scan worker uses — wait for
  ``frame_count_value()`` to advance, then take ``get_current_frame()``.
* It does not read the DISPLAY pixmap. The picker's view paints the crosshair and
  the target rings onto its pixmap, so a crop of that would bake our own
  annotation ring into the training image. ``get_current_frame()`` returns the
  raw BGR array, which is also the frame ``um_per_px`` and
  ``pixel_to_stage_offset`` are defined against.

The stage position is sampled WITH the frame and travels with it, so the caller
crops against the geometry that was true at capture time rather than a later poll.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

from PySide6.QtCore import QThread, Signal

logger = logging.getLogger(__name__)


class SpheroidCropWorker(QThread):
    """Settle → wait for N fresh frames → emit ``(frame_bgr, stage_um, um_per_px)``.

    ``failed`` carries an operator-readable reason; nothing here raises into the
    GUI thread.
    """

    captured = Signal(object, object, float)
    failed = Signal(str)

    def __init__(self, cam, stage_reader, um_per_px: float, *,
                 settle_ms: int = 300, fresh_frames: int = 3,
                 timeout_s: float = 2.5, parent=None):
        super().__init__(parent)
        self._cam = cam
        self._stage_reader = stage_reader
        self._um_per_px = float(um_per_px)
        self._settle_ms = max(0, int(settle_ms))
        self._fresh_frames = max(1, int(fresh_frames))
        self._timeout_s = float(timeout_s)
        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        cam = self._cam
        if cam is None:
            self.failed.emit("No microscope camera is running.")
            return
        try:
            if self._settle_ms and not self._stop:
                time.sleep(self._settle_ms / 1000.0)

            start = None
            counter = getattr(cam, "frame_count_value", None)
            if callable(counter):
                try:
                    start = counter()
                except Exception:
                    start = None
            if start is not None:
                deadline = time.time() + self._timeout_s
                while time.time() < deadline and not self._stop:
                    try:
                        if counter() - start >= self._fresh_frames:
                            break
                    except Exception:
                        break
                    time.sleep(0.02)

            if self._stop:
                return
            frame = cam.get_current_frame()
            if frame is None or getattr(frame, "size", 0) == 0:
                self.failed.emit(
                    "No camera frame arrived — is the microscope feed running?")
                return

            stage = None
            if callable(self._stage_reader):
                try:
                    stage = self._stage_reader()
                except Exception as exc:
                    logger.debug("crop worker stage read failed: %s", exc)
            if stage is None:
                self.failed.emit("Could not read the stage position.")
                return
            self.captured.emit(frame, (float(stage[0]), float(stage[1])),
                               self._um_per_px)
        except Exception as exc:            # pragma: no cover - defensive
            logger.exception("spheroid crop worker crashed")
            self.failed.emit(str(exc))
