"""
capture_controller.py — the one orchestrator behind the capture buttons.

v7.14. `CameraFeedView` is built at ~18 sites; putting capture logic in it
would mean 18 copies of the blocking work. Instead the view gets buttons and
this gets every decision, every blocking call and every byte written.

Blocking work — `capture_raw_average`, a full-resolution stream restart, video
encoding — runs on daemon threads and comes back through queued Qt signals, the
pattern used by `SafeTravelWorker` and the v7.13 signal optimizer.

Recording ingress deliberately connects to the CAMERA's ``frame_captured``, not
to the view's ``_on_frame``: that early-returns when the page is hidden, and a
recording must keep running while the operator works elsewhere.
"""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Callable, Optional

from PySide6.QtCore import QObject, QTimer, Signal

logger = logging.getLogger(__name__)

# Frames buffered between the GUI ingress slot and the encoder thread. Bounded:
# on overflow a frame is DROPPED and counted, never blocking the GUI thread.
QUEUE_DEPTH_SECONDS = 4

# A recording with no frame for this long has lost its camera.
STALL_GRACE_S = 5.0


def to_bgr(frame):
    """Normalise a delivered frame to a contiguous BGR uint8 ndarray.

    v7.15 — THE RECORDING BUG. ``CameraWidget.frame_captured`` is
    ``Signal(object)`` carrying a **QImage** (RGB888), not a numpy array. The
    encoder assumed ndarray, ``orient_array`` swallowed the type error and
    returned a non-image, and ``_open_writer``'s ``h, w = img.shape[:2]`` then
    raised inside a daemon thread with no ``try`` — killing the session so it
    could neither stop itself nor save. One conversion at the boundary, so
    nothing downstream has to know which type arrived.

    ⚠ The QImage is **RGB**; ``cv2.VideoWriter`` wants **BGR**. Fixing only the
    type would have produced silently colour-swapped video.

    Returns None when the frame cannot be interpreted — the caller reports
    that rather than encoding garbage.
    """
    if frame is None:
        return None
    try:
        import numpy as np
    except ImportError:      # pragma: no cover
        return None

    # Already an ndarray: accept mono or BGR, reject anything else.
    if isinstance(frame, np.ndarray):
        if frame.ndim == 2:
            try:
                import cv2
                return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            except Exception:
                return np.ascontiguousarray(
                    np.repeat(frame[:, :, None], 3, axis=2))
        if frame.ndim == 3 and frame.shape[2] >= 3:
            return np.ascontiguousarray(frame[:, :, :3])
        return None

    # QImage → ndarray. Convert to a known format first: the emitted image is
    # RGB888, but a converted/scaled QImage elsewhere may not be, and reading
    # the buffer of an unexpected format yields a sheared picture.
    try:
        from PySide6.QtGui import QImage
    except ImportError:      # pragma: no cover
        return None
    if not isinstance(frame, QImage):
        return None
    try:
        img = frame.convertToFormat(QImage.Format.Format_RGB888)
        w, h = img.width(), img.height()
        if w <= 0 or h <= 0:
            return None
        ptr = img.constBits()
        # bytesPerLine >= 3*w: Qt pads rows to a 4-byte boundary, so the
        # buffer must be read by STRIDE and then trimmed, never reshaped to
        # (h, w, 3) directly.
        stride = img.bytesPerLine()
        buf = np.frombuffer(memoryview(ptr)[:stride * h], dtype=np.uint8)
        rgb = buf.reshape(h, stride)[:, :w * 3].reshape(h, w, 3)
        return np.ascontiguousarray(rgb[:, :, ::-1])      # RGB → BGR
    except Exception as exc:
        logger.debug(f"QImage → ndarray failed: {exc}")
        return None


class CaptureController(QObject):
    """Stills + recording for one camera slot."""

    captured = Signal(str, str)        # path, note
    failed = Signal(str)               # operator-readable reason
    record_state = Signal(bool, float, int)   # active, elapsed_s, frames
    # v7.15: the encoder thread ended itself (limit reached, stall, error).
    # Emitted FROM that thread; Qt queues it onto the GUI thread, so the
    # recording is finalized at once instead of waiting for the next tick.
    _session_ended = Signal()

    def __init__(self, camera_manager, cam_idx: int, parent=None):
        super().__init__(parent)
        self._mgr = camera_manager
        self._cam_idx = int(cam_idx)
        self._busy = False                 # a still is in flight
        self._context_fn: Optional[Callable[[], dict]] = None
        self._orientation_fn: Optional[Callable[[], tuple]] = None
        self._rec = None                   # _RecordingSession
        self._tick = QTimer(self)
        self._tick.setInterval(500)
        self._tick.timeout.connect(self._emit_record_state)
        self._session_ended.connect(self._on_session_ended)

    # ── Wiring from the host page/view ────────────────────────────

    def set_context_provider(self, fn):
        """``fn() -> {"channel": …, "well": …, "plate": …}``.

        The page knows what it is imaging; the hardware only knows a slot
        number. Whatever this returns wins over the hardware reading.
        """
        self._context_fn = fn

    def set_orientation_provider(self, fn):
        """``fn() -> (rotation_deg, flip_x, flip_y)`` — the transform the view
        is displaying, so a saved 'as seen' image matches the screen."""
        self._orientation_fn = fn

    @property
    def is_recording(self) -> bool:
        return self._rec is not None and self._rec.active

    @property
    def is_busy(self) -> bool:
        return self._busy

    # ── Settings ──────────────────────────────────────────────────

    def settings(self) -> dict:
        from SupportClasses.CaptureSpec import merged_settings
        stored = None
        try:
            from SupportClasses.CaptureContext import get_settings
            s = get_settings()
            if s is not None:
                stored = s.get_section("capture")
        except Exception:
            stored = None
        return merged_settings(stored)

    def _context(self) -> dict:
        try:
            return dict(self._context_fn() or {}) if self._context_fn else {}
        except Exception:
            return {}

    def _orientation(self) -> tuple:
        try:
            if self._orientation_fn:
                return tuple(self._orientation_fn())
        except Exception:
            pass
        try:
            from SupportClasses.MosaicCalibration import (
                resolve_camera_orientation)
            # v7.16 🐞 this unpacked FOUR values from a 3-tuple
            # (``_resolve_orientation`` is the 4-tuple one, with the mosaic's
            # display-only output rotation appended). The ValueError landed in
            # the bare ``except`` below, so whenever no ``_orientation_fn`` was
            # supplied every capture was stamped with a NEUTRAL orientation —
            # silently claiming ``pixels_stage_aligned`` for a rotated camera.
            rot, fx, fy = resolve_camera_orientation(
                camera_manager=self._mgr, cam_idx=self._cam_idx)
            return (rot, fx, fy)
        except Exception:
            return (0.0, False, False)

    # ── Still capture ─────────────────────────────────────────────

    def capture_still(self) -> bool:
        """Kick off a capture. False when one is already running."""
        if self._busy:
            return False
        self._busy = True
        cfg = self.settings()
        ctx = self._context()
        orient = self._orientation()
        threading.Thread(target=self._still_worker, args=(cfg, ctx, orient),
                         daemon=True, name="CaptureStill").start()
        return True

    def _still_worker(self, cfg, ctx, orient):
        try:
            path, note = self._do_still(cfg, ctx, orient)
        except Exception as exc:
            logger.exception("capture failed")
            self._busy = False
            self.failed.emit(str(exc))
            return
        self._busy = False
        if path is None:
            self.failed.emit(note)
        else:
            logger.info(f"Captured {path}")
            self.captured.emit(str(path), note)
            # v7.17: offer the still to LabLink — never raises, never blocks,
            # and a no-op unless the operator enabled the feature.
            from SupportClasses.LabLinkPublish import publish_capture
            publish_capture(str(path), note, kind="still")

    def _do_still(self, cfg, ctx, orient):
        from SupportClasses.CaptureImageWriter import write_image
        from SupportClasses.CaptureMetadata import collect, to_tokens
        from SupportClasses.CaptureOrientation import orient_array
        from SupportClasses.CaptureSpec import (
            open_unique, render_template, resolve_output_dir, still_extension,
            validate_still)

        # v7.15: validate_still, NOT validate — the combined check meant a
        # video-only problem refused to take a photograph.
        problems = validate_still(cfg)
        if problems:
            return None, problems[0]

        raw_mode = (cfg.get("still_source") == "raw")
        restore_to = None
        try:
            arr, source_mode, restore_to = self._grab(cfg, raw_mode)
            if arr is None:
                # NOTE the restore below still runs: a FAILED raw grab is
                # exactly when the camera must not be left at full
                # resolution, and an early return here would skip it.
                return None, source_mode      # carries the reason

            rot, fx, fy = orient
            img = orient_array(arr, mirrored=fx, flip_y=fy, rotation_deg=rot)
            h, w = img.shape[:2]
            meta = collect(
                camera_manager=self._mgr, cam_idx=self._cam_idx,
                captured_wh=(w, h), kind="still", source_mode=source_mode,
                orientation=orient, controller=_controller(),
                microscope=_microscope(), config_store=_config_store(),
                cfg=cfg, extra=ctx)
            stem, unknown = render_template(
                cfg.get("still_template", ""), to_tokens(meta))
            directory = resolve_output_dir(cfg)
            path, fh = open_unique(directory, stem, still_extension(cfg))
            fh.close()
            write_image(path, img, meta, embed=bool(cfg.get("embed_metadata")),
                        sidecar=bool(cfg.get("write_sidecar")))
            note = f"{w}×{h} {source_mode}"
            if unknown:
                note += f" · unknown token(s): {', '.join(unknown)}"
            return path, note
        finally:
            if restore_to is not None:
                from SupportClasses.CaptureResolution import restore
                restore(self._mgr, self._cam_idx, restore_to)

    def _grab(self, cfg, raw_mode):
        """Return ``(array, source_mode, resolution_to_restore)``.

        A raw request that cannot be served is REFUSED — never quietly served
        a display frame instead. An auto-scaled display frame is not the same
        measurement, and a file that looks right and is not quantitative is
        the worse outcome.
        """
        restore_to = None
        if raw_mode:
            n = max(1, int(cfg.get("still_raw_avg_frames", 1)))
            if cfg.get("still_full_res"):
                try:
                    from SupportClasses.CaptureResolution import switch_to_max
                    restore_to = switch_to_max(self._mgr, self._cam_idx)
                    if restore_to is not None:
                        time.sleep(0.4)      # let the restarted stream settle
                except Exception as exc:
                    logger.warning(f"full-res capture switch failed: {exc}")
            try:
                arr = self._mgr.capture_raw_average(self._cam_idx, n,
                                                    timeout_s=15.0)
            except Exception as exc:
                return None, f"raw capture failed: {exc}", restore_to
            if arr is None:
                return (None,
                        "This camera cannot deliver raw frames — switch the "
                        "capture source to Display in the capture settings.",
                        restore_to)
            mode = f"raw(avg {n})" if n > 1 else "raw"
            return arr, mode, restore_to

        cam = None
        try:
            cam = self._mgr.cameras[self._cam_idx]
        except (AttributeError, IndexError):
            pass
        arr = None
        if cam is not None:
            try:
                arr = cam.capture_fresh_frame(discard_n_frames=1, settle_ms=60)
            except Exception:
                arr = None
            if arr is None:
                try:
                    arr = cam.get_current_frame()
                except Exception:
                    arr = None
        if arr is None:
            return None, "The camera is not delivering frames.", restore_to
        return arr, "display", restore_to

    # ── Recording ─────────────────────────────────────────────────

    def start_recording(self) -> bool:
        if self.is_recording:
            return False
        cfg = self.settings()
        # v7.15: the recording path never validated its own settings. Refuse
        # up front rather than producing an unplayable file.
        from SupportClasses.CaptureSpec import validate_video
        problems = validate_video(cfg)
        if problems:
            self.failed.emit(problems[0])
            return False
        try:
            self._rec = _RecordingSession(self, cfg, self._context(),
                                          self._orientation())
            self._rec.start()
        except Exception as exc:
            logger.exception("recording could not start")
            self._rec = None
            self.failed.emit(str(exc))
            return False
        self._tick.start()
        self._emit_record_state()
        return True

    def stop_recording(self):
        rec, self._rec = self._rec, None
        self._tick.stop()
        if rec is None:
            return
        result = rec.stop()
        self.record_state.emit(False, 0.0, 0)
        if result is None:
            return
        if result.ok:
            self.captured.emit(str(result.path), result.describe())
            # v7.17: offer the recording to LabLink. GUI thread — which is
            # exactly why publish_capture never blocks (put_nowait inside).
            from SupportClasses.LabLinkPublish import publish_capture
            publish_capture(str(result.path), result.describe(), kind="video")
        else:
            self.failed.emit(result.describe())

    def _on_session_ended(self):
        """Queued from the encoder thread when it terminates itself."""
        if self._rec is not None and not self._rec.active:
            self.stop_recording()

    def _emit_record_state(self):
        """Timer backstop. ``_session_ended`` normally gets there first; this
        still runs so a session that died without signalling cannot linger."""
        rec = self._rec
        if rec is None:
            self.record_state.emit(False, 0.0, 0)
            return
        if not rec.active:
            # The session ended itself (stall, size lock, limit) — surface it.
            self.stop_recording()
            return
        self.record_state.emit(True, rec.elapsed_s, rec.frames)

    def shutdown(self):
        """Finalize any recording — called on app close and page teardown."""
        if self.is_recording:
            self.stop_recording()


class _RecordingSession:
    """One recording: GUI-thread ingress, encoder on a daemon thread."""

    def __init__(self, owner: CaptureController, cfg, ctx, orient):
        import queue
        self._owner = owner
        self._cfg = cfg
        self._orient = orient
        self._fps = max(0.1, float(cfg.get("video_fps", 15)))
        self._raw = (cfg.get("video_source") == "raw_timelapse")
        self._q = queue.Queue(maxsize=max(4, int(self._fps *
                                                 QUEUE_DEPTH_SECONDS)))
        self._stop = threading.Event()
        self._thread = None
        self._cam = None
        self.active = False
        self.frames = 0
        self.dropped = 0
        self.started_at = 0.0
        self._result = None
        self._writer = None
        self._ctx = ctx
        self._reason = ""
        self._unknown_tokens: list = []
        self._meta = None

    @property
    def elapsed_s(self) -> float:
        return max(0.0, time.monotonic() - self.started_at) if self.started_at else 0.0

    def start(self):
        mgr, idx = self._owner._mgr, self._owner._cam_idx
        try:
            self._cam = mgr.cameras[idx]
        except (AttributeError, IndexError):
            raise RuntimeError("camera unavailable")
        self.started_at = time.monotonic()
        self.active = True
        self._cam.frame_captured.connect(self._ingest)
        self._thread = threading.Thread(target=self._encode_loop, daemon=True,
                                        name="CaptureRecord")
        self._thread.start()

    def _ingest(self, frame):
        """GUI-thread slot: normalise, copy and hand off.

        v7.15: the conversion happens HERE, at the one place a delivered frame
        enters, so the encoder only ever sees a BGR ndarray. It is cheap (a
        strided view + a channel reverse) and keeps the GUI slot's cost the
        same order as the ``.copy()`` it replaces.
        """
        if not self.active or frame is None:
            return
        bgr = to_bgr(frame)
        if bgr is None:
            self.dropped += 1
            return
        try:
            self._q.put_nowait((time.monotonic(), bgr))
        except Exception:
            self.dropped += 1

    def _encode_loop(self):
        """Encoder thread.

        v7.15: the whole body is guarded and ``active`` is cleared in a
        ``finally``. Previously any raise here killed the thread BEFORE
        ``active = False``, so the session was stranded as permanently
        "recording": the 1 Hz tick never saw it end, the max-duration check
        never ran again, and a manual stop found no writer and reported
        "no frames were recorded". An unhandled exception in a daemon thread
        goes to ``threading.excepthook`` → stderr, which a windowed app
        discards, so it left no trace either.
        """
        import queue as _q
        from SupportClasses.CaptureOrientation import orient_array
        last_seen = time.monotonic()
        max_s = float(self._cfg.get("video_max_seconds", 0) or 0)
        max_b = float(self._cfg.get("video_max_gb", 0) or 0) * (1024 ** 3)
        # v7.21.8: with no time cap the size cap is the only automatic stop, so
        # it has to be one the CONTAINER can actually honour — an AVI is clamped
        # to what RIFF's 32-bit offsets can describe. A raw time-lapse is a
        # directory of TIFFs, not one container, so it keeps the operator's value.
        if not self._raw:
            from SupportClasses.CaptureVideoWriter import container_byte_limit
            max_b = container_byte_limit(
                self._cfg.get("video_container", "mp4"), max_b)
        try:
            while not self._stop.is_set():
                try:
                    t_mono, frame = self._q.get(timeout=0.25)
                except _q.Empty:
                    if time.monotonic() - last_seen > STALL_GRACE_S:
                        self._reason = "camera stopped delivering frames"
                        break
                    continue
                last_seen = time.monotonic()
                rot, fx, fy = self._orient
                img = orient_array(frame, mirrored=fx, flip_y=fy,
                                   rotation_deg=rot)
                if img is None:
                    self.dropped += 1
                    continue
                if self._writer is None and not self._open_writer(img):
                    self._reason = "no available video encoder"
                    break
                if not self._write(img, t_mono):
                    self._reason = "the capture resolution changed mid-recording"
                    break
                if max_s and self.elapsed_s >= max_s:
                    self._reason = f"reached the {max_s:.0f} s limit"
                    break
                if max_b and self._bytes_written() >= max_b:
                    self._reason = (f"reached the "
                                    f"{max_b / (1024 ** 3):.1f} GB limit")
                    break
        except Exception as exc:
            logger.exception("recording encoder failed")
            self._reason = f"encoder error: {exc}"
        finally:
            self.active = False
            try:
                self._owner._session_ended.emit()
            except Exception:
                pass

    def _bytes_written(self) -> float:
        """Size on disk so far, for the max-size limit. 0 when unknowable."""
        w = self._writer
        try:
            if self._raw:
                return float(sum(f.stat().st_size
                                 for f in w.dir.glob("*.tif")))
            return float(w.path.stat().st_size)
        except Exception:
            return 0.0

    def _open_writer(self, img) -> bool:
        from SupportClasses.CaptureMetadata import collect, to_tokens
        from SupportClasses.CaptureSpec import (
            open_unique, render_template, resolve_output_dir, unique_dir)
        h, w = img.shape[:2]
        meta = collect(camera_manager=self._owner._mgr,
                       cam_idx=self._owner._cam_idx, captured_wh=(w, h),
                       kind="video",
                       source_mode="raw_timelapse" if self._raw else "display",
                       orientation=self._orient, controller=_controller(),
                       microscope=_microscope(), config_store=_config_store(),
                       cfg=self._cfg, extra=self._ctx)
        self._meta = meta
        stem, unknown = render_template(
            self._cfg.get("video_template", ""), to_tokens(meta))
        if unknown:
            # v7.15: the still path reports these; the video path dropped them,
            # so a typo'd token silently vanished from every recording's name.
            self._unknown_tokens = list(unknown)
        directory = resolve_output_dir(self._cfg)
        if self._raw:
            from SupportClasses.CaptureVideoWriter import RawFrameSequenceWriter
            self._writer = RawFrameSequenceWriter(
                unique_dir(directory, stem), meta,
                interval_s=float(self._cfg.get("raw_timelapse_interval_s", 1)))
            return True
        from SupportClasses.CaptureSpec import video_extension
        from SupportClasses.CaptureVideoWriter import EncodedVideoWriter
        container = str(self._cfg.get("video_container", "mp4"))
        path, fh = open_unique(directory, stem, video_extension(self._cfg))
        fh.close()
        try:
            path.unlink()      # cv2 opens its own handle; the reservation
        except OSError:        # already proved the name was ours
            pass
        self._writer = EncodedVideoWriter(
            path, self._fps, (w, h), container=container,
            quality=int(self._cfg.get("video_quality", 80)))
        return self._writer.open()

    def _write_sidecar(self, result):
        """A video container cannot carry our metadata, so the JSON beside it
        is the only record. v7.15: ``write_sidecar`` was a stills-only setting
        despite its own tooltip calling the sidecar "the only metadata a video
        can carry"."""
        if (not self._cfg.get("write_sidecar") or self._meta is None
                or not result.ok or result.path is None):
            return
        try:
            from SupportClasses.CaptureMetadata import to_json
            side = result.path.with_suffix(result.path.suffix + ".json")
            side.write_text(to_json(self._meta), encoding="utf-8")
        except Exception as exc:
            logger.warning(f"video sidecar not written: {exc}")

    def _write(self, img, t_mono) -> bool:
        if self._raw:
            ok = self._writer.add(img, t_wall=time.time(), t_mono=t_mono)
            self.frames = self._writer.frames
            return True if ok else True     # a bad frame skips, never aborts
        from SupportClasses.CaptureVideoWriter import plan_frame_repeats
        repeats = plan_frame_repeats(self.elapsed_s, self._fps,
                                     self._writer.frames)
        if repeats == 0:
            return True                     # camera ahead of the target rate
        if not self._writer.add(img, repeats=repeats):
            return False                    # size changed → finalize cleanly
        self.frames = self._writer.frames
        return True

    def stop(self):
        self.active = False
        self._stop.set()
        try:
            if self._cam is not None:
                self._cam.frame_captured.disconnect(self._ingest)
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        if self._writer is None:
            from SupportClasses.CaptureVideoWriter import VideoResult
            return VideoResult(ok=False,
                               reason=self._reason or "no frames were recorded")
        res = self._writer.close(reason=self._reason)
        res.dropped = self.dropped
        if self._unknown_tokens:
            res.warnings.append("unknown token(s) in the video file name: "
                                + ", ".join(self._unknown_tokens))
        self._write_sidecar(res)
        if self.dropped and res.frames:
            frac = self.dropped / float(self.dropped + res.frames)
            if frac > 0.01:
                res.warnings.append(
                    f"{frac * 100:.0f}% of frames were dropped — the encoder "
                    f"could not keep up")
        return res


# ── Lazily-resolved app singletons (never imported at module scope) ──

def _controller():
    try:
        from SupportClasses.CaptureContext import get_stage_controller
        return get_stage_controller()
    except Exception:
        return None


def _microscope():
    try:
        from SupportClasses.MicroscopeControl import get_microscope
        return get_microscope()
    except Exception:
        return None


def _config_store():
    try:
        from SupportClasses.MicroscopeConfigStore import get_store
        return get_store()
    except Exception:
        return None
