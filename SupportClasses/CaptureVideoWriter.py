"""
CaptureVideoWriter.py — encode a live feed to a playable video file.

v7.14. The first video encoder in this repo (the "walkthrough recorder" writes
GUI screenshots for tutorials, not camera frames), so two things are stated
explicitly rather than assumed:

**1. The frame rate is the PLAYBACK rate.** A camera at 4 fps recorded into a
15 fps file would otherwise play back at nearly 4× slow motion while the
operator believes they recorded real time. :func:`plan_frame_repeats` decides
how many copies of the newest frame to write so the file's duration tracks the
wall clock — a slow camera repeats frames, a fast one drops them, and either
way one second of recording is one second of video.

**2. ``avc1`` is deliberately excluded.** On this machine it reports
``isOpened() == True`` while the OpenH264 DLL fails to load, producing a file
that never plays. It looks like the obvious quality upgrade, so it is named
here and pinned by a test. Do not add it back without a decodable file on the
bench.

Pure: cv2 + numpy, no Qt.
"""

from __future__ import annotations

import json
import logging
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

try:
    import cv2
    _CV2 = True
except ImportError:      # pragma: no cover
    cv2 = None
    _CV2 = False

# Container → fourcc candidates, best first. See the module docstring for why
# avc1/H.264 is absent.
FOURCC_CHAIN: dict = {
    "mp4": ("mp4v",),
    "avi": ("XVID", "MJPG"),
}

# A file smaller than this at finalize never contained real video.
MIN_USABLE_BYTES = 1024


@dataclass
class VideoResult:
    """Outcome of one recording — always reported, success or not."""
    path: Optional[Path] = None
    ok: bool = False
    frames: int = 0
    duration_s: float = 0.0
    fourcc: str = ""
    bytes: int = 0
    dropped: int = 0
    reason: str = ""
    container_substituted: bool = False
    quality_applied: bool = False
    warnings: list = field(default_factory=list)

    def describe(self) -> str:
        if not self.ok:
            return f"recording failed: {self.reason}"
        txt = (f"{self.frames} frames · {self.duration_s:.1f} s · "
               f"{self.bytes / (1024 * 1024):.1f} MB")
        if self.container_substituted:
            txt += " · saved as .avi (no MP4 encoder available)"
        if self.dropped:
            txt += f" · {self.dropped} frame(s) dropped"
        return txt


def plan_frame_repeats(elapsed_s: float, fps: float, frames_written: int,
                       *, max_repeat: int = 8) -> int:
    """How many copies of the newest frame keep playback at real time.

    ``elapsed_s`` is wall-clock since recording started. The file should hold
    ``elapsed_s × fps`` frames by now; the difference is how many to write.
    Returns 0 when the camera is running AHEAD of the requested rate (the frame
    is dropped), and is clamped so one long stall cannot dump hundreds of
    duplicate frames.
    """
    try:
        want = int(float(elapsed_s) * float(fps)) + 1
    except (TypeError, ValueError):
        return 1
    need = want - int(frames_written)
    if need <= 0:
        return 0
    return min(int(need), max(1, int(max_repeat)))


class EncodedVideoWriter:
    """cv2.VideoWriter with a verified open, size lock and honest finalize."""

    def __init__(self, path: Path, fps: float, size: tuple,
                 *, container: str = "mp4", quality: int = 80,
                 writer_factory=None):
        self.path = Path(path)
        self.fps = max(0.1, float(fps))
        self.size = (int(size[0]), int(size[1]))
        self.container = str(container or "mp4").lower()
        self.quality = max(1, min(100, int(quality)))
        self._factory = writer_factory or self._make_writer
        self._w = None
        self.fourcc = ""
        self.frames = 0
        self.container_substituted = False
        self.quality_applied = False

    @staticmethod
    def _make_writer(path, fourcc, fps, size):      # pragma: no cover - cv2
        if not _CV2:
            return None
        return cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*fourcc),
                               float(fps), (int(size[0]), int(size[1])))

    def open(self) -> bool:
        """Try each fourcc, VERIFYING the writer actually opened.

        A writer that reports success and then writes nothing decodable is the
        failure this loop exists to catch (see the module docstring).
        """
        attempts = [(self.container, fc)
                    for fc in FOURCC_CHAIN.get(self.container, ("MJPG",))]
        if self.container == "mp4":
            # Falling back to AVI beats failing outright — but it is REPORTED,
            # so the operator is not surprised by the extension later.
            attempts += [("avi", fc) for fc in FOURCC_CHAIN["avi"]]
        for cont, fourcc in attempts:
            path = self.path if cont == self.container else \
                self.path.with_suffix("." + cont)
            try:
                w = self._factory(path, fourcc, self.fps, self.size)
            except Exception as exc:
                logger.debug(f"video writer {fourcc} raised: {exc}")
                continue
            if w is not None and w.isOpened():
                self._w = w
                self.fourcc = fourcc
                if path != self.path:
                    self.container_substituted = True
                    self.path = path
                self._apply_quality()
                logger.info(f"Recording to {self.path.name} ({fourcc}, "
                            f"{self.fps:g} fps, {self.size[0]}x{self.size[1]})")
                return True
            try:
                if w is not None:
                    w.release()
            except Exception:
                pass
        return False

    def _apply_quality(self):
        """Ask for the quality and RECORD whether it took (mp4v ignores it)."""
        if not _CV2 or self._w is None:
            return
        try:
            prop = getattr(cv2, "VIDEOWRITER_PROP_QUALITY", None)
            if prop is None:
                return
            self._w.set(prop, float(self.quality))
            got = float(self._w.get(prop))
            self.quality_applied = abs(got - self.quality) < 5.0
        except Exception:
            self.quality_applied = False

    def add(self, frame, repeats: int = 1) -> bool:
        """Write ``repeats`` copies of ``frame``. False if the size mismatched.

        A frame whose size differs from the first one is REFUSED rather than
        written: cv2 writes garbage for a mismatched size, and a resolution
        change mid-record must end the file cleanly, not corrupt it.
        """
        if self._w is None or frame is None:
            return False
        h, w = frame.shape[:2]
        if (int(w), int(h)) != self.size:
            return False
        for _ in range(max(0, int(repeats))):
            self._w.write(frame)
            self.frames += 1
        return True

    def close(self, *, reason: str = "") -> VideoResult:
        """Release and judge the result. A zero-frame file is DELETED.

        Leaving a 0-byte .mp4 on disk that looks like a recording is worse than
        reporting the failure.
        """
        if self._w is not None:
            try:
                self._w.release()
            except Exception:
                pass
            self._w = None
        size = 0
        try:
            size = self.path.stat().st_size
        except OSError:
            pass
        res = VideoResult(
            path=self.path, frames=self.frames,
            duration_s=self.frames / self.fps if self.fps else 0.0,
            fourcc=self.fourcc, bytes=size, reason=reason,
            container_substituted=self.container_substituted,
            quality_applied=self.quality_applied)
        if self.frames <= 0 or size < MIN_USABLE_BYTES:
            try:
                self.path.unlink()
            except OSError:
                pass
            res.ok = False
            res.path = None
            res.reason = reason or (
                "no frames were recorded" if self.frames <= 0
                else "the encoder produced an empty file")
        else:
            res.ok = True
        return res


class RawFrameSequenceWriter:
    """Raw 16-bit recording: numbered TIFFs plus a manifest.

    cv2 video is 8-bit only, so "raw video" cannot be a movie file. And the
    raw path in this app is a ONE-SHOT averaged request serviced by the
    backend's reader thread — looping it captures some frames and misses the
    rest — so this is presented honestly as a **time-lapse**: every saved frame
    is real and carries its own wall-clock timestamp, and nothing claims the
    sequence is continuous.
    """

    def __init__(self, directory: Path, meta=None, *, interval_s: float = 1.0):
        self.dir = Path(directory)
        self.meta = meta
        self.interval_s = max(0.0, float(interval_s))
        self.frames = 0
        self._records: list = []

    def add(self, array, *, t_wall: float, t_mono: float) -> bool:
        from SupportClasses.CaptureImageWriter import write_image
        self.dir.mkdir(parents=True, exist_ok=True)
        self.frames += 1
        name = f"frame_{self.frames:06d}.tif"
        try:
            write_image(self.dir / name, array, self.meta,
                        embed=(self.frames == 1), sidecar=False)
        except Exception as exc:
            logger.warning(f"raw time-lapse frame {self.frames} failed: {exc}")
            self.frames -= 1
            return False
        self._records.append({"i": self.frames, "file": name,
                              "t_wall": t_wall, "t_mono": t_mono})
        return True

    def close(self, *, reason: str = "") -> VideoResult:
        if self.frames <= 0:
            try:
                self.dir.rmdir()
            except OSError:
                pass
            return VideoResult(ok=False, reason=reason or "no frames captured")
        span = 0.0
        if len(self._records) > 1:
            span = self._records[-1]["t_mono"] - self._records[0]["t_mono"]
        manifest = {
            "version": "1.0",
            "kind": "raw_timelapse",
            "interval_s": self.interval_s,
            "frames": self._records,
        }
        if self.meta is not None:
            try:
                from SupportClasses.CaptureMetadata import to_json
                manifest["meta"] = json.loads(to_json(self.meta))
            except Exception:
                pass
        (self.dir / "manifest.json").write_text(
            json.dumps(manifest, indent=2), encoding="utf-8")
        total = sum(f.stat().st_size for f in self.dir.glob("*.tif"))
        return VideoResult(path=self.dir, ok=True, frames=self.frames,
                           duration_s=span, bytes=total, reason=reason,
                           fourcc="tiff-sequence")


def enough_disk_space(directory: Path, projected_bytes: float,
                      *, headroom: float = 2.0) -> tuple[bool, str]:
    """Refuse a recording that plainly will not fit (with headroom)."""
    try:
        free = shutil.disk_usage(str(Path(directory).parent
                                     if not Path(directory).exists()
                                     else directory)).free
    except OSError:
        return True, ""      # unknowable → do not block the operator
    need = float(projected_bytes) * float(headroom)
    if free >= need:
        return True, ""
    return False, (f"not enough free space: {free / 1e9:.1f} GB available, "
                   f"~{need / 1e9:.1f} GB needed for this recording")
