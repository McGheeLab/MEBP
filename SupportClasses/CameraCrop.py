"""Camera frame cropping — a square (or tighter) centre crop, applied at the
frame SOURCE so every surface sees the same pixels.

v7.16 — operator: *"it turns out that the image sensor is too wide, and we see
some of the dark circles of the field of view. can we crop the image into a
square in the camera calibration part of the camera detection settings page.
this cropping should be assigned to all surfaces including mosaic, live view,
image, recording etc."*

A microscope's illuminated field is a CIRCLE. A sensor wider than that circle
images the unlit tube wall at the left/right edges — those pixels carry no
specimen, they drag the mosaic's flat-field estimate toward black, and the
raster still steps by their width, so they cost tiles as well as quality.

WHY THE CROP LIVES AT THE SOURCE
--------------------------------
This is a GEOMETRIC change to the field of view, not a display preference like
brightness or gamma. If it were applied per consumer, the raw frame cache, the
mosaic tile, the detection ROI and the click→stage map would each have to agree
about it independently — four chances to disagree about how many pixels a frame
has. Cropping once, where the frame enters the application, makes every
downstream surface (live view, mosaic, still capture, video recording,
detection, autofocus, calibration) inherit it with no code of their own.

The trade-off, stated plainly: the discarded pixels are GONE, so unlike the
mirror/rotation (which are applied per consumer precisely so the mosaic can
re-blend against a *changing* orientation) a crop cannot be undone after the
fact. That is correct here — a vignetted edge has no information to preserve.

WHY A FRACTION, NOT ABSOLUTE PIXELS
-----------------------------------
``scale`` is a fraction of the short side, so the crop describes a physical
region of the field and survives a capture-resolution change. Storing pixels
would silently mean a different physical area after a 2x2 binning switch --
the same class of stale-number bug as an unstamped µm/px resolution.

WHERE THE CROP SITS, AND WHY THAT COSTS SOMETHING
-------------------------------------------------
The illuminated circle is centred on the OPTICAL AXIS, which need not pass
through the middle of the sensor — a C-mount adapter or a slightly off-axis tube
lens puts it elsewhere, and then a centred square still clips one side. So the
crop can be offset (``offset_x`` / ``offset_y``, fractions of the frame).

That is not free. ``CameraManager.pixel_to_stage_offset`` maps a click by its
offset from the frame CENTRE, so moving the crop off-centre moves the pixel that
means "the stage is here" — and every click, every overlay and every mosaic tile
placement inherits that shift (up to ~1 mm at a small crop on this rig). It is
therefore **compensated**, not merely documented: ``center_offset_px`` reports
the displacement and the three places that convert pixel↔stage add it back, so
an offset crop maps to exactly the same stage coordinates a full frame would.
The operator can re-aim the crop without invalidating a taught plate.

``center_offset_px`` is derived from the ACTUAL rect, never from the requested
fraction, so a clamped offset cannot silently disagree with what is compensated
for. At zero offset the centre is preserved to the pixel: ``rect_for`` matches
the crop's parity to the frame's.

µm/px IS UNCHANGED BY A CROP
----------------------------
Cropping removes pixels; it does not change what one pixel spans. Any code that
rescales a stored µm/px between resolutions (``µm/px ∝ 1/width``) must therefore
compare CAPTURE widths, never the delivered width -- see
``CameraManager.capture_width``. Getting that wrong would rescale by the crop
fraction (27 % on this rig's Tucsen) in the direction that opens mosaic gaps.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Optional

logger = logging.getLogger(__name__)

MODE_NONE = "none"
MODE_SQUARE = "square"
VALID_MODES = (MODE_NONE, MODE_SQUARE)

#: Smallest crop the model will produce. A frame below this is unusable for
#: registration and detection, so an absurd ``scale`` is clamped rather than
#: allowed to deliver a handful of pixels.
MIN_SIDE_PX = 16

def _clamped(value, lo: float, hi: float, default: float) -> float:
    """``value`` as a float clamped to [lo, hi]; ``default`` for junk or NaN."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return default
    if v != v:                                  # NaN
        return default
    return max(lo, min(hi, v))


MIN_SCALE = 0.10
MAX_SCALE = 1.0

#: Offset range, as a fraction of the frame's width / height. Half the frame in
#: either direction is more than any real optical misalignment; the crop is
#: additionally clamped to stay inside the frame, so the usable range is
#: whatever slack the chosen ``scale`` leaves.
MAX_OFFSET = 0.5


@dataclass(frozen=True)
class CameraCrop:
    """A crop of the camera frame — square by default, positionable.

    ``mode``     -- ``"none"`` (no crop) or ``"square"``.
    ``scale``    -- fraction of the SHORT side to keep, 0.10..1.0. 1.0 is the
                    largest square that fits; lower it when the illuminated
                    circle is smaller than the sensor's short side and the
                    corners are still dark.
    ``offset_x`` -- horizontal placement, as a fraction of the frame WIDTH.
                    0 = centred, + = toward +x (right in raw frame coords).
    ``offset_y`` -- vertical placement, as a fraction of the frame HEIGHT.

    The offsets exist because the illuminated circle is centred on the optical
    axis, which need not be the middle of the sensor. They are clamped to keep
    the crop inside the frame, and the resulting displacement is reported by
    :meth:`center_offset_px` so the pixel↔stage conversions can cancel it.
    """

    mode: str = MODE_NONE
    scale: float = 1.0
    offset_x: float = 0.0
    offset_y: float = 0.0

    def __post_init__(self):
        mode = str(self.mode or MODE_NONE).strip().lower()
        if mode not in VALID_MODES:
            mode = MODE_NONE
        object.__setattr__(self, "mode", mode)
        object.__setattr__(self, "scale", _clamped(
            self.scale, MIN_SCALE, MAX_SCALE, 1.0))
        object.__setattr__(self, "offset_x", _clamped(
            self.offset_x, -MAX_OFFSET, MAX_OFFSET, 0.0))
        object.__setattr__(self, "offset_y", _clamped(
            self.offset_y, -MAX_OFFSET, MAX_OFFSET, 0.0))

    # ── Queries ──────────────────────────────────────────────────────

    @property
    def enabled(self) -> bool:
        """True when this crop asks for anything at all."""
        return self.mode != MODE_NONE

    def is_active_for(self, width: int, height: int) -> bool:
        """True when applying this crop to a ``width`` x ``height`` frame would
        actually remove pixels. A full-scale square crop of an already-square
        frame is a no-op, and reporting that honestly keeps the fast paths
        (and the operator's readout) truthful."""
        rect = self.rect_for(width, height)
        if rect is None:
            return False
        x0, y0, cw, ch = rect
        return not (x0 == 0 and y0 == 0 and cw == int(width)
                    and ch == int(height))

    def rect_for(self, width: int, height: int
                 ) -> Optional[tuple[int, int, int, int]]:
        """``(x0, y0, w, h)`` of the crop within a ``width`` x ``height`` frame,
        or None when no crop applies (disabled, or a degenerate frame).

        At zero offset the box is exactly centred: its width/height share the
        frame's parity, so the optical centre stays at the centre to the pixel.
        A non-zero offset shifts it by a whole number of pixels and is CLAMPED to
        keep the box inside the frame — a crop reaching past the sensor edge
        would deliver undefined pixels, and silently sliding it back without
        saying so is why :meth:`center_offset_px` reads the achieved rect rather
        than the requested fraction.
        """
        try:
            w = int(width)
            h = int(height)
        except (TypeError, ValueError):
            return None
        if w <= 0 or h <= 0 or not self.enabled:
            return None

        side = int(round(min(w, h) * self.scale))
        side = max(MIN_SIDE_PX, min(side, min(w, h)))

        cw = self._match_parity(side, w)
        ch = self._match_parity(side, h)
        x0 = (w - cw) // 2 + int(round(self.offset_x * w))
        y0 = (h - ch) // 2 + int(round(self.offset_y * h))
        # Clamp into the frame. max(0, ...) second so a crop as large as the
        # frame lands at 0 rather than negative.
        x0 = max(0, min(x0, w - cw))
        y0 = max(0, min(y0, h - ch))
        return (x0, y0, cw, ch)

    def center_offset_px(self, width: int, height: int) -> tuple[float, float]:
        """Displacement of the crop's centre from the FRAME's centre, in pixels.

        THE quantity the pixel↔stage conversions cancel. Positive x means the
        crop sits toward +x, so a feature at the cropped frame's centre is
        really ``+x`` pixels from where the stage is pointing.

        Derived from :meth:`rect_for` — the achieved box — so a clamped or
        rounded offset is compensated for exactly as it was applied. Deriving it
        from ``offset_x`` instead would drift from reality at the clamp, which is
        the one place it matters most.
        """
        rect = self.rect_for(width, height)
        if rect is None:
            return (0.0, 0.0)
        x0, y0, cw, ch = rect
        return (float(x0) + cw / 2.0 - float(width) / 2.0,
                float(y0) + ch / 2.0 - float(height) / 2.0)

    def reference_pixel(self, width: int, height: int) -> tuple[float, float]:
        """Where the stage's own position lands in the CROPPED frame.

        The frame centre for a centred crop; displaced by ``-center_offset_px``
        once the crop is moved. This is where a crosshair belongs — drawing it
        at the geometric middle of an offset crop would aim the operator at a
        point the stage is not on.
        """
        rect = self.rect_for(width, height)
        if rect is None:
            return (float(width) / 2.0, float(height) / 2.0)
        _x0, _y0, cw, ch = rect
        dx, dy = self.center_offset_px(width, height)
        return (cw / 2.0 - dx, ch / 2.0 - dy)

    @staticmethod
    def _match_parity(side: int, full: int) -> int:
        """``side`` nudged so ``full - side`` is even (an exactly centred box),
        without ever exceeding ``full`` or dropping below the floor."""
        if (full - side) % 2 == 0:
            return side
        if side + 1 <= full:
            return side + 1
        if side - 1 >= MIN_SIDE_PX:
            return side - 1
        return side

    def size_for(self, width: int, height: int) -> tuple[int, int]:
        """Delivered ``(w, h)`` after cropping a ``width`` x ``height`` frame."""
        rect = self.rect_for(width, height)
        if rect is None:
            return (int(width), int(height))
        return (rect[2], rect[3])

    # ── Application ──────────────────────────────────────────────────

    def apply(self, frame):
        """Return ``frame`` cropped, or ``frame`` itself when no crop applies.

        Works for 2-D (raw mono, uint16) and 3-D (BGR) arrays alike -- only the
        first two axes are indexed, so a raw still and a display frame are
        cropped identically. Returns a VIEW, not a copy: callers that already
        copy (``get_current_frame``, ``capture_fresh_frame``) keep doing so, and
        callers that hand the array straight to cv2 are unaffected.
        """
        if frame is None or not self.enabled:
            return frame
        shape = getattr(frame, "shape", None)
        if not shape or len(shape) < 2:
            return frame
        rect = self.rect_for(int(shape[1]), int(shape[0]))
        if rect is None:
            return frame
        x0, y0, cw, ch = rect
        if x0 == 0 and y0 == 0 and cw == int(shape[1]) and ch == int(shape[0]):
            return frame
        return frame[y0:y0 + ch, x0:x0 + cw]

    # ── Persistence ──────────────────────────────────────────────────

    def to_dict(self) -> dict:
        d = {"mode": self.mode, "scale": round(float(self.scale), 4)}
        # Offsets emitted only when non-zero, so a centred crop's stored entry
        # stays exactly what pre-offset builds wrote.
        if abs(self.offset_x) > 1e-9:
            d["offset_x"] = round(float(self.offset_x), 5)
        if abs(self.offset_y) > 1e-9:
            d["offset_y"] = round(float(self.offset_y), 5)
        return d

    @classmethod
    def from_dict(cls, data: Any) -> "CameraCrop":
        """Tolerant reader — anything unrecognised means "no crop".

        A malformed entry must not crop by accident: an unexpected crop is a
        silently wrong field of view, where no crop is merely the old behaviour.
        A missing offset means centred, which is what every pre-offset entry is.
        """
        if isinstance(data, CameraCrop):
            return data
        if not isinstance(data, dict):
            return cls()
        return cls(mode=data.get("mode", MODE_NONE),
                   scale=data.get("scale", 1.0),
                   offset_x=data.get("offset_x", 0.0),
                   offset_y=data.get("offset_y", 0.0))

    def describe(self, width: int = 0, height: int = 0) -> str:
        """Short human summary for a readout line."""
        if not self.enabled:
            return "off (full sensor)"
        pct = f"{self.scale * 100:.0f}%"
        off = ""
        if abs(self.offset_x) > 1e-9 or abs(self.offset_y) > 1e-9:
            off = (f", offset {self.offset_x * 100:+.0f}%/"
                   f"{self.offset_y * 100:+.0f}%")
        if width and height:
            cw, ch = self.size_for(width, height)
            return (f"square {pct}{off} — {cw}x{ch} of "
                    f"{int(width)}x{int(height)}")
        return f"square {pct}{off} of the short side"


#: The neutral crop — module-level so callers can compare identity cheaply.
NO_CROP = CameraCrop()
