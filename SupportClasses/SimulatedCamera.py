"""
SimulatedCamera — Synthetic microscope frame generator for MEBP v7.3.0.

Generates BGR camera frames that simulate an inverted microscope view:
  - Black plate surface with red well holes (transmitted light from below)
  - Needle tip as a dark circle, with visibility and blur governed by a
    Gaussian depth-of-field (DOF) envelope per objective magnification
  - Wells translate as the XY stage moves

Focus model:
  Each objective has a characteristic DOF half-width (mm). The needle's
  visibility follows: opacity = exp(-0.5 * (defocus / dof_hw)^2).
  When defocus >> dof_hw, the needle is completely invisible.

  Typical DOF half-widths (approximated from NA / magnification):
    2×  → ~0.20 mm     10× → ~0.04 mm
    4×  → ~0.10 mm     20× → ~0.02 mm

Usage::

    sim = SimulatedCamera(
        resolution=(916, 686),
        micron_per_pixel=1.67,   # 4× objective
        plate=WellPlate.from_format(96),
        plate_origin_um=(50000.0, 50000.0),
    )
    sim.set_stage_position(50000.0, 50000.0, z_mm=0.0)
    frame = sim.generate_frame()
"""

from __future__ import annotations

import logging
import math

import cv2
import numpy as np

logger = logging.getLogger(__name__)

try:
    from SupportClasses.WellPlate import WellPlate
except ImportError:
    WellPlate = None


class SimulatedCamera:
    """
    Generates synthetic microscope frames based on virtual world state.

    Simulates an inverted microscope: camera looks up through the objective,
    well plate sits above, needle descends from above into the FOV.

    The XY stage moves the camera. Moving stage right → camera center moves
    right → you see more of the world to the right.
    """

    # Well rendering — bright red with enough luminance for edge detection
    # Grayscale ≈ 125, giving gradient ~115 against background (passes Canny)
    WELL_COLOR_BGR = (80, 80, 230)     # Bright red in BGR
    WELL_EDGE_BGR = (40, 40, 180)      # Darker red well wall ring
    WELL_EDGE_WIDTH = 2                # Edge ring thickness in pixels
    BACKGROUND_BGR = (10, 10, 10)      # Near-black plate surface

    # Needle rendering
    NEEDLE_COLOR_BGR = (0, 0, 0)       # Black (blocks transmitted light)
    NEEDLE_RING_BGR = (40, 40, 40)     # Subtle gray ring at sharp focus

    # Needle visibility threshold — below this opacity, skip drawing entirely
    VISIBILITY_CUTOFF = 0.01

    def __init__(
        self,
        resolution: tuple[int, int] = (916, 686),
        micron_per_pixel: float = 3.34,
        plate: WellPlate | None = None,
        plate_origin_um: tuple[float, float] | None = None,
        plate_center_um: tuple[float, float] | None = None,
        needle_od_um: float = 910.0,
        needle_id_um: float = 0.0,
        focal_z_mm: float = 0.0,
        dof_halfwidth_mm: float | None = None,
    ):
        """
        Args:
            resolution: Frame (width, height) in pixels.
            micron_per_pixel: µm per pixel at current magnification.
            plate: WellPlate geometry object (None = no wells drawn).
            plate_origin_um: (x, y) µm — absolute stage position of well A1.
                             If None and plate_center_um is given, A1 is
                             auto-computed from plate center using ANSI/SLAS geometry.
            plate_center_um: (x, y) µm — plate center position. Used to
                             auto-compute A1 when plate_origin_um is None.
                             Defaults to stage center (65000, 42500) based on
                             130×85mm travel.
            needle_od_um: Needle outer diameter in µm.
            needle_id_um: Needle inner diameter in µm (0 = solid).
            focal_z_mm: Z position (mm, zero-ref) where needle is in focus.
            dof_halfwidth_mm: Depth-of-field half-width in mm. If None,
                computed from micron_per_pixel (objective magnification).
        """
        self._width, self._height = resolution
        self._um_per_px = micron_per_pixel
        self._plate = plate
        self._needle_od_um = needle_od_um
        self._needle_id_um = needle_id_um
        self._focal_z_mm = focal_z_mm

        # Resolve plate origin: explicit A1 > auto from center > legacy default
        if plate_origin_um is not None:
            self._plate_origin_um = plate_origin_um
        elif plate is not None:
            center = plate_center_um or (65000.0, 42500.0)
            self._plate_origin_um = plate.get_a1_from_plate_center(*center)
        else:
            self._plate_origin_um = plate_center_um or (65000.0, 42500.0)

        # Depth of field: either explicit or derived from pixel scale
        if dof_halfwidth_mm is not None:
            self._dof_hw_mm = dof_halfwidth_mm
        else:
            self._dof_hw_mm = self._compute_dof_halfwidth(micron_per_pixel)

        # Current state (updated externally or via controller)
        self._stage_x_um = self._plate_origin_um[0]
        self._stage_y_um = self._plate_origin_um[1]
        self._z_mm = 0.0

        # Show needle in frame (can be toggled)
        self._show_needle = True

        # Controller reference for auto-position pull
        self._controller = None

        # FOV in µm
        self._fov_w_um = self._width * self._um_per_px
        self._fov_h_um = self._height * self._um_per_px

    # ── DOF Model ──────────────────────────────────────────────

    @staticmethod
    def _compute_dof_halfwidth(um_per_px: float) -> float:
        """
        Estimate DOF half-width (mm) from pixel scale.

        Higher magnification (smaller µm/px) → narrower DOF.
        Approximation: dof_hw ≈ um_per_px * 0.06 mm.

        Results:
            2×  (3.34 µm/px) → 0.200 mm
            4×  (1.67 µm/px) → 0.100 mm
            10× (0.668 µm/px) → 0.040 mm
            20× (0.334 µm/px) → 0.020 mm
        """
        return um_per_px * 0.06

    def _needle_visibility(self, defocus_mm: float) -> float:
        """
        Gaussian DOF envelope: returns 0.0–1.0 opacity for the needle.

        At defocus == 0 → 1.0 (perfectly sharp).
        At defocus >> dof_hw → ~0.0 (invisible).
        """
        if self._dof_hw_mm <= 0:
            return 1.0
        return math.exp(-0.5 * (defocus_mm / self._dof_hw_mm) ** 2)

    # Blur scaling constant (µm). At 1 DOF half-width of defocus, the
    # Gaussian blur sigma in pixels equals BLUR_REFERENCE_UM / um_per_px.
    # 15 µm gives moderate but detectable blur at 1 DOF across all objectives:
    #   2×  → ~4.5 px    10× → ~22 px
    #   4×  → ~9 px      20× → ~45 px
    BLUR_REFERENCE_UM = 15.0

    def _needle_blur_sigma(self, defocus_mm: float) -> float:
        """
        Compute Gaussian blur sigma (in pixels) for the needle.

        Blur scales with the optical system (magnification), not the object
        size. Higher magnification amplifies the defocus blur disk.

        At 1 DOF half-width of defocus, sigma ≈ BLUR_REFERENCE_UM / um_per_px.
        """
        if self._dof_hw_mm <= 0:
            return 0.0
        normalized_defocus = abs(defocus_mm) / self._dof_hw_mm
        # Blur per DOF: scales with magnification (1/um_per_px)
        base_blur_px = self.BLUR_REFERENCE_UM / self._um_per_px
        return normalized_defocus * base_blur_px

    # ── Configuration ──────────────────────────────────────────

    def set_plate(
        self,
        plate,
        plate_origin_um: tuple[float, float] | None = None,
        plate_center_um: tuple[float, float] | None = None,
    ):
        """Set or change the well plate geometry.

        Args:
            plate: WellPlate geometry object.
            plate_origin_um: Explicit A1 position (µm). Takes priority.
            plate_center_um: Plate center (µm). A1 is auto-computed.
        """
        self._plate = plate
        if plate_origin_um is not None:
            self._plate_origin_um = plate_origin_um
        elif plate is not None and plate_center_um is not None:
            self._plate_origin_um = plate.get_a1_from_plate_center(*plate_center_um)

    def set_needle(self, od_um: float, id_um: float = 0.0):
        """Set needle dimensions."""
        self._needle_od_um = od_um
        self._needle_id_um = id_um

    def set_focal_z(self, focal_z_mm: float):
        """Set the Z position of perfect focus."""
        self._focal_z_mm = focal_z_mm

    def set_dof_halfwidth(self, dof_hw_mm: float):
        """Explicitly set the DOF half-width (mm)."""
        self._dof_hw_mm = max(0.001, dof_hw_mm)

    def set_stage_position(self, x_um: float, y_um: float, z_mm: float = 0.0):
        """Set current stage position (XY in µm, Z in mm)."""
        self._stage_x_um = x_um
        self._stage_y_um = y_um
        self._z_mm = z_mm

    def set_controller(self, controller):
        """Set StageController reference for auto position pull."""
        self._controller = controller

    @property
    def show_needle(self) -> bool:
        return self._show_needle

    @show_needle.setter
    def show_needle(self, value: bool):
        self._show_needle = value

    @property
    def dof_halfwidth_mm(self) -> float:
        return self._dof_hw_mm

    # ── Coordinate Mapping ─────────────────────────────────────

    def _world_to_pixel(self, world_x_um: float, world_y_um: float) -> tuple[float, float]:
        """Convert world coordinate (µm) to pixel position in frame."""
        dx_um = world_x_um - self._stage_x_um
        dy_um = world_y_um - self._stage_y_um
        px = self._width / 2.0 + dx_um / self._um_per_px
        py = self._height / 2.0 + dy_um / self._um_per_px
        return (px, py)

    def _um_to_px(self, distance_um: float) -> float:
        """Convert a distance in µm to pixels."""
        return distance_um / self._um_per_px

    # ── Frame Generation ───────────────────────────────────────

    def _pull_position(self, cached=True):
        """Pull current position from controller if available.

        Args:
            cached: If False, forces a fresh position read from hardware.
        """
        if self._controller is None:
            return
        try:
            xy = self._controller.get_xy_position(cached=cached)
            if xy[0] is not None:
                self._stage_x_um = float(xy[0])
                self._stage_y_um = float(xy[1])
        except Exception:
            pass
        try:
            zp = self._controller.get_zp_position(cached=cached)
            if isinstance(zp, (list, tuple)) and len(zp) >= 1 and zp[0] is not None:
                zero_z = 0.0
                if hasattr(self._controller, 'zero_position'):
                    zero_z = self._controller.zero_position.get("Z", 0.0)
                self._z_mm = float(zp[0]) - zero_z
        except Exception:
            pass

    def generate_frame(self) -> np.ndarray:
        """
        Generate a single BGR frame for the current world state.

        Returns:
            BGR numpy array of shape (height, width, 3), dtype uint8.
        """
        self._pull_position()

        # Start with black plate surface
        frame = np.full(
            (self._height, self._width, 3),
            self.BACKGROUND_BGR,
            dtype=np.uint8,
        )

        # Draw wells (red holes in the black plate)
        if self._plate is not None:
            self._draw_wells(frame)

        # Draw needle with DOF-based visibility
        if self._show_needle:
            defocus_mm = abs(self._z_mm - self._focal_z_mm)
            opacity = self._needle_visibility(defocus_mm)
            if opacity >= self.VISIBILITY_CUTOFF:
                self._draw_needle(frame, defocus_mm, opacity)

        return frame

    def _draw_wells(self, frame: np.ndarray) -> None:
        """Draw all visible wells onto the frame as red holes in the plate."""
        plate = self._plate
        ox, oy = self._plate_origin_um

        # FOV bounds in world µm (with margin for partially visible wells)
        half_w = self._fov_w_um / 2.0
        half_h = self._fov_h_um / 2.0
        fov_left = self._stage_x_um - half_w
        fov_right = self._stage_x_um + half_w
        fov_top = self._stage_y_um - half_h
        fov_bottom = self._stage_y_um + half_h

        well_radius_um = plate.well_diameter * 1000.0 / 2.0
        well_radius_px = self._um_to_px(well_radius_um)

        for well in plate.get_all_wells():
            # Well absolute position in µm
            wx_um = ox + well.x * 1000.0
            wy_um = oy + well.y * 1000.0

            # Bounds check: skip wells that can't overlap the FOV
            if (wx_um + well_radius_um < fov_left or
                wx_um - well_radius_um > fov_right or
                wy_um + well_radius_um < fov_top or
                wy_um - well_radius_um > fov_bottom):
                continue

            # Convert to pixel coordinates
            px, py = self._world_to_pixel(wx_um, wy_um)
            cx, cy = int(round(px)), int(round(py))
            r = max(1, int(round(well_radius_px)))

            # Filled red circle (well hole showing illumination from below)
            cv2.circle(frame, (cx, cy), r, self.WELL_COLOR_BGR, -1)
            # Edge ring (well wall shadow)
            cv2.circle(frame, (cx, cy), r, self.WELL_EDGE_BGR, self.WELL_EDGE_WIDTH)

    def _draw_needle(self, frame: np.ndarray, defocus_mm: float,
                     opacity: float) -> None:
        """Draw the needle tip at frame center with DOF-based blur and opacity."""
        cx = self._width // 2
        cy = self._height // 2

        od_px = max(1, int(round(self._um_to_px(self._needle_od_um))))
        id_px = max(0, int(round(self._um_to_px(self._needle_id_um))))
        od_r = od_px // 2
        id_r = id_px // 2

        blur_sigma = self._needle_blur_sigma(defocus_mm)

        # Draw needle shape onto a temporary alpha layer
        needle_layer = np.zeros((self._height, self._width), dtype=np.uint8)

        if id_r > 0 and id_r < od_r:
            # Hollow needle: dark annulus (lumen is transparent)
            cv2.circle(needle_layer, (cx, cy), od_r, 255, -1)
            cv2.circle(needle_layer, (cx, cy), id_r, 0, -1)
        else:
            # Solid needle tip
            cv2.circle(needle_layer, (cx, cy), od_r, 255, -1)

        # Apply defocus blur to the needle mask
        if blur_sigma > 0.5:
            ksize = int(blur_sigma * 6) | 1  # Must be odd
            ksize = max(3, min(ksize, 501))
            needle_layer = cv2.GaussianBlur(
                needle_layer, (ksize, ksize), blur_sigma)

        # Scale mask by the DOF opacity envelope
        alpha = needle_layer.astype(np.float32) * (opacity / 255.0)

        # Composite: blend frame toward NEEDLE_COLOR where alpha > 0
        for c in range(3):
            channel = frame[:, :, c].astype(np.float32)
            frame[:, :, c] = (
                channel * (1.0 - alpha) + self.NEEDLE_COLOR_BGR[c] * alpha
            ).astype(np.uint8)

        # Draw subtle gray ring at outer edge only when mostly in focus
        if opacity > 0.5 and blur_sigma < 3.0:
            cv2.circle(frame, (cx, cy), od_r, self.NEEDLE_RING_BGR, 1)
            if id_r > 0:
                cv2.circle(frame, (cx, cy), id_r, self.NEEDLE_RING_BGR, 1)

    # ── cv2.VideoCapture-Compatible API ────────────────────────

    def read(self) -> tuple[bool, np.ndarray]:
        """Generate and return the next frame (VideoCapture API)."""
        return True, self.generate_frame()

    def read_fresh(self) -> tuple[bool, np.ndarray]:
        """Generate a frame with a fresh (uncached) position read.

        Use this after stage movement to ensure the frame reflects
        the current hardware position, not stale cached data.
        """
        self._pull_position(cached=False)
        return True, self.generate_frame()

    def isOpened(self) -> bool:
        """Always ready."""
        return True

    def release(self) -> None:
        """No resources to release."""
        pass

    def get(self, prop_id: int) -> float:
        """Mimic cv2.VideoCapture.get() for common properties."""
        if prop_id == cv2.CAP_PROP_FRAME_WIDTH:
            return float(self._width)
        elif prop_id == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(self._height)
        elif prop_id == cv2.CAP_PROP_FPS:
            return 30.0
        return 0.0
