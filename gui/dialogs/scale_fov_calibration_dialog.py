"""
scale_fov_calibration_dialog.py — stage-motion scale + FOV auto-calibration.

v7.5.x (operator): "the camera setup should do an auto calibration of the pixel
to micron conversion based on the stage motion … and we should be able to
calculate the extents of the camera in microns by looking at the features near
the edge and moving to those features to do an auto registration and alignment.
This should all check out with the known pixels incoming from the camera and the
micron to pixel space."

Unlike ``PixelCalibrationDialog`` (one small move + whole-frame phase
correlation, which loses lock once the content shifts more than the overlap),
this dialog picks a textured FEATURE and moves the stage a LARGE, known baseline
so the feature crosses much of the frame, then re-finds it with template
matching (``VisionDetector.find_template``, TM_CCOEFF_NORMED). A big pixel
baseline makes µm/px far more accurate, and it directly yields the camera's
field-of-view EXTENT in microns:

    µm/px      = move_distance_µm / feature_pixel_displacement
    FOV_w_µm   = frame_width_px  × µm/px      (the extent, cross-checked)
    FOV_h_µm   = frame_height_px × µm/px

Everything is anchored to the camera's ACTUAL captured resolution
(``frame.shape``), not an assumed one, so the value the mosaic later scales by
matches the real pixels coming off the camera. A move along +X and a move along
+Y give an independent µm/px per axis (they agree for square pixels) plus the
camera's in-plane rotation vs the stage.

Results (read after ``exec()`` returns ``Accepted``):
    result_um_per_px   : float   — mean µm/px
    result_rotation_deg: float|None — camera in-plane rotation vs stage (deg)
    result_resolution  : (w, h)  — the captured resolution the value holds at
    result_fov_um      : (w, h)  — field-of-view extent in microns

Display-only mirror control: correcting the view never touches the raw frame the
measurement runs on, so it can't corrupt the result.

Requires a running camera and a connected stage controller.
"""

from __future__ import annotations

import logging
import math
from enum import Enum, auto

import numpy as np

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLabel, QDoubleSpinBox, QPushButton, QCheckBox,
    QDialogButtonBox, QGroupBox, QMessageBox,
)

from gui.styles import COLORS
from gui.scaling import s, scaled_font_size

logger = logging.getLogger(__name__)

try:
    from SupportClasses.VisionDetector import (
        select_trackable_patch, find_template)
    VISION_AVAILABLE = True
except ImportError:
    VISION_AVAILABLE = False
    logger.warning("VisionDetector unavailable — scale/FOV calibration disabled")

try:
    from gui.dialogs.pixel_calibration_dialog import plus_column_direction_deg
except Exception:
    def plus_column_direction_deg(commanded_deg, dx, dy=0.0):  # fallback
        phi = math.degrees(math.atan2(dy, dx))
        return ((float(commanded_deg) - phi - 180.0 + 180.0) % 360.0) - 180.0


def derive_camera_stage_orientation(dxX, dyX, dxY, dyY):
    """Decompose the camera→stage orientation from a two-axis stage move.

    v7.5.x (operator: "the placement of the images is not on the correct side …
    the +X direction on the stage is not the same as the camera's +X direction,
    and same for Y"). ``plus_column_direction_deg`` models the camera as a pure
    rotation and **explicitly assumes a non-mirrored image**, so a mirrored (or
    180°-mounted) microscope can never be detected from motion — the operator is
    forced to guess flip-X/flip-Y by eye and lands "upside down". The two
    measured content displacements DO encode the full 2×2 relationship,
    including **handedness** (a mirror), in the sign of their cross product.

    ``(dxX, dyX)`` / ``(dxY, dyY)`` are the tracked-feature image displacements
    (px) for a **+X** and a **+Y** stage move. Returns the canonical
    ``(rotation_deg, flip_x, flip_y)`` such that
    ``MosaicBuilder._orient_tile`` / ``CameraManager.pixel_to_stage_offset``
    (both ``R(θ)·diag(mx, my)``, mx=−1 iff flip_x, my=−1 iff flip_y) orient a raw
    tile into the STAGE frame — so the mosaic stitches AND back-projects
    correctly, and a live-view click maps to the right XY.

    Derivation (image→stage direction ``M = −(d/µmpx)·P⁻¹``, ``P`` = the columns
    of the two content displacements): ``sign(det M) == sign(det P)``, and
    ``det`` = ``mx·my`` ⇒ a NEGATIVE cross product means an odd number of flips
    (a genuine mirror). Canonically fix ``mx=+1`` and put the handedness on
    ``my`` (``flip_y = det < 0``); ``θ`` is the direction of ``M``'s first
    column. For a non-mirrored camera this reduces EXACTLY to
    ``plus_column_direction_deg`` (no regression); a mirror-X camera comes out as
    ``(θ=180°, flip_y=True)`` (≡ flip-X), a 180° mount as ``(θ=180°, no flips)``.
    Returns None when the two moves are collinear (degenerate — can't solve).
    Signs locked by ``test_derive_camera_stage_orientation`` against
    ``_orient_tile``.
    """
    det_p = dxX * dyY - dxY * dyX          # == cross product; sign = handedness
    if abs(det_p) < 1e-9:
        return None
    c = -1.0 / det_p                       # sign carrier (magnitude irrelevant)
    m00 = c * dyY
    m10 = c * (-dyX)
    theta = math.degrees(math.atan2(m10, m00))
    theta = ((theta + 180.0) % 360.0) - 180.0
    if theta == -180.0:
        theta = 180.0
    flip_x = False
    flip_y = det_p < 0.0                    # odd handedness → one axis mirrored
    return theta, flip_x, flip_y

try:
    from gui.widgets.camera_feed_view import CameraFeedView
    FEED_AVAILABLE = True
except ImportError:
    CameraFeedView = None
    FEED_AVAILABLE = False

# Minimum template-match confidence (TM_CCOEFF_NORMED peak) for a trusted track.
_MIN_TRACK_CONF = 0.35
# Minimum pixel displacement for a meaningful baseline.
_MIN_DISP_PX = 20.0


class _S(Enum):
    READY = auto()
    BEFORE_X = auto()
    AFTER_X = auto()
    BEFORE_Y = auto()
    AFTER_Y = auto()
    DONE = auto()
    ERROR = auto()


class ScaleFovCalibrationDialog(QDialog):
    """Measure µm/px + FOV extent by tracking a feature across a large move."""

    def __init__(self, camera_manager, controller, cam_idx: int = 0, *,
                 resolution_getter=None, safe_z=None, align_key=None,
                 objective=None, cam_key=None, scan_settings=None, parent=None):
        super().__init__(parent)
        self._mgr = camera_manager
        self._controller = controller
        self._cam_idx = cam_idx
        self._resolution_getter = resolution_getter
        # v7.5.x: the REAL mosaic-scan settings, so the "verify with a mosaic"
        # self-check builds the same way the actual scan will (it used to pass an
        # empty dict and silently fall back to unrelated defaults).
        self._scan_settings = dict(scan_settings) if scan_settings else {}
        # Objective identity for the self-check correction dialog, so its
        # corrections propagate as ground truth (mosaic + click mapping).
        self._align_key = align_key
        self._objective = objective
        self._cam_key = cam_key
        # Safe retract Z for the self-check mosaic. When None the self-check
        # images at the CURRENT height (it only ever moves XY, never descends —
        # see _verify_mosaic — so it is safe regardless of the plate).
        self._safe_z = safe_z
        self._verify_worker = None
        self._verify_builder = None
        self._mosaic_rgb = None            # keep the QImage backing buffer alive
        self._state = _S.READY

        # Per-run scratch.
        self._patch = None
        self._patch_center = (0.0, 0.0)
        self._frame_wh = (0, 0)
        self._disp_x = None          # (dx, dy, conf) for the +X move
        self._disp_y = None          # (dx, dy, conf) for the +Y move

        # Results.
        self.result_um_per_px: float | None = None
        self.result_rotation_deg: float | None = None
        self.result_resolution: tuple[int, int] | None = None
        self.result_fov_um: tuple[float, float] | None = None
        # v7.5.x: the camera→stage handedness DERIVED from the two-axis move
        # (a mirror can't be expressed as a rotation, so it needs its own flag).
        # None until measured; the caller persists them alongside the rotation.
        self.result_flip_x: bool | None = None
        self.result_flip_y: bool | None = None

        # v7.5.x: current camera view orientation, so the feed shows the
        # corrected upright/un-mirrored view and the mirror checkbox seeds it.
        try:
            vo = getattr(camera_manager, "view_orientation", None)
            self._view_mir, self._view_rot = (
                vo(cam_idx) if callable(vo) else (False, 0.0))
        except Exception:
            self._view_mir, self._view_rot = False, 0.0

        self.setWindowTitle("Auto-calibrate scale + FOV (stage motion)")
        self.setMinimumWidth(s(940))
        self.setMinimumHeight(s(540))
        self.setStyleSheet(f"background-color: {COLORS['base']}; "
                           f"color: {COLORS['text']};")
        self._build_ui()
        self._start_feed()

    # ── UI ────────────────────────────────────────────────────────

    def _build_ui(self):
        outer = QHBoxLayout(self)
        outer.setContentsMargins(s(12), s(12), s(12), s(12))
        outer.setSpacing(s(12))

        left_col = QVBoxLayout()
        left_col.setSpacing(s(6))
        if FEED_AVAILABLE and self._mgr is not None:
            self._feed = CameraFeedView(
                camera_manager=self._mgr, cam_idx=self._cam_idx,
                show_crosshair=True,
                label=f"Camera {self._cam_idx + 1} — live",
                enable_settings=False, auto_orient=True)
            self._feed.setMinimumSize(s(480), s(300))
            try:
                self._feed.set_view_orientation(self._view_mir, self._view_rot)
            except Exception:
                pass
            left_col.addWidget(self._feed, stretch=3)
        else:
            self._feed = None
            ph = QLabel("Live view unavailable")
            ph.setAlignment(Qt.AlignCenter)
            ph.setMinimumSize(s(480), s(300))
            left_col.addWidget(ph, stretch=3)

        # Self-check mosaic preview — hidden until "Verify" builds one.
        self._mosaic_label = QLabel(
            "A self-check mosaic appears here after Verify.")
        self._mosaic_label.setAlignment(Qt.AlignCenter)
        self._mosaic_label.setMinimumHeight(s(150))
        self._mosaic_label.setStyleSheet(
            f"background-color: #181825; color: {COLORS['subtext0']}; "
            f"border: 1px solid {COLORS['surface1']};")
        self._mosaic_label.setVisible(False)
        left_col.addWidget(self._mosaic_label, stretch=2)
        outer.addLayout(left_col, stretch=1)

        side = QVBoxLayout()
        side.setSpacing(s(10))
        outer.addLayout(side, stretch=0)

        instr = QLabel(
            "Point the scope at a textured region (well edges / debris) and "
            "click Measure. The stage moves a known distance twice (X then Y); "
            "a feature is tracked across the frame to derive µm/px and the "
            "camera's field-of-view in microns — anchored to the camera's "
            "actual captured resolution.")
        instr.setWordWrap(True)
        instr.setMaximumWidth(s(360))
        instr.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {scaled_font_size(9)}pt;")
        side.addWidget(instr)

        grp = QGroupBox("Settings")
        grp.setStyleSheet(self._group_style())
        form = QFormLayout(grp)

        self._spin_move = QDoubleSpinBox()
        self._spin_move.setRange(100.0, 10000.0)
        self._spin_move.setValue(1200.0)
        self._spin_move.setSuffix(" µm")
        self._spin_move.setDecimals(0)
        self._spin_move.setToolTip(
            "Baseline stage move. Larger = more accurate, but the feature must "
            "stay in view — auto-sized from the current µm/px when the camera "
            "is running.")
        form.addRow("Baseline move:", self._spin_move)

        self._spin_settle = QDoubleSpinBox()
        self._spin_settle.setRange(200, 4000)
        self._spin_settle.setValue(600)
        self._spin_settle.setSuffix(" ms")
        self._spin_settle.setDecimals(0)
        form.addRow("Settlement:", self._spin_settle)

        self._chk_mirror = QCheckBox("Camera shows a mirrored image")
        self._chk_mirror.setToolTip(
            "Flip the displayed feed left↔right (display-only; saved as the "
            "camera's mirror flag).")
        self._chk_mirror.setChecked(bool(self._view_mir))
        self._chk_mirror.toggled.connect(self._on_mirror_toggled)
        form.addRow("Mirror view:", self._chk_mirror)
        side.addWidget(grp)

        self._lbl_status = QLabel("Ready — click Measure.")
        self._lbl_status.setWordWrap(True)
        self._lbl_status.setMaximumWidth(s(360))
        self._lbl_status.setStyleSheet(
            f"color: {COLORS['yellow']}; font-size: {scaled_font_size(9)}pt; "
            f"padding: 4px;")
        side.addWidget(self._lbl_status)

        self._res_grp = QGroupBox("Result")
        self._res_grp.setStyleSheet(self._group_style())
        rf = QFormLayout(self._res_grp)
        self._lbl_res = QLabel("—")
        rf.addRow("Resolution:", self._lbl_res)
        self._lbl_umpx = QLabel("—")
        self._lbl_umpx.setStyleSheet(
            f"color: {COLORS['green']}; font-weight: bold; "
            f"font-size: {scaled_font_size(11)}pt;")
        rf.addRow("µm/px:", self._lbl_umpx)
        self._lbl_fov = QLabel("—")
        rf.addRow("FOV extent:", self._lbl_fov)
        self._lbl_rot = QLabel("—")
        rf.addRow("Rotation vs stage:", self._lbl_rot)
        self._lbl_detail = QLabel("—")
        self._lbl_detail.setWordWrap(True)
        self._lbl_detail.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {scaled_font_size(8)}pt;")
        rf.addRow(self._lbl_detail)
        # Self-check: build a small mosaic with the measured µm/px so the
        # operator can confirm the tiles register before committing.
        self._btn_verify = QPushButton("🔍 Verify & correct (mosaic)…")
        self._btn_verify.setToolTip(
            "Open the full mosaic correction tool at the current position with "
            "the measured µm/px — build a small mosaic, align the overlaps with "
            "the X/Y spacing sliders, and flip the camera to the X/Y axis if the "
            "mosaic is mirrored. Whatever you set there becomes ground truth "
            "everywhere. Images only at the current height (never descends).")
        self._btn_verify.clicked.connect(self._verify_mosaic)
        rf.addRow(self._btn_verify)
        self._res_grp.setVisible(False)
        side.addWidget(self._res_grp)
        side.addStretch()

        row = QHBoxLayout()
        self._btn_measure = QPushButton("Measure")
        self._btn_measure.setObjectName("accentBtn")
        self._btn_measure.clicked.connect(self._start_measure)
        row.addWidget(self._btn_measure)
        self._btn_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok
            | QDialogButtonBox.StandardButton.Cancel)
        self._btn_box.button(QDialogButtonBox.StandardButton.Ok).setText("Accept")
        self._btn_box.button(QDialogButtonBox.StandardButton.Ok).setEnabled(False)
        self._btn_box.accepted.connect(self._accept_result)
        self._btn_box.rejected.connect(self.reject)
        row.addWidget(self._btn_box)
        side.addLayout(row)

    def _group_style(self) -> str:
        return (
            f"QGroupBox {{ border: 1px solid {COLORS['surface1']}; "
            f"border-radius: 4px; margin-top: 8px; padding-top: 14px; "
            f"color: {COLORS['text']}; }}"
            f"QGroupBox::title {{ subcontrol-position: top left; "
            f"padding: 2px 6px; }}")

    # ── Feed lifecycle / mirror ───────────────────────────────────

    def _start_feed(self):
        mgr = self._mgr
        if mgr is None:
            return
        try:
            if not mgr.is_running(self._cam_idx):
                mgr.start(self._cam_idx)
        except Exception as e:
            logger.debug(f"ScaleFov: feed start skipped — {e}")

    def showEvent(self, event):
        super().showEvent(event)
        QTimer.singleShot(200, self._seed_default_move)

    def _seed_default_move(self):
        """Auto-size the baseline so the feature crosses ~40% of the frame —
        big enough to be accurate, small enough to stay in view."""
        try:
            cam = self._mgr.cameras[self._cam_idx]
            frame = cam.get_current_frame()
            if frame is None:
                return
            fh, fw = frame.shape[:2]
            upp = float(self._mgr.get_um_per_px(self._cam_idx) or 0.0)
            if upp <= 0:
                return
            d = 0.40 * min(fw, fh) * upp
            d = max(100.0, min(10000.0, d))
            self._spin_move.setValue(round(d / 10.0) * 10.0)
        except Exception as e:
            logger.debug(f"ScaleFov: default move seed skipped — {e}")

    def _on_mirror_toggled(self, on: bool):
        """Flip the PREVIEW only — this checkbox tells the dialog what the
        operator is looking at, so the feed reads correctly while measuring.

        v7.5.x: it no longer PERSISTS the mirror (nor pushes it to the shared
        manager). It used to write ``CameraCalibrationStore.set_mirrored`` the
        instant it was ticked — so merely opening this dialog and toggling the
        checkbox permanently changed the camera's stored calibration even if the
        operator then pressed Cancel. The handedness is MEASURED from the two-axis
        move (``derive_camera_stage_orientation`` → ``result_flip_x/_flip_y``) and
        committed only when the calibration is accepted.
        """
        on = bool(on)
        self._view_mir = on
        if self._feed is not None:
            try:
                self._feed.set_view_orientation(on, self._view_rot)
            except Exception:
                pass

    # ── Measurement state machine ─────────────────────────────────

    def _set_status(self, text, color="yellow"):
        self._lbl_status.setText(text)
        self._lbl_status.setStyleSheet(
            f"color: {COLORS[color]}; font-size: {scaled_font_size(9)}pt; "
            f"padding: 4px;")

    def _busy(self, on):
        self._btn_measure.setEnabled(not on)
        self._spin_move.setEnabled(not on)
        self._spin_settle.setEnabled(not on)
        btn = getattr(self, "_btn_verify", None)
        if btn is not None:
            btn.setEnabled(not on and bool(self.result_um_per_px))

    def _capture(self):
        try:
            return self._mgr.cameras[self._cam_idx].capture_fresh_frame()
        except Exception as e:
            logger.debug(f"ScaleFov: capture failed — {e}")
            return None

    def _move(self, dx, dy):
        self._controller.move_xy_relative_um(float(dx), float(dy))

    def _start_measure(self):
        if not VISION_AVAILABLE:
            QMessageBox.warning(self, "Unavailable", "Vision module unavailable.")
            return
        if self._mgr is None or not self._mgr.is_running(self._cam_idx):
            QMessageBox.warning(self, "Camera required",
                                "Start the camera before calibrating.")
            return
        if self._controller is None:
            QMessageBox.warning(self, "Stage required",
                                "Stage controller not connected.")
            return
        self._busy(True)
        self._res_grp.setVisible(False)
        self._btn_box.button(QDialogButtonBox.StandardButton.Ok).setEnabled(False)
        self._disp_x = self._disp_y = None
        if self._feed is not None:
            self._feed.set_overlay_vector(None, None)
        self._state = _S.BEFORE_X
        self._set_status("Selecting a feature to track…")
        QTimer.singleShot(50, self._before_x)

    def _before_x(self):
        frame = self._capture()
        if frame is None:
            return self._fail("Could not capture a frame.")
        sel = select_trackable_patch(frame)
        if sel is None:
            return self._fail("No trackable feature — aim at a textured region.")
        cx, cy, patch = sel
        self._patch = patch
        self._patch_center = (cx, cy)
        self._frame_wh = (int(frame.shape[1]), int(frame.shape[0]))
        d = float(self._spin_move.value())
        self._set_status(f"Moving +X {d:.0f} µm and tracking the feature…")
        try:
            self._move(d, 0.0)
        except Exception as e:
            return self._fail(f"Stage move failed: {e}")
        self._state = _S.AFTER_X
        QTimer.singleShot(int(self._spin_settle.value()), self._after_x)

    def _after_x(self):
        frame = self._capture()
        d = float(self._spin_move.value())
        try:
            self._move(-d, 0.0)                 # return to origin
        except Exception as e:
            logger.warning(f"ScaleFov: X move-back failed — {e}")
        res = find_template(frame, self._patch) if frame is not None else None
        if res is None:
            return self._fail("Lost the feature on the X move — try a smaller "
                              "baseline or a more textured region.")
        cx, cy, conf = res
        dx = cx - self._patch_center[0]
        dy = cy - self._patch_center[1]
        self._disp_x = (dx, dy, conf)
        if self._feed is not None:
            self._feed.set_overlay_vector(
                dx, dy, f"X {math.hypot(dx, dy):.0f}px")
        if conf < _MIN_TRACK_CONF or math.hypot(dx, dy) < _MIN_DISP_PX:
            return self._fail(
                f"Weak X track (conf {conf:.2f}, {math.hypot(dx, dy):.0f}px). "
                f"Use a more textured region or a larger baseline.")
        self._state = _S.BEFORE_Y
        self._set_status("Feature tracked in X. Preparing Y move…")
        QTimer.singleShot(int(self._spin_settle.value()), self._before_y)

    def _before_y(self):
        frame = self._capture()
        if frame is None:
            return self._fail("Could not capture a frame for the Y move.")
        sel = select_trackable_patch(frame)
        if sel is None:
            return self._fail("No trackable feature for the Y move.")
        cx, cy, patch = sel
        self._patch = patch
        self._patch_center = (cx, cy)
        d = float(self._spin_move.value())
        self._set_status(f"Moving +Y {d:.0f} µm and tracking the feature…")
        try:
            self._move(0.0, d)
        except Exception as e:
            return self._fail(f"Stage move failed: {e}")
        self._state = _S.AFTER_Y
        QTimer.singleShot(int(self._spin_settle.value()), self._after_y)

    def _after_y(self):
        frame = self._capture()
        d = float(self._spin_move.value())
        try:
            self._move(0.0, -d)                 # return to origin
        except Exception as e:
            logger.warning(f"ScaleFov: Y move-back failed — {e}")
        res = find_template(frame, self._patch) if frame is not None else None
        if res is None:
            return self._fail("Lost the feature on the Y move.")
        cx, cy, conf = res
        dx = cx - self._patch_center[0]
        dy = cy - self._patch_center[1]
        self._disp_y = (dx, dy, conf)
        if conf < _MIN_TRACK_CONF or math.hypot(dx, dy) < _MIN_DISP_PX:
            return self._fail(
                f"Weak Y track (conf {conf:.2f}, {math.hypot(dx, dy):.0f}px).")
        self._compute()

    def _compute(self):
        d = float(self._spin_move.value())
        dxX, dyX, cX = self._disp_x
        dxY, dyY, cY = self._disp_y
        magX = math.hypot(dxX, dyX)
        magY = math.hypot(dxY, dyY)
        umpx_x = d / magX
        umpx_y = d / magY
        umpx = (umpx_x + umpx_y) / 2.0
        fw, fh = self._frame_wh
        fov_w = fw * umpx
        fov_h = fh * umpx
        # v7.5.x: derive the FULL camera→stage orientation (rotation AND
        # handedness) from BOTH moves. A mirror/180° mount is invisible to the
        # rotation-only model, so this measures the flip the operator otherwise
        # had to guess by eye. Falls back to the rotation-only estimate if the
        # two moves came out collinear (degenerate).
        orient = derive_camera_stage_orientation(dxX, dyX, dxY, dyY)
        if orient is not None:
            rot, flip_x, flip_y = orient
        else:
            rot, flip_x, flip_y = plus_column_direction_deg(0.0, dxX, dyX), False, False

        self.result_um_per_px = umpx
        self.result_rotation_deg = rot
        self.result_flip_x = bool(flip_x)
        self.result_flip_y = bool(flip_y)
        # v7.16: stamp the CAPTURE resolution, not the delivered one. µm/px is
        # unchanged by a centred crop, so the stamp has to name the sensor mode
        # the measurement belongs to — otherwise turning the crop on or off
        # later would rescale a perfectly good calibration by the crop fraction.
        # (fw, fh) remains what the FOV is computed from, since that IS what the
        # camera delivers.
        cap = None
        try:
            getter = getattr(self._mgr, "capture_resolution", None)
            cap = getter(self._cam_idx) if callable(getter) else None
        except Exception:
            cap = None
        self.result_resolution = (
            (int(cap[0]), int(cap[1])) if cap and cap[0] and cap[1]
            else (int(fw), int(fh)))
        self.result_fov_um = (fov_w, fov_h)

        flips = []
        if flip_x:
            flips.append("flip X")
        if flip_y:
            flips.append("flip Y")
        flip_txt = (" · " + ", ".join(flips)) if flips else ""
        self._lbl_res.setText(f"{fw} × {fh} px (from the camera)")
        self._lbl_umpx.setText(f"{umpx:.4f} µm/px")
        self._lbl_fov.setText(f"{fov_w:.0f} × {fov_h:.0f} µm")
        self._lbl_rot.setText(f"{rot:.1f}°{flip_txt}")
        # A large X-vs-Y disagreement means one axis lost lock / left the frame.
        disagree = abs(umpx_x - umpx_y) / max(umpx, 1e-6) * 100.0
        self._lbl_detail.setText(
            f"X: {umpx_x:.4f} µm/px ({magX:.0f}px, conf {cX:.2f}) · "
            f"Y: {umpx_y:.4f} µm/px ({magY:.0f}px, conf {cY:.2f}) · "
            f"axis mismatch {disagree:.1f}%")
        self._res_grp.setVisible(True)
        self._state = _S.DONE
        self._busy(False)
        if disagree > 15.0:
            self._set_status(
                f"Done, but X and Y µm/px disagree by {disagree:.0f}% — the "
                f"feature may have left the frame on one axis. Re-measure with "
                f"a smaller baseline for confidence.", "yellow")
        else:
            self._set_status(
                "Good — Accept to save µm/px + FOV to the camera/objective.",
                "green")
        self._btn_box.button(QDialogButtonBox.StandardButton.Ok).setEnabled(True)

    # ── Self-check mosaic (verify the measured µm/px produces a good mosaic) ──

    def _verify_mosaic(self):
        """Open the FULL mosaic-correction tool (MosaicCalibrationDialog) at the
        current position, seeded with the measured µm/px + the camera's
        orientation, so the operator gets the manual X/Y spacing sliders AND the
        flip-camera-to-X/Y-axis buttons — and whatever they set there propagates
        as ground truth (mosaic + click mapping). It only ever moves XY at the
        current height (never descends → safe regardless of the plate)."""
        if not (self.result_um_per_px and self.result_um_per_px > 0):
            self._set_status("Measure first, then verify.", "yellow")
            return
        if (self._mgr is None or not self._mgr.is_running(self._cam_idx)
                or self._controller is None):
            self._set_status("Camera + stage required to verify.", "red")
            return
        try:
            from gui.dialogs.mosaic_calibration_dialog import (
                MosaicCalibrationDialog)
        except Exception as e:
            self._set_status(f"Correction dialog unavailable: {e}", "red")
            return

        fw, fh = self._frame_wh
        if fw <= 0 or fh <= 0:
            try:
                f = self._mgr.cameras[self._cam_idx].get_current_frame()
                fh, fw = int(f.shape[0]), int(f.shape[1])
            except Exception:
                self._set_status("No frame size for verify.", "red")
                return
        try:
            xy = self._controller.get_xy_position(cached=False)
            cx, cy = float(xy[0]), float(xy[1])
        except Exception:
            cx, cy = 0.0, 0.0
        # Retract only to the CURRENT height (never descends → safe); use the
        # supplied safe Z if one was passed.
        safe_z = self._safe_z
        if safe_z is None:
            try:
                z = self._controller.get_zp_position_zero_ref(cached=True)
                safe_z = (float(z.get("Z"))
                          if isinstance(z, dict) and z.get("Z") is not None
                          else 0.0)
            except Exception:
                safe_z = 0.0
        store = None
        try:
            from SupportClasses.MosaicAlignmentStore import get_store
            store = get_store()
        except Exception:
            store = None

        # v7.5.x: pass the REAL scan settings, not ``{}``. An empty dict made the
        # dialog fall back to its own defaults, so this "verify" step silently ran
        # at 25 % overlap on a 5x5 grid regardless of what the operator had
        # configured (3x3 here) — i.e. it verified something other than the scan
        # it was supposed to be checking. ``fov_um`` / ``spacing_um`` are NOT
        # forwarded: the freshly measured µm/px below must size the tiles.
        verify_settings = dict(self._scan_settings or {})
        verify_settings.pop("fov_um", None)
        verify_settings.pop("spacing_um", None)
        # v7.16: snapshot the manager BEFORE the verify dialog, so afterwards we
        # can tell an actual correction from "nothing happened". See below.
        def _mgr_eff():
            try:
                v = float(self._mgr.effective_um_per_px(self._cam_idx, fw))
                return v if v > 0 else None
            except Exception:
                return None

        before = _mgr_eff()

        dlg = MosaicCalibrationDialog(
            self._controller, self._mgr, self._cam_idx,
            safe_z=safe_z, align_key=self._align_key, store=store,
            settings=verify_settings, center_um=(cx, cy), frame_size=(fw, fh),
            um_per_px_camera=float(self.result_um_per_px),
            cam_key=self._cam_key, objective=self._objective, parent=self)
        dlg.exec()

        # Re-sync the µm/px ONLY when the verify step actually CORRECTED it.
        #
        # ⚠ This used to adopt the manager's value unconditionally, which
        # destroyed the measurement this dialog exists to make. The manager
        # holds whatever was last pushed into the slot — including another
        # camera's stored calibration — so simply opening Verify and changing
        # nothing replaced a fresh reading with a stale one, silently, with the
        # label still saying "measured".
        #
        # That is the whole 400-minute mosaic on this rig: the microscope slot
        # had been seeded with the ToupTek's 0.3891 µm/px (the objective store
        # was keyed by camera *model name*, so the Tucsen read the ToupTek's
        # block), the operator measured the Tucsen, opened Verify, and Accept
        # then saved 0.3891 stamped at the Tucsen's 2600x2048 — a FOV of
        # 1012 um instead of ~3305, i.e. 18,972 tiles instead of ~1,800.
        #
        # Comparing against the pre-dialog snapshot is what distinguishes the
        # two cases without needing the verify dialog to report back.
        try:
            after = _mgr_eff()
            corrected = (
                after is not None and before is not None
                and abs(after - before) > 1e-9)
            if after is not None and before is None:
                # No comparable "before" (an unstamped slot cannot be rescaled
                # to this width) — the measurement stands. Adopting here is how
                # an unrescalable value reaches Accept wearing a fresh stamp.
                corrected = False
            if corrected:
                self.result_um_per_px = after
                self._lbl_umpx.setText(f"{after:.4f} µm/px")
                if fw > 0 and fh > 0:
                    self.result_fov_um = (fw * after, fh * after)
                    self._lbl_fov.setText(
                        f"{fw * after:.0f} × {fh * after:.0f} µm")
                logger.info(
                    "ScaleFov: verify corrected um/px %.4f -> %.4f",
                    before, after)
            else:
                logger.info(
                    "ScaleFov: verify made no um/px correction — keeping the "
                    "measured %.4f um/px", float(self.result_um_per_px or 0.0))
        except Exception:
            pass
        try:
            fo = getattr(self._mgr, "full_orientation", None)
            if callable(fo):
                mir, fy, rot = fo(self._cam_idx)
            else:
                mir, rot = self._mgr.view_orientation(self._cam_idx)
                fy = False
            self.result_rotation_deg = float(rot)
            self.result_flip_x = bool(mir)
            self.result_flip_y = bool(fy)
            flips = ([("flip X") ] if mir else []) + ([("flip Y")] if fy else [])
            flip_txt = (" · " + ", ".join(flips)) if flips else ""
            self._lbl_rot.setText(f"{rot:.1f}°{flip_txt}")
        except Exception:
            pass
        self._set_status(
            "Correction applied. Accept to save, or re-measure.", "green")

    def build_test_mosaic(self, cal) -> None:
        """Build the confirmation mosaic for a CANDIDATE calibration.

        v7.5.x: the "final check with the new settings" (operator). Uses the
        candidate's µm/px + orientation + overlap rather than anything stored —
        the stores are deliberately not written until the operator accepts — and
        routes through the same ``MosaicCalibrationDialog`` build machinery the
        real scan uses, so what is confirmed is what will happen.

        Safe: XY only, at the CURRENT height. ``_verify_mosaic`` passes
        ``target_z_mm=None`` so the needle never descends.
        """
        self.result_um_per_px = float(cal.um_per_px)
        # v7.16: the CAPTURE resolution (falling back to the delivered one when
        # there is no crop) — the stamp names a sensor mode, not a frame size.
        stamp = getattr(cal, "capture_resolution", None) or cal.live_resolution
        if stamp:
            self.result_resolution = tuple(stamp)
        self.result_rotation_deg = float(cal.rotation_deg)
        self.result_flip_x = bool(cal.flip_x)
        self.result_flip_y = bool(cal.flip_y)
        prev = dict(self._scan_settings or {})
        try:
            self._scan_settings = dict(prev)
            self._scan_settings["overlap_pct"] = int(
                round(cal.overlap_frac * 100))
            self._verify_mosaic()
        finally:
            self._scan_settings = prev

    def _on_verify_progress(self, done, total):
        self._set_status(f"Self-check mosaic: {done}/{total} tiles…")

    def _on_verify_finished(self, comp, extent, scale, frames, dets):
        self._verify_worker = None
        self._busy(False)
        if comp is None:
            self._set_status("Self-check produced no mosaic.", "red")
            return
        self._show_mosaic(comp)
        self._set_status(
            f"Self-check mosaic built ({frames} tiles). If the features line up "
            f"across tiles, the µm/px is good — Accept to save.", "green")

    def _on_verify_failed(self, msg):
        self._verify_worker = None
        self._busy(False)
        self._set_status(f"Self-check failed: {msg}", "red")

    def _show_mosaic(self, comp):
        from PySide6.QtGui import QImage, QPixmap
        try:
            h, w = int(comp.shape[0]), int(comp.shape[1])
            rgb = np.ascontiguousarray(comp[:, :, ::-1])   # BGR → RGB
            self._mosaic_rgb = rgb                          # keep alive
            img = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888)
            pm = QPixmap.fromImage(img)
            self._mosaic_label.setVisible(True)
            lw = max(self._mosaic_label.width(), s(200))
            lh = max(self._mosaic_label.height(), s(150))
            self._mosaic_label.setPixmap(pm.scaled(
                lw, lh, Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))
        except Exception as e:
            logger.debug(f"show mosaic failed: {e}")

    def _stop_verify(self):
        w = self._verify_worker
        self._verify_worker = None
        if w is not None:
            for sig in (w.progress, w.finished_ok, w.failed):
                try:
                    sig.disconnect()
                except Exception:
                    pass
            try:
                w.stop()
                if w.isRunning():
                    w.wait(4000)
            except Exception:
                pass
        self._verify_builder = None

    def closeEvent(self, event):
        self._stop_verify()
        super().closeEvent(event)

    def reject(self):
        self._stop_verify()
        super().reject()

    def _fail(self, msg):
        self._state = _S.ERROR
        self._busy(False)
        self._set_status(msg, "red")

    def _accept_result(self):
        if self.result_um_per_px and self.result_um_per_px > 0:
            logger.info(
                f"Scale/FOV calibration accepted: {self.result_um_per_px:.4f} "
                f"µm/px @ {self.result_resolution}, FOV {self.result_fov_um}, "
                f"rotation {self.result_rotation_deg}, "
                f"flip_x {self.result_flip_x}, flip_y {self.result_flip_y}")
            self.accept()
        else:
            self.reject()
