"""
pixel_calibration_dialog.py — Modal dialog for empirical µm/px calibration.

v7.3.3: Measures the actual µm/px ratio by moving the stage a known distance
and correlating the resulting pixel displacement between two captured frames.

v7.5.x: Live view + selectable move direction. The needle cameras are 90°
apart from each other, mounted symmetric about the stage +X axis at +45° and
−45°, so a stage move in the wrong direction drives the needle *along the
camera's optical axis* — it just goes in/out of focus and shows almost no
lateral motion. The dialog now shows the live feed and overlays the detected
phase-correlation displacement as an arrow, and lets the operator pick the
move direction (presets + free angle) until they get strong lateral motion.
The accepted direction is reported as the camera's column→stage mount
direction (``result_rotation_deg``) for the needle-centering aligner, and the
measured vector's deviation from parallel as the sensor roll
(``result_view_roll_deg``) for the display orientation — two different
angles; only the roll may tilt the live view.

Workflow:
    1. Capture frame at current position
    2. Move stage a known distance along the chosen direction
    3. Wait for settlement, capture second frame
    4. Phase-correlate to measure pixel displacement (shown as an arrow)
    5. µm/px = distance_moved / pixel_displacement_magnitude

Requires a running camera and connected stage controller.
"""

from __future__ import annotations

import logging
import math
from enum import Enum, auto

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLabel, QDoubleSpinBox, QPushButton,
    QDialogButtonBox, QGroupBox, QMessageBox, QWidget, QCheckBox,
)

from gui.styles import COLORS
from gui.scaling import s, scaled_font_size

logger = logging.getLogger(__name__)

try:
    from SupportClasses.VisionDetector import measure_pixel_displacement
    VISION_AVAILABLE = True
except ImportError:
    VISION_AVAILABLE = False
    logger.warning("VisionDetector not available — pixel calibration disabled")

try:
    from gui.widgets.camera_feed_view import CameraFeedView
    FEED_AVAILABLE = True
except ImportError:
    CameraFeedView = None
    FEED_AVAILABLE = False


class _CalState(Enum):
    """Calibration state machine."""
    READY = auto()
    CAPTURING_BEFORE = auto()
    MOVING = auto()
    SETTLING = auto()
    CAPTURING_AFTER = auto()
    RESULT = auto()
    ERROR = auto()


def plus_column_direction_deg(commanded_deg: float, measured_dx_px: float,
                              measured_dy_px: float = 0.0) -> float:
    """Stage angle (deg, CCW from +X) along which displacing the needle
    INCREASES its image column — the value `TwoCameraNeedleAligner` expects.

    Derived from the *measured* content displacement, NOT the commanded preset.
    The commanded angle alone has two problems: its sign is arbitrary (for a
    ~45° mount both opposite "lateral" presets look valid), and it is quantized
    to the preset the operator clicked — if the camera's true lateral axis sits
    a few degrees off that preset, the leftover tilt biases the two-camera solve
    and the needle lands off-center.

    Model the stage→image map as scale·R(α) (rotation by α, no mirror). Moving
    the cameras by ``+m = D·(cosθ, sinθ)`` makes stationary content shift by
    ``p = −scale·R(α)·m`` (measured ``p = (dx, dy)``, image convention +x =
    right). The stage direction that maps to image +column is therefore
    ``−α = θ − atan2(dy, dx) − 180°`` — independent of which preset θ the
    operator picked, and exact for off-axis moves. On a purely lateral move
    this reduces to the simple ±180° sign rule.

    (HW note: assumes a non-mirrored image — the aligner only models a single
    rotation per camera anyway. If both cameras come out 180° off, the
    ``measure_pixel_displacement`` / ``cv2.phaseCorrelate`` sign is inverted on
    this build; negate the ``180.0`` term.)
    """
    phi = math.degrees(math.atan2(measured_dy_px, measured_dx_px))
    angle = float(commanded_deg) - phi - 180.0
    return ((angle + 180.0) % 360.0) - 180.0  # normalize to [−180, 180)


def column_direction_to_camera_rotation_deg(commanded_deg: float,
                                            raw_deg: float,
                                            mirrored: bool = False,
                                            flip_y: bool = False) -> float:
    """Convert ``plus_column_direction_deg``'s output into the camera rotation
    ``θ`` that belongs beside the ALREADY-STORED ``mirrored`` / ``flip_y``.

    That helper models the stage→image map as ``scale·R(α)`` and its docstring
    says so — but the repo's convention (``CameraManager.pixel_to_stage_offset``,
    ``camera_feed_view.view_transform_coeffs``, ``MosaicBuilder._orient_tile``)
    is ``s = R(θ)·F·u·p`` with ``F = diag(mx, my)``, ``mx = −1`` iff mirrored,
    ``my = −1`` iff flip_y. A commanded move ``m`` at stage angle ``c`` shifts
    image content by ``d = −(1/u)·F⁻¹·R(−θ)·m`` (from ``pto``'s own contract:
    a plate feature's stage LABEL ``xy + pto(P)`` is invariant under stage
    motion).

    Two separate things then go wrong, and BOTH must be undone:

    1. ``diag(−1,−1) == R(180)``, so ``R(θ)·F`` always reduces to
       ``R(θ_eff)·diag(1, my_eff)`` with ``θ_eff = θ + 180`` and
       ``my_eff = −my`` whenever ``mx = −1``. The helper can only ever report
       ``θ_eff``, i.e. it is **180° out for any mirrored camera**.
    2. Conjugating a rotation by a reflection reverses it, so for
       ``det F = −1`` the helper returns ``2c − θ_eff`` — a reflection *about
       the commanded angle*, not a plain sign flip. It therefore changes with
       whichever direction preset the operator happened to click, which is why
       a single reading cannot look wrong.

    Inverting both, in order::

        θ_eff = (2c − raw) if det F == −1 else raw
        θ     = θ_eff − 180 if mirrored else θ_eff

    Exactly a no-op for an unmirrored, unflipped camera. Verified numerically
    against a forward simulation of ``pixel_to_stage_offset`` for all four flip
    combinations.
    """
    theta = float(raw_deg)
    if bool(mirrored) != bool(flip_y):          # det F == -1
        theta = 2.0 * float(commanded_deg) - theta
    if bool(mirrored):                          # diag(-1, .) carries an R(180)
        theta -= 180.0
    return ((theta + 180.0) % 360.0) - 180.0    # normalize to [-180, 180)


# v7.10: fold_parallel_deg moved to the Qt-free SupportClasses module so the
# live rotation tracker and this dialog cannot drift apart on a sign-carrying
# helper. Re-exported here, unchanged, for every existing importer.
from SupportClasses.CameraRotationTracker import (  # noqa: E402
    fold_parallel_deg, wrap_deg)  # noqa: F401


def view_roll_from_displacement(dx_px: float, dy_px: float,
                                mirrored: bool = False,
                                flip_y: bool = False) -> float:
    """The ``set_view_orientation`` rotation (deg) that renders the measured
    stage-motion displacement LEVEL (parallel to the image horizontal).

    This is the needle side-camera's sensor ROLL — the deviation of the
    drawn motion vector from parallel — which is the only rotation that
    belongs on the DISPLAY. The full column→stage mount direction
    (``plus_column_direction_deg``, ±45° on the current rig) feeds the
    two-camera needle aligner instead and must never tilt the live view.

    The display chain applies flips first, then R(θ)
    (``camera_feed_view.view_transform_coeffs``), so the flips are applied
    to the raw vector before measuring its angle; a raw vector at display
    angle φ′ renders at φ′ + θ, hence θ = −fold(φ′). The sign is pinned by
    a test composing this with ``view_transform_coeffs``.
    """
    dxf = -float(dx_px) if mirrored else float(dx_px)
    dyf = -float(dy_px) if flip_y else float(dy_px)
    phi = math.degrees(math.atan2(dyf, dxf))
    return -fold_parallel_deg(phi)


# Move-direction presets (stage-frame angle, degrees CCW from +X).
_DIRECTION_PRESETS = [
    ("X →", 0.0),
    ("Y ↑", 90.0),
    ("Diag ↗", 45.0),
    ("Diag ↘", -45.0),
]


class PixelCalibrationDialog(QDialog):
    """Modal dialog for empirical camera µm/px calibration via stage movement."""

    def __init__(self, camera_manager, controller, cam_idx: int = 0,
                 parent=None, needle_mode: bool = False,
                 cam_key: str | None = None, objective: str | None = None):
        super().__init__(parent)
        self._camera_manager = camera_manager
        self._controller = controller
        self._cam_idx = cam_idx
        # v7.16: which camera + which objective this calibration is FOR.
        # Needed to size the in-frame move bound: the bound depends on µm/px,
        # but the move happens BEFORE the measurement exists, so the estimate
        # has to come from the camera's native scale (a sensor property,
        # derivable from ANY calibrated objective) divided by THIS objective's
        # magnification. See ``_expected_um_per_px``.
        self._cam_key = cam_key
        self._objective = objective
        self._state = _CalState.READY
        # v7.10: a needle SIDE camera gets a second measurement leg — a Z move,
        # which moves the needle and nothing else. See the module docstring of
        # SupportClasses/NeedleCameraCalibration for why one leg is not enough.
        self._needle_mode = bool(needle_mode)
        self.result_needle_axes = None      # NeedleCameraAxes | None
        self._lateral_px = None             # (dx, dy) of the accepted XY leg

        self._frame_before = None
        self._frame_after = None
        self.result_um_per_px: float | None = None
        # v7.5.x: the accepted move direction = the camera's in-plane lateral
        # stage direction (deg from +X), fed to the needle-centering aligner.
        self.result_rotation_deg: float | None = None
        # v7.5.x (rotated rig): the sensor ROLL — deviation of the measured
        # motion vector from parallel — the only rotation that belongs on the
        # needle cameras' DISPLAY orientation (result_rotation_deg is the
        # ±45° mount direction and must not tilt the live view).
        self.result_view_roll_deg: float | None = None

        # v7.5.x: the camera's current view orientation, so the calibration feed
        # shows the corrected upright/un-mirrored view and the Mirror-view
        # checkbox seeds from it. Measurement runs on RAW frames, so the mirror
        # is display-only and cannot corrupt the µm/px result.
        try:
            vo = getattr(camera_manager, "view_orientation", None)
            self._view_mir, self._view_rot = (
                vo(cam_idx) if callable(vo) else (False, 0.0))
        except Exception:
            self._view_mir, self._view_rot = False, 0.0

        self.setWindowTitle("Calibrate µm/px")
        self.setMinimumWidth(s(900))
        self.setMinimumHeight(s(520))
        self.setStyleSheet(f"background-color: {COLORS['base']}; "
                           f"color: {COLORS['text']};")

        self._build_ui()
        self._start_feed()

    # ── UI construction ───────────────────────────────────────────

    def _build_ui(self):
        outer = QHBoxLayout(self)
        outer.setContentsMargins(s(12), s(12), s(12), s(12))
        outer.setSpacing(s(12))

        # ── Left: live feed ───────────────────────────────────────
        if FEED_AVAILABLE and self._camera_manager is not None:
            self._feed = CameraFeedView(
                camera_manager=self._camera_manager,
                cam_idx=self._cam_idx,
                show_crosshair=True,
                label=f"Camera {self._cam_idx + 1} — live",
                # No settings gear here: this dialog PRODUCES a µm/px
                # calibration, and changing capture resolution mid-measurement
                # would change the effective µm/px and corrupt the result.
                enable_settings=False,
                # Keep this calibration view consistent with the live/mosaic
                # views by tracking the camera's saved orientation.
                auto_orient=True,
            )
            self._feed.setMinimumSize(s(480), s(380))
            # Show the corrected upright/un-mirrored view (display-only).
            try:
                self._feed.set_view_orientation(self._view_mir, self._view_rot)
            except Exception:
                pass
            outer.addWidget(self._feed, stretch=1)
        else:
            self._feed = None
            placeholder = QLabel("Live view unavailable")
            placeholder.setAlignment(Qt.AlignCenter)
            placeholder.setMinimumSize(s(480), s(380))
            placeholder.setStyleSheet(
                f"background-color: #181825; color: {COLORS['subtext0']};")
            outer.addWidget(placeholder, stretch=1)

        # ── Right: controls ───────────────────────────────────────
        side = QVBoxLayout()
        side.setSpacing(s(10))
        outer.addLayout(side, stretch=0)

        instr = QLabel(
            "Move the stage a known distance and measure the pixel shift "
            "(phase correlation). The detected motion is drawn as a green "
            "arrow on the feed.\n\n"
            "The needle cameras sit symmetric about +X at ±45° — if a move "
            "just changes focus with little arrow, the needle is moving "
            "along the camera's optical axis. Pick the direction that gives "
            "the longest arrow.")
        instr.setWordWrap(True)
        instr.setMaximumWidth(s(340))
        instr.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {scaled_font_size(9)}pt;")
        side.addWidget(instr)

        # Settings group
        settings_grp = QGroupBox("Settings")
        settings_grp.setStyleSheet(self._group_style())
        form = QFormLayout(settings_grp)

        self._spin_distance = QDoubleSpinBox()
        self._spin_distance.setRange(50.0, 2000.0)
        self._spin_distance.setValue(200.0)
        self._spin_distance.setSuffix(" µm")
        self._spin_distance.setDecimals(0)
        form.addRow("Move distance:", self._spin_distance)

        self._spin_direction = QDoubleSpinBox()
        self._spin_direction.setRange(-180.0, 180.0)
        self._spin_direction.setValue(45.0)
        self._spin_direction.setSuffix("°")
        self._spin_direction.setDecimals(1)
        self._spin_direction.setToolTip(
            "Stage move direction in the XY plane (0° = +X, 90° = +Y).")
        form.addRow("Move direction:", self._spin_direction)

        # Direction preset buttons.
        preset_row = QHBoxLayout()
        preset_row.setSpacing(s(4))
        for text, ang in _DIRECTION_PRESETS:
            b = QPushButton(text)
            b.setMaximumWidth(s(70))
            b.clicked.connect(
                lambda _c=False, a=ang: self._spin_direction.setValue(a))
            preset_row.addWidget(b)
        preset_holder = QWidget()
        preset_holder.setLayout(preset_row)
        form.addRow("Presets:", preset_holder)

        self._spin_settle = QDoubleSpinBox()
        self._spin_settle.setRange(200, 3000)
        self._spin_settle.setValue(500)
        self._spin_settle.setSuffix(" ms")
        self._spin_settle.setDecimals(0)
        form.addRow("Settlement time:", self._spin_settle)

        # v7.5.x: mirror the camera output in this view (display-only; the raw
        # frame the measurement uses is untouched). Persisted to the camera so
        # the live feed / mosaics / click-mapping stay consistent.
        self._chk_mirror = QCheckBox("Camera shows a mirrored image")
        self._chk_mirror.setToolTip(
            "Flip the displayed feed left↔right so what you see is not "
            "mirrored. Saved as the camera's mirror flag.")
        self._chk_mirror.setChecked(bool(self._view_mir))
        self._chk_mirror.toggled.connect(self._on_mirror_toggled)
        form.addRow("Mirror view:", self._chk_mirror)

        side.addWidget(settings_grp)

        # Status label
        self._lbl_status = QLabel("Ready — click Measure to move & detect.")
        self._lbl_status.setWordWrap(True)
        self._lbl_status.setMaximumWidth(s(340))
        self._lbl_status.setStyleSheet(
            f"color: {COLORS['yellow']}; font-size: {scaled_font_size(9)}pt; "
            f"padding: 4px;")
        side.addWidget(self._lbl_status)

        # Result display
        self._result_group = QGroupBox("Result")
        self._result_group.setStyleSheet(self._group_style())
        result_form = QFormLayout(self._result_group)

        self._lbl_displacement = QLabel("—")
        result_form.addRow("Pixel displacement:", self._lbl_displacement)
        self._lbl_angle = QLabel("—")
        result_form.addRow("Detected angle:", self._lbl_angle)
        self._lbl_confidence = QLabel("—")
        result_form.addRow("Confidence:", self._lbl_confidence)
        self._lbl_umpx = QLabel("—")
        self._lbl_umpx.setStyleSheet(
            f"color: {COLORS['green']}; font-weight: bold; "
            f"font-size: {scaled_font_size(11)}pt;")
        result_form.addRow("Computed µm/px:", self._lbl_umpx)

        self._result_group.setVisible(False)
        side.addWidget(self._result_group)

        # ── v7.10: Z leg (needle side cameras only) ───────────────
        self._z_group = QGroupBox("Step 2 — Z leg (needle motion)")
        self._z_group.setStyleSheet(self._group_style())
        z_form = QFormLayout(self._z_group)

        z_note = QLabel(
            "An XY move slides the whole scene (the cameras ride the stage). "
            "A Z move moves the NEEDLE and nothing else, which is what makes "
            "the sensor roll, the Z direction and the true µm/px measurable "
            "instead of assumed.\n\n"
            "This leg stands alone — run it on its own for a rotation/scale "
            "correction. The XY leg is only needed for the needle aligner's "
            "±45° mount direction.\n\nThe needle retracts UP and returns.")
        z_note.setWordWrap(True)
        z_note.setMaximumWidth(s(340))
        z_note.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {scaled_font_size(9)}pt;")
        z_form.addRow(z_note)

        self._spin_z = QDoubleSpinBox()
        self._spin_z.setRange(20.0, 2000.0)
        self._spin_z.setValue(200.0)
        self._spin_z.setSuffix(" µm")
        self._spin_z.setDecimals(0)
        self._spin_z.setToolTip(
            "How far to retract the needle (up — away from the plate) for the "
            "Z leg. Kept small enough that the needle stays in frame.")
        z_form.addRow("Z move (up):", self._spin_z)

        self._btn_z = QPushButton("Measure Z leg")
        self._btn_z.clicked.connect(self._start_z_leg)
        # v7.10: NOT gated on the XY leg. The Z leg is the independent, more
        # trustworthy measurement; requiring the lateral one first would make a
        # rotation correction depend on the very measurement it supersedes.
        self._btn_z.setEnabled(self._needle_mode)
        z_form.addRow(self._btn_z)

        self._lbl_z_result = QLabel(
            "Run this on its own for rotation + µm/px, or after the XY leg to "
            "also get the aligner's mount direction.")
        self._lbl_z_result.setWordWrap(True)
        self._lbl_z_result.setMaximumWidth(s(340))
        self._lbl_z_result.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {scaled_font_size(9)}pt;")
        z_form.addRow(self._lbl_z_result)

        self._z_group.setVisible(self._needle_mode)
        side.addWidget(self._z_group)

        side.addStretch()

        # Buttons
        btn_layout = QHBoxLayout()
        self._btn_start = QPushButton("Measure")
        self._btn_start.setObjectName("accentBtn")
        self._btn_start.setStyleSheet(
            f"QPushButton {{ background-color: {COLORS['blue']}; "
            f"color: {COLORS['base']}; padding: 6px 16px; "
            f"border-radius: 4px; font-weight: bold; }}")
        self._btn_start.clicked.connect(self._start_calibration)
        btn_layout.addWidget(self._btn_start)

        self._btn_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok |
            QDialogButtonBox.StandardButton.Cancel)
        self._btn_box.button(QDialogButtonBox.StandardButton.Ok).setText("Accept")
        self._btn_box.button(QDialogButtonBox.StandardButton.Ok).setEnabled(False)
        self._btn_box.accepted.connect(self._accept_result)
        self._btn_box.rejected.connect(self.reject)
        btn_layout.addWidget(self._btn_box)

        side.addLayout(btn_layout)

    def _on_mirror_toggled(self, on: bool) -> None:
        """Flip the PREVIEW only, so the feed reads correctly while measuring.

        v7.5.x: no longer persists (nor pushes to the shared manager). It used to
        write ``CameraCalibrationStore.set_mirrored`` the moment it was ticked, so
        opening this dialog and toggling the checkbox permanently altered the
        camera's stored calibration even on Cancel. Orientation is committed only
        by the calibration flow that owns it.
        """
        on = bool(on)
        self._view_mir = on
        if self._feed is not None:
            try:
                self._feed.set_view_orientation(on, self._view_rot)
            except Exception:
                pass

    def _group_style(self) -> str:
        return (
            f"QGroupBox {{ border: 1px solid {COLORS['surface1']}; "
            f"border-radius: 4px; margin-top: 8px; padding-top: 14px; "
            f"color: {COLORS['text']}; }}"
            f"QGroupBox::title {{ subcontrol-position: top left; "
            f"padding: 2px 6px; }}")

    # ── Camera feed lifecycle ─────────────────────────────────────

    def _start_feed(self):
        """Ensure the camera is running so the live feed shows."""
        mgr = self._camera_manager
        if mgr is None:
            return
        try:
            if not mgr.is_running(self._cam_idx):
                mgr.start(self._cam_idx)
        except Exception as e:
            logger.debug(f"PixelCalibrationDialog: feed start skipped — {e}")

    # ── State machine ─────────────────────────────────────────────

    def _set_status(self, text: str, color: str = "yellow"):
        self._lbl_status.setText(text)
        self._lbl_status.setStyleSheet(
            f"color: {COLORS[color]}; font-size: {scaled_font_size(9)}pt; "
            f"padding: 4px;")

    def _move_vector(self) -> tuple[float, float]:
        """Stage (dx, dy) µm for the current distance + direction."""
        distance = self._spin_distance.value()
        theta = math.radians(self._spin_direction.value())
        return (distance * math.cos(theta), distance * math.sin(theta))

    def _start_calibration(self):
        """Begin the calibration sequence."""
        if not VISION_AVAILABLE:
            QMessageBox.warning(self, "Unavailable",
                                "Vision module not available.")
            return

        mgr = self._camera_manager
        if mgr is None or not mgr.is_running(self._cam_idx):
            QMessageBox.warning(self, "Camera Required",
                                "Start the camera before calibrating.")
            return

        if self._controller is None:
            QMessageBox.warning(self, "Stage Required",
                                "Stage controller is not connected.")
            return

        # v7.10: refuse a move that would carry the tracked content off the
        # sensor. A too-large move and a move along the optical axis both come
        # back as "almost no displacement", so bounding it up front is what
        # makes the failure message trustworthy. Silent when µm/px is unknown —
        # the first calibration has to be allowed to run.
        try:
            from SupportClasses.NeedleCameraCalibration import in_frame_refusal
            fw, fh = self._frame_wh()
            # v7.16: size the bound from THIS camera + THIS objective, not from
            # whatever µm/px happens to be sitting in the manager. See
            # _expected_um_per_px — the old path produced a bogus 199 µm limit
            # from another camera's unstamped value.
            if self.result_um_per_px:
                u_known, src = float(self.result_um_per_px), "this measurement"
            else:
                u_known, src = self._expected_um_per_px(fw)
            why = in_frame_refusal(
                self._spin_distance.value(), u_known, fw, fh, what="XY move")
            if why:
                # Always say WHERE the limit came from: a bound the operator
                # can see is wrong is one they can act on, and this exact
                # number ("199 µm") was wrong and unexplained.
                self._set_status(f"{why} (scale from {src}.)", "red")
                return
        except Exception as exc:
            logger.debug(f"XY leg in-frame check skipped: {exc}")

        self._state = _CalState.CAPTURING_BEFORE
        self._btn_start.setEnabled(False)
        self._result_group.setVisible(False)
        self._btn_box.button(QDialogButtonBox.StandardButton.Ok).setEnabled(False)
        if self._feed is not None:
            self._feed.set_overlay_vector(None, None)
        self._set_status("Capturing frame 1...")

        QTimer.singleShot(50, self._capture_before)

    def _capture_before(self):
        """Capture the first frame, then initiate stage move."""
        cam_widget = self._camera_manager.cameras[self._cam_idx]
        self._frame_before = cam_widget.capture_fresh_frame()

        if self._frame_before is None:
            self._set_status("Failed to capture frame 1. Is the camera running?",
                             "red")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            return

        dx, dy = self._move_vector()
        self._state = _CalState.MOVING
        self._set_status(
            f"Moving stage {self._spin_distance.value():.0f} µm at "
            f"{self._spin_direction.value():.0f}° "
            f"(dx={dx:.0f}, dy={dy:.0f})...")

        try:
            self._controller.move_xy_relative_um(dx, dy)
        except Exception as e:
            self._set_status(f"Stage move failed: {e}", "red")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            return

        self._state = _CalState.SETTLING
        settle_ms = int(self._spin_settle.value())
        self._set_status(f"Waiting {settle_ms} ms for stage to settle...")
        QTimer.singleShot(settle_ms, self._capture_after)

    def _capture_after(self):
        """Capture the second frame and compute displacement."""
        cam_widget = self._camera_manager.cameras[self._cam_idx]
        self._frame_after = cam_widget.capture_fresh_frame()

        if self._frame_after is None:
            self._set_status("Failed to capture frame 2.", "red")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            self._move_back()
            return

        self._state = _CalState.CAPTURING_AFTER
        self._set_status("Computing displacement...")
        self._move_back()

        try:
            dx, dy, confidence = measure_pixel_displacement(
                self._frame_before, self._frame_after)
        except Exception as e:
            self._set_status(f"Phase correlation failed: {e}", "red")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            return

        magnitude = math.sqrt(dx * dx + dy * dy)
        distance_um = self._spin_distance.value()

        # Always show the detected vector so the operator can see the motion.
        if self._feed is not None:
            self._feed.set_overlay_vector(dx, dy, f"{magnitude:.0f}px")
        img_angle = math.degrees(math.atan2(dy, dx))
        self._lbl_displacement.setText(
            f"dx={dx:.2f}, dy={dy:.2f} px  (|d|={magnitude:.2f} px)")
        self._lbl_angle.setText(f"{img_angle:.1f}° (image)")
        self._lbl_confidence.setText(f"{confidence:.3f}")
        self._result_group.setVisible(True)

        if magnitude < 3.0:
            self._set_status(
                f"Very little lateral motion ({magnitude:.1f} px). The needle "
                f"is likely moving along this camera's optical axis (in/out of "
                f"focus). Try a different move direction (e.g. ±45°).", "red")
            self._lbl_umpx.setText("—")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            self._btn_start.setText("Measure")
            return

        if confidence < 0.15:
            self._set_status(
                f"Low confidence ({confidence:.2f}). The field may be too "
                f"featureless, or the motion is mostly out-of-focus. Try a "
                f"different direction or a textured target.", "red")
            self._lbl_umpx.setText("—")
            self._state = _CalState.ERROR
            self._btn_start.setEnabled(True)
            self._btn_start.setText("Measure")
            return

        um_per_px = distance_um / magnitude
        self._state = _CalState.RESULT

        conf_color = "green" if confidence >= 0.3 else "yellow"
        self._lbl_confidence.setStyleSheet(
            f"color: {COLORS[conf_color]}; font-weight: bold;")
        self._lbl_umpx.setText(f"{um_per_px:.4f} µm/px")

        self.result_um_per_px = um_per_px
        # v7.5.x: the rotation fed to TwoCameraNeedleAligner must be the stage
        # direction along which displacing the needle INCREASES its image column
        # (the "+column" direction), NOT just the commanded move angle whose sign
        # is arbitrary for a ~45°-mounted camera — that was the wrong-direction
        # auto-center bug. Resolve the sign from the measured displacement.
        # Parity flips must match the display chain (flips first, then R(θ)).
        flip_y = False
        try:
            gfy = getattr(self._camera_manager, "get_flip_y", None)
            if callable(gfy):
                flip_y = bool(gfy(self._cam_idx))
        except Exception:
            flip_y = False
        commanded = float(self._spin_direction.value())
        # v7.10: plus_column_direction_deg's model assumes a NON-mirrored image
        # (its own docstring says so), so its output must be converted into the
        # θ that belongs beside the already-stored flips. Exactly a no-op for an
        # unmirrored, unflipped camera; on this rig's microscope (Andor,
        # mirrored, θ=180°) the raw value at the default 45° preset is 90°, and
        # committing that would rotate click→stage, every mosaic tile and every
        # per-objective entry by 90°.
        self.result_rotation_deg = column_direction_to_camera_rotation_deg(
            commanded, plus_column_direction_deg(commanded, dx, dy),
            mirrored=bool(self._view_mir), flip_y=flip_y)
        # v7.5.x (rotated rig): the sensor roll = deviation of the measured
        # vector from parallel, for the needle cameras' display orientation.
        self.result_view_roll_deg = view_roll_from_displacement(
            dx, dy, mirrored=bool(self._view_mir), flip_y=flip_y)

        # v7.10: keep the accepted XY leg so the Z leg can solve both together.
        # Re-measuring the XY leg invalidates any previous solve — the pair must
        # come from the same framing.
        self._lateral_px = (dx, dy)
        if self._needle_mode:
            self.result_needle_axes = None
            self._btn_z.setEnabled(True)
            self._set_z_note(
                "XY leg captured. Now measure the Z leg — it is what gives the "
                "true µm/px, the sensor roll and the Z direction.", "yellow")
            self._set_status(
                "XY leg good. Measure the Z leg before accepting: on its own "
                "this µm/px is over-estimated by however far the move was off "
                "this camera's lateral direction.", "yellow")
        else:
            self._set_status(
                "Good lateral motion — Accept to use this µm/px and direction, "
                "or try other directions to compare.", "green")
        self._btn_box.button(QDialogButtonBox.StandardButton.Ok).setEnabled(True)
        self._btn_start.setEnabled(True)
        self._btn_start.setText("Re-measure")

    def _move_back(self):
        """Move stage back to the original position."""
        dx, dy = self._move_vector()
        try:
            self._controller.move_xy_relative_um(-dx, -dy)
        except Exception as e:
            logger.warning(f"Failed to move stage back: {e}")

    # ── v7.10: Z leg — the needle as the reference object ─────────

    def _expected_um_per_px(self, frame_w: int) -> tuple:
        """``(um_per_px, source)`` expected for this camera + objective, or
        ``(0.0, "")`` when genuinely unknown.

        v7.16 — operator: *"when calibrating the rotation of the microscope
        camera, it says that it needs to move less than 199 microns. this is
        fully not true. for it to know how far it can move it needs to know the
        objective its on and the measured magnification of the objective."*

        They are right, and the 199 µm was measurable proof: the bound had been
        sized from ``CameraManager.effective_um_per_px``, which held **another
        camera's** 0.389135 µm/px (unstamped, so it passed straight through
        un-rescaled). 0.25 x 2048 px x 0.389135 = **199.2 µm** exactly. At this
        camera's real scale the same bound is ~651 µm.

        Precedence, measurement-first and never a bare passthrough:

        1. this objective's own stored calibration, rescaled to the live width;
        2. **predicted from the camera's other objectives** — µm/px x
           magnification x width is a sensor property, so the camera's native
           1x scale divides by THIS objective's magnification. This is the leg
           the operator described, and it is the only one available for an
           objective that has never been calibrated;
        3. the live manager, but ONLY when it is both calibrated AND carries
           the resolution it was measured at. An unstamped value cannot be
           rescaled, and treating it as valid at whatever width happens to be
           running is what produced the 199.
        """
        try:
            from SupportClasses.ObjectiveCalibration import get_store
            store = get_store()
        except Exception:
            store = None

        # v7.16: both store legs rescale by width, so they need the CAPTURE
        # width — a centred crop shrinks the delivered frame without changing
        # what a pixel spans. The bound itself is still measured against the
        # DELIVERED frame (below), which is what has to stay in view.
        from SupportClasses.MosaicCalibration import capture_width_px
        cap_w = capture_width_px(self._camera_manager, self._cam_idx, frame_w)

        if store is not None and self._cam_key and self._objective:
            try:
                cal = store.get_calibration(
                    str(self._cam_key), str(self._objective))
                if cal:
                    base = float(cal.get("measured_um_per_px") or 0.0)
                    res = cal.get("resolution") or ()
                    cw = float(res[0]) if res else 0.0
                    if base > 0 and cw > 0 and cap_w > 0:
                        return (base * cw / float(cap_w),
                                f"stored {self._objective} calibration")
            except Exception:
                pass

        if store is not None and self._cam_key and self._objective:
            try:
                pred = store.predicted_um_per_px(
                    str(self._cam_key), str(self._objective), float(cap_w))
                if pred and pred > 0:
                    return (pred,
                            f"this camera's other objectives / "
                            f"{self._objective} magnification")
            except Exception:
                pass

        mgr = self._camera_manager
        try:
            stamped = None
            getter = getattr(mgr, "get_um_per_px_resolution", None)
            if callable(getter):
                stamped = getter(self._cam_idx)
            checker = getattr(mgr, "is_um_per_px_calibrated", None)
            ok = bool(checker(self._cam_idx)) if callable(checker) else False
            if ok and stamped:
                v = float(mgr.effective_um_per_px(self._cam_idx, frame_w) or 0)
                if v > 0:
                    return (v, "the camera's stored µm/px")
        except Exception:
            pass
        return (0.0, "")

    def _frame_wh(self) -> tuple:
        """(w, h) of the live frame, or (0, 0)."""
        try:
            frame = self._camera_manager.cameras[self._cam_idx].get_current_frame()
            if frame is not None and getattr(frame, "shape", None):
                return (int(frame.shape[1]), int(frame.shape[0]))
        except Exception:
            pass
        return (0, 0)

    def _set_z_note(self, text: str, color: str = "subtext0") -> None:
        self._lbl_z_result.setText(text)
        self._lbl_z_result.setStyleSheet(
            f"color: {COLORS.get(color, COLORS['subtext0'])}; "
            f"font-size: {scaled_font_size(9)}pt;")

    def _start_z_leg(self):
        """Retract the needle a known distance, track IT, and solve both legs.

        Safety: the move is in the HEIGHT frame via ``move_z_user_relative``, so
        it is polarity-safe on either ``z_up_sign``, and it goes **UP first** —
        away from the plate — then returns. A calibration that descended a blind
        200 µm could put a needle through glass.

        Tracking is template-based, NOT the whole-frame phase correlation the XY
        leg uses. On a Z move only the needle moves; the background is
        stationary, so phase correlation would faithfully report the
        background's zero displacement. A near-zero reading is therefore
        surfaced as "the tracker locked onto the background", which is a thing
        the operator can actually fix.
        """
        mgr = self._camera_manager
        if mgr is None or not mgr.is_running(self._cam_idx):
            self._set_z_note("Start the camera first.", "red")
            return
        mover = getattr(self._controller, "move_z_user_relative", None)
        if not callable(mover):
            self._set_z_note(
                "This controller cannot jog Z in the height frame, so the Z leg "
                "cannot run safely here.", "red")
            return

        dz = float(self._spin_z.value())
        # In-frame pre-check. Without it, "the move was along the optical axis"
        # and "the feature left the sensor" arrive as the SAME symptom — a tiny,
        # low-confidence displacement — and the operator cannot tell which they
        # are looking at.
        fw, fh = self._frame_wh()
        try:
            from SupportClasses.NeedleCameraCalibration import in_frame_refusal
            # v7.16: same resolution order as the XY leg — never a bare
            # passthrough from the manager (see _expected_um_per_px).
            if self.result_um_per_px:
                u_guess, src = float(self.result_um_per_px), "this measurement"
            else:
                u_guess, src = self._expected_um_per_px(fw)
            why = in_frame_refusal(dz, u_guess, fw, fh, what="Z move")
            if why:
                self._set_z_note(f"{why} (scale from {src}.)", "red")
                return
        except Exception as exc:
            logger.debug(f"Z leg in-frame check skipped: {exc}")

        try:
            from SupportClasses.VisionDetector import (
                select_trackable_patch, find_template)
        except ImportError:
            self._set_z_note("Vision module unavailable.", "red")
            return

        self._btn_z.setEnabled(False)
        self._set_z_note(f"Retracting the needle {dz:.0f} µm…", "yellow")
        self._lbl_z_result.repaint()

        cam = mgr.cameras[self._cam_idx]
        before = cam.capture_fresh_frame()
        if before is None:
            self._set_z_note("Could not capture the reference frame.", "red")
            self._btn_z.setEnabled(True)
            return
        picked = select_trackable_patch(before)
        if picked is None:
            self._set_z_note(
                "No trackable feature — the view is too flat. Focus on the "
                "needle so its edge gives the tracker something to hold.", "red")
            self._btn_z.setEnabled(True)
            return
        cx0, cy0, patch = picked

        moved = False
        try:
            mover(dz / 1000.0)          # height frame, mm, + = up = safe
            moved = True
            QTimer.singleShot(int(self._spin_settle.value()),
                              lambda: self._finish_z_leg(
                                  dz, cx0, cy0, patch, find_template))
        except Exception as exc:
            if moved:
                self._return_z(dz, mover)
            self._set_z_note(f"Z move failed: {exc}", "red")
            self._btn_z.setEnabled(True)

    def _return_z(self, dz: float, mover) -> None:
        try:
            mover(-dz / 1000.0)
        except Exception as exc:
            logger.warning(f"Failed to return Z after the calibration leg: {exc}")

    def _finish_z_leg(self, dz, cx0, cy0, patch, find_template):
        """Capture after the Z move, solve, and ALWAYS put Z back."""
        mover = getattr(self._controller, "move_z_user_relative", None)
        try:
            cam = self._camera_manager.cameras[self._cam_idx]
            after = cam.capture_fresh_frame()
            if after is None:
                self._set_z_note("Could not capture the moved frame.", "red")
                return
            found = find_template(after, patch)
            if found is None:
                self._set_z_note(
                    "Lost the tracked feature after the Z move. Try a smaller "
                    "Z step.", "red")
                return
            cx1, cy1, conf = found
            if conf < 0.35:
                self._set_z_note(
                    f"Low match confidence ({conf:.2f}) after the Z move — the "
                    f"feature may have left the frame or defocused. Try a "
                    f"smaller Z step.", "red")
                return
            self._solve_needle_axes(dz, cx1 - cx0, cy1 - cy0, conf)
        finally:
            if callable(mover):
                self._return_z(dz, mover)
            self._btn_z.setEnabled(True)

    def _solve_needle_axes(self, dz, zdx, zdy, conf):
        from SupportClasses.NeedleCameraCalibration import (
            solve_needle_camera_axes)
        kw = {}
        if self._lateral_px is not None:
            kw = dict(lateral_um=float(self._spin_distance.value()),
                      lateral_dx_px=self._lateral_px[0],
                      lateral_dy_px=self._lateral_px[1])
        axes = solve_needle_camera_axes(z_um=dz, z_dx_px=zdx, z_dy_px=zdy, **kw)
        why = axes.refusal()
        if why:
            self.result_needle_axes = None
            self._set_z_note(why, "red")
            return
        self.result_needle_axes = axes
        # The Z leg's µm/px is the one that cannot be foreshortened, so it
        # supersedes the XY leg's value — and the roll it measured supersedes
        # the one inferred from the lateral vector.
        self.result_um_per_px = axes.um_per_px
        self.result_view_roll_deg = -axes.roll_deg
        self._lbl_umpx.setText(f"{axes.um_per_px:.4f} µm/px")
        lines = [f"Z leg: {axes.z_travel_px:.0f} px (match {conf:.2f})"]
        if axes.has_lateral:
            lines.append(
                f"µm/px {axes.um_per_px:.4f} (XY leg alone said "
                f"{axes.um_per_px_lateral:.4f})")
            lines.append(
                f"roll {axes.roll_deg:+.2f}°, axes "
                f"{90 + axes.orthogonality_err_deg:.1f}° apart")
        else:
            lines.append(f"µm/px {axes.um_per_px:.4f}")
            lines.append(f"roll {axes.roll_deg:+.2f}°")
        lines += axes.advisories()
        self._set_z_note("\n".join(lines), "green")
        # A Z-only run is a complete, committable result.
        self._btn_box.button(
            QDialogButtonBox.StandardButton.Ok).setEnabled(True)

    def _accept_result(self):
        """Accept the calibration result and close."""
        if self.result_um_per_px is not None:
            logger.info(
                f"Pixel calibration accepted: {self.result_um_per_px:.4f} "
                f"µm/px @ {self.result_rotation_deg:.1f}°")
            self.accept()
        else:
            self.reject()
