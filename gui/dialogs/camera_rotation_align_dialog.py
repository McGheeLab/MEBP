"""camera_rotation_align_dialog.py — square up a camera mount, live.

v7.10 (operator): "we have a great way to detect camera rotation, but what I
want to do is fix the camera rotation to as close to 0, 90, 180, 270 as
possible by physically rotating the actual camera at the mount. The software
will first capture the rotation via calibration, then we can monitor the
features as I rotate the frame, it tells me to keep turning the camera until I
rotate it into where it needs to be … rotate the calibrated view to the exact
rotation we want, then set its alpha such that when I physically rotate the
hardware it will align with the frame. Then we can recalculate the alignment."

The app already MEASURES camera rotation and corrects for it in software. This
dialog helps REMOVE it physically, so the correction becomes a lossless
quarter-turn instead of an arbitrary resample.

    1. Freeze a reference frame at the camera's current physical angle.
    2. Show that reference pre-rotated to the target, translucent, over the
       live feed — turn the camera until the two coincide.
    3. Read a LIVE "degrees still to turn" number, measured by Fourier-Mellin
       against the frozen reference.
    4. Re-run the existing rotation calibration to confirm and commit.

NO STAGE MOTION happens in steps 1-3 — the measurement is purely optical, so
the needle is never at risk while the operator has their hands on the rig. Step
4 delegates to the host's existing calibration handler (which does move the
stage), and the worker is stopped before it launches.

THIS DIALOG WRITES NOTHING. Everything it displays is guidance; the committed
value always comes from step 4's stage-motion measurement.

Threading: Fourier-Mellin costs ~10 ms per sample at 256², and the camera
views already spend ~6-33 ms per frame in ``QImage.transformed``. Sampling runs
on a daemon worker (it genuinely releases the GIL — 2.4x measured) and results
arrive on the GUI thread through a bridge QObject, matching
``timing_calibration_workflow``.
"""

from __future__ import annotations

import logging
import threading
import time

import numpy as np

from PySide6.QtCore import Qt, QObject, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QGroupBox, QHBoxLayout, QLabel, QMessageBox, QPushButton, QSizePolicy,
    QSlider, QVBoxLayout, QWidget,
)

from gui.styles import COLORS
from gui.scaling import s, sf, scaled_font_size
from gui.widgets.camera_feed_view import CameraFeedView

logger = logging.getLogger(__name__)

try:
    from SupportClasses.CameraRotationTracker import (
        RotationTracker, TRACK_SIZE, edge_rgba, make_ghost,
        nearest_square_rotation, prepare_frame, target_image_rotation_deg,
        wrap_deg,
    )
    TRACKER_AVAILABLE = True
except Exception as _e:                                   # pragma: no cover
    TRACKER_AVAILABLE = False
    logger.warning(f"CameraRotationTracker unavailable — square-up disabled: {_e}")

SAMPLE_PERIOD_S = 0.12
# Longest the ghost's larger side is rendered at. The view stretches it onto
# the displayed pixmap anyway, and a full-res warpAffine on a 10 Mpx sensor is
# pure waste.
GHOST_MAX_PX = 720
# No new camera frame for this long ⇒ the feed died. CameraWidget._grab_frame
# calls stop() on a read failure, after which get_current_frame() returns the
# same array forever — without this the readout would hold its last value while
# the operator kept turning.
FEED_WATCHDOG_S = 1.5
DEFAULT_TOLERANCE_DEG = 1.0


# ── live signed-residual indicator ────────────────────────────────────────

class _ResidualDial(QWidget):
    """Centre-zero bar + big signed number: how far there is left to turn."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMinimumHeight(s(96))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self._residual: float | None = None
        self._tolerance = DEFAULT_TOLERANCE_DEG
        self._span = 15.0          # full-scale, auto-grown to fit
        self._state = "idle"       # idle | ok | live | lost
        self._note = ""

    def set_tolerance(self, deg: float) -> None:
        self._tolerance = max(0.05, float(deg))
        self.update()

    def set_reading(self, residual: float | None, state: str = "live",
                    note: str = "") -> None:
        self._residual = None if residual is None else float(residual)
        self._state = str(state)
        self._note = str(note)
        if self._residual is not None:
            self._span = max(15.0, abs(self._residual) * 1.25)
        self.update()

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor(COLORS.get("mantle", "#181825")))
        pad = s(8)
        bar_h = s(14)
        bar_y = h - pad - bar_h
        x0, bw = pad, max(1, w - 2 * pad)
        cx = x0 + bw / 2.0

        # number
        p.setFont(QFont("", max(10, int(scaled_font_size(22))), QFont.Bold))
        if self._residual is None:
            col = COLORS.get("overlay0", "#6c7086")
            txt = "—"
        elif abs(self._residual) <= self._tolerance:
            col = COLORS.get("green", "#a6e3a1")
            txt = f"{self._residual:+.2f}°"
        else:
            col = COLORS.get("peach", "#fab387")
            txt = f"{self._residual:+.2f}°"
        p.setPen(QColor(col))
        p.drawText(int(x0), int(pad), int(bw), int(bar_y - pad * 2),
                   Qt.AlignCenter, txt)

        if self._note:
            p.setFont(QFont("", max(7, int(sf(9)))))
            p.setPen(QColor(COLORS.get("subtext0", "#a6adc8")))
            p.drawText(int(x0), int(bar_y - s(16)), int(bw), s(14),
                       Qt.AlignCenter, self._note)

        # track
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(COLORS.get("surface0", "#313244")))
        p.drawRoundedRect(int(x0), int(bar_y), int(bw), int(bar_h),
                          bar_h / 2, bar_h / 2)
        # tolerance band
        tol_w = max(2.0, bw * (self._tolerance / self._span))
        p.setBrush(QColor(COLORS.get("green", "#a6e3a1")))
        p.setOpacity(0.30)
        p.drawRoundedRect(int(cx - tol_w), int(bar_y), int(tol_w * 2),
                          int(bar_h), bar_h / 2, bar_h / 2)
        p.setOpacity(1.0)
        # centre tick
        p.setPen(QPen(QColor(COLORS.get("surface2", "#585b70")), s(1)))
        p.drawLine(int(cx), int(bar_y - s(3)), int(cx), int(bar_y + bar_h + s(3)))

        if self._residual is not None:
            frac = max(-1.0, min(1.0, self._residual / self._span))
            mx = cx + frac * (bw / 2.0)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(col))
            r = bar_h / 2.0 + s(2)
            p.drawEllipse(int(mx - r), int(bar_y + bar_h / 2 - r),
                          int(r * 2), int(r * 2))
        p.end()


# ── worker → GUI bridge ───────────────────────────────────────────────────

class _AlignBridge(QObject):
    """Owns the cross-thread signals. Separate from the dialog so a queued
    emit arriving after teardown cannot land on a deleted C++ object."""
    sample = Signal(float, float, float, float, str)   # rot, resid, conf, hint_i, hint
    rejected = Signal()
    ghost = Signal(object)                             # ndarray (BGR or RGBA)
    reference = Signal(bool, str)                      # ok, message


class _AlignWorker:
    """Samples the camera and owns ALL mutable tracker state on one thread.

    Commands from the GUI (freeze a reference, retarget, switch ghost style)
    are queued under a lock and applied at the top of the loop, so the tracker
    is never touched from two threads.
    """

    def __init__(self, bridge: _AlignBridge, camera_widget, target_deg: float):
        self._bridge = bridge
        self._cam = camera_widget
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._tracker = RotationTracker(target_deg=target_deg)
        self._ref_raw = None
        self._pending_ref = True          # freeze on the first good frame
        self._pending_target: float | None = None
        self._pending_style = False
        self._edges = False
        self._last_seq = -1
        self._thread: threading.Thread | None = None

    # -- commands (GUI thread) -------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="camera-square-up")
        self._thread.start()

    def stop(self, timeout: float = 0.6) -> None:
        self._stop.set()
        t = self._thread
        if t is not None and t.is_alive():
            # Short, bounded join: never block the GUI thread on a worker that
            # may be mid-emit.
            t.join(timeout=timeout)
        self._thread = None

    def request_reference(self) -> None:
        with self._lock:
            self._pending_ref = True

    def set_target(self, phi: float) -> None:
        with self._lock:
            self._pending_target = float(phi)

    def set_edges(self, on: bool) -> None:
        with self._lock:
            self._edges = bool(on)
            self._pending_style = True

    # -- loop (worker thread) --------------------------------------------
    def _grab(self):
        try:
            return self._cam.get_current_frame()
        except Exception:
            return None

    def _emit_ghost(self, phi: float, edges: bool) -> None:
        if self._ref_raw is None:
            return
        try:
            a = self._ref_raw
            h, w = a.shape[:2]
            m = max(h, w)
            if m > GHOST_MAX_PX:
                import cv2
                k = GHOST_MAX_PX / float(m)
                a = cv2.resize(a, (max(1, int(w * k)), max(1, int(h * k))),
                               interpolation=cv2.INTER_AREA)
            rot = make_ghost(a, phi)
            out = edge_rgba(rot) if edges else rot
            self._bridge.ghost.emit(np.ascontiguousarray(out))
        except Exception as e:                            # pragma: no cover
            logger.debug(f"ghost build failed: {e}")

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                want_ref = self._pending_ref
                new_target = self._pending_target
                restyle = self._pending_style
                edges = self._edges
                self._pending_ref = False
                self._pending_target = None
                self._pending_style = False

            if new_target is not None:
                self._tracker.set_target(new_target)

            if want_ref:
                frame = self._grab()
                gray = None
                if frame is not None:
                    try:
                        gray = prepare_frame(frame)
                    except Exception as e:
                        logger.debug(f"prepare_frame failed: {e}")
                if gray is None:
                    self._bridge.reference.emit(
                        False, "No frame from the camera yet.")
                elif not self._tracker.set_reference(gray):
                    self._ref_raw = None
                    self._bridge.reference.emit(
                        False, "Not enough texture to track. Point the camera "
                               "at a feature (a well edge, a slide mark) and "
                               "focus, then freeze again.")
                else:
                    self._ref_raw = np.array(frame, copy=True)
                    self._bridge.reference.emit(True, "")
                    self._emit_ghost(self._tracker.target_deg, edges)
            elif (new_target is not None or restyle) and self._ref_raw is not None:
                self._emit_ghost(self._tracker.target_deg, edges)

            if self._tracker.has_reference:
                try:
                    seq = int(self._cam.frame_count_value())
                except Exception:
                    seq = self._last_seq + 1
                if seq != self._last_seq:
                    self._last_seq = seq
                    frame = self._grab()
                    got = None
                    if frame is not None:
                        try:
                            got = self._tracker.update(prepare_frame(frame),
                                                       time.monotonic())
                        except Exception as e:            # pragma: no cover
                            logger.debug(f"rotation sample failed: {e}")
                    if got is None:
                        self._bridge.rejected.emit()
                    else:
                        hint = self._tracker.direction_hint()
                        self._bridge.sample.emit(
                            float(self._tracker.rotated_deg or 0.0),
                            float(self._tracker.residual_deg or 0.0),
                            float(got.conf),
                            float(self._tracker.translation_px[0]),
                            hint)
            self._stop.wait(SAMPLE_PERIOD_S)


# ── the dialog ────────────────────────────────────────────────────────────

class CameraRotationAlignDialog(QDialog):
    """Guides the operator through physically squaring a camera in its mount.

    ``remeasure`` is a zero-arg callable supplied by the host that runs the
    EXISTING per-slot rotation calibration and returns True only when a value
    was committed. Re-using it keeps one commit path (manager + store +
    per-objective sync) rather than growing a second.
    """

    def __init__(self, camera_manager, cam_idx: int, *, role=None,
                 remeasure=None, parent=None):
        super().__init__(parent)
        self._mgr = camera_manager
        self._cam_idx = int(cam_idx)
        self._role = role
        self._remeasure = remeasure
        self._worker: _AlignWorker | None = None
        self._bridge = _AlignBridge()
        self._last_sample_t = 0.0
        self._last_frame_seq = -1
        self._last_frame_t = time.monotonic()
        self._theta0: float | None = None
        self._nominal = 0.0
        self._phi_target = 0.0
        self._flip_sign = 1.0            # operator's "flip ghost direction"
        self._orient_key = None

        self.setWindowTitle(f"Square up camera mount — Cam {self._cam_idx + 1}")
        self.setModal(True)
        self._build_ui()
        self._refresh_target(force=True)
        self._start_feed()
        self._start_worker()

        self._tick_timer = QTimer(self)
        self._tick_timer.setInterval(300)
        self._tick_timer.timeout.connect(self._tick)
        self._tick_timer.start()

    # -- construction -----------------------------------------------------
    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setSpacing(s(8))

        self._lbl_head = QLabel("")
        self._lbl_head.setWordWrap(True)
        outer.addWidget(self._lbl_head)

        self._lbl_warn = QLabel("")
        self._lbl_warn.setWordWrap(True)
        self._lbl_warn.setStyleSheet(f"color: {COLORS.get('yellow', '#f9e2af')};")
        self._lbl_warn.setVisible(False)
        outer.addWidget(self._lbl_warn)

        body = QHBoxLayout()
        body.setSpacing(s(8))
        outer.addLayout(body, stretch=1)

        # Live feed. auto_orient is OFF on purpose: correct what is NOT
        # changing (the flips are a fixed property of the optical path) and
        # show what IS (the rotation), so the operator watches the thing they
        # are actually adjusting. enable_settings off — a resolution change
        # mid-align would invalidate the frozen reference.
        self._feed = CameraFeedView(
            camera_manager=self._mgr, cam_idx=self._cam_idx,
            show_crosshair=True, enable_settings=False, auto_orient=False,
            label=f"Camera {self._cam_idx + 1} — live + ghost")
        self._feed.setMinimumSize(s(460), s(340))
        body.addWidget(self._feed, stretch=3)

        side = QVBoxLayout()
        side.setSpacing(s(6))
        body.addLayout(side, stretch=2)

        self._dial = _ResidualDial()
        side.addWidget(self._dial)

        self._lbl_state = QLabel("Freezing reference…")
        self._lbl_state.setWordWrap(True)
        side.addWidget(self._lbl_state)

        grp = QGroupBox("Ghost")
        gl = QVBoxLayout(grp)
        row = QHBoxLayout()
        row.addWidget(QLabel("Target"))
        self._cmb_target = QComboBox()
        self._cmb_target.addItem("Nearest", None)
        for v in (0.0, 90.0, 180.0, -90.0):
            self._cmb_target.addItem(f"{v:g}°", v)
        self._cmb_target.currentIndexChanged.connect(
            lambda *_: self._refresh_target(force=True))
        row.addWidget(self._cmb_target, stretch=1)
        gl.addLayout(row)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Opacity"))
        self._sld_alpha = QSlider(Qt.Horizontal)
        self._sld_alpha.setRange(0, 100)
        self._sld_alpha.setValue(45)
        self._sld_alpha.valueChanged.connect(lambda *_: self._push_ghost())
        row2.addWidget(self._sld_alpha, stretch=1)
        gl.addLayout(row2)

        self._chk_edges = QCheckBox("Edge outlines instead of a photo")
        self._chk_edges.toggled.connect(self._on_edges_toggled)
        gl.addWidget(self._chk_edges)

        self._chk_follow = QCheckBox("Keep the ghost centred on the live image")
        self._chk_follow.setChecked(True)
        self._chk_follow.setToolTip(
            "A mount axis that is not the optical axis slides the image as it "
            "turns. Tracking that out lets you judge angle alone.")
        gl.addWidget(self._chk_follow)

        brow = QHBoxLayout()
        self._btn_freeze = QPushButton("Freeze reference")
        self._btn_freeze.clicked.connect(self._on_freeze)
        brow.addWidget(self._btn_freeze)
        self._btn_flip = QPushButton("Flip ghost direction")
        self._btn_flip.setToolTip(
            "If re-measuring shows you moved the wrong way by twice the "
            "angle, the ghost was inverted for this optical path — flip it "
            "and turn back.")
        self._btn_flip.clicked.connect(self._on_flip)
        brow.addWidget(self._btn_flip)
        gl.addLayout(brow)
        side.addWidget(grp)

        grp2 = QGroupBox("Confirm")
        g2 = QVBoxLayout(grp2)
        trow = QHBoxLayout()
        trow.addWidget(QLabel("Tolerance"))
        self._spn_tol = QDoubleSpinBox()
        self._spn_tol.setRange(0.1, 5.0)
        self._spn_tol.setSingleStep(0.1)
        self._spn_tol.setDecimals(2)
        self._spn_tol.setValue(DEFAULT_TOLERANCE_DEG)
        self._spn_tol.setSuffix(" °")
        self._spn_tol.valueChanged.connect(
            lambda v: self._dial.set_tolerance(float(v)))
        trow.addWidget(self._spn_tol)
        trow.addStretch(1)
        g2.addLayout(trow)
        self._btn_remeasure = QPushButton("⟳ Re-measure rotation…")
        self._btn_remeasure.setToolTip(
            "Lock the mount first. This moves the stage to measure the "
            "camera's true rotation and commits it.")
        self._btn_remeasure.clicked.connect(self._on_remeasure)
        self._btn_remeasure.setEnabled(self._remeasure is not None)
        g2.addWidget(self._btn_remeasure)
        self._lbl_result = QLabel("Not re-measured yet.")
        self._lbl_result.setWordWrap(True)
        g2.addWidget(self._lbl_result)
        side.addWidget(grp2)
        side.addStretch(1)

        box = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        box.rejected.connect(self.reject)
        box.accepted.connect(self.accept)
        outer.addWidget(box)

        self._bridge.sample.connect(self._on_sample)
        self._bridge.rejected.connect(self._on_rejected)
        self._bridge.ghost.connect(self._on_ghost)
        self._bridge.reference.connect(self._on_reference)

    # -- orientation / target --------------------------------------------
    def _orientation(self) -> tuple[bool, bool, float | None]:
        """(flip_x, flip_y, theta) read LIVE — every one of these can be
        changed on the slot card behind this dialog, and flipping either
        parity inverts the required image rotation."""
        flip_x = flip_y = False
        theta = None
        try:
            fo = getattr(self._mgr, "full_orientation", None)
            if callable(fo):
                flip_x, flip_y, theta = fo(self._cam_idx)
            else:
                flip_x = bool(self._mgr.get_mirrored(self._cam_idx))
                theta = self._mgr.get_rotation_deg(self._cam_idx)
        except Exception:
            pass
        try:
            t = self._mgr.get_rotation_deg(self._cam_idx)
            theta = None if t is None else float(t)
        except Exception:
            pass
        return bool(flip_x), bool(flip_y), theta

    def _refresh_target(self, force: bool = False) -> None:
        flip_x, flip_y, theta = self._orientation()
        pinned = self._cmb_target.currentData()
        key = (flip_x, flip_y, None if theta is None else round(theta, 4),
               pinned, self._flip_sign)
        if not force and key == self._orient_key:
            return
        self._orient_key = key
        if theta is None:
            self._lbl_head.setText(
                "This camera's rotation has not been measured yet — run "
                "<b>⟳ Rotation…</b> on the slot card first, then come back.")
            self._phi_target = 0.0
            self._dial.set_reading(None, "idle", "no calibration")
            return
        if self._theta0 is None:
            self._theta0 = float(theta)
        nominal = (nearest_square_rotation(theta)[0] if pinned is None
                   else float(pinned))
        self._nominal = nominal
        delta = wrap_deg(float(theta) - nominal)
        mirrored_net = (flip_x != flip_y)
        self._phi_target = self._flip_sign * target_image_rotation_deg(
            theta, mirrored_net, nominal)
        self._lbl_head.setText(
            f"Measured rotation vs stage <b>{float(theta):+.2f}°</b> → target "
            f"<b>{nominal:g}°</b>. Turn the camera in its mount until the "
            f"reading below reaches 0 (that is <b>{abs(delta):.2f}°</b> of "
            f"rotation), then lock it and re-measure.")
        warn = []
        if mirrored_net:
            warn.append(
                "This camera's image is mirrored. A mirror cannot be removed "
                "by rotating — only the tilt is fixed here; the flip stays "
                "corrected in software.")
        if abs(delta) > 60.0:
            warn.append(
                f"{abs(delta):.0f}° is a large turn. Check the mount can "
                "actually rotate that far before loosening it.")
        self._lbl_warn.setText("  ".join(warn))
        self._lbl_warn.setVisible(bool(warn))
        # Push the display flips (fixed) but NOT the rotation (the thing being
        # adjusted), and retarget the worker.
        try:
            self._feed.set_view_orientation(mirrored=flip_x, rotation_deg=0.0,
                                            flip_y=flip_y)
        except Exception:
            pass
        if self._worker is not None:
            self._worker.set_target(self._phi_target)

    # -- lifecycle --------------------------------------------------------
    def _start_feed(self) -> None:
        try:
            if not self._mgr.is_running(self._cam_idx):
                self._mgr.start(self._cam_idx)
        except Exception as e:                            # pragma: no cover
            logger.debug(f"square-up feed start skipped: {e}")

    def _camera_widget(self):
        try:
            return self._mgr.cameras[self._cam_idx]
        except Exception:
            return None

    def _start_worker(self) -> None:
        if not TRACKER_AVAILABLE:
            self._lbl_state.setText(
                "Rotation tracking is unavailable (OpenCV missing).")
            return
        cam = self._camera_widget()
        if cam is None:
            self._lbl_state.setText("Camera not available.")
            return
        self._worker = _AlignWorker(self._bridge, cam, self._phi_target)
        self._worker.set_edges(self._chk_edges.isChecked())
        self._worker.start()

    def _stop_worker(self) -> None:
        w, self._worker = self._worker, None
        if w is not None:
            w.stop()
        try:
            self._feed.set_alignment_ghost(None)
        except Exception:
            pass

    def hideEvent(self, event):
        self._tick_timer.stop()
        self._stop_worker()
        super().hideEvent(event)

    def closeEvent(self, event):
        self._tick_timer.stop()
        self._stop_worker()
        super().closeEvent(event)

    def done(self, r):
        self._tick_timer.stop()
        self._stop_worker()
        super().done(r)

    # -- slots ------------------------------------------------------------
    def _on_freeze(self) -> None:
        if self._worker is not None:
            self._worker.request_reference()
            self._lbl_state.setText("Freezing reference…")

    def _on_flip(self) -> None:
        self._flip_sign = -self._flip_sign
        self._refresh_target(force=True)

    def _on_edges_toggled(self, on: bool) -> None:
        if self._worker is not None:
            self._worker.set_edges(bool(on))

    def _on_reference(self, ok: bool, message: str) -> None:
        if ok:
            self._lbl_state.setText(
                "Reference frozen. Start turning the camera either way — the "
                "arrow locks on after a couple of degrees.")
        else:
            self._lbl_state.setText(message)
            self._dial.set_reading(None, "idle", "no reference")

    def _on_ghost(self, arr) -> None:
        self._ghost_arr = arr
        self._push_ghost()

    def _push_ghost(self) -> None:
        arr = getattr(self, "_ghost_arr", None)
        if arr is None:
            return
        try:
            a = np.ascontiguousarray(arr)
            h, w = a.shape[:2]
            if a.ndim == 3 and a.shape[2] == 4:
                img = QImage(a.data, w, h, w * 4,
                             QImage.Format.Format_RGBA8888).copy()
            elif a.ndim == 3:
                img = QImage(a.data, w, h, w * 3,
                             QImage.Format.Format_BGR888).copy()
            else:
                img = QImage(a.data, w, h, w,
                             QImage.Format.Format_Grayscale8).copy()
            self._feed.set_alignment_ghost(
                img, opacity=self._sld_alpha.value() / 100.0,
                offset_px=getattr(self, "_ghost_offset", (0.0, 0.0)))
        except Exception as e:                            # pragma: no cover
            logger.debug(f"ghost push failed: {e}")

    def _on_rejected(self) -> None:
        # Do not blank immediately — a single low-confidence frame during a
        # fast turn is normal. _tick() blanks once the gap exceeds STALE.
        pass

    def _on_sample(self, rotated: float, residual: float, conf: float,
                   dx_px: float, hint: str) -> None:
        self._last_sample_t = time.monotonic()
        tol = float(self._spn_tol.value())
        if abs(residual) <= tol:
            note = "squared up — lock the mount and re-measure"
        elif hint == "good":
            note = "keep turning this way"
        elif hint == "reverse":
            note = "wrong way — turn back"
        else:
            note = "turn either way to lock the direction on"
        self._dial.set_reading(residual, "live", note)
        self._lbl_state.setText(
            f"turned {rotated:+.2f}°   ·   confidence {conf:.2f}")
        if self._chk_follow.isChecked():
            self._apply_ghost_offset(dx_px)

    def _apply_ghost_offset(self, dx_px: float) -> None:
        """Translate the ghost so it stays overlaid as the image slides.

        ``dx_px`` arrives in prepared-frame (TRACK_SIZE²) pixels; the ghost is
        drawn in displayed pixels, so scale by the displayed width. Only the
        magnitude matters for legibility — the operator judges angle."""
        try:
            pm = self._feed._last_pixmap
            if pm is None or pm.isNull():
                return
            k = pm.width() / float(TRACK_SIZE)
            self._ghost_offset = (dx_px * k, 0.0)
        except Exception:
            self._ghost_offset = (0.0, 0.0)

    def _feed_watchdog_s(self, cam) -> float:
        """How long without a new frame means the feed really stopped.

        v7.14: ``frame_count_value()`` now counts SENSOR frames rather than
        display-timer ticks, so a camera running a long exposure advances it
        slowly BY DESIGN. A flat 1.5 s would call a perfectly healthy 2 s
        exposure "feed stopped" and tell the operator to restart the camera.
        Scale the threshold to what one frame actually costs; the flat value
        stays the floor for a fast camera.
        """
        try:
            from SupportClasses.CaptureTiming import frame_period_s
            getter = getattr(cam, "get_hw_settings", None)
            period = frame_period_s(getter()) if callable(getter) else None
        except Exception:
            period = None
        if not period:
            return FEED_WATCHDOG_S
        return max(FEED_WATCHDOG_S, 3.0 * float(period))

    def _tick(self) -> None:
        self._refresh_target()
        now = time.monotonic()
        cam = self._camera_widget()
        try:
            seq = int(cam.frame_count_value()) if cam is not None else -1
        except Exception:
            seq = -1
        if seq != self._last_frame_seq:
            self._last_frame_seq = seq
            self._last_frame_t = now
        elif (now - self._last_frame_t) > self._feed_watchdog_s(cam):
            self._dial.set_reading(None, "lost", "feed stopped")
            self._lbl_state.setText(
                "No new frames from the camera — the feed stopped. Restart it "
                "on the slot card.")
            return
        if (self._worker is not None
                and (now - self._last_sample_t) > FEED_WATCHDOG_S
                and self._last_sample_t > 0.0):
            self._dial.set_reading(None, "lost", "lost lock")
            self._lbl_state.setText(
                "Lost the reference — turn back toward it, or freeze a new "
                "reference at the current angle.")

    def _on_remeasure(self) -> None:
        if self._remeasure is None:
            return
        # The calibration MOVES THE STAGE. Stop sampling first: this tool's
        # whole safety story is that nothing moves while it runs.
        self._tick_timer.stop()
        self._stop_worker()
        try:
            committed = bool(self._remeasure())
        except Exception as e:
            logger.exception("re-measure failed")
            QMessageBox.warning(self, "Re-measure", f"Calibration failed: {e}")
            committed = False
        if not committed:
            self._lbl_result.setText(
                "Not re-measured — the calibration was cancelled or refused. "
                "Nothing was written.")
        else:
            self._report_result()
        self._theta0 = None
        self._refresh_target(force=True)
        self._start_worker()
        self._tick_timer.start()
        if self._worker is not None:
            self._worker.request_reference()

    def _report_result(self) -> None:
        _fx, _fy, theta = self._orientation()
        if theta is None:
            self._lbl_result.setText("Re-measured, but no rotation was stored.")
            return
        nominal, delta = nearest_square_rotation(theta)
        tol = float(self._spn_tol.value())
        msg = (f"Re-measured <b>{float(theta):+.2f}°</b> — "
               f"{abs(delta):.2f}° from {nominal:g}°.")
        if abs(delta) <= tol:
            msg += " ✓ squared up."
        self._lbl_result.setText(msg)
        self._lbl_result.setStyleSheet(
            f"color: {COLORS.get('green' if abs(delta) <= tol else 'peach')};")


def suggest_reverification(role=None) -> str:
    """What a physical rotation invalidates. Advisory — all of these fail
    SAFELY (they refuse or look stale); none drives the hardware anywhere."""
    lines = [
        "Re-save the mosaic re-anchor feature and any needle focus templates: "
        "those are raw image patches and template matching is not "
        "rotation-invariant, so they will simply stop matching.",
        "Saved mosaic composites stay valid — tiles are baked into STAGE axes, "
        "not sensor axes.",
        "µm/px is unchanged by a pure rotation. If the mount is a C-mount "
        "thread, though, turning it also moves the sensor along the optical "
        "axis (~0.10 mm for 45°) — re-check focus and µm/px in that case.",
    ]
    return "\n\n".join(lines)
