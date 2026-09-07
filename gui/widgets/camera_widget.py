"""
Camera Widget — Live microscope camera feed for calibration verification.

Provides a live camera feed widget that can be embedded in any page.
Uses OpenCV (cv2) for camera capture and converts frames to QImage
for display in a QLabel.

Features:
    - Auto-detect available cameras
    - Live feed with configurable FPS
    - Software brightness / contrast / gamma adjustment (hardware-independent)
    - Crosshair overlay for needle alignment
    - Snapshot capture (save to file)
    - Compact mode for multi-camera layouts

Falls back gracefully if OpenCV is not installed.

Usage::

    from gui.widgets.camera_widget import CameraWidget, CV2_AVAILABLE
    cam = CameraWidget(camera_label="Cam 1", compact=True, parent=self)
    cam.start_with_index(0)
    cam.set_brightness(20)
    cam.set_gamma(1.5)
    cam.stop()
"""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from datetime import datetime

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QCheckBox, QGroupBox, QFileDialog, QSlider,
    QFrame, QSizePolicy, QSpinBox,
)
from PySide6.QtCore import Qt, QTimer, Signal, QEvent, QObject
from PySide6.QtGui import QImage, QPixmap, QPainter, QPen, QColor

from gui.scaling import s, scaled_font_size
from SupportClasses.CameraCrop import CameraCrop

logger = logging.getLogger(__name__)

# Try to import OpenCV
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    logger.info("OpenCV (cv2) not installed — camera widget disabled")

# v7.3-camera: Try to import ToupCam backend
try:
    from gui.widgets.toupcam_backend import ToupCamBackend, TOUPCAM_AVAILABLE
except ImportError:
    TOUPCAM_AVAILABLE = False
    ToupCamBackend = None
    logger.info("ToupCam backend not available")

# v7.5.x: Try to import Andor SDK3 backend (ANDOR Zyla sCMOS via pylablib).
# Guarded/lazy exactly like ToupCam — importing never loads the SDK DLLs.
try:
    from gui.widgets.andor_backend import AndorBackend, ANDOR_AVAILABLE
except ImportError:
    ANDOR_AVAILABLE = False
    AndorBackend = None
    logger.info("Andor backend not available")

# v7.9.x: Try to import the Tucsen TUCam backend (Libra / Dhyana / Aries / FL).
# Guarded/lazy exactly like ToupCam and Andor — importing never loads TUCam.dll.
try:
    from gui.widgets.tucam_backend import TUCamBackend, TUCAM_AVAILABLE
except ImportError:
    TUCAM_AVAILABLE = False
    TUCamBackend = None
    logger.info("TUCam backend not available")

# v7.3.0: Try to import SimulatedCamera backend
try:
    from SupportClasses.SimulatedCamera import SimulatedCamera
    SIM_AVAILABLE = True
except ImportError:
    SIM_AVAILABLE = False
    SimulatedCamera = None

# v7.3-camera: Unified availability flag
CAMERA_AVAILABLE = (CV2_AVAILABLE or bool(TOUPCAM_AVAILABLE)
                    or bool(ANDOR_AVAILABLE) or bool(TUCAM_AVAILABLE)
                    or SIM_AVAILABLE)



def detect_cameras(max_index: int = 4) -> list[int]:
    """Probe camera indices and return those that are available.

    v7.2.5 S7: Reduced max_index 8->4, suppresses OpenCV errors,
    stops after 2 consecutive failures for speed.
    """
    if not CV2_AVAILABLE:
        return []

    # Suppress OpenCV error spam during probing
    old_log_level = None
    try:
        old_log_level = cv2.getLogLevel()
        cv2.setLogLevel(0)  # SILENT
    except (AttributeError, cv2.error):
        pass

    available = []
    consecutive_fails = 0
    # v7.5.x: open via DirectShow so OpenCV indices line up with the
    # DirectShow device enumeration used for per-camera identity
    # (gui/widgets/camera_identity.py). CAP_DSHOW is a Windows no-op
    # elsewhere; the constant exists in all OpenCV builds.
    _dshow = getattr(cv2, "CAP_DSHOW", 700)
    try:
        for idx in range(max_index):
            try:
                cap = cv2.VideoCapture(idx, _dshow)
                if cap.isOpened():
                    available.append(idx)
                    cap.release()
                    consecutive_fails = 0
                else:
                    consecutive_fails += 1
                    if consecutive_fails >= 2:
                        break
            except Exception:
                consecutive_fails += 1
                if consecutive_fails >= 2:
                    break
    finally:
        if old_log_level is not None:
            try:
                cv2.setLogLevel(old_log_level)
            except (AttributeError, cv2.error):
                pass

    logger.info(f"Camera detection: found {len(available)} camera(s) "
               f"at indices {available}")
    return available


def detect_toupcam_cameras() -> list[dict]:
    """v7.3-camera: Detect ToupTek/Bestscope cameras.

    Returns list of dicts with 'id' and 'displayname' keys.
    """
    if not TOUPCAM_AVAILABLE or ToupCamBackend is None:
        return []
    try:
        return ToupCamBackend.enumerate()
    except Exception as e:
        logger.warning(f"ToupCam detection error: {e}")
        return []


def detect_andor_cameras() -> list[dict]:
    """v7.5.x: Detect ANDOR SDK3 cameras (Zyla sCMOS).

    Returns list of dicts with 'id' (serial) and 'displayname' keys.
    """
    if not ANDOR_AVAILABLE or AndorBackend is None:
        return []
    try:
        return AndorBackend.enumerate()
    except Exception as e:
        logger.warning(f"Andor detection error: {e}")
        return []


def detect_tucam_cameras() -> list[dict]:
    """v7.9.x: Detect Tucsen TUCam cameras (Libra / Dhyana / Aries / FL).

    Returns list of dicts with 'id' (device index) and 'displayname' keys.
    """
    if not TUCAM_AVAILABLE or TUCamBackend is None:
        return []
    try:
        return TUCamBackend.enumerate()
    except Exception as e:
        logger.warning(f"TUCam detection error: {e}")
        return []




def detect_cameras_async(callback, max_index: int = 4):
    """
    v7.2.6: Detect cameras in a background thread.

    Args:
        callback: Called with list[int] of found camera indices.
                  Called from background thread — use QTimer.singleShot(0, fn)
                  to marshal back to main thread.
        max_index: Max camera index to probe.
    """
    import threading

    def _worker():
        indices = detect_cameras(max_index)
        callback(indices)

    t = threading.Thread(target=_worker, daemon=True, name="CameraDetect")
    t.start()
    return t


class CameraWidget(QWidget):
    """
    Live camera feed widget with crosshair overlay and software image
    adjustments (brightness, gamma).

    Args:
        camera_label: Display name shown in the header (e.g. "Camera 1").
        compact:      If True, uses a smaller minimum size suitable for
                      multi-camera grid layouts.
        show_controls: If True, show the built-in control row.  When False,
                       the camera is controlled externally (e.g. from a
                       context-panel).

    Emits:
        frame_captured(QImage): Every time a new frame is captured.
    """

    frame_captured = Signal(object)  # QImage

    # v7.4.4: Edge-pick mode (Needle Location workflow). Emits the
    # frame-pixel coordinate (cx_px, cy_px) of each user click while
    # edge-pick mode is enabled. The caller decides whether the click
    # is "left edge" or "right edge".
    pixel_clicked = Signal(float, float)

    # v7.5.x: internal — carries the result of an async device open from the
    # worker thread back to the GUI thread (queued connection). The blocking
    # cv2.VideoCapture / ToupCam / Andor open runs off the GUI thread (see
    # start_async) so it can't freeze the UI at startup; this signal marshals
    # the opened handle back so the display QTimer is started on the GUI thread.
    _open_result = Signal(object)  # dict payload

    def __init__(
        self,
        camera_label: str = "Camera",
        compact: bool = False,
        show_controls: bool = True,
        parent=None,
    ):
        super().__init__(parent)
        self._capture = None
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._grab_frame)
        self._camera_index = 0
        self._show_crosshair = True
        self._fps = 15
        # v7.6: set while a print needs the GUI event loop (see set_throttled).
        self._throttled = False
        self._running = False
        self._camera_label = camera_label
        self._compact = compact

        # Software image adjustments (display-only — applied AFTER the raw
        # frame is cached for detection, so vision/calibration see real
        # sensor data).
        self._brightness: int = 0       # -100 … +100  (additive offset)
        self._contrast: float = 1.0     # 0.1 … 3.0    (×, pivots on mid-gray)
        self._gamma: float = 1.0        # 0.1 … 3.0
        self._gamma_lut = None          # Precomputed LUT for speed

        # v7.3-camera: ToupCam backend state
        self._toupcam = None
        # v7.5.x: Andor SDK3 backend state (ANDOR Zyla sCMOS)
        self._andor = None
        # v7.9.x: Tucsen TUCam backend state (Libra / Dhyana / Aries / FL)
        self._tucam = None
        # "opencv" | "toupcam" | "andor" | "tucam" | "simulated"
        self._backend_type = "opencv"

        # v7.5.x: async-open state (see start_async). ``_open_token`` supersedes
        # an in-flight open when stop()/another start happens; ``_opening`` marks
        # an open in progress so re-entrant start calls are ignored.
        self._open_token = 0
        self._opening = False
        self._pending_on_done = None
        self._open_result.connect(self._on_open_result)

        # v7.5.x: mirrored-view flag. When True the frame is flipped
        # horizontally at the SOURCE (in _grab_frame, before the raw cache) so
        # the whole pipeline — the displayed feed AND the cached raw frame used
        # for detection / mosaic / click-mapping — sees an un-mirrored image.
        # This corrects a physically-mirrored camera so what the operator sees
        # is NOT mirrored, and downstream code needs no further mirror handling.
        self._mirrored = False

        # v7.16: centred crop, applied at the frame SOURCE (all three egress
        # points below) so every surface — live view, mosaic, still capture,
        # video recording, detection — sees the same pixels. Default: no crop.
        self._crop = CameraCrop()
        # Last PRE-crop frame size (w, h), or None until a frame arrives. This
        # is the CAPTURE size: µm/px rescaling is only valid between capture
        # resolutions, so it must never be read off a cropped frame.
        self._capture_size: tuple[int, int] | None = None

        # v7.3.0: Thread-safe frame buffer for detection workers
        self._current_frame = None        # Latest BGR numpy array (or None)
        self._frame_lock = threading.Lock()
        # v7.5.x: monotonic frame counter, incremented each grab. Lets a worker
        # thread (e.g. the mosaic scanner) wait for N genuinely-new frames from
        # the GUI display timer after a stage move — draining any buffered
        # backlog so the captured frame is post-move — without touching the
        # camera backend itself (which only the grab timer reads).
        self._frame_seq = 0
        # v7.14: the backend's own acquisition count as of the last grab, so
        # _grab_frame can tell a NEW sensor frame from a repeat of the cached
        # one. None on backends that cannot report it (OpenCV / simulated,
        # where every read() is genuinely new anyway).
        self._frame_src_seq = None

        # v7.4.4: Edge-pick mode (Needle Location workflow)
        self._edge_pick_mode = False

        self._setup_ui(show_controls)

    # ── UI Construction ──────────────────────────────────────────

    def _setup_ui(self, show_controls: bool):
        layout = QVBoxLayout(self)
        layout.setSpacing(4)
        layout.setContentsMargins(0, 0, 0, 0)

        if not CAMERA_AVAILABLE:
            layout.addWidget(QLabel(
                "Camera unavailable — install OpenCV or ToupTek SDK:\n"
                "  pip install opencv-python"
            ))
            return

        if show_controls:
            # Header / controls row
            header = QHBoxLayout()
            header.setSpacing(4)

            header.addWidget(QLabel(f"<b>{self._camera_label}</b>"))

            header.addWidget(QLabel("Src:"))
            self.camera_combo = QComboBox()
            self.camera_combo.setMinimumWidth(s(100))
            self.camera_combo.setMaximumWidth(s(180))
            self._populate_cameras()
            header.addWidget(self.camera_combo)

            self.btn_start = QPushButton("▶ Start")
            self.btn_start.setMinimumWidth(s(64))
            self.btn_start.setToolTip("Start / Stop camera")
            self.btn_start.clicked.connect(self.toggle)
            header.addWidget(self.btn_start)

            self.chk_crosshair = QCheckBox("✛ Crosshair")
            self.chk_crosshair.setChecked(True)
            self.chk_crosshair.setToolTip("Toggle crosshair overlay")
            self.chk_crosshair.toggled.connect(self._on_crosshair_toggle)
            header.addWidget(self.chk_crosshair)

            btn_snap = QPushButton("📷 Snap")
            btn_snap.setMinimumWidth(s(56))
            btn_snap.setToolTip("Save snapshot to file")
            btn_snap.clicked.connect(self.take_snapshot)
            header.addWidget(btn_snap)

            # v7.3.2: Per-camera settings toggle
            self._btn_settings = QPushButton("⚙ Settings")
            self._btn_settings.setMinimumWidth(s(72))
            self._btn_settings.setToolTip("Brightness, gamma, FPS")
            self._btn_settings.setCheckable(True)
            self._btn_settings.toggled.connect(self._toggle_settings_panel)
            header.addWidget(self._btn_settings)

            header.addStretch()
            layout.addLayout(header)

            # v7.3.2: Collapsible per-camera settings panel
            self._settings_panel = QFrame()
            self._settings_panel.setObjectName("cardFrame")
            self._settings_panel.setVisible(False)
            sp_layout = QVBoxLayout(self._settings_panel)
            sp_layout.setSpacing(3)
            sp_layout.setContentsMargins(6, 4, 6, 4)

            # Brightness slider
            bri_row = QHBoxLayout()
            bri_row.addWidget(QLabel("Brightness:"))
            self._sld_brightness = QSlider(Qt.Horizontal)
            self._sld_brightness.setRange(-100, 100)
            self._sld_brightness.setValue(0)
            self._sld_brightness.valueChanged.connect(self._on_brightness_slider)
            bri_row.addWidget(self._sld_brightness)
            self._lbl_brightness = QLabel("0")
            self._lbl_brightness.setMinimumWidth(s(28))
            bri_row.addWidget(self._lbl_brightness)
            sp_layout.addLayout(bri_row)

            # Contrast slider
            con_row = QHBoxLayout()
            con_row.addWidget(QLabel("Contrast:"))
            self._sld_contrast = QSlider(Qt.Horizontal)
            self._sld_contrast.setRange(10, 300)
            self._sld_contrast.setValue(100)
            self._sld_contrast.valueChanged.connect(self._on_contrast_slider)
            con_row.addWidget(self._sld_contrast)
            self._lbl_contrast = QLabel("1.00")
            self._lbl_contrast.setMinimumWidth(s(28))
            con_row.addWidget(self._lbl_contrast)
            sp_layout.addLayout(con_row)

            # Gamma slider
            gam_row = QHBoxLayout()
            gam_row.addWidget(QLabel("Gamma:"))
            self._sld_gamma = QSlider(Qt.Horizontal)
            self._sld_gamma.setRange(10, 300)
            self._sld_gamma.setValue(100)
            self._sld_gamma.valueChanged.connect(self._on_gamma_slider)
            gam_row.addWidget(self._sld_gamma)
            self._lbl_gamma = QLabel("1.00")
            self._lbl_gamma.setMinimumWidth(s(28))
            gam_row.addWidget(self._lbl_gamma)
            sp_layout.addLayout(gam_row)

            # FPS spinner
            fps_row = QHBoxLayout()
            fps_row.addWidget(QLabel("FPS:"))
            self._spn_fps = QSpinBox()
            self._spn_fps.setRange(1, 60)
            self._spn_fps.setValue(self._fps)
            self._spn_fps.valueChanged.connect(self._on_fps_spinner)
            fps_row.addWidget(self._spn_fps)
            fps_row.addStretch()
            sp_layout.addLayout(fps_row)

            layout.addWidget(self._settings_panel)
        else:
            # Even without controls, provide a combo for internal use
            self.camera_combo = QComboBox()
            self.camera_combo.setVisible(False)
            self._populate_cameras()
            layout.addWidget(self.camera_combo)

        # Video display
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        if self._compact:
            self.video_label.setMinimumSize(s(200), s(150))
        else:
            self.video_label.setMinimumSize(s(320), s(240))
        self.video_label.setStyleSheet(
            "background-color: #181825; border: 1px solid #45475a;"
        )
        self.video_label.setText(f"{self._camera_label} — stopped")
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.video_label, stretch=1)

        # v7.4.4: Install event filter so we can intercept clicks on the
        # video label when edge-pick mode is active. Stays installed
        # regardless of mode — the filter early-outs when the mode is
        # off, so this has no effect outside the workflow.
        self.video_label.installEventFilter(self)

        # v7.3.0: Detection overlay (transparent, sits on top of video_label)
        try:
            from gui.widgets.detection_overlay import DetectionOverlay
            self._detection_overlay = DetectionOverlay(parent=self.video_label)
            self._detection_overlay.setGeometry(self.video_label.rect())
            self._detection_overlay.show()
        except ImportError:
            self._detection_overlay = None

    def _populate_cameras(self):
        """Show placeholder in camera combo -- no hardware probe.

        v7.2.5 S7: Camera detection is now lazy. This method just
        sets a placeholder. Call refresh_cameras() or start() to
        actually probe hardware.
        """
        self.camera_combo.clear()
        if not CV2_AVAILABLE:
            return
        self._cameras_detected = False
        self.camera_combo.addItem("Click Detect or Start", -1)

    def refresh_cameras(self, probe: "dict | None" = None):
        """Detect cameras and populate the combo.

        v7.3-camera: Detects both OpenCV and ToupCam cameras.
        Combo item data is a tuple: ("opencv", index) or ("toupcam", device_id).

        v7.5.x: ``probe`` is an optional pre-computed source inventory
        ``{"opencv": [indices], "dshow": [...], "toupcam": [...]}``. The
        CameraManager probes ONCE and passes the same inventory to every
        widget, so the slow OpenCV device probe runs a single time (and can be
        run off the GUI thread at startup) instead of each widget re-probing.
        When None, this widget probes synchronously itself (the lazy
        ``start()`` path).
        """
        self.camera_combo.clear()
        if not CAMERA_AVAILABLE:
            return

        if probe is None:
            # Synchronous self-probe (lazy first-start). Mirrors the inventory
            # the CameraManager builds.
            ds_cams = []
            opencv_indices = []
            if CV2_AVAILABLE:
                try:
                    from gui.widgets.camera_identity import enumerate_directshow_cameras
                    ds_cams = enumerate_directshow_cameras()
                except Exception:
                    ds_cams = []
                opencv_indices = detect_cameras()
            probe = {"opencv": opencv_indices, "dshow": ds_cams,
                     "toupcam": detect_toupcam_cameras(),
                     "andor": detect_andor_cameras(),
                     "tucam": detect_tucam_cameras()}

        # OpenCV cameras — v7.5.x: label with the DirectShow friendly name
        # + USB port tag (e.g. "Teslong Camera (port 6&29d1719c&2)") so two
        # identical cameras are distinguishable. Falls back to the index
        # when no DirectShow info is available.
        if CV2_AVAILABLE:
            ds_cams = probe.get("dshow") or []
            try:
                from gui.widgets.camera_identity import label_for
            except Exception:
                label_for = None
            for idx in (probe.get("opencv") or []):
                entry = next(
                    (c for c in ds_cams if c.get("index") == idx), None)
                if entry and label_for is not None:
                    text = label_for(entry["name"], entry["device_path"])
                else:
                    text = f"Camera {idx}"
                self.camera_combo.addItem(text, ("opencv", idx))

        # ToupCam cameras  (v7.3-camera)
        for tc_dev in (probe.get("toupcam") or []):
            name = tc_dev.get('displayname', 'ToupCam')
            dev_id = tc_dev.get('id', '')
            self.camera_combo.addItem(f"TC: {name}", ("toupcam", dev_id))

        # Andor SDK3 cameras (v7.5.x — ANDOR Zyla sCMOS)
        for an_dev in (probe.get("andor") or []):
            name = an_dev.get('displayname', 'Andor Zyla')
            dev_id = an_dev.get('id', '')
            self.camera_combo.addItem(f"Andor: {name}", ("andor", dev_id))

        # Tucsen TUCam cameras (v7.9.x — Libra / Dhyana / Aries / FL). The
        # displayname is the model the SDK itself reports (e.g. "Libra 25").
        for tu_dev in (probe.get("tucam") or []):
            name = tu_dev.get('displayname', 'Tucsen')
            dev_id = tu_dev.get('id', '')
            self.camera_combo.addItem(f"Tucsen: {name}", ("tucam", dev_id))

        # v7.3.0: Simulated camera
        if SIM_AVAILABLE:
            self.camera_combo.addItem(
                "SIM: Microscope", ("simulated", "microscope"))

        if self.camera_combo.count() == 0:
            self.camera_combo.addItem("No cameras found", -1)
        self._cameras_detected = True

    def start(self):
        """Start the camera feed using the currently selected combo index.

        v7.2.5 S7: Triggers lazy camera detection on first start.
        """
        # v7.3-camera: Support either backend
        if not CAMERA_AVAILABLE or self._running:
            return

        # Lazy detection: probe hardware on first start
        if not getattr(self, "_cameras_detected", False):
            self.refresh_cameras()

        cam_data = self.camera_combo.currentData()
        if cam_data is None or cam_data == -1:
            self.video_label.setText("No camera available")
            return

        # v7.3-camera: Route by backend type
        if isinstance(cam_data, tuple) and len(cam_data) == 2:
            backend_type, identifier = cam_data
            if backend_type == "toupcam":
                self._start_toupcam(identifier)
                return
            elif backend_type == "andor":
                self._start_andor(identifier)
                return
            elif backend_type == "tucam":
                self._start_tucam(identifier)
                return
            elif backend_type == "simulated":
                self._start_simulated(identifier)
                return
            else:
                self.start_with_index(identifier)
                return

        # Legacy: plain integer index (backward compat)
        if isinstance(cam_data, int) and cam_data >= 0:
            self.start_with_index(cam_data)

    def start_async(self, on_done=None):
        """Open the camera WITHOUT blocking the GUI thread.

        The blocking device open (``cv2.VideoCapture`` / ToupCam / Andor — each
        can take several seconds) runs on a daemon thread; when it completes the
        opened handle is marshalled back to the GUI thread via ``_open_result``,
        which assigns it and starts the display QTimer (``_on_open_result``).
        This is the fix for "the camera boot-up freezes the UI" at startup.

        ``on_done(success: bool)`` (optional) is invoked on the GUI thread once
        the camera is running or the open failed. The cheap simulated backend and
        the unavailable/already-running cases complete synchronously.
        """
        if not CAMERA_AVAILABLE or self._running or self._opening:
            if on_done:
                on_done(self._running)
            return

        # Lazy detection on first start (mirrors start()). Combos are normally
        # already populated by the CameraManager, so this is a no-op at startup.
        if not getattr(self, "_cameras_detected", False):
            self.refresh_cameras()

        cam_data = self.camera_combo.currentData()
        if cam_data is None or cam_data == -1:
            self.video_label.setText("No camera available")
            if on_done:
                on_done(False)
            return

        # Resolve backend + identifier.
        if isinstance(cam_data, tuple) and len(cam_data) == 2:
            backend_type, identifier = cam_data
        elif isinstance(cam_data, int) and cam_data >= 0:
            backend_type, identifier = "opencv", cam_data
        else:
            # Unknown shape — fall back to the synchronous path.
            self.start()
            if on_done:
                on_done(self._running)
            return

        # Simulated backend opens instantly — no worker thread needed.
        if backend_type == "simulated":
            self._start_simulated(identifier)
            if on_done:
                on_done(self._running)
            return

        self._opening = True
        self._open_token += 1
        token = self._open_token
        self._pending_on_done = on_done
        self.video_label.setText(f"{self._camera_label} — opening…")

        def _worker():
            payload = {"token": token, "backend": backend_type,
                       "identifier": identifier, "handle": None, "meta": None}
            try:
                if backend_type == "opencv":
                    cap = cv2.VideoCapture(
                        int(identifier), getattr(cv2, "CAP_DSHOW", 700))
                    if cap.isOpened():
                        payload["handle"] = cap
                    else:
                        try:
                            cap.release()
                        except Exception:
                            pass
                elif (backend_type == "toupcam" and TOUPCAM_AVAILABLE
                      and ToupCamBackend is not None):
                    tc = ToupCamBackend()
                    if tc.open(identifier):
                        payload["handle"] = tc
                        payload["meta"] = tc.get_resolution()
                elif (backend_type == "andor" and ANDOR_AVAILABLE
                      and AndorBackend is not None):
                    an = AndorBackend()
                    if an.open(identifier):
                        payload["handle"] = an
                        payload["meta"] = an.get_resolution()
                elif (backend_type == "tucam" and TUCAM_AVAILABLE
                      and TUCamBackend is not None):
                    tu = TUCamBackend()
                    if tu.open(identifier):
                        payload["handle"] = tu
                        payload["meta"] = tu.get_resolution()
            except Exception as exc:
                logger.warning(
                    f"{self._camera_label}: async camera open failed: {exc}")
                payload["handle"] = None
            # Marshal back to the GUI thread (queued — receiver lives there).
            self._open_result.emit(payload)

        threading.Thread(
            target=_worker, daemon=True,
            name=f"CamOpen-{self._camera_label}").start()

    def _on_open_result(self, payload):
        """GUI-thread continuation of start_async: adopt the opened handle and
        start the display timer, or report failure. Discards a stale result if
        stop()/another start superseded this open while it was in flight."""
        on_done = self._pending_on_done
        self._pending_on_done = None
        self._opening = False
        backend = payload.get("backend")
        handle = payload.get("handle")

        # Superseded (stop() or another start bumped the token) or something else
        # already started this widget → release the just-opened handle and bail.
        if payload.get("token") != self._open_token or self._running:
            self._release_handle(backend, handle)
            if on_done:
                on_done(self._running)
            return

        if handle is None:
            self.video_label.setText(f"Failed to open {backend} camera")
            if on_done:
                on_done(False)
            return

        if backend == "opencv":
            self._capture = handle
            self._camera_index = payload.get("identifier", self._camera_index)
        elif backend == "toupcam":
            self._toupcam = handle
        elif backend == "andor":
            self._andor = handle
        elif backend == "tucam":
            self._tucam = handle
        self._backend_type = backend
        self._running = True
        self._timer.start(int(1000 / self._fps))
        if hasattr(self, "btn_start"):
            self.btn_start.setText("⏹ Stop" if backend == "opencv" else "⏹")
        meta = payload.get("meta")
        if meta:
            logger.info(f"{self._camera_label}: {backend} started "
                        f"({meta[0]}x{meta[1]}) at {self._fps} FPS")
        else:
            logger.info(f"{self._camera_label}: {backend} started "
                        f"at {self._fps} FPS")
        if on_done:
            on_done(True)

    def _release_handle(self, backend, handle):
        """Best-effort release of an opened-but-discarded backend handle."""
        if handle is None:
            return
        try:
            # cv2.VideoCapture / ToupCam / Andor / TUCam all expose release().
            handle.release()
        except Exception:
            pass

    def start_with_index(self, camera_index: int):
        """Start the camera feed with a specific device index."""
        if not CV2_AVAILABLE or self._running:
            return

        # v7.5.x: match the CAP_DSHOW backend used during detection so the
        # opened device corresponds to the enumerated identity (see
        # detect_cameras / camera_identity).
        self._capture = cv2.VideoCapture(camera_index, getattr(cv2, "CAP_DSHOW", 700))
        if not self._capture.isOpened():
            self.video_label.setText(f"Failed to open camera {camera_index}")
            self._capture = None
            return

        self._camera_index = camera_index
        self._running = True
        # v7.5.x: assert the backend so a slot reused after a ToupCam/sim run
        # can never report a stale source/controls (see get_hw_settings).
        self._backend_type = "opencv"
        self._timer.start(int(1000 / self._fps))
        if hasattr(self, 'btn_start'):
            self.btn_start.setText("⏹ Stop")
        logger.info(f"{self._camera_label}: camera {camera_index} started at {self._fps} FPS")

    def stop(self):
        """Stop the camera feed."""
        # v7.5.x: supersede any async open still in flight (its _on_open_result
        # will see the bumped token, release the handle, and bail) so a stop
        # during startup can't leave a camera streaming after we think it's off.
        self._open_token += 1
        self._opening = False
        self._timer.stop()
        self._running = False
        if self._capture:
            self._capture.release()
            self._capture = None
        # v7.5.x: release the ToupCam SDK stream too (previously stop() only
        # released the OpenCV capture, so a ToupCam kept streaming and
        # _backend_type stayed stale — making get_hw_settings mislabel a later
        # OpenCV source as source='toupcam').
        tc = getattr(self, '_toupcam', None)
        if tc is not None:
            try:
                tc.release()
            except Exception:
                pass
            self._toupcam = None
        # v7.5.x: release the Andor SDK3 stream (stops acquisition + reader
        # thread), same rationale as the ToupCam release above.
        an = getattr(self, '_andor', None)
        if an is not None:
            try:
                an.release()
            except Exception:
                pass
            self._andor = None
        # v7.9.x: release the Tucsen stream (stops acquisition, joins the reader
        # thread, frees the SDK buffer and drops the API refcount) — same
        # rationale as the ToupCam/Andor releases above.
        tu = getattr(self, '_tucam', None)
        if tu is not None:
            try:
                tu.release()
            except Exception:
                pass
            self._tucam = None
        self._backend_type = "opencv"
        # v7.3.0: Clear simulated camera reference
        if hasattr(self, '_simulated_camera'):
            self._simulated_camera = None
        # v7.3.0: Clear frame buffer
        with self._frame_lock:
            self._current_frame = None
        self.video_label.setText(f"{self._camera_label} — stopped")
        if hasattr(self, 'btn_start'):
            self.btn_start.setText("▶ Start")
        logger.info(f"{self._camera_label}: camera stopped")

    def _start_toupcam(self, device_id: str):
        """v7.3-camera: Start a ToupCam camera feed."""
        if not TOUPCAM_AVAILABLE or ToupCamBackend is None or self._running:
            return

        self._toupcam = ToupCamBackend()
        if not self._toupcam.open(device_id):
            self.video_label.setText(f"Failed to open ToupCam")
            self._toupcam = None
            return

        w, h = self._toupcam.get_resolution()
        self._running = True
        self._backend_type = "toupcam"
        self._timer.start(int(1000 / self._fps))
        if hasattr(self, 'btn_start'):
            self.btn_start.setText("\u23f9")
        logger.info(f"{self._camera_label}: ToupCam started ({w}x{h}) at {self._fps} FPS")

    def _start_andor(self, device_id: str):
        """v7.5.x: Start an ANDOR SDK3 (Zyla) camera feed."""
        if not ANDOR_AVAILABLE or AndorBackend is None or self._running:
            return

        self._andor = AndorBackend()
        if not self._andor.open(device_id):
            self.video_label.setText("Failed to open Andor camera")
            self._andor = None
            return

        w, h = self._andor.get_resolution()
        self._running = True
        self._backend_type = "andor"
        self._timer.start(int(1000 / self._fps))
        if hasattr(self, 'btn_start'):
            self.btn_start.setText("\u23f9")
        logger.info(f"{self._camera_label}: Andor started ({w}x{h}) at {self._fps} FPS")

    def _start_tucam(self, device_id: str):
        """v7.9.x: Start a Tucsen (TUCam SDK) camera feed."""
        if not TUCAM_AVAILABLE or TUCamBackend is None or self._running:
            return

        self._tucam = TUCamBackend()
        if not self._tucam.open(device_id):
            self.video_label.setText("Failed to open Tucsen camera")
            self._tucam = None
            return

        w, h = self._tucam.get_resolution()
        self._running = True
        self._backend_type = "tucam"
        self._timer.start(int(1000 / self._fps))
        if hasattr(self, 'btn_start'):
            self.btn_start.setText("⏹")
        logger.info(f"{self._camera_label}: Tucsen started ({w}x{h}) "
                    f"at {self._fps} FPS")

    def _start_simulated(self, mode: str = "microscope"):
        """v7.3.0: Start the simulated microscope camera."""
        if not SIM_AVAILABLE or SimulatedCamera is None or self._running:
            return

        self._simulated_camera = SimulatedCamera()
        self._capture = self._simulated_camera  # read() compatible API
        self._backend_type = "simulated"
        self._running = True
        self._timer.start(int(1000 / self._fps))
        if hasattr(self, 'btn_start'):
            self.btn_start.setText("\u23f9")
        logger.info(f"{self._camera_label}: Simulated camera started at {self._fps} FPS")

    @property
    def simulated_camera(self) -> SimulatedCamera | None:
        """Access the SimulatedCamera instance (if backend is simulated)."""
        return getattr(self, '_simulated_camera', None)

    def toggle(self):
        """Toggle camera on/off."""
        if self._running:
            self.stop()
        else:
            self.start()

    # ── Image Adjustment Properties ───────────────────────────────

    def set_brightness(self, value: int):
        """Set software brightness offset (-100 to +100)."""
        self._brightness = max(-100, min(100, int(value)))
        sld = getattr(self, "_sld_brightness", None)
        if sld is not None and sld.value() != self._brightness:
            sld.blockSignals(True)
            sld.setValue(self._brightness)
            sld.blockSignals(False)
        if hasattr(self, "_lbl_brightness"):
            self._lbl_brightness.setText(str(self._brightness))

    def set_contrast(self, value: float):
        """Set software contrast multiplier (0.1 to 3.0).  1.0 = no change.

        Pivots on mid-gray (128) so raising contrast does not also brighten;
        combined with brightness in a single ``convertScaleAbs`` pass.
        """
        self._contrast = max(0.1, min(3.0, float(value)))
        sld = getattr(self, "_sld_contrast", None)
        if sld is not None:
            iv = int(round(self._contrast * 100))
            if sld.value() != iv:
                sld.blockSignals(True)
                sld.setValue(iv)
                sld.blockSignals(False)
        if hasattr(self, "_lbl_contrast"):
            self._lbl_contrast.setText(f"{self._contrast:.2f}")

    def set_gamma(self, value: float):
        """Set software gamma (0.1 to 3.0).  1.0 = no change."""
        self._gamma = max(0.1, min(3.0, float(value)))
        # Precompute a lookup table for speed
        inv_gamma = 1.0 / self._gamma
        self._gamma_lut = np.array(
            [((i / 255.0) ** inv_gamma) * 255 for i in range(256)]
        ).astype("uint8")
        sld = getattr(self, "_sld_gamma", None)
        if sld is not None:
            iv = int(round(self._gamma * 100))
            if sld.value() != iv:
                sld.blockSignals(True)
                sld.setValue(iv)
                sld.blockSignals(False)
        if hasattr(self, "_lbl_gamma"):
            self._lbl_gamma.setText(f"{self._gamma:.2f}")

    def set_mirrored(self, value: bool):
        """v7.5.x: store the camera's mirrored-view flag.

        Flag ONLY — the frame is NOT flipped here. Orientation (mirror +
        rotation) is applied per consumer: the DISPLAY via
        ``CameraFeedView.set_view_orientation`` (so what the operator sees is
        un-mirrored), click→stage via ``CameraManager.pixel_to_stage_offset``,
        and the mosaic via ``MosaicBuilder``. Keeping frames raw here lets the
        mosaic re-blend against a *changing* orientation (the interactive
        mosaic-orientation adjustment)."""
        self._mirrored = bool(value)

    @property
    def mirrored(self) -> bool:
        return bool(getattr(self, "_mirrored", False))

    # ── v7.16: centred crop (applied at the frame source) ─────────

    def set_crop(self, crop) -> None:
        """Set the centred crop applied to EVERY frame this widget delivers.

        Unlike ``set_mirrored`` (a flag consumers apply themselves), this one
        really does change the pixels — at the source, so the live view, the
        mosaic tiles, still captures, video recording, detection and the
        click→stage map all see one consistent frame size. See
        :mod:`SupportClasses.CameraCrop` for why it belongs here.
        """
        self._crop = CameraCrop.from_dict(crop) if not isinstance(
            crop, CameraCrop) else crop

    def crop(self) -> CameraCrop:
        return getattr(self, "_crop", None) or CameraCrop()

    def capture_size(self) -> "tuple[int, int] | None":
        """Last PRE-crop frame size ``(w, h)``, or None if none seen yet.

        This is what a µm/px calibration must be stamped with and rescaled
        against: cropping removes pixels without changing what a pixel spans,
        so comparing DELIVERED widths across a crop change would rescale µm/px
        by the crop fraction — a wrong scale that looks entirely plausible.
        """
        return getattr(self, "_capture_size", None)

    def _apply_crop(self, frame):
        """Crop one frame and record its pre-crop size. The single place the
        crop is applied, shared by all three frame egress points."""
        if frame is None:
            return None
        shape = getattr(frame, "shape", None)
        if shape and len(shape) >= 2:
            self._capture_size = (int(shape[1]), int(shape[0]))
        crop = getattr(self, "_crop", None)
        return crop.apply(frame) if crop is not None else frame

    def image_correction(self) -> dict:
        """Snapshot of the current correction as a plain dict (for persistence)."""
        return {
            "brightness": int(self._brightness),
            "contrast": float(self._contrast),
            "gamma": float(self._gamma),
        }

    def reset_image_correction(self):
        """Restore neutral brightness/contrast/gamma (no correction)."""
        self.set_brightness(0)
        self.set_contrast(1.0)
        self.set_gamma(1.0)

    @property
    def brightness(self) -> int:
        return self._brightness

    @property
    def contrast(self) -> float:
        return self._contrast

    @property
    def gamma(self) -> float:
        return self._gamma

    # ── Hardware (camera-side) controls (v7.5.x) ──────────────────
    # These drive the *camera firmware* via its SDK/driver (ToupCam SDK, or
    # OpenCV CAP_PROP_* for UVC) — NOT the software post-processing above.
    # All getters read back FROM the device so a readout can be trusted.

    def hardware_capabilities(self) -> dict:
        """What the live backend can control, with ranges where known.

        ``controllable`` is False for stopped / simulated cameras. ``controls``
        maps a control name to ``{"range": (min,max,def) | None}``.
        """
        backend = getattr(self, "_backend_type", "opencv")
        caps = {"source": "none", "controllable": False,
                "resolution": False, "controls": {}, "device_name": ""}
        if not self._running:
            return caps
        if backend == "toupcam" and getattr(self, "_toupcam", None) is not None:
            tc = self._toupcam
            caps.update(source="toupcam", controllable=True, resolution=True,
                        device_name=getattr(tc, "_device_id", "ToupCam"))
            caps["controls"] = {
                "auto_exposure": {"range": None},
                "exposure_us": {"range": tc.get_exposure_time_range()},
                "exposure_gain_pct": {"range": tc.get_exposure_gain_range()},
                "gamma": {"range": tc.HW_RANGES["gamma"]},
                "brightness": {"range": tc.HW_RANGES["brightness"]},
                "contrast": {"range": tc.HW_RANGES["contrast"]},
            }
        elif backend == "andor" and getattr(self, "_andor", None) is not None:
            an = self._andor
            caps.update(source="andor", controllable=True, resolution=True,
                        device_name=getattr(an, "_device_id", "Andor Zyla"))
            # The Zyla exposes exposure as its only true HARDWARE control
            # (omitting the ToupTek ISP keys makes the settings dialog
            # auto-hide them; software brightness/contrast/gamma still apply).
            # The andor_* keys control the backend's mono16→8-bit DISPLAY
            # scaling — the per-frame percentile auto-scale that makes the
            # image look like it's auto-adjusting, plus fixed black/white
            # levels for when it's turned off.
            caps["controls"] = {
                "exposure_us": {"range": an.get_exposure_time_range()},
                "andor_auto_scale": {"range": None},
                "andor_scale_lo": {"range": an.get_display_level_range()},
                "andor_scale_hi": {"range": an.get_display_level_range()},
            }
            # v7.13 — sensor-quality features (cooling / readout rate / gain
            # mode / noise + blemish filters). Only features the OPEN camera
            # actually probed appear, so the dialog auto-hides the rest; enum
            # combos are populated from the camera's own runtime values.
            try:
                for key, spec in an.sensor_feature_specs().items():
                    caps["controls"][key] = {"range": None,
                                             "kind": spec.get("kind"),
                                             "values": spec.get("values")}
            except Exception:
                pass
            # Raw 16-bit frame statistics (histogram / saturation) available?
            if hasattr(an, "get_raw_frame_stats"):
                caps["controls"]["andor_raw_stats"] = {"range": None}
        elif backend == "tucam" and getattr(self, "_tucam", None) is not None:
            tu = self._tucam
            caps.update(source="tucam", controllable=True, resolution=True,
                        device_name=getattr(tu, "_model", "Tucsen") or "Tucsen")
            # Only advertise what THIS camera actually implements: each range
            # comes from the SDK's own TUCAM_Prop_GetAttr, and a property the
            # model lacks returns None → the control is omitted → the settings
            # dialog auto-hides it. That is what keeps the unverified property-ID
            # table from presenting a control that does nothing.
            ctrls: dict = {}
            for key, rng in (
                ("exposure_us", tu.get_exposure_time_range()),
                ("exposure_gain_pct", tu.get_exposure_gain_range()),
                ("gamma", tu.get_gamma_range()),
                ("brightness", tu.get_brightness_range()),
                ("contrast", tu.get_contrast_range()),
            ):
                if rng is not None:
                    ctrls[key] = {"range": rng}
            if tu.get_auto_exposure() is not None:
                ctrls["auto_exposure"] = {"range": None}
            # v7.19 — the SENSOR's own auto black/white points. Gated on a
            # non-None getter for the same reason as every range above: a model
            # without the capability must not be offered a dead control.
            if tu.get_auto_levels() is not None:
                ctrls["auto_levels"] = {"range": None}
            # Mono→8-bit display scaling: the same controls (and the same
            # historical `andor_*` keys) the Zyla uses, so both mono cameras are
            # adjusted identically during the A/B evaluation.
            lvl = tu.get_display_level_range()
            ctrls["andor_auto_scale"] = {"range": None}
            ctrls["andor_scale_lo"] = {"range": lvl}
            ctrls["andor_scale_hi"] = {"range": lvl}
            # v7.13 — raw 16-bit statistics (histogram / saturation): the
            # Tucsen gets the same treatment as the Zyla; the key gates the
            # dialog's Signal section and the live-feed SATURATED badge.
            if hasattr(tu, "get_raw_frame_stats"):
                ctrls["andor_raw_stats"] = {"range": None}
            caps["controls"] = ctrls
        elif backend == "opencv" and self._capture is not None:
            caps.update(source="opencv", controllable=True, resolution=True,
                        device_name=f"OpenCV #{self._camera_index}")
            # UVC ranges are driver-specific and not reliably queryable;
            # expose generic 0–255 ranges and let the readback show reality.
            caps["controls"] = {
                "auto_exposure": {"range": None},
                "exposure_us": {"range": None},
                "exposure_gain_pct": {"range": (0, 255, 0)},
                "gamma": {"range": (0, 255, 128)},
                "brightness": {"range": (0, 255, 128)},
                "contrast": {"range": (0, 255, 128)},
            }
        elif backend == "simulated":
            caps["source"] = "simulated"
        return caps

    def get_hw_settings(self) -> dict:
        """Read current hardware settings back FROM the device.

        Returns a dict that always carries a ``source`` key
        (``"toupcam"``/``"opencv"``/``"simulated"``/``"none"``) so a caller can
        prove whether the values came from the camera or not.
        """
        backend = getattr(self, "_backend_type", "opencv")
        if not self._running:
            return {"source": "none"}
        if backend == "toupcam" and getattr(self, "_toupcam", None) is not None:
            d = self._toupcam.get_settings()
            d["source"] = "toupcam"
            return d
        if backend == "andor" and getattr(self, "_andor", None) is not None:
            d = self._andor.get_settings()
            d["source"] = "andor"
            return d
        if backend == "tucam" and getattr(self, "_tucam", None) is not None:
            d = self._tucam.get_settings()
            d["source"] = "tucam"
            return d
        if backend == "opencv" and self._capture is not None:
            cap = self._capture
            try:
                # UVC/DirectShow returns -1 for properties the driver doesn't
                # support; surface those as None ("unknown") so the readout
                # never presents a sentinel as a real device value.
                def _q(prop):
                    v = cap.get(prop)
                    return None if (v is None or v < 0) else v
                # DirectShow auto-exposure convention: 0.75 = auto, 0.25 =
                # manual. Anything else (incl. -1 unsupported) = unknown/None,
                # so bool() can't flip a manual/unsupported cam to "auto".
                raw_ae = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
                auto = (True if raw_ae == 0.75
                        else (False if raw_ae == 0.25 else None))
                return {
                    "source": "opencv",
                    "device_id": f"index {self._camera_index}",
                    "brightness": _q(cv2.CAP_PROP_BRIGHTNESS),
                    "contrast": _q(cv2.CAP_PROP_CONTRAST),
                    "gamma": _q(cv2.CAP_PROP_GAMMA),
                    "exposure_us": _q(cv2.CAP_PROP_EXPOSURE),
                    "exposure_gain_pct": _q(cv2.CAP_PROP_GAIN),
                    "auto_exposure": auto,
                    "resolution": (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                                   int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))),
                    "eSize": None,
                    "resolutions": [],
                }
            except Exception as exc:
                logger.debug(f"OpenCV get_hw_settings failed: {exc}")
                return {"source": "opencv", "error": str(exc)}
        return {"source": backend}

    def set_hw_auto_exposure(self, enabled: bool) -> bool:
        backend = getattr(self, "_backend_type", "opencv")
        if backend == "toupcam" and getattr(self, "_toupcam", None) is not None:
            return self._toupcam.set_auto_exposure(bool(enabled))
        if backend == "andor" and getattr(self, "_andor", None) is not None:
            return self._andor.set_auto_exposure(bool(enabled))
        if backend == "tucam" and getattr(self, "_tucam", None) is not None:
            return self._tucam.set_auto_exposure(bool(enabled))
        if backend == "opencv" and self._capture is not None:
            # DirectShow: 0.75 = auto, 0.25 = manual.
            return bool(self._capture.set(
                cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if enabled else 0.25))
        return False

    def set_hw_auto_levels(self, enabled: bool) -> bool:
        """Enable/disable the camera's HARDWARE auto black/white levels.

        v7.19. Only the Tucsen implements this today; every other backend
        returns False, and the capability is advertised only where the getter
        answers, so the settings dialog and the fluorescence preset both skip it
        silently rather than issuing a write that does nothing.

        ⚠ Distinct from ``set_hw_andor_auto_scale``, which is the SOFTWARE
        mono16→8-bit display mapping. This one moves the sensor's own levels and
        therefore changes captured pixel values.
        """
        backend = getattr(self, "_backend_type", "opencv")
        if backend == "tucam" and getattr(self, "_tucam", None) is not None:
            return self._tucam.set_auto_levels(bool(enabled))
        return False

    def set_hw_exposure_us(self, microseconds) -> bool:
        backend = getattr(self, "_backend_type", "opencv")
        if backend == "toupcam" and getattr(self, "_toupcam", None) is not None:
            return self._toupcam.put_exposure_time(microseconds)
        if backend == "andor" and getattr(self, "_andor", None) is not None:
            return self._andor.put_exposure_time(microseconds)
        if backend == "tucam" and getattr(self, "_tucam", None) is not None:
            return self._tucam.put_exposure_time(microseconds)
        if backend == "opencv" and self._capture is not None:
            return bool(self._capture.set(cv2.CAP_PROP_EXPOSURE, microseconds))
        return False

    def set_hw_exposure_gain(self, percent) -> bool:
        backend = getattr(self, "_backend_type", "opencv")
        if backend == "toupcam" and getattr(self, "_toupcam", None) is not None:
            return self._toupcam.put_exposure_gain(percent)
        if backend == "andor" and getattr(self, "_andor", None) is not None:
            return self._andor.put_exposure_gain(percent)
        if backend == "tucam" and getattr(self, "_tucam", None) is not None:
            return self._tucam.put_exposure_gain(percent)
        if backend == "opencv" and self._capture is not None:
            return bool(self._capture.set(cv2.CAP_PROP_GAIN, percent))
        return False

    def set_hw_gamma(self, value) -> bool:
        backend = getattr(self, "_backend_type", "opencv")
        if backend == "toupcam" and getattr(self, "_toupcam", None) is not None:
            return self._toupcam.put_gamma(value)
        if backend == "andor" and getattr(self, "_andor", None) is not None:
            return self._andor.put_gamma(value)
        if backend == "tucam" and getattr(self, "_tucam", None) is not None:
            return self._tucam.put_gamma(value)
        if backend == "opencv" and self._capture is not None:
            return bool(self._capture.set(cv2.CAP_PROP_GAMMA, value))
        return False

    def set_hw_brightness(self, value) -> bool:
        backend = getattr(self, "_backend_type", "opencv")
        if backend == "toupcam" and getattr(self, "_toupcam", None) is not None:
            return self._toupcam.put_brightness(value)
        if backend == "andor" and getattr(self, "_andor", None) is not None:
            return self._andor.put_brightness(value)
        if backend == "tucam" and getattr(self, "_tucam", None) is not None:
            return self._tucam.put_brightness(value)
        if backend == "opencv" and self._capture is not None:
            return bool(self._capture.set(cv2.CAP_PROP_BRIGHTNESS, value))
        return False

    def set_hw_contrast(self, value) -> bool:
        backend = getattr(self, "_backend_type", "opencv")
        if backend == "toupcam" and getattr(self, "_toupcam", None) is not None:
            return self._toupcam.put_contrast(value)
        if backend == "andor" and getattr(self, "_andor", None) is not None:
            return self._andor.put_contrast(value)
        if backend == "tucam" and getattr(self, "_tucam", None) is not None:
            return self._tucam.put_contrast(value)
        if backend == "opencv" and self._capture is not None:
            return bool(self._capture.set(cv2.CAP_PROP_CONTRAST, value))
        return False

    def _mono_display_backend(self):
        """The live backend that owns mono→8-bit DISPLAY scaling, or None.

        v7.9.x: both scientific cameras convert a mono sensor to 8-bit BGR for
        display through the same shared math (gui/widgets/mono_display.py), so
        both expose the same scaling API. Resolving the backend here keeps the
        three delegates below from duplicating the branch per camera type.

        The ``andor_*`` method/key names are HISTORICAL — this control shipped
        first on the Zyla — and are kept because they are the names the settings
        dialog gates on and ``hw_controls`` persists.
        """
        backend = getattr(self, "_backend_type", "")
        if backend == "andor":
            return getattr(self, "_andor", None)
        if backend == "tucam":
            return getattr(self, "_tucam", None)
        return None

    def set_hw_andor_auto_scale(self, enabled: bool) -> bool:
        """Mono cameras: toggle the mono→8-bit per-frame display auto-scale."""
        be = self._mono_display_backend()
        return be.set_display_auto_scale(bool(enabled)) if be is not None else False

    def set_hw_andor_scale_lo(self, counts) -> bool:
        """Mono cameras: manual display black level (raw sensor counts)."""
        be = self._mono_display_backend()
        return be.put_display_black(counts) if be is not None else False

    def set_hw_andor_scale_hi(self, counts) -> bool:
        """Mono cameras: manual display white level (raw sensor counts)."""
        be = self._mono_display_backend()
        return be.put_display_white(counts) if be is not None else False

    def set_hw_andor_feature(self, key: str, value) -> bool:
        """v7.13 — set a probed Andor sensor-quality feature by settings key.

        Andor-only today (the sensor feature table lives on that backend);
        the seam mirrors _mono_display_backend so a future Tucsen adoption
        widens one resolver, not five call sites.
        """
        backend = getattr(self, "_backend_type", "")
        be = getattr(self, "_andor", None) if backend == "andor" else None
        if be is None or not hasattr(be, "set_sensor_feature"):
            return False
        return bool(be.set_sensor_feature(key, value))

    def reset_andor_sensor_defaults(self) -> bool:
        """v7.13 — re-apply the backend's low-noise sensor defaults (live)."""
        backend = getattr(self, "_backend_type", "")
        be = getattr(self, "_andor", None) if backend == "andor" else None
        if be is None or not hasattr(be, "apply_sensor_defaults"):
            return False
        return bool(be.apply_sensor_defaults())

    def get_raw_frame_stats(self):
        """v7.13 — latest raw 16-bit frame statistics from a mono backend.

        Lock-snapshot read (no SDK call) — safe from GUI timers. None for
        backends without raw retention (OpenCV / simulated / stopped).
        """
        be = self._mono_display_backend()
        if be is None or not hasattr(be, "get_raw_frame_stats"):
            return None
        try:
            return be.get_raw_frame_stats()
        except Exception:
            return None

    def capture_raw_average(self, n: int, timeout_s: float = 10.0):
        """v7.13 — blocking averaged raw capture (worker threads ONLY).

        Returns a uint16 2-D mean of n consecutive new frames, or None when
        the backend can't do it (see AndorBackend.capture_raw_average).

        v7.16: cropped like every other frame this widget delivers. This is the
        THIRD egress point — it reads the raw sensor buffer straight from the
        backend — and leaving it out would make a raw still the one image in
        the app that still showed the vignetted edges.
        """
        be = self._mono_display_backend()
        if be is None or not hasattr(be, "capture_raw_average"):
            return None
        try:
            return self._apply_crop(be.capture_raw_average(n, timeout_s=timeout_s))
        except Exception as exc:
            logger.debug(f"{self._camera_label}: capture_raw_average failed: {exc}")
            return None

    def set_capture_resolution(self, width: int, height: int):
        """Reconfigure the *device* capture resolution. Returns actual (w,h).

        SDK backends (ToupCam / Andor / TUCam): maps (w,h) to the nearest
        supported resolution index and restarts the stream. OpenCV: sets
        CAP_PROP_FRAME_WIDTH/HEIGHT. Returns the resolution actually adopted
        (read back from the device), or None.

        NOTE: changing capture resolution changes the effective µm/px, so any
        existing pixel-scale calibration for this camera must be redone.
        """
        backend = getattr(self, "_backend_type", "opencv")
        # v7.9.x: the three SDK backends expose the identical
        # get_resolution_list / set_resolution_index / get_resolution contract,
        # so one branch serves all of them (the ToupCam and Andor branches were
        # byte-identical copies; adding a third would have been triplication).
        sdk_attr = {"toupcam": "_toupcam", "andor": "_andor",
                    "tucam": "_tucam"}.get(backend)
        sdk = getattr(self, sdk_attr, None) if sdk_attr else None
        if sdk is not None:
            res = sdk.get_resolution_list()
            if not res:
                return None
            target_area = int(width) * int(height)
            idx = min(range(len(res)),
                      key=lambda i: abs(res[i][0] * res[i][1] - target_area))
            ok = sdk.set_resolution_index(idx)
            actual = sdk.get_resolution()
            logger.info(
                f"{self._camera_label}: capture resolution -> {actual} "
                f"(eSize {idx}){'' if ok else ' [FAILED]'} — µm/px calibration "
                f"may need redoing")
            return actual if ok else None
        if backend == "opencv" and self._capture is not None:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
            actual = (int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                      int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            logger.info(
                f"{self._camera_label}: capture resolution -> {actual} — "
                f"µm/px calibration may need redoing")
            return actual
        return None

    def log_hw_settings(self, prefix: str = ""):
        """Log the device's current hardware settings to the terminal.

        Reads every value back from the camera via getters and prints a
        labelled block stating the SOURCE, so the operator can confirm the
        settings come from the camera (not a software default). Returns
        ``(settings_dict, formatted_text)``.
        """
        st = self.get_hw_settings()
        src = st.get("source", "none")
        head = f"{prefix}{self._camera_label}: hardware settings — source = {src}"
        lines = [head]
        if src in ("toupcam", "opencv", "andor", "tucam"):
            lines.append(f"  device       : {st.get('device_id', '?')}")
            lines.append(f"  resolution   : {st.get('resolution')}"
                         + (f"  (eSize {st.get('eSize')})"
                            if st.get("eSize") is not None else ""))
            lines.append(f"  auto-exposure: {st.get('auto_exposure')}")
            lines.append(f"  exposure     : {st.get('exposure_us')} µs"
                         + (f"  range={st.get('exposure_range_us')}"
                            if st.get("exposure_range_us") else ""))
            lines.append(f"  gain         : {st.get('exposure_gain_pct')} %")
            lines.append(f"  gamma        : {st.get('gamma')}")
            lines.append(f"  brightness   : {st.get('brightness')}")
            lines.append(f"  contrast     : {st.get('contrast')}")
            if src in ("andor", "tucam"):
                lines.append(
                    f"  display scale: "
                    f"{'auto (per-frame)' if st.get('andor_auto_scale') else 'manual'}"
                    f"  levels {st.get('andor_scale_lo')}..{st.get('andor_scale_hi')}")
        else:
            lines.append(f"  (no controllable camera backend — source={src})")
        text = "\n".join(lines)
        logger.info(text)
        return st, text

    # ── Frame Capture ─────────────────────────────────────────────

    def _grab_frame(self):
        """Capture, adjust, and display one frame.

        v7.3-camera: Reads from OpenCV or an SDK backend.
        v7.9.x: SDK backends (ToupCam / Andor / TUCam) share one read path —
        each exposes the same isOpened()/read() contract and returns BGR8.
        """
        backend = getattr(self, '_backend_type', 'opencv')
        sdk_attr = {'toupcam': '_toupcam', 'andor': '_andor',
                    'tucam': '_tucam'}.get(backend)
        try:
            if sdk_attr:
                sdk = getattr(self, sdk_attr, None)
                if sdk is None or not sdk.isOpened():
                    self.stop()
                    return
                ret, frame = sdk.read()
                # v7.14: ask the backend how many DISTINCT frames the sensor
                # has delivered, so _frame_seq below can tell a new frame from
                # a repeat of the cached one. None => backend cannot report.
                try:
                    getter = getattr(sdk, "frames_acquired", None)
                    src_seq = int(getter()) if callable(getter) else None
                except Exception:
                    src_seq = None
            else:
                if not self._capture or not self._capture.isOpened():
                    self.stop()
                    return
                ret, frame = self._capture.read()
                # OpenCV / SimulatedCamera: read() advances the driver FIFO or
                # generates on demand, so every successful read IS a new frame
                # and no separate accounting is needed.
                src_seq = None
        except Exception as exc:
            # A camera can open successfully and still be unable to deliver a
            # frame — an idle OBS Virtual Camera, or any device unplugged
            # mid-stream. OpenCV then raises out of this slot on EVERY timer
            # tick: the console fills with identical tracebacks, the feed never
            # recovers, and the wasted work is continuous (observed at ~7
            # minutes of CPU in one session). isOpened() does not catch it,
            # because the capture still reports itself open.
            #
            # Stop this feed instead. One clear line, and the label reads
            # "stopped" so the operator can see which camera died rather than
            # hunting a scrolling traceback.
            logger.error("%s: frame read failed (%s backend) — stopping this "
                         "feed: %s", self._camera_label, backend, exc)
            self.stop()
            self.video_label.setText(
                f"{self._camera_label} — read failed, feed stopped")
            return

        if not ret or frame is None:
            return

        # v7.16: crop FIRST — before the raw cache and before the display
        # conversion — so the cached frame (detection / mosaic / calibration),
        # the emitted QImage (live view / video recording) and the on-screen
        # pixmap are all the same pixels. Cropping later, per consumer, is how
        # four surfaces end up disagreeing about how big a frame is.
        frame = self._apply_crop(frame)

        # Import numpy/cv2 for processing
        try:
            import cv2
            import numpy as np
        except ImportError:
            return

        # v7.3.0: Store raw BGR frame for detection workers (thread-safe).
        # Frames are kept RAW — orientation (mirror + rotation) is applied per
        # consumer (display / click-mapping / mosaic), so the mosaic can re-blend
        # against a changing orientation. See set_mirrored.
        # v7.14: advance _frame_seq ONLY when the sensor actually delivered a
        # new frame.
        #
        # This timer runs at a fixed 15 fps regardless of how fast the camera
        # is, and the SDK backends' read() is non-blocking — it returns the
        # SAME cached frame as often as it is asked. So the old unconditional
        # increment counted timer ticks, not frames. While the camera outran
        # the timer that was harmless; at 2048x2048 with a 200 ms exposure the
        # Zyla delivers ~5 fps, so a mosaic worker waiting for "3 fresh frames"
        # was satisfied in ~3 ticks (200 ms) having seen ZERO new frames — and
        # stitched the frame exposed DURING the stage move. That is the
        # full-resolution blur.
        #
        # The gate lives HERE, not in frame_count_value(), so the counter and
        # _current_frame stay paired: reading the backend's counter directly
        # would let it advance while this timer is stopped (a hidden page),
        # and a worker would then be told "new frame" while _current_frame
        # still held the old one.
        with self._frame_lock:
            advanced = (src_seq is None or self._frame_src_seq is None
                        or src_seq != self._frame_src_seq)
            self._frame_src_seq = src_seq
            self._current_frame = frame.copy()
            if advanced:
                self._frame_seq += 1

        # Convert BGR -> RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Apply software brightness + contrast in one pass.
        # out = contrast*(in - 128) + 128 + brightness
        #     = contrast*in + (128 - 128*contrast + brightness)
        # Contrast pivots on mid-gray (so it doesn't also brighten); reduces to
        # the legacy brightness-only path when contrast == 1.0.
        if self._brightness != 0 or abs(self._contrast - 1.0) > 0.01:
            beta = 128.0 - 128.0 * self._contrast + self._brightness
            rgb = cv2.convertScaleAbs(rgb, alpha=self._contrast, beta=beta)

        # Apply software gamma via LUT
        if self._gamma_lut is not None and abs(self._gamma - 1.0) > 0.01:
            rgb = cv2.LUT(rgb, self._gamma_lut)

        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        # Make a copy so the numpy buffer stays valid for QImage
        q_img = QImage(rgb.copy().data, w, h, bytes_per_line, QImage.Format.Format_RGB888)

        # Draw crosshair overlay
        pixmap = QPixmap.fromImage(q_img)
        if self._show_crosshair:
            self._draw_crosshair(pixmap)

        # Scale to fit label
        scaled = pixmap.scaled(
            self.video_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.video_label.setPixmap(scaled)

        self.frame_captured.emit(q_img)


    def _draw_crosshair(self, pixmap: QPixmap):
        """Draw a crosshair overlay on the pixmap.

        v7.16: the crosshair marks where the STAGE is pointing, which is the
        middle of the frame only while the crop is centred. Once the crop is
        moved off-centre that point shifts, and a crosshair left at the
        geometric middle would aim the operator at somewhere the stage is not.
        """
        painter = QPainter(pixmap)
        pen = QPen(QColor("#f38ba8"), 1, Qt.PenStyle.DashLine)
        painter.setPen(pen)

        cx = pixmap.width() // 2
        cy = pixmap.height() // 2
        crop = getattr(self, "_crop", None)
        cap = getattr(self, "_capture_size", None)
        if crop is not None and crop.enabled and cap:
            try:
                rx, ry = crop.reference_pixel(cap[0], cap[1])
                cw, ch = crop.size_for(cap[0], cap[1])
                if cw > 0 and ch > 0:
                    cx = int(round(rx * pixmap.width() / cw))
                    cy = int(round(ry * pixmap.height() / ch))
            except Exception:
                pass

        # Horizontal line
        painter.drawLine(0, cy, pixmap.width(), cy)
        # Vertical line
        painter.drawLine(cx, 0, cx, pixmap.height())

        # Center circle
        pen.setStyle(Qt.PenStyle.SolidLine)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawEllipse(cx - 15, cy - 15, 30, 30)

        painter.end()

    def _on_crosshair_toggle(self, checked: bool):
        self._show_crosshair = checked

    # ── v7.3.2: Per-camera settings panel callbacks ───────────────

    def _toggle_settings_panel(self, checked: bool):
        if hasattr(self, '_settings_panel'):
            self._settings_panel.setVisible(checked)

    def _on_brightness_slider(self, value: int):
        self.set_brightness(value)

    def _on_contrast_slider(self, value: int):
        self.set_contrast(value / 100.0)

    def _on_gamma_slider(self, value: int):
        self.set_gamma(value / 100.0)

    def _on_fps_spinner(self, value: int):
        self._fps = value
        self._throttled = False
        if self._running:
            self._timer.setInterval(int(1000 / value))

    def set_throttled(self, on: bool) -> None:
        """v7.6: halve the display grab rate during time-critical work.

        ``_grab_frame`` runs on the GUI thread (blocking backend read + colour
        conversion + pixmap scaling), so at 15 FPS it can starve the event loop
        — which is what made the live needle position stutter during prints.
        Halving the rate while a print runs buys the loop back without stopping
        the feed. Restores the configured FPS when turned off.
        """
        on = bool(on)
        if on == bool(getattr(self, "_throttled", False)):
            return
        self._throttled = on
        if self._running:
            fps = max(1.0, self._fps / (2.0 if on else 1.0))
            try:
                self._timer.setInterval(int(1000 / fps))
            except Exception:
                pass

    # ── Frame Access (v7.3.0 — for detection workers) ────────────

    def get_current_frame(self):
        """
        Get a copy of the latest captured frame as a BGR numpy array.

        Thread-safe — can be called from any thread (e.g. DetectionWorker).
        Returns None if no frame has been captured yet or camera is stopped.

        Returns:
            np.ndarray (BGR, uint8) or None
        """
        with self._frame_lock:
            if self._current_frame is not None:
                return self._current_frame.copy()
            return None

    def frame_count_value(self) -> int:
        """Monotonic count of DISTINCT camera frames in the display buffer.

        Thread-safe. A worker thread can sample this before/after a stage move
        and wait for it to advance by N to guarantee N genuinely-new frames
        have arrived before reading ``get_current_frame()``.

        v7.14: this now counts SENSOR frames, not display-timer ticks. It
        previously incremented on every tick, so on a camera slower than the
        15 fps timer (full resolution, or any long exposure) it advanced
        without a single new frame having been delivered — see ``_grab_frame``.
        """
        with self._frame_lock:
            return self._frame_seq

    def frames_acquired(self) -> "int | None":
        """The BACKEND's own acquisition count, or None if it cannot report.

        Diagnostic. Prefer ``frame_count_value()`` for post-move waits: this
        one advances even while the display timer is stopped, so it is not
        paired with what ``get_current_frame()`` would return.
        """
        backend = getattr(self, '_backend_type', 'opencv')
        sdk_attr = {'toupcam': '_toupcam', 'andor': '_andor',
                    'tucam': '_tucam'}.get(backend)
        if not sdk_attr:
            return None
        try:
            sdk = getattr(self, sdk_attr, None)
            getter = getattr(sdk, "frames_acquired", None)
            return int(getter()) if callable(getter) else None
        except Exception:
            return None

    def capture_fresh_frame(self, discard_n_frames: int = 0,
                            settle_ms: int = 0):
        """Capture a fresh frame directly from the camera backend.

        Unlike get_current_frame() which returns the last timer-grabbed frame,
        this forces a new read() call. Essential after stage movement to ensure
        the frame content matches the current stage position.

        v7.5.x: optional ``settle_ms`` (sleep first — lets the exposure window /
        callback advance past the move) and ``discard_n_frames`` (pull+discard
        buffered frames before returning the final one). On OpenCV this drains
        the driver's FIFO backlog (back-to-back ``read()`` advances it); on
        ToupCam the callback fills a single slot, so the settle is what makes
        the frame post-move and the discard loop adds a small inter-frame gap so
        a new callback frame can arrive. CAUTION: this touches the backend
        directly — do NOT call it concurrently with the display grab timer from
        another thread; for off-thread use prefer ``frame_count_value()`` +
        ``get_current_frame()``.

        Supports OpenCV, ToupCam, and SimulatedCamera backends.

        Returns:
            np.ndarray (BGR, uint8) or None
        """
        if settle_ms and settle_ms > 0:
            time.sleep(settle_ms / 1000.0)

        backend = getattr(self, '_backend_type', 'opencv')

        def _read_once():
            if backend == 'toupcam':
                tc = getattr(self, '_toupcam', None)
                if tc is None or not tc.isOpened():
                    return None
                ret, frame = tc.read()
            elif backend == 'andor':
                an = getattr(self, '_andor', None)
                if an is None or not an.isOpened():
                    return None
                ret, frame = an.read()
            elif backend == 'tucam':
                # v7.14 — the Tucsen branch was MISSING: this fell through to
                # ``self._capture``, which is None on a tucam slot, so
                # capture_fresh_frame returned None on the Libra and every
                # caller (mosaic tiles, calibration grabs, the capture button)
                # silently got nothing.
                tu = getattr(self, '_tucam', None)
                if tu is None or not tu.isOpened():
                    return None
                ret, frame = tu.read()
            else:
                if not self._capture or not self._capture.isOpened():
                    return None
                # SimulatedCamera exposes read_fresh() for an uncached position.
                if hasattr(self._capture, 'read_fresh'):
                    ret, frame = self._capture.read_fresh()
                else:
                    ret, frame = self._capture.read()
            return frame if (ret and frame is not None) else None

        last = None
        for _ in range(max(0, int(discard_n_frames))):
            f = _read_once()
            if f is not None:
                last = f
            # ToupCam / Andor return the same buffered frame back-to-back; give
            # the async callback / reader thread a moment to deliver a newer one.
            if backend in ('toupcam', 'andor'):
                time.sleep(0.02)

        # v7.16: this path reads the backend DIRECTLY, bypassing _grab_frame, so
        # it must crop for itself — otherwise a mosaic tile (which comes through
        # here) would be full-frame while the live view beside it is cropped.
        final = _read_once()
        if final is not None:
            return self._apply_crop(final).copy()
        return self._apply_crop(last).copy() if last is not None else None

    # ── Snapshot ──────────────────────────────────────────────────

    def take_snapshot(self):
        """Save current frame to file."""
        if not self._capture or not self._capture.isOpened():
            return

        ret, frame = self._capture.read()
        if not ret:
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"snapshot_{self._camera_label.replace(' ', '_')}_{ts}.png"

        filepath, _ = QFileDialog.getSaveFileName(
            self, "Save Snapshot", default_name,
            "Images (*.png *.jpg *.bmp)"
        )
        if filepath:
            cv2.imwrite(filepath, frame)
            logger.info(f"Snapshot saved to {filepath}")

    # ── Detection Overlay Access (v7.3.0) ─────────────────────────

    @property
    def detection_overlay(self):
        """Access the detection overlay widget (or None if unavailable)."""
        return getattr(self, '_detection_overlay', None)

    # ── v7.4.4: Edge-Pick Mode (Needle Location workflow) ────────

    def set_edge_pick_mode(self, enabled: bool) -> None:
        """Enable/disable edge-pick click capture on this widget.

        While enabled, every left-click inside `video_label` is mapped
        from label-pixel coordinates back to frame-pixel coordinates
        and emitted via `pixel_clicked(cx_px, cy_px)`. The cursor
        switches to a crosshair while the mode is active.
        """
        if not hasattr(self, 'video_label'):
            return
        self._edge_pick_mode = bool(enabled)
        if self._edge_pick_mode:
            self.video_label.setCursor(Qt.CursorShape.CrossCursor)
        else:
            self.video_label.unsetCursor()

    def is_edge_pick_mode(self) -> bool:
        return bool(self._edge_pick_mode)

    def _label_to_frame_px(
        self, label_x: float, label_y: float
    ) -> tuple[float, float] | None:
        """Map a click on `video_label` back to original frame pixels.

        Mirrors the KeepAspectRatio + AlignCenter math from
        `DetectionOverlay._display_transform`. Returns None when there
        is no current frame or the click falls in the letterbox.
        """
        with self._frame_lock:
            frame = self._current_frame
        if frame is None or frame.ndim < 2:
            return None
        fh, fw = frame.shape[:2]
        ow, oh = self.video_label.width(), self.video_label.height()
        if fw <= 0 or fh <= 0 or ow <= 0 or oh <= 0:
            return None
        scale = min(ow / fw, oh / fh)
        if scale <= 0:
            return None
        displayed_w = fw * scale
        displayed_h = fh * scale
        offset_x = (ow - displayed_w) / 2.0
        offset_y = (oh - displayed_h) / 2.0
        fx = (label_x - offset_x) / scale
        fy = (label_y - offset_y) / scale
        if fx < 0 or fy < 0 or fx >= fw or fy >= fh:
            return None
        return float(fx), float(fy)

    def eventFilter(self, obj: QObject, event: QEvent) -> bool:
        if (self._edge_pick_mode
                and obj is getattr(self, 'video_label', None)
                and event.type() == QEvent.Type.MouseButtonPress
                and event.button() == Qt.MouseButton.LeftButton):
            pos = event.position()
            mapped = self._label_to_frame_px(pos.x(), pos.y())
            if mapped is not None:
                self.pixel_clicked.emit(mapped[0], mapped[1])
            return True  # Consume the click so click-to-move can't also fire
        return super().eventFilter(obj, event)

    # ── Cleanup ───────────────────────────────────────────────────

    def resizeEvent(self, event):
        """Keep detection overlay sized to match video_label."""
        super().resizeEvent(event)
        overlay = getattr(self, '_detection_overlay', None)
        if overlay is not None and hasattr(self, 'video_label'):
            # rect() not geometry() — overlay is a child of video_label,
            # so coordinates must be relative to video_label, not parent.
            overlay.setGeometry(self.video_label.rect())

    def closeEvent(self, event):
        self.stop()
        super().closeEvent(event)

    @property
    def is_running(self) -> bool:
        return self._running
