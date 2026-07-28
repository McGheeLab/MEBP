"""
mosaic_calibration_dialog.py — Pop-out small-mosaic alignment calibration.

v7.5.x: Builds a SMALL, user-defined NxN mosaic (default 5×5) around a chosen
point, runs the same global registration as the full plate scan, and stores the
resulting approximate **global delta** (shift, µm) for the current microscope
camera + objective. A subsequent full mosaic pre-seeds that delta
(`MosaicBuilder(initial_shift_um=...)`) so it starts pre-registered and the live
global registration has little to correct.

A 5×5 (or larger) grid is used so there are enough textured overlaps to estimate
the systematic shift robustly (the median ignores featureless tiles).

Self-contained: it owns a ``_MosaicScanWorker`` for the calibration scan (which
suspends the position poller and does tile-0 safe-travel + pure XY thereafter,
so the ZP board isn't hammered). The page hands it the controller, camera,
store, key, centre and frame info.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QApplication, QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QFormLayout,
    QGroupBox, QLabel, QSpinBox, QDoubleSpinBox, QComboBox, QCheckBox,
    QPushButton, QSlider, QScrollArea, QSplitter, QWidget,
)

from gui.styles import COLORS
from gui.scaling import s

logger = logging.getLogger(__name__)


def centered_scan_bounds(cx, cy, cols, rows, step_x, step_y, fov_w, fov_h):
    """Bounds (min_x, min_y, max_x, max_y) µm that make
    ``MosaicBuilder.generate_raster_positions`` (which insets by half-FOV) emit a
    ``cols × rows`` grid centred on ``(cx, cy)`` at the given step.
    """
    span_x = max(0, int(cols) - 1) * float(step_x) + float(fov_w)
    span_y = max(0, int(rows) - 1) * float(step_y) + float(fov_h)
    return (cx - span_x / 2.0, cy - span_y / 2.0,
            cx + span_x / 2.0, cy + span_y / 2.0)


class MosaicCalibrationDialog(QDialog):
    """Modal pop-out that builds a small calibration mosaic + stores the delta."""

    def __init__(self, controller, camera_manager, cam_idx, *, safe_z,
                 align_key, store, settings, center_um, frame_size,
                 um_per_px_camera, plate=None, well_positions=None,
                 safety_limits=None, zero_offset=(0.0, 0.0),
                 needle_od_um=None, cam_key=None, objective=None,
                 plate_flip_180=None, parent=None):
        super().__init__(parent)
        self._controller = controller
        self._mgr = camera_manager
        self._cam_idx = cam_idx
        # v7.5.x: render the tiles in the SAME plate-display frame as the full
        # plate view (JogWorkspaceView rotates the mosaic 180° when
        # plate_flip_180), so an orientation tuned here is WYSIWYG vs the full
        # mosaic. Resolve from the controller when not passed explicitly.
        if plate_flip_180 is None:
            try:
                pf = getattr(controller, "plate_flip_180", None)
                plate_flip_180 = bool(pf()) if callable(pf) else False
            except Exception:
                plate_flip_180 = False
        self._plate_flip_180 = bool(plate_flip_180)
        self._safe_z = safe_z
        self._align_key = align_key
        self._store = store
        # v7.5.x: when the objective identity is known, a correction here
        # (Store FOV/spacing) also writes the objective store + live manager so
        # the corrected µm/px is GROUND TRUTH everywhere (mosaic + click mapping
        # + restore), not just the mosaic-alignment learned value.
        self._cam_key = cam_key
        self._objective = objective
        self._settings = dict(settings or {})
        self._center_um = center_um            # (cx, cy) abs µm — "plate centre"
        self._fw, self._fh = frame_size
        self._um_camera = float(um_per_px_camera)
        # Render context for the embedded plate view (so the operator can slide
        # the calibration mosaic onto the well grid by eye, inside this dialog).
        self._plate = plate
        self._well_positions = dict(well_positions) if well_positions else {}
        self._safety_limits = safety_limits
        try:
            self._zero_offset = (float(zero_offset[0]), float(zero_offset[1]))
        except Exception:
            self._zero_offset = (0.0, 0.0)
        self._needle_od_um = needle_od_um

        # v7.5.x: camera orientation (flip X = mirror, flip Y, rotation). The
        # operator flips/rotates the scanned images here until the mosaic reads
        # correctly, then it saves as the camera's calibration. Seeded from the
        # camera's current calibration (all three via full_orientation).
        self._orient_mir, self._orient_fy, self._orient_rot = False, False, 0.0
        try:
            fo = getattr(camera_manager, "full_orientation", None)
            if callable(fo):
                self._orient_mir, self._orient_fy, self._orient_rot = \
                    fo(cam_idx)
            else:
                vo = getattr(camera_manager, "view_orientation", None)
                if callable(vo):
                    self._orient_mir, self._orient_rot = vo(cam_idx)
        except Exception:
            self._orient_mir, self._orient_fy, self._orient_rot = \
                False, False, 0.0

        self._worker = None
        self._builder = None
        self._applied = None          # tuned settings to push to the full mosaic
        self._result_comp = None      # last built composite (BGR) for handoff
        self._result_ext = None       # its absolute-µm extent (base + Δ)
        self._result_base_ext = None  # un-shifted canvas extent (base for Δ)

        self.setWindowTitle("Mosaic Alignment Calibration")
        self.setModal(True)
        self.setMinimumWidth(s(560))
        self._build_ui()

    # ── UI ─────────────────────────────────────────────────────────

    def _int_spin(self, lo, hi, val, suffix=""):
        sb = QSpinBox()
        sb.setRange(lo, hi)
        sb.setValue(int(val))
        if suffix:
            sb.setSuffix(suffix)
        return sb

    def _build_ui(self):
        # Two columns: LEFT = settings (top) + delta sliders (bottom) in a scroll
        # area; RIGHT = the live stitched mosaic (zoom/pan + per-tile outlines).
        # The action-button row is PINNED below both, so Build/Close are always
        # reachable regardless of screen height.
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(12), s(12), s(12))
        outer.setSpacing(s(8))

        splitter = QSplitter(Qt.Horizontal)
        outer.addWidget(splitter, stretch=1)

        # ── LEFT column (scrollable): settings + delta sliders + status ──
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        left = QWidget()
        root = QVBoxLayout(left)         # left-column layout
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(s(8))

        intro = QLabel(
            "Build a small mosaic over a textured region (wells). The stitched "
            "mosaic appears on the right — scroll to zoom, drag to pan. Register "
            "it (auto, or by eye with the Δ sliders). Stored per camera + objective.")
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color: {COLORS['subtext0']};")
        root.addWidget(intro)

        # Settings — COLLAPSIBLE so the column stays short.
        g = self._settings
        self._form_box = QGroupBox("Calibration mosaic settings")
        self._form_box.setCheckable(True)
        self._form_box.setChecked(True)
        box_lay = QVBoxLayout(self._form_box)
        box_lay.setContentsMargins(s(6), s(4), s(6), s(6))
        self._form_inner = QWidget()
        form = QFormLayout(self._form_inner)
        form.setContentsMargins(0, 0, 0, 0)
        form.setVerticalSpacing(s(4))
        self._spin_cols = self._int_spin(2, 15, g.get("cal_cols", 5))
        self._spin_rows = self._int_spin(2, 15, g.get("cal_rows", 5))
        form.addRow(QLabel("Grid columns"), self._spin_cols)
        form.addRow(QLabel("Grid rows"), self._spin_rows)
        self._combo_center = QComboBox()
        self._combo_center.addItem("Plate centre", "plate")
        self._combo_center.addItem("Current stage position", "current")
        form.addRow(QLabel("Centre on"), self._combo_center)
        self._spin_overlap = self._int_spin(5, 50, g.get("overlap_pct", 25), " %")
        form.addRow(QLabel("Tile overlap"), self._spin_overlap)
        self._spin_settle = self._int_spin(0, 5000, g.get("settle_ms", 300), " ms")
        form.addRow(QLabel("Settle after move"), self._spin_settle)
        self._spin_fresh = self._int_spin(1, 30, g.get("fresh_frames", 3))
        form.addRow(QLabel("Fresh frames to wait"), self._spin_fresh)
        self._spin_fov = self._int_spin(0, 50000, g.get("fov_um", 0), " µm")
        form.addRow(QLabel("Camera FOV width (0=auto)"), self._spin_fov)
        self._spin_maxshift = self._int_spin(0, 5000, g.get("max_shift_um", 0),
                                             " µm")
        form.addRow(QLabel("Max alignment shift (0=auto)"), self._spin_maxshift)
        box_lay.addWidget(self._form_inner)
        self._form_box.toggled.connect(self._form_inner.setVisible)
        root.addWidget(self._form_box)

        # Spacing calibration — UNDER the settings. Move the tiles toward/away
        # from each other (adjust the inter-tile SPACING) until features line up
        # in the overlap regions. From that, we derive the corrected effective
        # FOV (µm/px) + grid step — the best settings for the full mosaic.
        self._align_box = QGroupBox("Spacing calibration (align the overlaps)")
        ag = QGridLayout(self._align_box)
        ag.setContentsMargins(s(8), s(4), s(8), s(4))
        ag.addWidget(QLabel("X gap %"), 0, 0)
        self._align_dx = QSlider(Qt.Horizontal)
        self._align_dx.setRange(-50, 50)
        self._align_dx.setSingleStep(1)
        self._align_dx.setPageStep(5)
        self._align_dx.valueChanged.connect(self._on_align_changed)
        ag.addWidget(self._align_dx, 0, 1)
        self._align_dx_lbl = QLabel("0%")
        self._align_dx_lbl.setMinimumWidth(s(44))
        ag.addWidget(self._align_dx_lbl, 0, 2)
        ag.addWidget(QLabel("Y gap %"), 1, 0)
        self._align_dy = QSlider(Qt.Horizontal)
        self._align_dy.setRange(-50, 50)
        self._align_dy.setSingleStep(1)
        self._align_dy.setPageStep(5)
        self._align_dy.valueChanged.connect(self._on_align_changed)
        ag.addWidget(self._align_dy, 1, 1)
        self._align_dy_lbl = QLabel("0%")
        ag.addWidget(self._align_dy_lbl, 1, 2)
        ag.addWidget(QLabel("Opacity"), 2, 0)
        self._align_op = QSlider(Qt.Horizontal)
        self._align_op.setRange(5, 100)
        self._align_op.setValue(60)
        self._align_op.valueChanged.connect(self._on_align_changed)
        ag.addWidget(self._align_op, 2, 1)
        self._align_op_lbl = QLabel("60%")
        ag.addWidget(self._align_op_lbl, 2, 2)
        self._align_readout = QLabel("")
        self._align_readout.setWordWrap(True)
        self._align_readout.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 8pt;")
        ag.addWidget(self._align_readout, 3, 0, 1, 3)
        align_btns = QHBoxLayout()
        self._align_store_btn = QPushButton("Store FOV/spacing")
        self._align_store_btn.setToolTip(
            "Save the corrected effective FOV (µm/px) for this camera + "
            "objective. Future mosaics use it so the overlap is correct.")
        self._align_store_btn.clicked.connect(self._store_manual_align)
        align_btns.addWidget(self._align_store_btn)
        self._align_reset_btn = QPushButton("Reset")
        self._align_reset_btn.clicked.connect(self._reset_align)
        align_btns.addWidget(self._align_reset_btn)
        self._align_clear_btn = QPushButton("Clear stored")
        self._align_clear_btn.setToolTip(
            "Forget this camera + objective's stored FOV/alignment so auto "
            "sets it on the next scan.")
        self._align_clear_btn.clicked.connect(self._clear_stored_align)
        align_btns.addWidget(self._align_clear_btn)
        ag.addLayout(align_btns, 4, 0, 1, 3)
        self._set_align_enabled(False)
        root.addWidget(self._align_box)

        # ── Camera orientation (flip / rotate the images) ──────────────
        # A mirrored / rotated camera makes the mosaic read wrong. Flip / rotate
        # the images here until it looks right, then Apply to save it as the
        # camera's mirror + rotation (used by the live feed, future mosaics, and
        # click-mapping). Since this is a small single-well mosaic it's fast.
        self._orient_box = QGroupBox("Camera orientation (flip / rotate images)")
        obl = QVBoxLayout(self._orient_box)
        obl.setContentsMargins(s(8), s(4), s(8), s(4))
        obl.setSpacing(s(4))
        oh = QLabel(
            "Flip / rotate the images until the mosaic reads correctly, then "
            "Apply to save it to the camera.")
        oh.setWordWrap(True)
        oh.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 8pt;")
        obl.addWidget(oh)
        orow1 = QHBoxLayout()
        orow1.setSpacing(s(4))
        self._o_fliph = QPushButton("⇄ Flip X axis")
        self._o_fliph.setToolTip("Flip the camera left↔right (mirror the X axis) "
                                 "— use if the mosaic is mirrored across X.")
        self._o_fliph.clicked.connect(self._orient_flip_h)
        self._o_flipv = QPushButton("⇅ Flip Y axis")
        self._o_flipv.setToolTip("Flip the camera top↔bottom (mirror the Y axis) "
                                 "— use if the mosaic is mirrored across Y.")
        self._o_flipv.clicked.connect(self._orient_flip_v)
        self._o_ccw = QPushButton("⟲ 90°")
        self._o_ccw.clicked.connect(lambda: self._orient_rotate(-90.0))
        self._o_cw = QPushButton("⟳ 90°")
        self._o_cw.clicked.connect(lambda: self._orient_rotate(90.0))
        for b in (self._o_fliph, self._o_flipv, self._o_ccw, self._o_cw):
            orow1.addWidget(b)
        obl.addLayout(orow1)
        orow2 = QHBoxLayout()
        orow2.setSpacing(s(4))
        orow2.addWidget(QLabel("Rotation °"))
        self._o_fine = QDoubleSpinBox()
        self._o_fine.setRange(-180.0, 180.0)
        self._o_fine.setSingleStep(0.5)
        self._o_fine.setDecimals(1)
        self._o_fine.blockSignals(True)
        self._o_fine.setValue(float(self._orient_rot))
        self._o_fine.blockSignals(False)
        self._o_fine.valueChanged.connect(self._orient_fine_changed)
        orow2.addWidget(self._o_fine, 1)
        self._o_reset = QPushButton("Reset")
        self._o_reset.clicked.connect(self._orient_reset)
        orow2.addWidget(self._o_reset)
        obl.addLayout(orow2)
        self._o_apply = QPushButton("Apply to camera setup")
        self._o_apply.setObjectName("accentBtn")
        self._o_apply.clicked.connect(self._orient_apply_to_camera)
        obl.addWidget(self._o_apply)
        self._orient_box.setEnabled(False)   # enabled after a build
        root.addWidget(self._orient_box)

        self._status = QLabel("")
        self._status.setWordWrap(True)
        self._status.setStyleSheet(f"color: {COLORS['subtext0']};")
        root.addWidget(self._status)
        root.addStretch(1)
        left_scroll.setWidget(left)

        # ── RIGHT column: the live stitched mosaic (zoom/pan + tile outlines) ──
        self._preview = None             # no QLabel thumbnail in this design
        self._view = None
        mosaic_panel = QWidget()
        mp = QVBoxLayout(mosaic_panel)
        mp.setContentsMargins(0, 0, 0, 0)
        mp.setSpacing(s(4))
        mhead = QHBoxLayout()
        mhead.addWidget(QLabel("Mosaic (live)"))
        mhead.addStretch()
        self._align_zoomreset_btn = QPushButton("Reset view")
        self._align_zoomreset_btn.setToolTip("Reset zoom & pan to fit.")
        self._align_zoomreset_btn.clicked.connect(
            lambda: self._view.reset_view() if self._view is not None else None)
        mhead.addWidget(self._align_zoomreset_btn)
        mp.addLayout(mhead)
        try:
            from gui.widgets.mosaic_registration_view import (
                MosaicRegistrationView)
            self._view = MosaicRegistrationView()
            self._view.setMinimumSize(s(360), s(260))
            # Show tiles in the plate-display frame → WYSIWYG vs the full mosaic.
            try:
                self._view.set_flip_180(self._plate_flip_180)
            except Exception:
                pass
            mp.addWidget(self._view, stretch=1)
        except Exception as e:
            logger.debug(f"Mosaic registration view unavailable: {e}")
            ph = QLabel("Mosaic view unavailable.")
            ph.setAlignment(Qt.AlignmentFlag.AlignCenter)
            mp.addWidget(ph, stretch=1)

        splitter.addWidget(left_scroll)
        splitter.addWidget(mosaic_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([s(310), s(540)])

        btn_row = QHBoxLayout()
        self._btn_build = QPushButton("Build & calibrate")
        self._btn_build.setObjectName("accentBtn")
        self._btn_build.clicked.connect(self._on_build)
        btn_row.addWidget(self._btn_build)
        self._btn_cancel = QPushButton("Stop")
        self._btn_cancel.setEnabled(False)
        self._btn_cancel.clicked.connect(self._stop_worker)
        btn_row.addWidget(self._btn_cancel)
        self._btn_apply = QPushButton("Apply to full mosaic")
        self._btn_apply.setToolTip(
            "Use the tuned overlap / settle / FOV / max-shift here as the "
            "full-plate mosaic's settings (the alignment delta is already "
            "stored for this camera + objective).")
        self._btn_apply.clicked.connect(self._on_apply_settings)
        btn_row.addWidget(self._btn_apply)
        btn_row.addStretch()
        self._btn_close = QPushButton("Close")
        self._btn_close.clicked.connect(self.close)
        btn_row.addWidget(self._btn_close)
        outer.addLayout(btn_row)         # PINNED below the scroll area

        # Never let the dialog grow taller than the screen — otherwise the
        # action buttons (Build/Close) fall off the bottom and can't be clicked.
        # The plate view has the layout stretch, so a capped height shrinks the
        # VIEW, never the pinned button row. Resizable + collapsible form let the
        # operator reclaim space; the view can be grown by enlarging the dialog.
        self._apply_height_cap(initial=True)

    def _apply_height_cap(self, initial: bool = False) -> None:
        """Cap the dialog to the current screen's available height so the pinned
        button row can't overflow. Re-applied in showEvent because self.screen()
        in __init__ can report the primary monitor, not the one a modal opens on
        (a shorter secondary monitor would otherwise get too generous a cap)."""
        try:
            scr = self.screen() or QApplication.primaryScreen()
            avail = scr.availableGeometry() if scr is not None else None
            if avail is None:
                return
            cap_h = max(s(420), avail.height() - s(80))
            self.setMaximumHeight(int(cap_h))
            if initial:
                self.resize(s(640), int(min(s(840), cap_h)))
        except Exception as e:
            logger.debug(f"Calibration dialog height cap skipped: {e}")

    def showEvent(self, event):
        # Now positioned on its real screen — recompute the cap for THAT monitor.
        self._apply_height_cap(initial=False)
        super().showEvent(event)

    # ── Build / run ────────────────────────────────────────────────

    def grid_values(self) -> tuple[int, int]:
        return int(self._spin_cols.value()), int(self._spin_rows.value())

    def applied_settings(self):
        """Tuned full-mosaic settings the operator chose to apply (or None)."""
        return self._applied

    def result_overlay(self):
        """(composite_bgr, extent_um) of the last built calibration mosaic, so
        the page can show it on the plate map for manual alignment."""
        return self._result_comp, self._result_ext

    def _on_apply_settings(self):
        """Capture the current tuned settings to hand back to the full mosaic.

        The grid columns/rows are calibration-only; the FOV / overlap / settle /
        fresh-frames / max-shift the operator dialed in here become the full
        mosaic's. The registration delta is already persisted per
        camera+objective and pre-applied automatically.
        """
        self._applied = {
            "overlap_pct": int(self._spin_overlap.value()),
            "settle_ms": int(self._spin_settle.value()),
            "fresh_frames": int(self._spin_fresh.value()),
            "fov_um": int(self._spin_fov.value()),
            "max_shift_um": int(self._spin_maxshift.value()),
        }
        self._status.setText(
            "These settings will be applied to the full-plate mosaic when you "
            "close this dialog.")

    def _on_build(self):
        if self._worker is not None:
            return
        try:
            self._start_calibration()
        except Exception as e:
            logger.exception("Mosaic calibration build failed")
            self._status.setText(f"Calibration failed: {e}")
            self._set_running(False)

    def _start_calibration(self):
        from SupportClasses.MosaicBuilder import MosaicBuilder
        from gui.pages.calibration import _MosaicScanWorker

        # CRITICAL SAFETY (defense-in-depth; the page opener also gates): never
        # run with an unset Safe Z while the ZP is connected — the retract would
        # fall back to zero-ref 0 (plate datum on ME3B V1) and crash the needle.
        if (self._safe_z is None
                and bool(getattr(self._controller, "is_zp_connected", False))):
            self._status.setText(
                "Set the Safe / Move Z first (Needle Offset tab) — refusing to "
                "run without a safe retract height.")
            return

        cam = self._mgr.cameras[self._cam_idx]
        cols = int(self._spin_cols.value())
        rows = int(self._spin_rows.value())
        overlap = float(self._spin_overlap.value()) / 100.0
        fov_um = float(self._spin_fov.value())
        eff = (fov_um / self._fw) if (fov_um > 0 and self._fw > 0) else self._um_camera
        fov_w = self._fw * eff
        fov_h = self._fh * eff
        step_x = fov_w * (1.0 - overlap)
        step_y = fov_h * (1.0 - overlap)

        # Centre.
        if self._combo_center.currentData() == "current":
            try:
                xy = self._controller.get_xy_position(cached=False)
                cx, cy = float(xy[0]), float(xy[1])
            except Exception:
                cx, cy = self._center_um
        else:
            cx, cy = self._center_um

        bounds = centered_scan_bounds(cx, cy, cols, rows, step_x, step_y,
                                      fov_w, fov_h)
        # Clip to the reachable XY envelope so no move clamps.
        env = None
        sl = getattr(self._controller, "safety_limits", None)
        if sl is not None:
            try:
                env = (float(sl.xy_min_x), float(sl.xy_min_y),
                       float(sl.xy_max_x), float(sl.xy_max_y))
                bounds = (max(bounds[0], env[0]), max(bounds[1], env[1]),
                          min(bounds[2], env[2]), min(bounds[3], env[3]))
            except Exception:
                env = None
        if bounds[2] <= bounds[0] or bounds[3] <= bounds[1]:
            self._status.setText(
                "The calibration grid is outside the reachable XY envelope.")
            return

        max_shift_um = float(self._spin_maxshift.value())
        # Pre-seed with any previously-learned shift so a calibration that
        # registers nothing (featureless region) PRESERVES the old value
        # instead of resetting it.
        prior = (0.0, 0.0)
        if self._store is not None:
            try:
                s = self._store.get_shift_um(self._align_key)
                if s:
                    prior = s
            except Exception:
                pass
        builder = MosaicBuilder(
            frame_size_px=(self._fw, self._fh), micron_per_pixel=eff,
            overlap=overlap, target_mosaic_px=1800,
            register=True, max_shift_um=max_shift_um, initial_shift_um=prior,
            # v7.5.x: apply the camera's current mirror + rotation so the tiles
            # read the way the operator will adjust them (retained frames let
            # tile_images_px re-orient on each flip/rotate).
            frame_rotation_deg=float(self._orient_rot),
            frame_mirrored=bool(self._orient_mir),
            frame_flip_y=bool(self._orient_fy))
        grid = builder.generate_raster_positions(
            bounds, overlap=overlap, step_x_um=step_x, step_y_um=step_y)
        if env is not None:
            grid = [(x, y) for (x, y) in grid
                    if env[0] <= x <= env[2] and env[1] <= y <= env[3]]
        if not grid:
            self._status.setText("No reachable calibration tiles.")
            return

        # Pre-scan retract (never crash).
        try:
            if hasattr(self._controller, "ensure_retracted_to"):
                self._controller.ensure_retracted_to(
                    self._safe_z if self._safe_z is not None else 0.0)
        except Exception:
            pass

        self._builder = builder
        self._total = len(grid)
        self._set_running(True)
        self._reset_align()
        self._status.setText(f"Building {len(grid)} tiles…")

        safe_z = self._safe_z if self._safe_z is not None else 0.0
        self._worker = _MosaicScanWorker(
            self._controller, cam, builder, grid, safe_z,
            expected_d_px=0.0, min_dist_px=1.0,          # skip detection
            fresh_frames=int(self._spin_fresh.value()),
            fresh_timeout_s=float(self._settings.get("fresh_timeout_s", 2.5)),
            settle_ms=int(self._spin_settle.value()),
            frame_orient=str(self._settings.get("frame_orient", "none")))
        self._worker.progress.connect(self._on_progress)
        self._worker.tile.connect(self._on_tile)
        self._worker.finished_ok.connect(self._on_finished)
        self._worker.failed.connect(self._on_failed)
        self._worker.start()

    def _set_running(self, running: bool):
        self._btn_build.setEnabled(not running)
        self._btn_cancel.setEnabled(running)
        for w in (self._spin_cols, self._spin_rows, self._combo_center,
                  self._spin_overlap, self._spin_settle, self._spin_fresh,
                  self._spin_fov, self._spin_maxshift):
            w.setEnabled(not running)
        # Clear-stored acts on the store, not the live overlay — usable whenever
        # not mid-build (even before the first mosaic exists).
        self._align_clear_btn.setEnabled(not running)
        if running:
            self._set_align_enabled(False)
            self._orient_box.setEnabled(False)

    # ── Manual global registration (slide mosaic onto wells by eye) ──

    def _set_align_enabled(self, on: bool):
        for w in (self._align_dx, self._align_dy, self._align_op,
                  self._align_store_btn, self._align_reset_btn):
            w.setEnabled(bool(on))

    # ── Spacing calibration (move tiles toward/away to align overlaps) ──

    def _on_align_changed(self, _v: int = 0):
        dx = int(self._align_dx.value())     # X gap %
        dy = int(self._align_dy.value())     # Y gap %
        op = int(self._align_op.value())
        self._align_dx_lbl.setText(f"{dx:+d}%")
        self._align_dy_lbl.setText(f"{dy:+d}%")
        self._align_op_lbl.setText(f"{op}%")
        kx = 1.0 + dx / 100.0
        ky = 1.0 + dy / 100.0
        if self._view is not None:
            self._view.set_spacing_factor(kx, ky)
            self._view.set_image_opacity(op / 100.0)
        self._update_readout(kx, ky)

    def _corrected_um_per_px(self, kx=None, ky=None) -> float:
        """Effective µm/px implied by the spacing factor. Spreading the tiles to
        align (k>1) means the assumed FOV was too large → µm/px = assumed / k."""
        if kx is None:
            kx = 1.0 + int(self._align_dx.value()) / 100.0
        if ky is None:
            ky = 1.0 + int(self._align_dy.value()) / 100.0
        k = (kx + ky) / 2.0
        return (self._um_camera / k) if k else self._um_camera

    def _update_readout(self, kx, ky):
        corr = self._corrected_um_per_px(kx, ky)
        fov = self._fw * corr
        overlap = float(self._spin_overlap.value()) / 100.0
        step = fov * (1.0 - overlap)
        self._align_readout.setText(
            f"Effective µm/px ≈ {corr:.3f} (was {self._um_camera:.3f}) · "
            f"FOV {fov:.0f} µm · step ≈ {step:.0f} µm @ "
            f"{int(overlap * 100)}% overlap")

    def _set_sliders(self, pct_x, pct_y) -> None:
        """Set the gap-% sliders (clamped) without firing live callbacks."""
        px = int(max(-50, min(50, round(float(pct_x)))))
        py = int(max(-50, min(50, round(float(pct_y)))))
        for sld, v in ((self._align_dx, px), (self._align_dy, py)):
            sld.blockSignals(True)
            sld.setValue(v)
            sld.blockSignals(False)
        self._align_dx_lbl.setText(f"{px:+d}%")
        self._align_dy_lbl.setText(f"{py:+d}%")
        kx = 1.0 + px / 100.0
        ky = 1.0 + py / 100.0
        if self._view is not None:
            self._view.set_spacing_factor(kx, ky)
        self._update_readout(kx, ky)

    def _seed_sliders_from_store(self) -> None:
        """Seed the gap-% from any stored effective µm/px (corrected = assumed/k
        → k = assumed/stored → pct = (k-1)·100), else 0."""
        pct = 0.0
        if self._store is not None and self._align_key and self._um_camera > 0:
            try:
                stored = self._store.get_um_per_px(self._align_key)
                if stored and stored > 0:
                    k = self._um_camera / stored
                    pct = (k - 1.0) * 100.0
            except Exception:
                pass
        self._set_sliders(pct, pct)

    def _reset_align(self):
        """Back to as-captured spacing (does NOT clear a stored value)."""
        self._set_sliders(0.0, 0.0)

    def _clear_stored_align(self):
        """Forget this camera + objective's stored FOV/alignment so auto sets it
        on the next scan."""
        if self._store is not None and self._align_key:
            try:
                self._store.clear(self._align_key)
            except Exception as e:
                logger.warning(f"Clear stored alignment failed: {e}")
        self._set_sliders(0.0, 0.0)
        self._status.setText(
            f"Cleared stored FOV/alignment for '{self._align_key}' — auto "
            f"sets it on the next scan.")

    def _store_manual_align(self):
        """Persist the corrected effective µm/px (from the spacing factor) for
        this camera + objective; full mosaics use it so the overlap is right.
        Also pushes the corrected FOV into the FOV field so it flows to the full
        mosaic via 'Apply to full mosaic'. Idempotent."""
        if self._result_comp is None:
            self._status.setText("Build a calibration mosaic first.")
            return
        corr = self._corrected_um_per_px()
        fov = int(round(self._fw * corr))
        key = self._align_key
        if self._store is not None and key and corr > 0:
            try:
                # Stamp the capture resolution the value was measured at, so a
                # consumer at a different frame width (e.g. the objective-
                # selectable fluorescence mosaic) can rescale it. Backward-
                # compatible: the store makes it optional; the full-plate scan
                # reads um_per_px without rescaling as before.
                self._store.set_um_per_px(
                    key, corr, source="quick_fov",
                    resolution=(self._fw, self._fh))
            except TypeError:
                # Older store signature without the resolution kwarg.
                self._store.set_um_per_px(key, corr, source="quick_fov")
            except Exception as e:
                logger.warning(f"FOV store failed: {e}")
        # v7.5.x: propagate as GROUND TRUTH — push to the live manager (click
        # mapping via effective_um_per_px) and, when the objective is known, the
        # objective store (mosaic fallback + restore across sessions). So a
        # correction here takes effect EVERYWHERE, not just as the mosaic
        # learned value.
        self._propagate_um_per_px(corr)
        # Reflect into the FOV field so "Apply to full mosaic" carries it.
        self._spin_fov.blockSignals(True)
        self._spin_fov.setValue(max(0, min(self._spin_fov.maximum(), fov)))
        self._spin_fov.blockSignals(False)
        self._status.setText(
            f"Stored effective µm/px {corr:.3f} for '{key}' (FOV {fov} µm). "
            f"Applied everywhere (mosaic + live view).")
        logger.info(f"Calibration FOV stored: key='{key}' µm/px={corr:.4f} "
                    f"fov={fov}µm")

    def _propagate_um_per_px(self, corr: float) -> None:
        """Push a corrected µm/px to every consumer so it is ground truth."""
        if not corr or corr <= 0:
            return
        res = (int(self._fw), int(self._fh))
        try:
            self._mgr.set_um_per_px(self._cam_idx, float(corr), resolution=res)
        except Exception as e:
            logger.debug(f"propagate µm/px to manager: {e}")
        if self._cam_key and self._objective:
            try:
                from SupportClasses.ObjectiveCalibration import (
                    get_store as _objs)
                rot = None
                try:
                    rot = self._mgr.get_rotation_deg(self._cam_idx)
                except Exception:
                    rot = None
                _objs().set_calibration(
                    str(self._cam_key), str(self._objective), float(corr), res,
                    rotation_deg=rot)
            except Exception as e:
                logger.debug(f"propagate µm/px to objective store: {e}")

    # ── Camera orientation (flip / rotate the images) ──────────────

    def _orient_set(self, rot: float, flip_x: bool, flip_y: bool) -> None:
        """Set the working orientation (flip X, flip Y, rotation — normalised),
        re-orient the tiles, and push + PERSIST live so every consumer (full
        mosaic, live view, click mapping) uses it at once."""
        rot = ((float(rot) + 180.0) % 360.0) - 180.0
        if rot == -180.0:
            rot = 180.0
        self._orient_rot = rot
        self._orient_mir = bool(flip_x)
        self._orient_fy = bool(flip_y)
        if abs(self._o_fine.value() - rot) > 1e-6:
            self._o_fine.blockSignals(True)
            self._o_fine.setValue(rot)
            self._o_fine.blockSignals(False)
        # Re-orient the displayed tiles.
        b = self._builder
        if b is not None and hasattr(b, "set_frame_orientation"):
            try:
                b.set_frame_orientation(rot, self._orient_mir, self._orient_fy)
                self._show_tiles()
            except Exception as e:
                logger.debug(f"orientation re-show: {e}")
        # Push live to the camera (feed correction + clicks + future scans).
        try:
            self._mgr.set_mirrored(self._cam_idx, self._orient_mir)
            self._mgr.set_rotation_deg(self._cam_idx, rot)
            sfy = getattr(self._mgr, "set_flip_y", None)
            if callable(sfy):
                sfy(self._cam_idx, self._orient_fy)
        except Exception:
            pass
        # v7.5.x: PERSIST immediately (per identity) so the FULL-PLATE mosaic
        # applies it when it places each tile — no separate "Apply" needed.
        self._persist_orientation(rot, self._orient_mir, self._orient_fy)
        self._status.setText(
            f"Orientation: flip X {'on' if self._orient_mir else 'off'}, "
            f"flip Y {'on' if self._orient_fy else 'off'}, rotation {rot:.1f}° "
            f"— saved (mosaic + live view).")

    def _persist_orientation(self, rot: float, flip_x: bool,
                             flip_y: bool) -> None:
        """Persist the working orientation to the per-identity ground-truth
        store so every consumer (full mosaic, live feed, click mapping) uses it."""
        try:
            ident = self._mgr.camera_identity(self._cam_idx)
        except Exception:
            ident = None
        if not (ident and ident[0]):
            return
        try:
            from SupportClasses.CameraCalibrationStore import get_store
            nm = ident[1] if len(ident) > 1 else ""
            st = get_store()
            st.set_rotation(ident[0], float(rot), name=nm)
            st.set_mirrored(ident[0], bool(flip_x), name=nm)
            sfy = getattr(st, "set_flip_y", None)
            if callable(sfy):
                sfy(ident[0], bool(flip_y), name=nm)
        except Exception as e:
            logger.debug(f"persist orientation: {e}")

    def _orient_rotate(self, delta: float) -> None:
        self._orient_set(self._orient_rot + delta, self._orient_mir,
                         self._orient_fy)

    def _orient_flip_h(self) -> None:
        # Flip X axis: toggle the horizontal flip (independent of flip Y).
        self._orient_set(self._orient_rot, not self._orient_mir,
                         self._orient_fy)

    def _orient_flip_v(self) -> None:
        # Flip Y axis: toggle the vertical flip (independent of flip X).
        self._orient_set(self._orient_rot, self._orient_mir,
                         not self._orient_fy)

    def _orient_fine_changed(self, val: float) -> None:
        self._orient_set(val, self._orient_mir, self._orient_fy)

    def _orient_reset(self) -> None:
        """Back to the camera's currently-saved orientation."""
        mir, fy, rot = False, False, 0.0
        try:
            fo = getattr(self._mgr, "full_orientation", None)
            if callable(fo):
                mir, fy, rot = fo(self._cam_idx)
            else:
                vo = getattr(self._mgr, "view_orientation", None)
                if callable(vo):
                    mir, rot = vo(self._cam_idx)
        except Exception:
            mir, fy, rot = False, False, 0.0
        self._orient_set(rot, mir, fy)

    def _orient_apply_to_camera(self) -> None:
        """Confirm the working orientation is saved (it is already pushed +
        persisted live by ``_orient_set``)."""
        from PySide6.QtWidgets import QMessageBox
        mir, fy, rot = self._orient_mir, self._orient_fy, self._orient_rot
        self._persist_orientation(rot, mir, fy)
        self._status.setText(
            f"Saved to the camera — flip X {'on' if mir else 'off'}, "
            f"flip Y {'on' if fy else 'off'}, rotation {rot:.1f}°.")
        QMessageBox.information(
            self, "Camera orientation",
            f"Saved to the camera — flip X {'on' if mir else 'off'}, "
            f"flip Y {'on' if fy else 'off'}, rotation {rot:.1f}°.\n\nThe live "
            "feed, mosaics, and click-mapping now use it.")

    # ── Worker signals ─────────────────────────────────────────────

    def _on_progress(self, done, total):
        if self._worker is None:
            return
        self._status.setText(f"Building calibration mosaic: {done}/{total} tiles…")

    def _show_tiles(self):
        """Push the INDIVIDUAL captured tiles (+ outlines) to the mosaic view so
        the operator can adjust spacing and align the overlaps."""
        if self._view is None:
            return
        b = self._builder
        try:
            tiles = b.tile_images_px() if b is not None else []
            scale = getattr(b, "_mosaic_scale", 1.0) if b is not None else 1.0
            self._view.set_tiles(tiles, scale or 1.0)
        except Exception as e:
            logger.debug(f"Mosaic tiles update skipped: {e}")

    def _on_tile(self, composite, extent):
        if self._worker is None:
            return
        self._show_tiles()

    def _on_finished(self, composite, extent, scale, frames, detections):
        if self._worker is None:
            return
        self._worker = None
        self._set_running(False)
        # Read builder state BEFORE clearing it.
        shift = getattr(self._builder, "_global_shift_um", (0.0, 0.0))
        n_meas = len(getattr(self._builder, "_measured_shifts", []) or [])
        # Keep the composite for the page handoff; show INDIVIDUAL tiles on the
        # right (so spacing can be adjusted to align the overlaps).
        if composite is not None:
            self._result_comp = composite
            self._result_ext = (tuple(float(v) for v in extent)
                                 if extent is not None else None)
        self._show_tiles()
        # v7.5.x: keep the builder alive (retained frames) so the Camera-
        # orientation controls can re-orient the tiles (flip/rotate → re-show).
        # Freed on stop / fail / close.
        if self._view is not None and self._view.has_content():
            self._set_align_enabled(True)
            self._orient_box.setEnabled(True)

        # Auto registration still records the residual GLOBAL SHIFT (separate
        # from the FOV/spacing); keep it unless a manual_align lock is in force.
        if n_meas == 0:
            self._status.setText(
                f"Built {frames} tiles. Adjust the X/Y gap sliders so features "
                f"line up in the OVERLAP regions (zoom/pan to inspect), then "
                f"“Store FOV/spacing”.")
        elif self._store is not None and self._store.is_manual(self._align_key):
            self._status.setText(
                f"Built {frames} tiles ({n_meas} overlaps registered). Kept your "
                f"stored manual shift. Align the gaps + “Store FOV/spacing”, or "
                f"“Clear stored” to let auto take over.")
        else:
            if self._store is not None:
                try:
                    self._store.set_shift_um(
                        self._align_key, shift[0], shift[1], frames=frames,
                        source="quick_cal")
                except Exception as e:
                    logger.warning(f"Calibration store failed: {e}")
            self._status.setText(
                f"Built {frames} tiles ({n_meas} overlaps). Auto shift "
                f"({shift[0]:.1f}, {shift[1]:.1f}) µm stored. Now adjust the X/Y "
                f"gap sliders to align the overlaps, then “Store FOV/spacing”.")
        self._seed_sliders_from_store()

    def _on_failed(self, msg):
        self._worker = None
        self._builder = None
        self._set_running(False)
        self._orient_box.setEnabled(False)
        self._status.setText(f"Calibration failed: {msg}")

    # ── Lifecycle ──────────────────────────────────────────────────

    def _stop_worker(self):
        w = self._worker
        self._worker = None
        if w is not None:
            for sig in (w.tile, w.progress, w.finished_ok, w.failed):
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
        self._set_running(False)
        self._builder = None
        self._orient_box.setEnabled(False)
        self._status.setText("Calibration stopped.")

    def closeEvent(self, event):
        self._stop_worker()
        super().closeEvent(event)
