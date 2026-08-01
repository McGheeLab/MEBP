"""fluorescence_mosaic_workflow.py — High-resolution multi-channel fluorescence
mosaic of a single well.

v7.5.x: Captures a high-resolution stitched mosaic of ONE well, once per
fluorescence channel (DAPI / FITC / mCherry / Cy5 …). There is no filter-wheel
hardware, so the operator switches the physical filter/illumination between
channels: the workflow rasters a full single-well mosaic for the current channel,
then prompts the operator to switch the filter and repeat for the next channel.

Because every channel of a well reuses the SAME raster grid + camera scale, their
composites register pixel-for-pixel; the workflow blends them into a false-colour
overlay using an operator-chosen pseudo-colour per channel. Captures persist
per (plate, well) in ``FluorescenceMosaicStore`` so ANY other workflow (Spheroid
Pick & Place, Cell Targeting, Cell Labeling, Quick Print, the Jog plate view …)
can show the fluorescence as a registered background — see
``gui/pages/workflows/_fluorescence_overlay.py``.

The single-well scan worker mirrors the Plate-Location mosaic worker
(``gui/pages/calibration.py::_MosaicScanWorker``): it commands stage moves +
samples frames thread-safely off the GUI thread, keeping the live feed responsive.
It deliberately omits the well-detection pass (we already know the well).
"""

from __future__ import annotations

import logging
import time
from typing import Optional

from PySide6.QtCore import QObject, Qt, QThread, QPointF, QRectF, Signal
from PySide6.QtGui import QColor, QPixmap, QPainter, QPen, QBrush
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox,
    QFrame, QSizePolicy, QSplitter, QMessageBox, QColorDialog,
    QSpinBox, QDoubleSpinBox, QGraphicsView, QGraphicsScene, QGraphicsItem,
    QGraphicsItemGroup, QGraphicsEllipseItem, QGraphicsRectItem,
)

from gui.styles import COLORS
from gui.scaling import s, sf, sp
from gui.widgets.components import Card
from gui.widgets.camera_feed_view import CameraFeedView
from gui.widgets.jog_well_plate import WellPlateNavigator
from gui.widgets.jog_workspace_view import pixmap_from_bgr
from gui.dialogs.workflow_settings_dialog import WorkflowSettingsDialog

from SupportClasses import FluorescenceMosaicStore as fms

try:
    from SupportClasses.HardwareConfig import CameraRole
except Exception:   # pragma: no cover
    CameraRole = None

logger = logging.getLogger(__name__)


class _SingleWellMosaicWorker(QThread):
    """Raster + grab + stitch ONE channel of a single-well mosaic on a
    background thread (mirrors calibration._MosaicScanWorker, no detection).

    Signals (queued → GUI-thread slots):
        progress(done, total)
        tile(composite_bgr_copy, extent_tuple)
        finished_ok(composite, extent, scale, frames, shift_um)
        failed(message)

    ``shift_um`` (v7.8) is the global registration shift the builder baked into
    ``extent``; the caller MUST persist it, because px → stage-µm
    back-projection needs ``extent[:2] − shift`` and the shift is otherwise
    unrecoverable from the saved mosaic.
    """

    progress = Signal(int, int)
    tile = Signal(object, object)
    finished_ok = Signal(object, object, float, int, object)
    failed = Signal(str)

    _MAX_CONSEC_NONE = 8

    def __init__(self, controller, cam, builder, positions, safe_z,
                 fresh_frames=3, fresh_timeout_s=2.5, settle_ms=300,
                 registration_method="fourier_mellin", parent=None):
        super().__init__(parent)
        self._controller = controller
        self._cam = cam
        self._builder = builder
        self._positions = list(positions)
        self._safe_z = safe_z
        self._fresh_frames = int(fresh_frames)
        self._fresh_timeout_s = float(fresh_timeout_s)
        self._settle_ms = max(0, int(settle_ms))
        self._registration_method = str(registration_method or "fourier_mellin")
        self._stop = False

    def stop(self):
        self._stop = True

    # v7.5.x: the per-tile ``_orient_frame`` (the retired coarse
    # ``mosaic_scan.frame_orient`` none/rot180/fliph/flipv transform) is GONE.
    # Frames now reach the builder RAW and the builder applies the MEASURED
    # camera->stage orientation in ``_orient_tile`` — the same single orientation
    # the full-plate scan, the live view and the click mapping use. Applying the
    # coarse string here was why this mosaic came out rotated but un-flipped
    # while the plate mosaic got both.

    def _read_global_shift(self) -> tuple:
        """The builder's global registration shift, ``(0, 0)`` when unavailable."""
        sh = getattr(self._builder, "_global_shift_um", None)
        try:
            return (float(sh[0]), float(sh[1]))
        except (TypeError, ValueError, IndexError):
            return (0.0, 0.0)

    def _grab_post_move_frame(self):
        cam = self._cam
        if self._settle_ms > 0 and not self._stop:
            time.sleep(self._settle_ms / 1000.0)
        try:
            c0 = cam.frame_count_value()
        except Exception:
            c0 = None
        if c0 is not None:
            t_end = time.time() + self._fresh_timeout_s
            while time.time() < t_end and not self._stop:
                try:
                    if cam.frame_count_value() - c0 >= self._fresh_frames:
                        break
                except Exception:
                    break
                time.sleep(0.02)
        try:
            return cam.get_current_frame()
        except Exception:
            return None

    def run(self):
        try:
            self._controller.suspend_position_poller()
        except Exception:
            pass
        try:
            total = len(self._positions)
            consecutive_none = 0
            for idx, (tx, ty) in enumerate(self._positions):
                if self._stop:
                    return
                try:
                    if idx == 0:
                        self._controller.safe_travel_to(
                            tx, ty, safe_z_mm=self._safe_z, target_z_mm=None)
                    else:
                        self._controller.move_xy_absolute_um(tx, ty)
                        try:
                            zero = self._controller.zero_position
                            self._controller.wait_for_xy_arrival(
                                (tx - float(zero.get("x", 0.0))) / 1000.0,
                                (ty - float(zero.get("y", 0.0))) / 1000.0)
                        except Exception:
                            pass
                except Exception as e:
                    logger.warning(
                        f"Fluor mosaic: move to ({tx:.0f},{ty:.0f}) failed: {e}")
                if self._stop:
                    return
                frame = self._grab_post_move_frame()
                if frame is None:
                    consecutive_none += 1
                    if consecutive_none >= self._MAX_CONSEC_NONE:
                        self.failed.emit(
                            "camera stopped delivering frames — scan aborted")
                        return
                    self.progress.emit(idx + 1, total)
                    continue
                consecutive_none = 0
                try:
                    xy = self._controller.get_xy_position(cached=False)
                    sx = xy[0] if xy and xy[0] is not None else tx
                    sy = xy[1] if xy and xy[1] is not None else ty
                except Exception:
                    sx, sy = tx, ty
                self._builder.add_raster_frame(frame, sx, sy, index=idx)
                self._builder.stitch_incremental()
                comp = self._builder.composite
                self.tile.emit(
                    comp.copy() if comp is not None else None,
                    self._builder.canvas_extent_um)
                self.progress.emit(idx + 1, total)

            if self._stop:
                return
            # v7.5.x: run the SAME two-step alignment the full-plate scan runs.
            # optimize_registration (pairwise Fourier-Mellin + weighted global
            # least-squares) was never called here, so this mosaic was stitched by
            # a strictly weaker algorithm than the plate scan — another way the
            # "same" mosaic came out different. It needs the retained
            # canvas-resolution tiles, which build_mosaic_builder now requests.
            try:
                if getattr(self._builder, "has_reorient_tiles", lambda: False)():
                    self._builder.optimize_registration(
                        method=self._registration_method)
            except Exception as e:
                logger.debug(f"Fluor mosaic optimize_registration skipped: {e}")
            try:
                self._builder.finalize_global_shift()
            except Exception as e:
                logger.debug(f"Fluor mosaic global shift skipped: {e}")
            composite = self._builder.composite
            extent = self._builder.canvas_extent_um
            scale = float(getattr(self._builder, "_mosaic_scale", 0.0) or 0.0)
            frames = self._builder.frame_count
            # canvas_extent_um ADDS the global shift while the tile pixels stay
            # in the raw stage frame, so the shift must travel with the extent
            # or px → stage-µm back-projection is wrong by up to 20% of a FOV.
            # Read it defensively, exactly as calibration._ploc_mosaic_world_shift does.
            shift = self._read_global_shift()
            self.finished_ok.emit(
                composite.copy() if composite is not None else None,
                extent, scale, frames, shift)
        except Exception as e:
            logger.exception("Fluor mosaic worker crashed")
            self.failed.emit(str(e))
        finally:
            try:
                self._controller.resume_position_poller()
            except Exception:
                pass


class _ChannelPill(QPushButton):
    """Checkable filter-cube pill. Click toggles inclusion; double-click opens
    the colour picker for the channel's display pseudo-colour."""

    color_requested = Signal()

    def __init__(self, text: str, parent: QWidget | None = None):
        super().__init__(text, parent)
        self.setCheckable(True)
        self.setChecked(True)
        self.setCursor(Qt.PointingHandCursor)

    def mouseDoubleClickEvent(self, event):
        self.color_requested.emit()
        super().mouseDoubleClickEvent(event)


class _ZoomImageView(QGraphicsView):
    """Minimal pan + scroll-to-zoom view of a single pixmap (the mosaic).

    Drag pans (ScrollHandDrag); the wheel zooms about the cursor. The first
    image (and any explicit :meth:`reset_fit`) fits to the view; later live
    updates preserve the operator's current zoom/pan.

    v7.8: opt-in :meth:`set_interactive_items` frees the left button for scene
    items (moving a detected-spheroid circle, grabbing its radius handle) and
    moves panning to the middle button — the ``_MappingView`` arrangement from
    ``mosaic_well_mapping_dialog``. Default OFF, so the Fluorescence Mosaic
    page's own behaviour is unchanged.

    Scene coordinates are mosaic pixels 1:1 (the pixmap is added at the origin
    and the scene rect is its bounding rect), which is what lets a host apply
    ``SpheroidDetector.back_project_px`` to a scene point directly.
    """

    # Left-click in interactive mode, in SCENE (= mosaic pixel) coords.
    scene_clicked = Signal(QPointF)
    scene_dragged = Signal(QPointF)
    scene_released = Signal(QPointF)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self._item = None
        self._grid_group = None
        self._fitted = False
        self._interactive_items = False
        self._panning = False
        self._pan_origin = None
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setRenderHints(
            QPainter.RenderHint.Antialiasing
            | QPainter.RenderHint.SmoothPixmapTransform)
        self.setBackgroundBrush(QColor(COLORS["base"]))
        self.setMinimumSize(s(220), s(180))

    # ── interactive-items mode (v7.8) ─────────────────────────────

    def set_interactive_items(self, enabled: bool) -> None:
        """Free the left button for scene items; pan with the middle button."""
        enabled = bool(enabled)
        if enabled == self._interactive_items:
            return
        self._interactive_items = enabled
        self.setDragMode(QGraphicsView.NoDrag if enabled
                         else QGraphicsView.ScrollHandDrag)

    def interactive_items(self) -> bool:
        return self._interactive_items

    def scene_obj(self) -> QGraphicsScene:
        """The scene, so a host can add/remove its own overlay items."""
        return self._scene

    def image_size(self) -> tuple[int, int]:
        """``(w, h)`` of the displayed mosaic pixmap, or ``(0, 0)``."""
        if self._item is None:
            return (0, 0)
        r = self._item.boundingRect()
        return (int(r.width()), int(r.height()))

    def reset_fit(self):
        self._fitted = False
        if self._item is not None:
            self.fitInView(self._item, Qt.KeepAspectRatio)
            self._fitted = True

    def prepare_fit(self):
        """Arm a fit for the NEXT image (used when switching wells)."""
        self._fitted = False

    def set_image(self, pixmap: QPixmap | None):
        self._scene.clear()
        self._item = None
        self._grid_group = None   # clear() destroyed any grid-preview items
        if pixmap is None or pixmap.isNull():
            self._fitted = False
            return
        self._item = self._scene.addPixmap(pixmap)
        self._scene.setSceneRect(QRectF(pixmap.rect()))
        if not self._fitted:
            self.fitInView(self._item, Qt.KeepAspectRatio)
            self._fitted = True

    def set_grid_preview(self, plan: dict | None):
        """Draw the planned raster (well boundary + tile footprints + centres)
        when there is no captured mosaic yet, so the operator can verify the
        scan will cover the whole well. ``plan`` is from
        FluorescenceMosaicWorkflowPage._compute_raster_plan()."""
        # Only valid when no real composite is shown (the caller guarantees this).
        self._scene.clear()
        self._item = None
        self._grid_group = None
        if not plan:
            self._fitted = False
            return
        eff = float(plan.get("eff_um_per_px") or 0.0)
        bounds = plan.get("bounds")
        grid = plan.get("grid") or []
        if eff <= 0 or not bounds:
            self._fitted = False
            return
        bx0, by0, bx1, by1 = bounds

        def to_px(ux, uy):
            return ((ux - bx0) / eff, (uy - by0) / eff)

        group = QGraphicsItemGroup()
        # Well boundary circle (yellow).
        wr = float(plan.get("well_radius_um") or 0.0)
        cx, cy = plan.get("center", ((bx0 + bx1) / 2.0, (by0 + by1) / 2.0))
        if wr > 0:
            pcx, pcy = to_px(cx, cy)
            rpx = wr / eff
            circ = QGraphicsEllipseItem(pcx - rpx, pcy - rpx, 2 * rpx, 2 * rpx)
            pen = QPen(QColor("#f9e2af"))
            pen.setCosmetic(True)
            pen.setWidthF(1.5)
            circ.setPen(pen)
            circ.setBrush(QBrush(Qt.NoBrush))
            group.addToGroup(circ)
        # Tile footprints (dashed blue) + centre dots.
        fov_w, fov_h = plan.get("fov_um", (0.0, 0.0))
        tw = (fov_w / eff) if fov_w else 0.0
        th = (fov_h / eff) if fov_h else 0.0
        tile_pen = QPen(QColor("#89b4fa"))
        tile_pen.setStyle(Qt.DashLine)
        tile_pen.setCosmetic(True)
        dot_r = max(1.0, min(tw, th) * 0.06) if (tw and th) else 2.0
        for (ux, uy) in grid:
            px, py = to_px(ux, uy)
            if tw and th:
                rect = QGraphicsRectItem(px - tw / 2.0, py - th / 2.0, tw, th)
                rect.setPen(tile_pen)
                rect.setBrush(QBrush(Qt.NoBrush))
                group.addToGroup(rect)
            dot = QGraphicsEllipseItem(px - dot_r, py - dot_r, 2 * dot_r, 2 * dot_r)
            dot.setPen(QPen(Qt.NoPen))
            dot.setBrush(QBrush(QColor("#89b4fa")))
            group.addToGroup(dot)
        self._scene.addItem(group)
        self._grid_group = group
        self._scene.setSceneRect(0.0, 0.0, (bx1 - bx0) / eff, (by1 - by0) / eff)
        self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
        # Arm a fit for the eventual real composite.
        self._fitted = False

    def clear_grid_preview(self):
        g = self._grid_group
        if g is not None:
            try:
                self._scene.removeItem(g)
            except Exception:
                pass
            self._grid_group = None

    def wheelEvent(self, event):
        if self._item is None and self._grid_group is None:
            super().wheelEvent(event)
            return
        factor = 1.25 if event.angleDelta().y() > 0 else 0.8
        self.scale(factor, factor)

    # ── mouse (interactive-items mode only) ───────────────────────

    def mousePressEvent(self, event):
        if not self._interactive_items:
            super().mousePressEvent(event)
            return
        if event.button() == Qt.MouseButton.MiddleButton:
            self._panning = True
            self._pan_origin = event.position().toPoint()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
            return
        if event.button() == Qt.MouseButton.LeftButton:
            # Let the base class start a drag when the press landed on a movable
            # item; otherwise report the click in scene coords. The host decides
            # what an empty-space click means (add a spheroid, place a rim point).
            it = self.itemAt(event.position().toPoint())
            if it is not None and bool(
                    it.flags() & QGraphicsItem.GraphicsItemFlag.ItemIsMovable):
                super().mousePressEvent(event)
                return
            self.scene_clicked.emit(
                self.mapToScene(event.position().toPoint()))
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._interactive_items and self._panning and self._pan_origin is not None:
            pos = event.position().toPoint()
            delta = pos - self._pan_origin
            self._pan_origin = pos
            hbar, vbar = self.horizontalScrollBar(), self.verticalScrollBar()
            hbar.setValue(hbar.value() - delta.x())
            vbar.setValue(vbar.value() - delta.y())
            event.accept()
            return
        if self._interactive_items:
            self.scene_dragged.emit(
                self.mapToScene(event.position().toPoint()))
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if (self._interactive_items
                and event.button() == Qt.MouseButton.MiddleButton):
            self._panning = False
            self._pan_origin = None
            self.setCursor(Qt.CursorShape.ArrowCursor)
            event.accept()
            return
        if self._interactive_items:
            self.scene_released.emit(
                self.mapToScene(event.position().toPoint()))
        super().mouseReleaseEvent(event)


def _contrast_fg(color: QColor) -> str:
    """Black or white text for legibility on ``color``."""
    lum = 0.299 * color.red() + 0.587 * color.green() + 0.114 * color.blue()
    return "#11111b" if lum > 140 else "#ffffff"


class FluorescenceMosaicWorkflowPage(QWidget):
    """High-resolution multi-channel fluorescence mosaic of a single well.

    v7.8: also embeddable. ``embedded=True`` drops the "← Back to Workflows"
    header row so a host page (the Spheroid Pick & Place survey tab) can mount a
    real INSTANCE of this page rather than reimplementing the scan — the same
    "re-home, don't rewrite" pattern ``FullPrintWorkflowPage`` uses for
    ``PrintingModePage``. There is therefore exactly one single-well mosaic scan
    implementation and it cannot diverge between the two surfaces.
    """

    back_requested = Signal()
    # Emitted (with the well name) whenever a mosaic for the selected well
    # becomes available — after a channel scan completes AND when a saved one is
    # loaded on a well change — so an embedding host can re-arm detection on
    # both paths.
    mosaic_ready = Signal(str)

    def __init__(self, controller, settings, camera_manager=None,
                 parent: QWidget | None = None, *, embedded: bool = False):
        super().__init__(parent)
        self._embedded = bool(embedded)
        self._controller = controller
        self._settings = settings
        self._camera_manager = camera_manager
        self._hw_config = None

        self._plate = None
        self._well_positions: dict[str, tuple[float, float]] | None = None
        self._safe_z: float | None = None

        self._camera_view: CameraFeedView | None = None
        self._camera_started_by_us = False

        # Per-channel display pseudo-colours (QColor), seeded from defaults.
        self._channel_colors: dict[str, QColor] = {
            ch: QColor(*fms.default_color(ch)) for ch in fms.CHANNELS
        }
        # Filter-cube toggle pills (checkable QPushButtons). Kept under the
        # _channel_checks name so isChecked()/setChecked()/_selected_channels()
        # stay identical to the checkbox version.
        self._channel_checks: dict[str, _ChannelPill] = {}

        # Capture run state
        self._worker: Optional[_SingleWellMosaicWorker] = None
        self._capture_queue: list[str] = []
        self._capture_index = 0
        self._scan_positions: list[tuple[float, float]] = []
        self._scan_bounds: tuple[float, float, float, float] | None = None
        self._scan_well: str | None = None
        self._scan_objective = ""
        self._scan_um_per_px = 0.0
        self._scan_frame_size = (0, 0)
        self._aborting = False

        # Settings popout (scan knobs)
        self._settings_dialog = WorkflowSettingsDialog(
            "fluorescence_mosaic", "Fluorescence Mosaic",
            parent=self, on_change=self._on_settings_changed)
        self._build_settings_dialog(self._settings_dialog)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(10), s(12), s(12))
        outer.setSpacing(s(10))
        if not self._embedded:
            outer.addLayout(self._build_header())
        # Top row: objective · filter-cube pills · selected well
        outer.addWidget(self._build_top_row())

        # Main area — three resizable sections:
        #   LEFT column (vertical split):  [top] well navigator  [bottom] live view
        #   RIGHT column:                  the mosaic, pan + scroll-to-zoom
        main_split = QSplitter(Qt.Horizontal, self)
        main_split.setChildrenCollapsible(False)

        left = QSplitter(Qt.Vertical, self)
        left.setChildrenCollapsible(False)

        self._navigator = WellPlateNavigator()
        self._navigator.well_clicked.connect(self._on_well_clicked)
        nav_card = Card("Well selection", flush=True)
        nav_card.add_widget(self._navigator)
        left.addWidget(nav_card)

        if camera_manager is not None:
            self._camera_view = CameraFeedView(
                camera_manager=camera_manager,
                cam_idx=self._resolve_microscope_cam_idx(),
                show_crosshair=True,
                auto_orient=True,   # v7.5.x: matches the mosaic's orientation
                label="Microscope feed — starts on this page",
            )
            cam_card = Card("Live view", flush=True)
            cam_card.add_widget(self._camera_view)
            left.addWidget(cam_card)
        left.setStretchFactor(0, 1)
        left.setStretchFactor(1, 1)
        main_split.addWidget(left)

        self._mosaic_view = _ZoomImageView()
        mosaic_card = Card("Mosaic (drag to pan · scroll to zoom)", flush=True)
        mosaic_card.add_widget(self._mosaic_view)
        fit_row = QHBoxLayout()
        fit_row.setContentsMargins(0, 0, 0, 0)
        fit_btn = QPushButton("Fit")
        fit_btn.setCursor(Qt.PointingHandCursor)
        fit_btn.clicked.connect(self._mosaic_view.reset_fit)
        fit_row.addStretch(1)
        fit_row.addWidget(fit_btn)
        mosaic_card.add_layout(fit_row)
        main_split.addWidget(mosaic_card)

        main_split.setStretchFactor(0, 2)
        main_split.setStretchFactor(1, 3)
        outer.addWidget(main_split, stretch=1)

        outer.addWidget(self._build_run_row())

        self._settings_dialog.load_last()
        self._refresh_channel_status()
        self._update_button_state()

    # ── UI construction ───────────────────────────────────────────

    def _build_header(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(s(8))
        back = QPushButton("← Back to Workflows")
        back.setCursor(Qt.PointingHandCursor)
        back.clicked.connect(self.back_requested.emit)
        row.addWidget(back)
        title = QLabel("Fluorescence Mosaic")
        title.setStyleSheet(
            f"color: {COLORS['blue']}; font-size: {sf(14)}pt; font-weight: 600;")
        row.addWidget(title)
        row.addStretch(1)
        settings_btn = QPushButton("⚙ Settings")
        settings_btn.setCursor(Qt.PointingHandCursor)
        settings_btn.clicked.connect(self._open_settings)
        row.addWidget(settings_btn)
        return row

    def _build_top_row(self) -> QFrame:
        """Top row: objective · filter-cube pills (toggle) · selected well."""
        frame = QFrame(self)
        row = QHBoxLayout(frame)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(8))

        row.addWidget(QLabel("Objective:"))
        self._objective_combo = QComboBox()
        self._objective_combo.setMinimumWidth(s(100))
        self._objective_combo.currentTextChanged.connect(self._on_objective_changed)
        row.addWidget(self._objective_combo)

        sep = QLabel("Filter cubes:")
        sep.setStyleSheet(f"color: {COLORS['subtext0']};")
        row.addSpacing(s(10))
        row.addWidget(sep)
        for ch in fms.CHANNELS:
            pill = _ChannelPill(ch)
            pill.setToolTip(
                f"{ch}: click to include in the capture, double-click to set "
                f"its display colour.")
            pill.toggled.connect(
                lambda _checked, c=ch: self._on_pill_toggled(c))
            pill.color_requested.connect(lambda c=ch: self._pick_color(c))
            self._channel_checks[ch] = pill
            self._apply_pill(ch)
            row.addWidget(pill)

        row.addStretch(1)
        row.addWidget(QLabel("Well:"))
        self._well_label = QLabel("—")
        self._well_label.setStyleSheet(
            f"color: {COLORS['text']}; font-size: {sf(13)}pt; font-weight: 600;")
        row.addWidget(self._well_label)
        if self._embedded:
            # The header (which normally carries ⚙ Settings) is suppressed when
            # embedded, so the scan knobs would otherwise be unreachable.
            scan_settings_btn = QPushButton("⚙ Scan settings")
            scan_settings_btn.setCursor(Qt.PointingHandCursor)
            scan_settings_btn.clicked.connect(self._open_settings)
            row.addSpacing(s(10))
            row.addWidget(scan_settings_btn)
        return frame

    def _on_pill_toggled(self, channel: str):
        self._apply_pill(channel)
        self._update_button_state()

    def _build_run_row(self) -> QFrame:
        frame = QFrame(self)
        row = QHBoxLayout(frame)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(10))
        self._start_btn = QPushButton("Start capture")
        self._start_btn.clicked.connect(self._on_start)
        row.addWidget(self._start_btn)
        self._abort_btn = QPushButton("Abort")
        self._abort_btn.setEnabled(False)
        self._abort_btn.clicked.connect(self._on_abort)
        row.addWidget(self._abort_btn)
        row.addStretch(1)
        self._status = QLabel("Idle.")
        self._status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(10)}pt;")
        self._status.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        row.addWidget(self._status, stretch=1)
        return frame

    # ── Settings popout ───────────────────────────────────────────

    def _dspin(self, lo, hi, val, suffix="", decimals=1, step=None):
        sb = QDoubleSpinBox()
        sb.setRange(lo, hi)
        sb.setDecimals(decimals)
        if suffix:
            sb.setSuffix(suffix)
        sb.setValue(val)
        if step is not None:
            sb.setSingleStep(step)
        return sb

    def _build_settings_dialog(self, dlg: WorkflowSettingsDialog):
        self._target_px = QSpinBox()
        self._target_px.setRange(500, 12000)
        self._target_px.setSingleStep(250)
        self._target_px.setValue(2500)
        self._well_margin = self._dspin(1.0, 2.0, 1.15, "×", 2, 0.05)
        sec = dlg.add_section("Scan")
        sec.add("target_px", "Mosaic resolution (px)", self._target_px, 2500)
        sec.add("well_margin", "Well coverage", self._well_margin, 1.15)
        # The stitch-critical scan parameters — camera-mount orientation
        # (frame_orient), FOV override, tile overlap, registration and the
        # camera-settle timing — are NOT duplicated here. They are inherited
        # from the SAME persisted ``mosaic_scan`` settings the full-plate
        # Plate-Location mosaic uses (see _scan_settings), so a single-well
        # fluorescence mosaic stitches with the exact same pattern as the
        # full-plate mosaic. (Tune them on Calibration → Plate Location →
        # Mosaic scan → Settings.)
        note = QLabel(
            "Camera orientation, FOV, overlap and settle timing are inherited "
            "from the full-plate Mosaic scan settings (Calibration → Plate "
            "Location → Mosaic scan → Settings).")
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        sec.add_widget(note)

        # ── Mosaic FOV calibration (mirrors the full-plate scan's Calibrate…) ──
        cal = dlg.add_section("Mosaic FOV calibration")
        cal.add_note(
            "Build a small mosaic for the CURRENT objective and tune the tile "
            "spacing so the overlaps line up. The learned FOV/spacing is stored "
            "per camera + objective and sizes the raster grid — do this before a "
            "long multi-channel scan so the tiles tile correctly.")
        self._cal_button = QPushButton("Calibrate…")
        self._cal_button.setCursor(Qt.PointingHandCursor)
        self._cal_button.setToolTip(
            "Open the small-mosaic FOV/spacing calibration for the selected "
            "objective (the same tool as the full-plate scan).")
        self._cal_button.clicked.connect(self._open_mosaic_calibration)
        cal.add_widget(self._cal_button)
        self._cal_status_lbl = QLabel("")
        self._cal_status_lbl.setWordWrap(True)
        cal.add_widget(self._cal_status_lbl)

        dlg.finalize()
        self._refresh_calibration_status()

    def _scan_settings(self) -> dict:
        """Stitch-critical scan parameters, sourced from the SAME persisted
        ``mosaic_scan`` settings the full-plate Plate-Location mosaic uses.

        This is the fix for the single-well mosaic misalignment: the camera on
        ME3B V1 is mounted rotated (``frame_orient="rot180"``) and the operator
        calibrated an explicit FOV (``fov_um``) + overlap for the full-plate
        mosaic. The fluorescence workflow previously kept its OWN copies of
        these (defaulting frame_orient to "none"), so every captured tile was
        un-rotated relative to its stage placement and the mosaic couldn't
        stitch. Reading the shared section makes the single-well scan follow
        the exact same pattern as the full-plate scan. Robust to a ``settings``
        object without ``get_section`` (returns the documented defaults)."""
        try:
            from gui.dialogs.mosaic_settings_dialog import merged_settings
            stored = None
            if (self._settings is not None
                    and hasattr(self._settings, "get_section")):
                stored = self._settings.get_section("mosaic_scan")
            return merged_settings(stored)
        except Exception:
            try:
                from gui.dialogs.mosaic_settings_dialog import MOSAIC_SCAN_DEFAULTS
                return dict(MOSAIC_SCAN_DEFAULTS)
            except Exception:
                return {}

    def _open_settings(self):
        self._settings_dialog.show()
        self._settings_dialog.raise_()
        self._settings_dialog.activateWindow()

    def _on_settings_changed(self):
        self._update_button_state()
        # Overlap / well-coverage / resolution changes the planned grid.
        self._refresh_grid_preview()

    # ── Channel pills / colours ───────────────────────────────────

    def _captured_channels(self) -> set[str]:
        plate_key = self._plate_key()
        if plate_key and self._scan_well:
            try:
                return set(fms.get_store().list_channels(plate_key, self._scan_well))
            except Exception:
                return set()
        return set()

    def _apply_pill(self, channel: str, captured: set[str] | None = None):
        pill = self._channel_checks[channel]
        c = self._channel_colors[channel]
        if captured is None:
            captured = self._captured_channels()
        pill.setText(f"{channel} ✓" if channel in captured else channel)
        if pill.isChecked():
            pill.setStyleSheet(
                f"QPushButton{{background-color:{c.name()};color:{_contrast_fg(c)};"
                f"border:1px solid {c.name()};border-radius:{sp(11)};"
                f"padding:{sp(3)} {sp(12)};font-weight:600;}}")
        else:
            pill.setStyleSheet(
                f"QPushButton{{background-color:{COLORS['surface0']};"
                f"color:{COLORS['subtext0']};border:1px solid {c.name()};"
                f"border-radius:{sp(11)};padding:{sp(3)} {sp(12)};}}")

    def _pick_color(self, channel: str):
        c = QColorDialog.getColor(
            self._channel_colors[channel], self, f"{channel} colour")
        if c.isValid():
            self._channel_colors[channel] = c
            self._apply_pill(channel)
            # If already captured, persist the colour change + refresh preview.
            plate_key = self._plate_key()
            if plate_key and self._scan_well:
                try:
                    fms.get_store().set_channel_color(
                        plate_key, self._scan_well, channel,
                        (c.red(), c.green(), c.blue()))
                except Exception:
                    pass
                self._refresh_preview()

    # ── Required by MainWindow ────────────────────────────────────

    def get_page_title(self) -> str:
        return "Fluorescence Mosaic"

    def get_sub_page_title(self) -> str:
        return "Fluorescence Mosaic"

    def get_context_widget(self):
        return None

    def on_status_update(self) -> None:
        pass

    def set_settings(self, settings) -> None:
        self._settings = settings

    def showEvent(self, event):
        self._start_camera()
        # Draw the planned-raster preview once now and again shortly after, so it
        # appears as soon as the camera starts delivering frames (needed for the
        # FOV/grid sizing).
        self._refresh_grid_preview()
        try:
            from PySide6.QtCore import QTimer
            QTimer.singleShot(900, self._refresh_grid_preview)
        except Exception:
            pass
        super().showEvent(event)

    def hideEvent(self, event):
        self._stop_camera()
        try:
            if self._settings_dialog.isVisible():
                self._settings_dialog.hide()
        except Exception:
            pass
        super().hideEvent(event)

    # ── hw_config + calibration routing ───────────────────────────

    def set_hardware_config(self, hw_config):
        self._hw_config = hw_config
        self._refresh_objectives()
        if self._camera_view is not None:
            cam_idx = self._resolve_microscope_cam_idx()
            try:
                if self._camera_view.cam_idx != cam_idx:
                    self._camera_view.set_camera(cam_idx)
            except Exception:
                pass
        self._refresh_channel_status()
        self._update_button_state()

    def set_calibration_data(self, plate, well_positions, safe_z) -> None:
        self._plate = plate
        self._well_positions = well_positions
        self._safe_z = safe_z
        if plate is not None:
            try:
                self._navigator.set_plate(plate)
            except Exception:
                pass
        if well_positions:
            try:
                self._navigator.set_calibrated_wells(set(well_positions.keys()))
                self._navigator.set_well_positions(well_positions)
            except Exception:
                pass
            if self._scan_well is None:
                self._select_well(self._default_well())
        self._refresh_channel_status()
        self._refresh_grid_preview()
        self._update_button_state()

    def set_z_references(self, refs) -> None:
        pass

    def _default_well(self) -> str | None:
        if self._well_positions:
            if "A1" in self._well_positions:
                return "A1"
            return next(iter(self._well_positions))
        if self._plate is not None:
            try:
                names = list(self._plate.well_names)
                return names[0] if names else None
            except Exception:
                return None
        return None

    # ── Objective ─────────────────────────────────────────────────

    def _resolve_microscope_cam_idx(self) -> int:
        if self._hw_config is not None and CameraRole is not None:
            try:
                idx = self._hw_config.camera_for_role(CameraRole.MICROSCOPE)
                if idx is not None:
                    return int(idx)
            except Exception:
                pass
        return 0

    def _camera_key(self) -> str | None:
        cam_idx = self._resolve_microscope_cam_idx()
        cfg = self._hw_config
        spec = getattr(getattr(cfg, "camera_config", None), "camera_spec", None)
        if spec is not None and getattr(spec, "name", None):
            return str(spec.name)
        return f"camera_{cam_idx}"

    def _current_objective_name(self) -> str:
        cfg = self._hw_config
        obj = ""
        if cfg is not None:
            obj = getattr(getattr(cfg, "camera_config", None),
                          "current_objective_name", "") or ""
        if not obj and getattr(self, "_objective_combo", None) is not None:
            obj = self._objective_combo.currentText()
        return obj

    def _objective_um_per_px(self, frame_w: float) -> float | None:
        """Resolution-rescaled µm/px for the CURRENTLY SELECTED objective, or
        ``None`` when that objective has no stored calibration for this camera.

        Per-camera + per-objective, so the raster FOV tracks the objective
        (2x / 4x / 10x). This is what makes the grid spacing follow the
        objective selection — mirrors calibration.py::_ploc_microscope_um_per_px
        (the objective's ``measured_um_per_px`` was taken at ``resolution``; if
        the camera now captures at a different width, µm/px scales inversely by
        ``cal_width / current_width``)."""
        try:
            from SupportClasses.ObjectiveCalibration import get_store as obj_store
            cam_key = self._camera_key()
            obj = self._current_objective_name()
            if cam_key and obj:
                cal = obj_store().get_calibration(str(cam_key), str(obj))
                if cal:
                    meas = float(cal.get("measured_um_per_px") or 0.0)
                    res = cal.get("resolution")
                    cal_w = float(res[0]) if (res and len(res) >= 1) else 0.0
                    if meas > 0 and cal_w > 0 and frame_w > 0:
                        return meas * (cal_w / float(frame_w))
        except Exception:
            pass
        return None

    def _microscope_um_per_px(self, frame_w: float, fallback: float) -> float:
        """Objective-store µm/px rescaled to the LIVE frame width (see
        :meth:`_objective_um_per_px`), falling back to ``fallback`` when the
        current objective has no stored calibration."""
        eff = self._objective_um_per_px(frame_w)
        return eff if (eff and eff > 0) else fallback

    # ── Mosaic FOV/spacing calibration (mirrors the full-plate scan) ──

    def _align_store(self):
        """The shared per-camera+objective mosaic-alignment store (learned FOV /
        spacing + registration shift) — the same store the full-plate scan's
        'Calibrate…' writes to."""
        try:
            from SupportClasses.MosaicAlignmentStore import get_store
            return get_store()
        except Exception:
            return None

    def _align_key(self) -> str:
        """Key for the alignment store: microscope camera IDENTITY + current
        objective — identical scheme to calibration.py::_ploc_camera_objective_key
        so a calibration done here (or in Plate Location) round-trips for the
        same camera + objective."""
        obj = self._current_objective_name() or "default"
        ident = None
        mgr = self._camera_manager
        cam_idx = self._resolve_microscope_cam_idx()
        if mgr is not None:
            try:
                res = mgr.camera_identity(cam_idx)   # (key, name) | None
                if res:
                    ident = res[0]
            except Exception:
                ident = None
        return f"{ident}|{obj}" if ident else str(obj)

    def _learned_um_per_px(self, frame_w: float) -> float | None:
        """The learned effective µm/px from a mosaic FOV/spacing calibration for
        THIS camera + objective, rescaled to the live frame width, or ``None``.

        Resolution-safe: the stored value carries the capture resolution it was
        measured at, so a value taken at 916 px is rescaled for a 3664 px live
        frame (µm/px ∝ 1/width). A LEGACY value with no recorded resolution is
        deliberately ignored here — without the resolution it can't be trusted
        across the objective-selectable widths this workflow runs at, and the
        resolution-safe objective-store value (``_objective_um_per_px``) is the
        better fallback."""
        store = self._align_store()
        key = self._align_key()
        if store is None or not key:
            return None
        try:
            val = store.get_um_per_px(key)
            if not val or val <= 0:
                return None
            res = store.get_resolution(key)
            if res and res[0] > 0 and frame_w > 0:
                return float(val) * (float(res[0]) / float(frame_w))
        except Exception:
            pass
        return None

    def _open_mosaic_calibration(self):
        """Open the small-mosaic FOV/spacing calibration for the CURRENT
        objective — the same pop-out the full-plate scan uses (Calibration →
        Plate Location → Mosaic scan → Calibrate…), wired to this workflow's
        camera + objective + selected well. Building a small mosaic and tuning
        the spacing stores a learned effective µm/px (per camera + objective)
        that :meth:`_learned_um_per_px` then feeds into the raster grid — so the
        tiles tile correctly for the chosen objective before a long scan."""
        if self._worker is not None and self._worker.isRunning():
            QMessageBox.information(
                self, "Mosaic calibration",
                "Wait for the current capture to finish before calibrating.")
            return
        if self._controller is None or self._camera_manager is None:
            QMessageBox.warning(
                self, "Mosaic calibration",
                "Stage controller and camera manager are required.")
            return
        # Safe-Z gate — same as the scan (the calibration mosaic retracts the
        # needle before every XY hop; on ME3B V1 (ZDIR=-1) a retract with no
        # Safe Z would drive the needle DOWN into the plate).
        if getattr(self._controller, "is_zp_connected", False) and self._safe_z is None:
            QMessageBox.warning(
                self, "Mosaic calibration",
                "Set the Safe / Move Z on the Calibration page first — the "
                "calibration mosaic retracts the needle before every move.")
            return
        cam_idx = self._resolve_microscope_cam_idx()
        try:
            cam = self._camera_manager.cameras[cam_idx]
        except (AttributeError, IndexError):
            QMessageBox.warning(
                self, "Mosaic calibration", f"Camera {cam_idx} not available.")
            return
        try:
            if not self._camera_manager.is_running(cam_idx):
                self._camera_manager.start(cam_idx)
                self._camera_started_by_us = True
        except Exception:
            pass
        if not self._camera_manager.is_um_per_px_calibrated(cam_idx):
            QMessageBox.warning(
                self, "Mosaic calibration",
                "Calibrate the microscope objective µm/pixel first "
                "(Hardware Setup → Cameras).")
            return
        frame = None
        try:
            frame = cam.get_current_frame()
            if frame is None and hasattr(cam, "capture_fresh_frame"):
                frame = cam.capture_fresh_frame(discard_n_frames=2, settle_ms=300)
        except Exception:
            frame = None
        if frame is None:
            QMessageBox.warning(
                self, "Mosaic calibration",
                "Microscope camera is not producing frames yet — start it and retry.")
            return
        fh, fw = frame.shape[:2]
        # Objective-resolved µm/px at the LIVE width (the "assumed" FOV the
        # spacing slider corrects). Same value the raster planner uses.
        base = 0.0
        try:
            base = float(self._camera_manager.effective_um_per_px(cam_idx, fw)
                         or self._camera_manager.get_um_per_px(cam_idx) or 0.0)
        except Exception:
            base = float(self._camera_manager.get_um_per_px(cam_idx) or 0.0)
        um_cam = self._microscope_um_per_px(fw, base)
        if um_cam <= 0:
            QMessageBox.warning(
                self, "Mosaic calibration", "Microscope µm/pixel not calibrated.")
            return
        # Centre the calibration mosaic on the selected well (texture + where the
        # scan happens), falling back to the plate centre.
        center = self._well_center_um(self._scan_well) if self._scan_well else None
        if center is None:
            try:
                center = self._controller.default_plate_center_um()
            except Exception:
                center = (0.0, 0.0)
        try:
            from gui.dialogs.mosaic_calibration_dialog import MosaicCalibrationDialog
        except Exception as e:
            logger.warning("Mosaic calibration dialog unavailable: %s", e)
            QMessageBox.warning(
                self, "Mosaic calibration",
                "The mosaic calibration dialog is unavailable.")
            return
        # Inherit the shared stitch settings (orientation / overlap / settle /
        # timing) BUT force fov_um=0 so the calibration mosaic sizes its tiles
        # from the objective-resolved µm/px we pass (``um_per_px_camera``), not
        # the shared full-plate FOV — which is calibrated for ONE objective and
        # would build a wrong-scale calibration mosaic for a different one.
        cal_settings = dict(self._scan_settings())
        cal_settings["fov_um"] = 0
        dlg = MosaicCalibrationDialog(
            self._controller, self._camera_manager, cam_idx,
            safe_z=self._safe_z,
            align_key=self._align_key(),
            store=self._align_store(),
            settings=cal_settings,
            center_um=center,
            frame_size=(fw, fh),
            um_per_px_camera=um_cam,
            parent=self)
        dlg.exec()
        # A stored learned FOV/spacing changes the grid; refresh preview + status.
        self._refresh_calibration_status()
        self._refresh_grid_preview()

    def _refresh_calibration_status(self):
        """Update the settings-popout label with the learned FOV/spacing state
        for the current camera + objective."""
        lbl = getattr(self, "_cal_status_lbl", None)
        if lbl is None:
            return
        obj = self._current_objective_name() or "—"
        store = self._align_store()
        key = self._align_key()
        val = None
        res = None
        if store is not None and key:
            try:
                val = store.get_um_per_px(key)
                res = store.get_resolution(key)
            except Exception:
                val = res = None
        if val and val > 0:
            res_txt = (f" @ {int(res[0])}px" if res else
                       " (no resolution — recalibrate)")
            lbl.setText(
                f"{obj}: calibrated · {float(val):.3f} µm/px{res_txt}")
            lbl.setStyleSheet(
                f"color: {COLORS['green']}; font-size: {sf(9)}pt;")
        else:
            lbl.setText(
                f"{obj}: not calibrated — using the objective µm/px "
                f"(Calibrate… to fine-tune the spacing).")
            lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")

    def _refresh_objectives(self):
        try:
            from SupportClasses.ObjectiveCalibration import get_store as obj_store
            names = obj_store().objective_names()
        except Exception:
            names = []
        current = ""
        cfg = self._hw_config
        if cfg is not None:
            current = getattr(
                getattr(cfg, "camera_config", None), "current_objective_name", "") or ""
        self._objective_combo.blockSignals(True)
        self._objective_combo.clear()
        self._objective_combo.addItems(names)
        if current:
            i = self._objective_combo.findText(current)
            if i >= 0:
                self._objective_combo.setCurrentIndex(i)
        self._objective_combo.blockSignals(False)
        # The combo was populated/selected under blockSignals, so
        # _on_objective_changed never fired — push the restored objective's
        # calibrated µm/px (+ resolution stamp) to the manager explicitly so
        # effective_um_per_px is correct app-wide (it's guarded / a no-op when
        # the objective has no stored calibration).
        cur = self._objective_combo.currentText()
        if cur:
            self._on_objective_changed(cur)

    def _on_objective_changed(self, name: str):
        """Push the objective's calibrated µm/px (+ rotation) to the camera
        manager and record it as the current objective — mirrors the Hardware
        Setup objective card so the mosaic scale is correct."""
        if not name:
            return
        cfg = self._hw_config
        if cfg is not None and hasattr(cfg, "camera_config"):
            try:
                cfg.camera_config.current_objective_name = name
            except Exception:
                pass
        cam_idx = self._resolve_microscope_cam_idx()
        cam_key = self._camera_key()
        if self._camera_manager is None or not cam_key:
            return
        try:
            from SupportClasses.ObjectiveCalibration import get_store as obj_store
            cal = obj_store().get_calibration(cam_key, name)
        except Exception:
            cal = None
        if not cal:
            return
        try:
            self._camera_manager.set_um_per_px(
                cam_idx, float(cal["measured_um_per_px"]),
                resolution=cal.get("resolution"))
            if cal.get("rotation_deg") is not None:
                self._camera_manager.set_rotation_deg(
                    cam_idx, float(cal["rotation_deg"]))
        except Exception as exc:
            logger.debug("Fluor mosaic objective apply failed: %s", exc)
        # The FOV (and thus the grid) depends on the objective scale; the learned
        # FOV/spacing is per-objective, so refresh its status readout too.
        self._refresh_calibration_status()
        self._refresh_grid_preview()

    # ── Well selection ────────────────────────────────────────────

    def _on_well_clicked(self, well_name: str):
        self._select_well(well_name)

    def _select_well(self, well_name: str | None):
        if not well_name:
            return
        self._scan_well = well_name
        self._well_label.setText(well_name)
        try:
            self._navigator.set_current_well(well_name)
        except Exception:
            pass
        self._refresh_channel_status()
        # A new well → fit the next mosaic image to the view.
        self._mosaic_view.prepare_fit()
        self._refresh_preview()
        self._refresh_grid_preview()
        self._update_button_state()
        self._notify_mosaic_ready()

    def _notify_mosaic_ready(self) -> None:
        """Tell an embedding host that this well's mosaic state changed.

        Fired on both paths a mosaic can appear — a completed channel scan and a
        well change that loads a saved one — so a host never has to guess which
        one happened. Emits with the well name, or "" when the well has none.
        """
        well = self._scan_well or ""
        try:
            plate_key = self._plate_key()
            has = bool(plate_key and well
                       and fms.get_store().has(plate_key, well))
        except Exception:
            has = False
        self.mosaic_ready.emit(well if has else "")

    # ── Embedding host API (v7.8) ─────────────────────────────────
    # Read-only accessors so a host can drive detection without reaching into
    # this page's privates. Nothing here commands motion.

    def mosaic_view(self):
        """The zoomable mosaic view, for overlaying host-owned scene items."""
        return self._mosaic_view

    def current_well(self) -> str | None:
        return self._scan_well

    def plate_key(self) -> str | None:
        return self._plate_key()

    def selected_channels(self) -> list[str]:
        return self._selected_channels()

    def stored_channels(self) -> list[str]:
        """Channels with a saved mosaic for the selected well."""
        plate_key = self._plate_key()
        if not plate_key or not self._scan_well:
            return []
        try:
            return fms.get_store().list_channels(plate_key, self._scan_well)
        except Exception:
            return []

    def is_scanning(self) -> bool:
        """True while a channel raster is driving the stage.

        A host MUST consult this before any manual travel: the worker holds the
        serial channel and toggles the non-refcounted position-poller suspend.
        """
        w = self._worker
        return w is not None and w.isRunning()

    def well_geometry(self):
        """``(center_um, radius_um)`` of the selected well, or ``(None, None)``.

        Absolute stage µm, straight from the calibrated/geometric well map —
        used to mask detections outside the well.
        """
        well = self._scan_well
        if not well:
            return (None, None)
        center = self._well_center_um(well)
        diam_mm = self._well_diameter_mm(well)
        if center is None or diam_mm <= 0:
            return (center, None)
        return (center, (diam_mm / 2.0) * 1000.0)

    def mosaic_context(self, channel: str | None = None) -> dict | None:
        """Everything needed to detect on this well and back-project the result.

        Returns None when the well has no stored mosaic. ``channel=None`` picks
        the first stored channel. ``image`` is that ONE channel's raw composite,
        deliberately not the blended overlay: the blend goes through grayscale
        and a saturating add, which clips overlapping channels and pushes a
        saturated blob's edge outward — i.e. it over-reads a diameter.

        ``scale`` is cross-checked against the image width so a channel whose
        stored ``mosaic_scale`` disagrees with its own pixels is reported rather
        than silently mis-projected.
        """
        plate_key = self._plate_key()
        well = self._scan_well
        if not plate_key or not well:
            return None
        store = fms.get_store()
        chans = store.list_channels(plate_key, well)
        if not chans:
            return None
        ch = str(channel) if channel and str(channel) in chans else chans[0]
        image = store.load_channel_image(plate_key, well, ch)
        extent = store.get_extent_um(plate_key, well, ch)
        if image is None or extent is None:
            return None
        stored_scale = store.get_mosaic_scale(plate_key, well, ch)
        span_x = float(extent[2]) - float(extent[0])
        derived_scale = (image.shape[1] / span_x) if span_x > 0 else 0.0
        scale_warning = ""
        scale = stored_scale or derived_scale
        if stored_scale and derived_scale > 0:
            rel = abs(stored_scale - derived_scale) / derived_scale
            if rel > 0.01:
                scale_warning = (
                    f"stored mosaic scale {stored_scale:.5f} px/µm disagrees "
                    f"with the image's own {derived_scale:.5f} px/µm "
                    f"({rel * 100:.1f}%) — re-scan this well")
        center_um, radius_um = self.well_geometry()
        return {
            "plate_key": plate_key,
            "well": well,
            "channel": ch,
            "channels": list(chans),
            "image": image,
            "extent_um": tuple(float(v) for v in extent),
            "mosaic_scale": float(scale or 0.0),
            "derived_scale": float(derived_scale),
            "scale_warning": scale_warning,
            "shift_um": store.get_shift_um(plate_key, well, ch),
            "has_shift": store.has_shift(plate_key, well, ch),
            "um_per_px": store.get_um_per_px(plate_key, well, ch) or 0.0,
            "objective": store.get_objective(plate_key, well),
            "well_center_um": center_um,
            "well_radius_um": radius_um,
        }

    # ── Live camera ───────────────────────────────────────────────

    def _start_camera(self):
        if self._camera_manager is None or self._camera_view is None:
            return
        cam_idx = self._resolve_microscope_cam_idx()
        try:
            if self._camera_view.cam_idx != cam_idx:
                self._camera_view.set_camera(cam_idx)
            if not self._camera_manager.is_running(cam_idx):
                self._camera_manager.start(cam_idx)
                self._camera_started_by_us = True
        except Exception as e:
            logger.debug("Fluor mosaic camera start failed: %s", e)

    def _stop_camera(self):
        if (self._camera_manager is None or self._camera_view is None
                or not self._camera_started_by_us):
            return
        try:
            self._camera_manager.stop(self._camera_view.cam_idx)
        except Exception:
            pass
        finally:
            self._camera_started_by_us = False

    # ── Geometry ──────────────────────────────────────────────────

    def _plate_key(self) -> str | None:
        cfg = self._hw_config
        if cfg is None:
            return None
        key = getattr(cfg, "active_plate_key", None) or getattr(cfg, "plate_name", None)
        return str(key) if key else None

    def _well_center_um(self, well: str) -> tuple[float, float] | None:
        if self._well_positions and well in self._well_positions:
            return self._well_positions[well]
        # Geometric fallback from the plate-centre seed.
        try:
            cx, cy = self._controller.default_plate_center_um()
            sign = (1.0, 1.0)
            if hasattr(self._controller, "plate_axis_sign"):
                sign = self._controller.plate_axis_sign()
            positions = self._plate.get_all_positions_from_plate_center(cx, cy, sign)
            return positions.get(well)
        except Exception:
            return None

    def _well_diameter_mm(self, well: str) -> float:
        plate = self._plate
        if plate is None:
            return 0.0
        try:
            if getattr(plate, "well_diameter", 0.0) > 0:
                return float(plate.well_diameter)
            for w in plate.get_all_wells():
                if w.name == well:
                    return float(getattr(w, "diameter", 0.0) or 0.0)
        except Exception:
            pass
        return 0.0

    # ── Unified mosaic calibration ────────────────────────────────

    def _mosaic_calibration(self, live_resolution):
        """The ONE resolved mosaic calibration for this page's camera/objective.

        v7.5.x: identical resolution to the full-plate and rosette scans (see
        ``SupportClasses/MosaicCalibration``), so all three mosaics are oriented
        and scaled the same way by construction. Returns None if unavailable.
        """
        try:
            from SupportClasses.MosaicCalibration import resolve
        except ImportError:
            return None
        cam_idx = self._resolve_microscope_cam_idx()
        try:
            return resolve(
                camera_manager=self._camera_manager, cam_idx=cam_idx,
                camera_name=self._camera_key(),
                objective=self._current_objective_name(),
                live_resolution=live_resolution,
                scan_settings=self._scan_settings())
        except Exception as exc:
            logger.debug("fluorescence mosaic calibration resolve failed: %s", exc)
            return None

    # ── Raster plan (shared by the grid preview AND the actual scan) ──

    def _compute_raster_plan(self, well) -> dict | None:
        """Resolve the raster plan for ``well``: tile centres, bounds, FOV, scale.

        Returns a dict (bounds, grid, eff_um_per_px, frame_size, fov_um, center,
        well_radius_um, cols, rows, overlap_frac, target_px) or None when the
        inputs aren't ready (no well/diameter/camera/µm-per-px). The SAME math
        feeds the grid preview and ``_on_start`` so the preview matches the scan.
        """
        if not well or self._camera_manager is None:
            return None
        center = self._well_center_um(well)
        diam_mm = self._well_diameter_mm(well)
        if center is None or diam_mm <= 0:
            return None
        cam_idx = self._resolve_microscope_cam_idx()
        try:
            cam = self._camera_manager.cameras[cam_idx]
        except (AttributeError, IndexError):
            return None
        fw = fh = 0
        try:
            frame = cam.get_current_frame()
            if frame is not None:
                fh, fw = frame.shape[:2]
        except Exception:
            pass
        if fw <= 0:
            fw, fh = self._scan_frame_size
        if fw <= 0 or fh <= 0:
            return None
        # v7.5.x: ONE resolver for every mosaic (see SupportClasses/
        # MosaicCalibration). This page is OBJECTIVE-SELECTABLE (2x/4x/10x), which
        # the resolver handles by keying µm/px on (camera, objective) — so the
        # right scale is used per objective without this page carrying its own
        # precedence. It previously had a DIFFERENT precedence from the full-plate
        # scan (and honoured the retired coarse ``frame_orient`` instead of the
        # measured orientation), which is why the same camera produced
        # differently-oriented, differently-scaled fluorescence and plate mosaics.
        cal = self._mosaic_calibration((fw, fh))
        if cal is None or cal.um_per_px <= 0:
            return None
        eff = cal.um_per_px
        overlap_frac = cal.overlap_frac
        step = None
        margin = float(self._well_margin.value())
        r_um = (diam_mm / 2.0) * 1000.0 * margin
        cx, cy = center
        bounds = self._clip_bounds((cx - r_um, cy - r_um, cx + r_um, cy + r_um))
        if bounds is None:
            return None
        try:
            from SupportClasses.MosaicCalibration import build_mosaic_builder
        except ImportError:
            return None
        target_px = int(self._target_px.value())
        tmpl = build_mosaic_builder(cal, target_mosaic_px=target_px)
        grid = tmpl.generate_raster_positions(
            bounds, overlap=overlap_frac, step_x_um=step, step_y_um=step)
        env = self._envelope()
        if env is not None:
            grid = [(x, y) for (x, y) in grid
                    if env[0] <= x <= env[2] and env[1] <= y <= env[3]]
        if not grid:
            return None
        cols = len({round(x, 1) for (x, _y) in grid})
        rows = len({round(y, 1) for (_x, y) in grid})
        return {
            "bounds": bounds, "grid": grid, "eff_um_per_px": eff,
            "frame_size": (fw, fh), "fov_um": (fw * eff, fh * eff),
            "center": (cx, cy), "well_radius_um": (diam_mm / 2.0) * 1000.0,
            "cols": cols, "rows": rows, "overlap_frac": overlap_frac,
            "target_px": target_px,
        }

    def _refresh_grid_preview(self):
        """Draw the planned raster (well boundary + tile footprints) in the
        mosaic viewer AND overlay a grid on the selected well in the navigator,
        so the operator can verify coverage before scanning. Skipped (and the
        captured mosaic shown instead) once a channel exists for the well."""
        view = getattr(self, "_mosaic_view", None)
        nav = getattr(self, "_navigator", None)
        if view is None:
            return
        try:
            well = self._scan_well
            plate_key = self._plate_key()
            # A captured mosaic is shown by _refresh_preview — don't draw the
            # grid over it.
            if not well or (plate_key and fms.get_store().has(plate_key, well)):
                view.clear_grid_preview()
                if nav is not None:
                    nav.set_raster_grid(0, 0)
                return
            plan = self._compute_raster_plan(well)
            if plan is None:
                view.clear_grid_preview()
                if nav is not None:
                    nav.set_raster_grid(0, 0)
                return
            view.set_grid_preview(plan)
            if nav is not None:
                nav.set_raster_grid(plan["cols"], plan["rows"])
            running = self._worker is not None and self._worker.isRunning()
            if not running:
                fx, fy = plan["fov_um"]
                self._status.setText(
                    f"{well}: planned raster {plan['cols']}×{plan['rows']} = "
                    f"{len(plan['grid'])} tiles/channel "
                    f"(FOV {fx / 1000.0:.2f}×{fy / 1000.0:.2f} mm).")
        except Exception as e:
            logger.debug("grid preview refresh failed: %s", e)

    # ── Start / capture sequence ──────────────────────────────────

    def _selected_channels(self) -> list[str]:
        return [ch for ch in fms.CHANNELS if self._channel_checks[ch].isChecked()]

    def _update_button_state(self, *_):
        running = self._worker is not None and self._worker.isRunning()
        ready = bool(self._scan_well) and bool(self._selected_channels())
        self._start_btn.setEnabled(ready and not running)
        self._abort_btn.setEnabled(running)

    def _on_start(self):
        if self._worker is not None and self._worker.isRunning():
            return
        well = self._scan_well
        if not well:
            self._status.setText("Select a well first.")
            return
        channels = self._selected_channels()
        if not channels:
            self._status.setText("Select at least one channel to capture.")
            return
        if self._camera_manager is None:
            self._status.setText("No camera manager available.")
            return
        cam_idx = self._resolve_microscope_cam_idx()
        try:
            cam = self._camera_manager.cameras[cam_idx]
        except (AttributeError, IndexError):
            self._status.setText(f"Camera index {cam_idx} not available.")
            return
        if not self._camera_manager.is_um_per_px_calibrated(cam_idx):
            self._status.setText(
                "Microscope camera µm/pixel not calibrated — calibrate the "
                "objective in Hardware Setup → Cameras.")
            return
        # ZP gate: the raster retracts the needle before each XY hop; without a
        # Safe Z, a retract on ME3B V1 (ZDIR=-1) would drive the needle DOWN.
        if getattr(self._controller, "is_zp_connected", False) and self._safe_z is None:
            self._status.setText(
                "Set the Safe / Move Z on the Calibration page first — the scan "
                "retracts the needle to it before every move.")
            return
        center = self._well_center_um(well)
        if center is None:
            self._status.setText(
                "Could not resolve the well position — calibrate the plate first.")
            return
        diam_mm = self._well_diameter_mm(well)
        if diam_mm <= 0:
            self._status.setText(
                "Unknown well diameter — check the active plate in Hardware Setup.")
            return

        # Grab a frame to size the FOV / µm/px.
        frame = None
        try:
            frame = cam.get_current_frame()
            if frame is None and hasattr(cam, "capture_fresh_frame"):
                frame = cam.capture_fresh_frame(discard_n_frames=2, settle_ms=300)
        except Exception:
            frame = None
        if frame is None:
            self._status.setText(
                "Microscope camera is not producing frames yet — start it and retry.")
            return

        # Resolve the raster plan (FOV/µm-per-px from the objective store, rescaled
        # to the live frame width — same math as the grid preview, so the scan
        # matches the preview and actually covers the whole well).
        plan = self._compute_raster_plan(well)
        if plan is None:
            self._status.setText(
                "Could not plan the raster for this well — check the objective "
                "µm/px calibration (Hardware Setup → Cameras) and the XY envelope.")
            return
        grid = plan["grid"]
        bounds = plan["bounds"]
        eff_um_per_px = plan["eff_um_per_px"]
        fw, fh = plan["frame_size"]

        n_tiles = len(grid)
        est_min = n_tiles * len(channels) * 1.5 / 60.0
        proceed = QMessageBox.question(
            self, "Fluorescence mosaic",
            f"This will raster {n_tiles} tiles ({plan['cols']}×{plan['rows']}) per "
            f"channel × {len(channels)} channel(s) (~{est_min:.0f} min total). "
            f"You'll be prompted to switch the filter between channels. The needle "
            f"stays retracted at Safe Z.\n\nStart?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No)
        if proceed != QMessageBox.StandardButton.Yes:
            return

        # Stash run params and start the per-channel sequence.
        self._scan_positions = grid
        self._scan_bounds = bounds
        self._scan_objective = self._current_objective_name()
        self._scan_um_per_px = eff_um_per_px
        self._scan_frame_size = (fw, fh)
        self._capture_queue = channels
        self._capture_index = 0
        self._aborting = False
        self._start_camera()
        self._prompt_next_channel()

    def _prompt_next_channel(self):
        if self._aborting:
            self._status.setText("Aborted.")
            self._update_button_state()
            return
        if self._capture_index >= len(self._capture_queue):
            self._status.setText(
                f"Done — captured {len(self._capture_queue)} channel(s) for "
                f"{self._scan_well}.")
            self._refresh_channel_status()
            self._refresh_preview()
            self._update_button_state()
            return
        channel = self._capture_queue[self._capture_index]
        num = fms.channel_number(channel)
        ch_label = (f"{channel} channel ({num})" if num is not None
                    else f"{channel} channel")
        resp = QMessageBox.information(
            self, "Set filter",
            f"Set the microscope filter / illumination for the "
            f"{ch_label}, focus if needed, then click OK to scan.\n\n"
            f"(Cancel stops the capture.)",
            QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Ok)
        if resp != QMessageBox.StandardButton.Ok:
            self._status.setText("Capture stopped by operator.")
            self._update_button_state()
            return
        self._start_channel_scan(channel)

    def _start_channel_scan(self, channel: str):
        cam_idx = self._resolve_microscope_cam_idx()
        try:
            cam = self._camera_manager.cameras[cam_idx]
        except (AttributeError, IndexError):
            self._status.setText("Camera unavailable.")
            return
        fw, fh = self._scan_frame_size
        # v7.5.x: EVERY stitch-critical parameter now comes from the ONE shared
        # resolver, so this single-well scan is identical to the full-plate and
        # rosette scans by construction.
        #
        # This is the fix for "rosette, full plate overview and fluorescence
        # should all have the exact same behaviour". Before, this path applied the
        # RETIRED coarse ``mosaic_scan.frame_orient`` string (a none/rot180/
        # fliph/flipv per-tile transform) instead of the measured camera->stage
        # orientation — so on a camera stored as rotation 180 + flip_y it rotated
        # but LOST THE FLIP, while the plate scan applied both. It also used its
        # own target_px and never ran optimize_registration.
        cal = self._mosaic_calibration((fw, fh))
        if cal is None:
            self._status.setText("Mosaic calibration unavailable.")
            return
        try:
            from SupportClasses.MosaicCalibration import (
                build_mosaic_builder, refuse_reason)
        except ImportError:
            self._status.setText("MosaicCalibration unavailable.")
            return
        why = refuse_reason(cal)
        if why:
            self._status.setText(why)
            logger.warning(f"Fluorescence mosaic refused: {why}")
            return
        overlap_frac = cal.overlap_frac
        step = None
        target_px = int(self._target_px.value())
        settle_ms = cal.settle_ms
        fresh_frames = cal.fresh_frames
        fresh_timeout_s = cal.fresh_timeout_s
        # retain_frames=False: a long scan otherwise accumulates ~2 MB/tile of
        # dead image data (this path used to keep them all).
        builder = build_mosaic_builder(
            cal, target_mosaic_px=target_px, retain_for_reorient=True,
            retain_frames=False)
        # CRITICAL: allocate the composite canvas on the SAME builder the worker
        # uses. generate_raster_positions() is the only thing that calls
        # _init_composite(); in _on_start the grid was generated on a THROWAWAY
        # template builder, so without this the worker's builder keeps
        # composite/extent = None → stitch_incremental no-ops → empty viewer AND
        # the channel save is skipped. (Same one-builder pattern as the
        # Plate-Location mosaic scanner.) The regenerated grid is identical to
        # self._scan_positions (same bounds/overlap/FOV); the return is ignored.
        if self._scan_bounds is not None:
            try:
                builder.generate_raster_positions(
                    self._scan_bounds, overlap=overlap_frac,
                    step_x_um=step, step_y_um=step)
            except Exception as e:
                logger.warning("Fluor mosaic: builder canvas init failed: %s", e)
        safe_z = self._safe_z if self._safe_z is not None else 0.0
        self._status.setText(
            f"[{self._capture_index + 1}/{len(self._capture_queue)}] "
            f"Scanning {channel}: 0/{len(self._scan_positions)}…")
        self._worker = _SingleWellMosaicWorker(
            self._controller, cam, builder, self._scan_positions, safe_z,
            fresh_frames=fresh_frames, fresh_timeout_s=fresh_timeout_s,
            settle_ms=settle_ms, registration_method=cal.reg_method)
        self._worker.progress.connect(self._on_channel_progress)
        self._worker.tile.connect(self._on_channel_tile)
        self._worker.finished_ok.connect(
            lambda comp, ext, scale, frames, shift, ch=channel:
            self._on_channel_finished(ch, comp, ext, scale, frames, shift))
        self._worker.failed.connect(self._on_channel_failed)
        self._worker.start()
        self._update_button_state()

    def _on_channel_progress(self, done: int, total: int):
        ch = (self._capture_queue[self._capture_index]
              if self._capture_index < len(self._capture_queue) else "")
        self._status.setText(
            f"[{self._capture_index + 1}/{len(self._capture_queue)}] "
            f"Scanning {ch}: {done}/{total}…")

    def _on_channel_tile(self, composite, extent):
        if composite is not None:
            self._set_preview_image(composite)

    def _on_channel_finished(self, channel, composite, extent, scale, frames,
                             shift_um=(0.0, 0.0)):
        self._worker = None
        if composite is not None and extent is not None:
            plate_key = self._plate_key() or "plate"
            color = self._channel_colors[channel]
            try:
                fms.get_store().save_channel(
                    plate_key, self._scan_well, channel, composite, extent,
                    color_rgb=(color.red(), color.green(), color.blue()),
                    objective=self._scan_objective,
                    um_per_px=self._scan_um_per_px, mosaic_scale=scale,
                    frames=frames, shift_um=shift_um)
            except Exception as exc:
                logger.warning("Fluor mosaic save failed: %s", exc)
        self._capture_index += 1
        self._refresh_channel_status()
        self._refresh_preview()
        self._notify_mosaic_ready()
        self._prompt_next_channel()

    def _on_channel_failed(self, msg: str):
        self._worker = None
        self._status.setText(f"Channel scan failed: {msg}")
        self._update_button_state()

    def _on_abort(self):
        self._aborting = True
        if self._worker is not None:
            self._worker.stop()
        self._status.setText("Abort requested…")

    # ── Preview + status ──────────────────────────────────────────

    def _refresh_channel_status(self):
        """Re-style every pill to reflect captured (✓) + checked state."""
        captured = self._captured_channels()
        for ch in fms.CHANNELS:
            if ch in self._channel_checks:
                self._apply_pill(ch, captured)

    def _refresh_preview(self):
        plate_key = self._plate_key()
        if not plate_key or not self._scan_well:
            return
        try:
            image, _extent = fms.get_store().composite_overlay(
                plate_key, self._scan_well)
        except Exception:
            image = None
        if image is None:
            self._mosaic_view.set_image(None)
            return
        self._set_preview_image(image)

    def _set_preview_image(self, image_bgr):
        pix = pixmap_from_bgr(image_bgr)
        if pix is None:
            return
        self._mosaic_view.set_image(pix)

    # ── Envelope clipping ─────────────────────────────────────────

    def _envelope(self):
        sl = getattr(self._controller, "safety_limits", None)
        if sl is None:
            return None
        try:
            return (float(sl.xy_min_x), float(sl.xy_min_y),
                    float(sl.xy_max_x), float(sl.xy_max_y))
        except Exception:
            return None

    def _clip_bounds(self, bounds):
        env = self._envelope()
        if env is None:
            return bounds
        min_x = max(bounds[0], env[0])
        min_y = max(bounds[1], env[1])
        max_x = min(bounds[2], env[2])
        max_y = min(bounds[3], env[3])
        if max_x <= min_x or max_y <= min_y:
            return None
        return (min_x, min_y, max_x, max_y)
