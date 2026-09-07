"""fluorescence_mosaic_workflow.py — High-resolution multi-channel fluorescence
mosaic of a single well.

v7.5.x: Captures a high-resolution stitched mosaic of ONE well, once per
fluorescence channel (DAPI / FITC / mCherry / Cy5 …). The workflow rasters a full
single-well mosaic for the current channel, then moves to the next channel.

⚠ WHICH FILTER CUBE IS IN THE LIGHT PATH IS A KNOWN, DRIVABLE FACT.
This docstring once asserted "there is no filter-wheel hardware, so the operator
switches the physical filter/illumination between channels". That is not true:
the Nikon Ti's ``FilterBlockCassette1`` is motorized and hardware-verified
switching all six slots with read-back, and ``MicroscopeController.set_filter``
has been public the whole time.

⚠⚠ v7.18 then replaced that claim with the OPPOSITE one — that the cube is
switched automatically — while the code to do it did not exist anywhere; every
channel still got an unconditional modal. **v7.19 is where the code landed**, so
the paragraph below now describes the shipped behaviour rather than an intent:
``_ensure_cube_for`` resolves the channel through
``FluorescenceMosaicStore.channel_slot`` (exact → normalized → operator alias →
the body's own name, REFUSING rather than guessing) and drives it with
``OpticsService.ensure_filter``, which verifies by read-back. The per-channel
prompt is the fallback, shown with the refusal reason when there is no body, the
slot is unnamed or empty, the channel name does not resolve to a cube (this
rig's cassette holds "TxRed" while the channel vocabulary says "mCherry", which
the software must never equate on its own), or the backend is SIMULATED — a
simulated switch is not a real one, and the operator still has a cube to move.

v7.19 also moved every acquisition control — objective, cubes, per-cube exposure
/ gain / averaging / display levels, the camera preset toggle and the scan order
— onto this workflow's own left context panel
(``gui/widgets/fluorescence_controls_panel.py``), above a live raw histogram, so
a signal is judged BEFORE the run rather than in a modal partway through it.

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

import dataclasses
import logging
import time
from typing import Optional

from PySide6.QtCore import QObject, Qt, QThread, QPointF, QRectF, QTimer, Signal
from PySide6.QtGui import QColor, QPixmap, QPainter, QPen, QBrush
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox,
    QFrame, QSizePolicy, QSplitter, QMessageBox, QColorDialog,
    QSpinBox, QDoubleSpinBox, QGraphicsView, QGraphicsScene, QGraphicsItem,
    QGraphicsItemGroup, QGraphicsEllipseItem, QGraphicsRectItem,
    QCheckBox, QDialog, QDialogButtonBox, QPlainTextEdit,
)

from gui.styles import COLORS
from gui.scaling import s, sf, sp
from gui.widgets.components import Card
from gui.widgets.section_stack import (
    PromotedSectionsPanel, wire_section_promotion)
from gui.widgets.camera_feed_view import CameraFeedView
from gui.widgets.jog_well_plate import WellPlateNavigator
from gui.widgets.jog_workspace_view import pixmap_from_bgr
from gui.worker_retirement import retire_worker
from gui.dialogs.workflow_settings_dialog import WorkflowSettingsDialog
# v7.19 — the workflow's own left context panel. `_ChannelPill` is re-exported
# from here under its historical name so every existing reference resolves to
# the one class.
from gui.widgets.fluorescence_controls_panel import (  # noqa: F401
    FluorescenceControlsPanel, _ChannelPill)
from gui.widgets.hw_controls_snapshot import (
    apply_hw_controls, fluorescence_preset, hw_controls_snapshot)

from SupportClasses import FluorescenceMosaicStore as fms
from SupportClasses.CaptureTiming import resolve_grab_timing

try:
    from SupportClasses.HardwareConfig import CameraRole
except Exception:   # pragma: no cover
    CameraRole = None

try:
    from SupportClasses.TileAutofocus import AfAbort, AfCancelled
except Exception:   # pragma: no cover
    class AfAbort(RuntimeError):
        ...

    class AfCancelled(RuntimeError):
        ...

try:
    from SupportClasses.MosaicFocusTracker import FocusSampleXY
except Exception:   # pragma: no cover
    FocusSampleXY = None

logger = logging.getLogger(__name__)

# v7.13 — coarse-AF search half-range at the probe position when no previous
# survey narrows it. The operator has just focused at the channel prompt, so
# this only needs to absorb ordinary hand mis-focus. Bench-tunable.
PROBE_HALF_RANGE_UM = 150.0


@dataclasses.dataclass
class ChannelPlan:
    """One channel's builder + signal recipe, for a tile-major run (v7.19).

    ``levels`` is the frozen mono16→8-bit conversion for THIS channel. The
    operator's explicit black/white beat the probe's measurement — they set
    them while watching the histogram, so the saved mosaic should look like the
    preview did. Left None, the probe measures them at the well centre.
    """

    channel: str
    builder: object
    exposure_us: float = 0.0
    gain_pct: "float | None" = None
    avg_frames: int = 1
    levels: "tuple[float, float] | None" = None
    frames_used: int = 0


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
    # v7.13: 6th arg = meta dict {display_levels, avg_frames, exposure_us,
    # levels_degenerate, af_note, af?, focus_map?}.
    finished_ok = Signal(object, object, float, int, object, object)
    # v7.19 tile-major: one per channel as it completes, then finished_all.
    # Same payload as finished_ok with the channel name prepended.
    channel_done = Signal(str, object, object, float, int, object, object)
    finished_all = Signal()
    failed = Signal(str)

    _MAX_CONSEC_NONE = 8
    _LEASE = "fluor_mosaic_af"

    def __init__(self, controller, cam, builder, positions, safe_z,
                 fresh_frames=3, fresh_timeout_s=2.5, settle_ms=300,
                 registration_method="fourier_mellin",
                 avg_frames=1, probe_xy=None, exposure_us=0.0,
                 scope=None, autofocus=None, tracker=None,
                 focus_predictor=None, optics=None,
                 probe_half_range_um=PROBE_HALF_RANGE_UM,
                 probe_sigma_um=None, channels=None, parent=None):
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
        # v7.19 — non-empty switches this worker to TILE-MAJOR. Empty keeps the
        # legacy single-channel path completely untouched.
        self._channels = list(channels or [])
        self._optics_service = None
        # v7.13 — averaged raw capture with per-channel FROZEN display levels.
        self._avg_frames = max(1, int(avg_frames))
        self._probe_xy = probe_xy
        self._exposure_us = float(exposure_us or 0.0)
        self._levels: "tuple[float, float] | None" = None
        self._levels_degenerate = False
        self._avg_used = 1
        self._avg_warned = False
        self._probed = False
        # v7.13 — per-tile autofocus. ``tracker`` (first channel: checkerboard
        # sweeps + running fit) or ``focus_predictor`` (later channels: replay
        # of the measured map). The worker owns the microscope LEASE for the
        # scan and restores the entry focus in ``finally``.
        self._scope = scope
        self._af = autofocus
        self._tracker = tracker
        self._predictor = focus_predictor
        self._optics = optics
        self._probe_half = float(probe_half_range_um or PROBE_HALF_RANGE_UM)
        self._probe_sigma = probe_sigma_um
        self._af_note = ""

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

    def _wait_settled(self) -> bool:
        """Settle + wait for genuinely-new frames after a move (the
        motion-settle guarantee every capture path shares).

        Returns True when a post-move frame is known to have arrived. False
        means the camera did not deliver one in time, so anything read now was
        exposed before or during the move.

        v7.14: the timeout is sized from the camera's own frame period.
        Fluorescence runs the longest exposures in the app, and at full
        resolution three frames can take longer than the flat configured
        timeout — which used to be abandoned silently.
        """
        cam = self._cam
        if self._settle_ms > 0 and not self._stop:
            time.sleep(self._settle_ms / 1000.0)
        try:
            c0 = cam.frame_count_value()
        except Exception:
            c0 = None
        if c0 is None:
            # Cannot report frame arrivals — the settle is all we have.
            return True
        n_frames, timeout_s, _period = resolve_grab_timing(
            cam, self._fresh_frames, self._fresh_timeout_s)
        t_end = time.time() + timeout_s
        while time.time() < t_end and not self._stop:
            try:
                if cam.frame_count_value() - c0 >= n_frames:
                    return True
            except Exception:
                return True
            time.sleep(0.02)
        if self._stop:
            return False
        logger.warning(
            "Fluor mosaic: no new camera frame within %.1f s after the move "
            "(needed %d) — dropping this tile rather than stitching a frame "
            "exposed before/during the move", timeout_s, n_frames)
        return False

    def _grab_post_move_frame(self):
        if not self._wait_settled():
            return None
        try:
            return self._cam.get_current_frame()
        except Exception:
            return None

    # ── v7.13: averaged raw capture with frozen per-channel levels ──

    def _avg_timeout_s(self) -> float:
        # Generous: long fluorescence exposures deliver frames slowly.
        return self._fresh_timeout_s + 0.5 * self._avg_frames

    def _freeze_levels(self, raw) -> "tuple[float, float] | None":
        """Choose the channel's fixed display mapping from a probe frame.

        P0.5 → black; P99.9 stretched by 20% headroom → white, so tiles
        BRIGHTER than the probe frame don't clip. A degenerate spread (empty
        well / lamp off) falls back to a narrow fixed window and is flagged
        in the meta rather than silently autoscaling noise to full range.
        """
        try:
            import numpy as np
            step = max(1, int(max(raw.shape) // 512))
            sample = raw[::step, ::step]
            lo = float(np.percentile(sample, 0.5))
            hi_raw = float(np.percentile(sample, 99.9))
            hi = min(65535.0, lo + max(0.0, hi_raw - lo) * 1.2)
            if hi - lo < 16.0:
                self._levels_degenerate = True
                hi = lo + 256.0
            return (lo, hi)
        except Exception:
            return None

    def _capture_tile(self):
        """One tile image: averaged raw (fixed levels) when available, else
        the legacy single display frame — byte-identical fallback."""
        if not self._wait_settled():
            # No confirmed post-move frame: skip the tile. Stitching one
            # exposed during the move places a smeared image at this canvas
            # spot and feeds the overlap registration a bogus measurement.
            return None
        if self._stop:
            return None
        if self._avg_frames > 1 and hasattr(self._cam, "capture_raw_average"):
            raw = None
            try:
                raw = self._cam.capture_raw_average(
                    self._avg_frames, timeout_s=self._avg_timeout_s())
            except Exception:
                raw = None
            if raw is not None:
                if self._levels is None:
                    self._levels = self._freeze_levels(raw)
                try:
                    from gui.widgets.mono_display import mono_to_bgr8
                    frame8 = mono_to_bgr8(raw, levels=self._levels)
                except Exception:
                    frame8 = None
                if frame8 is not None:
                    self._avg_used = self._avg_frames
                    return frame8
            if not self._avg_warned:
                # Do NOT fake-average display frames: each is independently
                # autoscaled, so their mean is not quantitative. Single-frame
                # fallback is honest and preserves today's behaviour exactly.
                logger.info(
                    "Fluor mosaic: raw frame averaging unavailable on this "
                    "camera — falling back to single display frames")
                self._avg_warned = True
        try:
            return self._cam.get_current_frame()
        except Exception:
            return None

    # ── v7.13: probe visit (frozen levels + coarse autofocus) ─────

    def _move_to(self, tx, ty, first: bool):
        """One raster move; ``first`` gets the retract-gated safe travel."""
        if first:
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

    def _do_probe(self):
        """Visit the well centre ONCE before the raster: run the coarse AF
        solve and freeze the channel's display levels there.

        The raster's first tile is a corner of the well's bounding square —
        guaranteed EMPTY GLASS on a circular well — so freezing levels (or
        seeding focus) from tile 1 would calibrate on background and clip
        every real structure. The well centre is where the sample is.
        """
        wants_levels = (self._avg_frames > 1
                        and hasattr(self._cam, "capture_raw_average"))
        wants_af = self._af is not None and self._tracker is not None
        if self._probe_xy is None or not (wants_levels or wants_af):
            return
        px, py = self._probe_xy
        try:
            self._move_to(px, py, first=True)
        except Exception as e:
            logger.warning(f"Fluor mosaic: probe move failed: {e}")
            return
        self._probed = True
        if self._stop:
            return
        self._wait_settled()

        # Coarse AF FIRST — the levels probe should be taken in focus.
        if wants_af:
            center = self._af.focus_now_um()
            if center is None or self._optics is None:
                self._af_note = ("autofocus off: focus position or objective "
                                 "optics unavailable")
                self._tracker = None
            else:
                peak, why = self._af.coarse_solve(
                    self._optics, float(center), self._probe_half,
                    prior_sigma_um=self._probe_sigma,
                    should_stop=lambda: self._stop)
                if peak is not None:
                    self._af.goto_focus(peak.z_um)
                    if FocusSampleXY is not None:
                        self._tracker.add(FocusSampleXY(
                            x_um=float(px), y_um=float(py),
                            focus_um=float(peak.z_um),
                            prominence=float(peak.prominence),
                            sigma_um=float(peak.sigma_z_um), tile_index=-1))
                else:
                    # Degrade to a normal no-AF scan and SAY so — never limp
                    # on with a fit seeded by a refused curve.
                    self._af_note = f"autofocus off for this scan: {why}"
                    self._tracker = None

        if wants_levels and not self._stop:
            raw = None
            try:
                raw = self._cam.capture_raw_average(
                    self._avg_frames, timeout_s=self._avg_timeout_s())
            except Exception:
                raw = None
            if raw is not None:
                self._levels = self._freeze_levels(raw)

    # ── v7.13: per-tile focus (sweep on the lattice, predict elsewhere) ──

    def _set_tile_focus(self, idx: int, tx: float, ty: float):
        af = self._af
        if af is None:
            return
        if self._tracker is not None:
            pred = self._tracker.predict(tx, ty)
            if self._tracker.should_af(idx, tx, ty):
                center = pred if pred is not None else af.focus_now_um()
                if center is None:
                    return
                peak, why = self._af.micro_sweep(
                    float(center), should_stop=lambda: self._stop)
                if peak is not None:
                    accepted = self._tracker.add(FocusSampleXY(
                        x_um=float(tx), y_um=float(ty),
                        focus_um=float(peak.z_um),
                        prominence=float(peak.prominence),
                        sigma_um=float(peak.sigma_z_um), tile_index=int(idx)))
                    target = (peak.z_um if accepted
                              else (self._tracker.predict(tx, ty) or peak.z_um))
                    af.goto_focus(float(target))
                else:
                    # Refused (empty glass, low prominence, …): the tile still
                    # gets the predicted focus; it never drags the fit.
                    self._tracker.note_refused()
                    if pred is not None:
                        af.goto_focus(float(pred))
            elif pred is not None:
                af.goto_focus(float(pred))
        elif self._predictor is not None:
            z = self._predictor.predict(tx, ty)
            if z is not None:
                af.goto_focus(float(z))

    def _restore_focus(self, entry_focus):
        if entry_focus is None or self._af is None:
            return
        try:
            back = self._af.goto_focus(float(entry_focus), lead_in=False)
            if abs(back - float(entry_focus)) > 5.0:
                logger.warning(
                    f"Fluor mosaic: focus restored to {back:.0f} µm, not the "
                    f"entry {float(entry_focus):.0f} µm — check the body")
        except Exception:
            logger.warning("Fluor mosaic: focus did not restore to its entry "
                           "value — check the body")

    # ── v7.19: tile-major (every colour per tile) ─────────────────

    def _optics(self):
        """An OpticsService built ON THIS THREAD, or None.

        Constructed with the SAME owner string this worker already holds the
        lease under: ``try_acquire`` is re-entrant per thread ident, and
        ``OpticsService._ensure`` releases only a lease it actually took, so
        the nesting is by design rather than by luck.
        """
        if self._optics_service is not None:
            return self._optics_service
        try:
            from gui.widgets.optics_ensure import build_service
            self._optics_service = build_service(self._LEASE)
        except Exception:
            self._optics_service = None
        return self._optics_service

    def _select_channel(self, plan) -> bool:
        """Put ``plan``'s cube in the path and apply its recipe. False = stop.

        A refusal ABORTS the run rather than capturing the tile through
        whatever cube happens to be fitted — a channel silently captured
        through the wrong cube is a result nothing downstream can detect.
        """
        svc = self._optics()
        if svc is None:
            self.failed.emit(
                "the microscope became unreachable — tile-major capture needs "
                "it to switch cubes at every tile")
            return False
        try:
            res = svc.ensure_filter(str(plan.channel))
        except Exception as exc:
            self.failed.emit(f"cube switch for {plan.channel} failed: {exc}")
            return False
        if not getattr(res, "ok", False):
            why = getattr(res, "why_not", "") or "the cube could not be set"
            self.failed.emit(f"{plan.channel}: {why}")
            return False
        # The recipe the operator dialled in against the histogram.
        if plan.exposure_us:
            try:
                self._cam.set_hw_exposure_us(int(round(plan.exposure_us)))
            except Exception:
                pass
        if plan.gain_pct is not None:
            try:
                self._cam.set_hw_exposure_gain(float(plan.gain_pct))
            except Exception:
                pass
        # These two drive _capture_tile, which is shared with the legacy path.
        self._levels = plan.levels
        self._avg_frames = int(plan.avg_frames or 1)
        return True

    def _probe_tile_major(self) -> bool:
        """One visit to the well centre: focus, then freeze levels PER CHANNEL.

        The probe deliberately visits the CENTRE — tile 0 of a bounding-square
        raster is empty glass on a circular well, so freezing display levels
        there would calibrate every channel on background.
        """
        if not self._probe_xy:
            return True
        try:
            self._do_probe()
        except AfCancelled:
            return False
        except AfAbort as e:
            self.failed.emit(str(e))
            return False
        for plan in self._channels:
            if self._stop:
                return False
            if not self._select_channel(plan):
                return False
            if not self._wait_settled():
                continue
            raw = None
            try:
                raw = self._cam.capture_raw_average(
                    1, timeout_s=self._avg_timeout_s())
            except Exception:
                raw = None
            if raw is not None:
                plan.levels = self._freeze_levels(raw)
        return True

    def _scan_tile_major(self):
        """Raster once, capturing every selected channel at each tile."""
        if not self._probe_tile_major():
            return
        if self._stop:
            return
        total = len(self._positions)
        consecutive_none = 0
        for idx, (tx, ty) in enumerate(self._positions):
            if self._stop:
                return
            try:
                self._move_to(tx, ty, first=(idx == 0 and not self._probed))
            except Exception as e:
                logger.warning(
                    f"Fluor mosaic: move to ({tx:.0f},{ty:.0f}) failed: {e}")
            if self._stop:
                return
            # ONE focus solve for the tile; every channel is captured at it.
            # ⚠ Per-channel chromatic (parfocal) offsets are NOT applied — the
            # documented v7.13 deferral. Within depth of field at 4×/10×.
            try:
                self._set_tile_focus(idx, tx, ty)
            except AfCancelled:
                return
            except AfAbort as e:
                self.failed.emit(str(e))
                return
            try:
                xy = self._controller.get_xy_position(cached=False)
                sx = xy[0] if xy and xy[0] is not None else tx
                sy = xy[1] if xy and xy[1] is not None else ty
            except Exception:
                sx, sy = tx, ty
            got_any = False
            for plan in self._channels:
                if self._stop:
                    return
                if not self._select_channel(plan):
                    return
                frame = self._capture_tile()
                if frame is None:
                    continue
                got_any = True
                plan.builder.add_raster_frame(frame, sx, sy, index=idx)
                plan.builder.stitch_incremental()
                plan.frames_used = plan.builder.frame_count
            if not got_any:
                consecutive_none += 1
                if consecutive_none >= self._MAX_CONSEC_NONE:
                    self.failed.emit(
                        "camera stopped delivering frames — scan aborted")
                    return
                self.progress.emit(idx + 1, total)
                continue
            consecutive_none = 0
            ref = self._channels[0]
            comp = ref.builder.composite
            self.tile.emit(comp.copy() if comp is not None else None,
                           ref.builder.canvas_extent_um)
            self.progress.emit(idx + 1, total)

        if self._stop:
            return
        self._finish_tile_major()

    def _finish_tile_major(self):
        """Solve the registration ONCE and replay it to every other channel.

        Solving per channel would give each its own positions and its own
        global shift, so the channels would overlay each other no better than
        if they had been captured minutes apart — which is precisely what
        tile-major exists to avoid.
        """
        ref = self._channels[0]
        try:
            if getattr(ref.builder, "has_reorient_tiles", lambda: False)():
                ref.builder.optimize_registration(
                    method=self._registration_method)
        except Exception as e:
            logger.debug(f"Fluor mosaic optimize_registration skipped: {e}")
        try:
            ref.builder.finalize_global_shift()
        except Exception as e:
            logger.debug(f"Fluor mosaic global shift skipped: {e}")
        for plan in self._channels[1:]:
            try:
                plan.builder.apply_registration_from(ref.builder)
            except Exception as e:
                logger.warning("Sharing registration to %s failed: %s",
                               plan.channel, e)
        for plan in self._channels:
            composite = plan.builder.composite
            extent = plan.builder.canvas_extent_um
            scale = float(getattr(plan.builder, "_mosaic_scale", 0.0) or 0.0)
            shift = getattr(plan.builder, "_global_shift_um", (0.0, 0.0))
            self._levels = plan.levels
            self._avg_used = int(plan.avg_frames or 1)
            self._exposure_us = float(plan.exposure_us or 0.0)
            self.channel_done.emit(
                plan.channel,
                composite.copy() if composite is not None else None,
                extent, scale, plan.builder.frame_count, shift,
                self._build_meta())
            # Free the float64 accumulators as soon as a channel is handed
            # over: tile-major holds N composites at once, and at the default
            # 2500 px canvas that is ~200 MB per channel.
            try:
                plan.builder.free_accumulators()
            except Exception:
                pass
        self.finished_all.emit()

    def _build_meta(self) -> dict:
        meta = {
            "display_levels": self._levels,
            "avg_frames": self._avg_used,
            "exposure_us": self._exposure_us,
            "levels_degenerate": self._levels_degenerate,
            "af_note": self._af_note,
        }
        if self._tracker is not None:
            meta["af"] = self._tracker.summary()
            meta["focus_map"] = self._tracker.sample_dicts()
        return meta

    def run(self):
        try:
            self._controller.suspend_position_poller()
        except Exception:
            pass
        lease_held = False
        entry_focus = None
        try:
            # v7.13 — the microscope lease, acquired ON THIS WORKER THREAD
            # (the controller's lease is thread-affine; acquiring it on the
            # GUI thread and releasing here would leak it — the B4 lesson).
            if self._af is not None:
                scope = self._scope
                acquired = False
                if scope is not None:
                    try:
                        acquired = scope.try_acquire(self._LEASE, timeout=2.0)
                    except Exception:
                        acquired = False
                if not acquired:
                    who = ""
                    try:
                        who = scope.lease_owner() if scope is not None else ""
                    except Exception:
                        who = ""
                    self.failed.emit(
                        "another part of the app is driving the microscope"
                        + (f" ({who})" if who else "")
                        + " — close it and re-run, or disable autofocus")
                    return
                lease_held = True
                entry_focus = self._af.focus_now_um()

            # v7.19 — tile-major: every colour captured at each tile, sharing
            # one focus and one registration. The legacy channel-major path
            # below is untouched and still runs whenever `channels` is unset.
            if self._channels:
                self._scan_tile_major()
                return

            try:
                self._do_probe()
            except AfCancelled:
                return
            except AfAbort as e:
                self.failed.emit(str(e))
                return
            if self._stop:
                return

            total = len(self._positions)
            consecutive_none = 0
            for idx, (tx, ty) in enumerate(self._positions):
                if self._stop:
                    return
                try:
                    # The probe visit already did the retract-gated first
                    # travel; without a probe, tile 0 keeps it.
                    self._move_to(tx, ty, first=(idx == 0 and not self._probed))
                except Exception as e:
                    logger.warning(
                        f"Fluor mosaic: move to ({tx:.0f},{ty:.0f}) failed: {e}")
                if self._stop:
                    return
                try:
                    self._set_tile_focus(idx, tx, ty)
                except AfCancelled:
                    return
                except AfAbort as e:
                    # A wedged / claimed body mid-scan: stop loudly rather
                    # than limp on capturing out-of-focus tiles.
                    self.failed.emit(str(e))
                    return
                frame = self._capture_tile()
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
                extent, scale, frames, shift, self._build_meta())
        except Exception as e:
            logger.exception("Fluor mosaic worker crashed")
            self.failed.emit(str(e))
        finally:
            # Restore the entry focus + release the lease on EVERY exit —
            # success, failure, abort. A leaked lease locks every microscope
            # surface in the app until restart.
            self._restore_focus(entry_focus)
            if lease_held and self._scope is not None:
                try:
                    self._scope.release(self._LEASE)
                except Exception:
                    pass
            try:
                self._controller.resume_position_poller()
            except Exception:
                pass


# v7.19: ``_ChannelPill`` moved to gui/widgets/fluorescence_controls_panel.py
# with the rest of the cube row. Re-exported (not re-declared) under its old
# name so existing references keep resolving to the SAME class — two pill
# classes would be a second, silently diverging definition of "a filter cube
# in the UI".


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
    #: The operator panned while following, so the view stopped following.
    follow_broken = Signal()

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
        # v7.9: opt-in follow mode (default OFF, so every existing consumer is
        # unchanged). While following, the host keeps calling `follow_point` and
        # the view holds that scene point at its centre — the map-app model.
        self._following = False
        self._follow_point: QPointF | None = None
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

    # ── follow mode (v7.9) ────────────────────────────────────────

    def set_follow(self, enabled: bool) -> None:
        """Hold ``follow_point`` at the view centre until the operator pans.

        ⚠ While following, zoom anchors on the VIEW CENTRE rather than the
        cursor. Cursor-anchored zoom is the normal behaviour, but it would drag
        the followed point off-centre — the opposite of "zoom out and still see
        the edges around where I am".
        """
        enabled = bool(enabled)
        if enabled == self._following:
            return
        self._following = enabled
        self.setTransformationAnchor(
            QGraphicsView.AnchorViewCenter if enabled
            else QGraphicsView.AnchorUnderMouse)
        if enabled and self._follow_point is not None:
            self.centerOn(self._follow_point)

    def is_following(self) -> bool:
        return self._following

    def follow_point(self, point: QPointF | None) -> None:
        """Where the followed thing is now, in scene (mosaic pixel) coords."""
        self._follow_point = point
        if self._following and point is not None:
            self.centerOn(point)

    def _break_follow(self) -> None:
        """A user pan stops the follow. Detected from the DRAG, deliberately.

        Not from ``scrollContentsBy``/scrollbar changes: ``centerOn`` fires those
        too, so the view would be fighting its own follow updates.
        """
        if not self._following:
            return
        self._following = False
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.follow_broken.emit()

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
        # Zooming must NOT break the follow — zooming out to see the mosaic
        # around the current position is the whole point of following.
        if self._following and self._follow_point is not None:
            self.centerOn(self._follow_point)

    # ── mouse (interactive-items mode only) ───────────────────────

    def mousePressEvent(self, event):
        if not self._interactive_items:
            super().mousePressEvent(event)
            return
        if event.button() == Qt.MouseButton.MiddleButton:
            self._panning = True
            self._pan_origin = event.position().toPoint()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            self._break_follow()
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
        # A hand-drag (ScrollHandDrag, left button) is handled by the base class,
        # so this is the only place to notice it. Dragging IS the unlock gesture.
        if (self._following and not self._interactive_items
                and event.buttons() != Qt.MouseButton.NoButton):
            self._break_follow()
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


class _ChannelPromptDialog(QDialog):
    """v7.13 — the per-channel 'set the filter' prompt, now with exposure.

    Replaces the plain QMessageBox so each channel can carry its own exposure:
    the spinbox pre-fills from the remembered per-channel value and applies
    LIVE (debounced ~300 ms) while the dialog is open, so the operator sees
    the running feed respond while choosing. 0 displays as "camera default"
    and means "don't touch the camera".

    v7.13.x — an "Auto" button runs the one-shot signal optimizer (raw-stats
    backends only): it iterates the exposure until the raw histogram's P99.9
    sits at ~70 % of full scale and writes the result into the spin. Exposure
    only — the mosaic scan freezes its own capture levels from the probe, so
    the live display scaling is deliberately left alone here.
    """

    # Optimizer worker → GUI thread (exposure_us | None, note).
    _auto_done = Signal(object, str)

    def __init__(self, channel: str, label_text: str, exposure_ms: float = 0.0,
                 on_apply_exposure=None, camera_manager=None, cam_idx=None,
                 parent=None):
        super().__init__(parent)
        self.setWindowTitle("Set filter")
        self.setModal(True)
        self._on_apply = on_apply_exposure
        self._mgr = camera_manager
        self._cam_idx = cam_idx
        self._optimizing = False
        self._auto_done.connect(self._on_auto_done)
        lay = QVBoxLayout(self)
        lay.setSpacing(s(10))
        text = QLabel(label_text)
        text.setWordWrap(True)
        lay.addWidget(text)

        row = QHBoxLayout()
        row.addWidget(QLabel(f"{channel} exposure:"))
        self._spin = QDoubleSpinBox()
        self._spin.setDecimals(2)
        self._spin.setRange(0.0, 10000.0)
        self._spin.setSuffix(" ms")
        self._spin.setSpecialValueText("camera default")
        self._spin.setKeyboardTracking(False)
        self._spin.setValue(max(0.0, float(exposure_ms or 0.0)))
        self._spin.setToolTip(
            "Exposure applied for THIS channel's scan (remembered per "
            "channel). Applies live while this dialog is open so the feed "
            "shows the result. 0 = leave the camera as it is.")
        row.addWidget(self._spin)
        self._auto_btn = QPushButton("Auto")
        self._auto_btn.setToolTip(
            "One-shot: find the exposure that puts the raw histogram's P99.9 "
            "at ~70% of full scale with no clipping, for THIS channel's "
            "filter/illumination as currently set.")
        self._auto_btn.clicked.connect(self._on_auto_clicked)
        self._auto_btn.setVisible(self._supports_auto())
        row.addWidget(self._auto_btn)
        row.addStretch(1)
        lay.addLayout(row)
        self._auto_note = QLabel("")
        self._auto_note.setWordWrap(True)
        lay.addWidget(self._auto_note)

        self._debounce = QTimer(self)
        self._debounce.setSingleShot(True)
        self._debounce.setInterval(300)
        self._debounce.timeout.connect(self._apply_now)
        self._spin.valueChanged.connect(
            lambda _v: self._debounce.start())

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=self)
        buttons.button(QDialogButtonBox.Ok).setText("Start scan")
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        lay.addWidget(buttons)

    def _apply_now(self):
        ms = self.exposure_ms()
        if self._on_apply is not None and ms > 0:
            try:
                self._on_apply(ms)
            except Exception:
                logger.debug("channel prompt: live exposure apply failed")

    def exposure_ms(self) -> float:
        return float(self._spin.value())

    # ── Auto (one-shot signal optimizer, v7.13.x) ─────────────────

    def _supports_auto(self) -> bool:
        """Only offered when the camera retains raw statistics (the same
        capability gate as the settings dialog's Signal section)."""
        if self._mgr is None or self._cam_idx is None:
            return False
        try:
            caps = self._mgr.hardware_capabilities(self._cam_idx)
            return "andor_raw_stats" in (caps.get("controls") or {})
        except Exception:
            return False

    def _on_auto_clicked(self):
        if self._optimizing or self._mgr is None:
            return
        self._optimizing = True
        self._auto_btn.setEnabled(False)
        self._auto_note.setText("optimizing exposure…")
        import threading
        threading.Thread(target=self._auto_worker, daemon=True,
                         name="ChannelAutoExpose").start()

    def _auto_worker(self):
        """Daemon worker — run_signal_optimize blocks on fresh raw frames."""
        try:
            from gui.widgets.mono_display import run_signal_optimize
            exp_us, note = run_signal_optimize(
                self._mgr, int(self._cam_idx), freeze_display=False)
        except Exception as exc:
            exp_us, note = None, f"auto-expose failed: {exc}"
        self._auto_done.emit(exp_us, note)

    def _on_auto_done(self, exp_us, note):
        self._optimizing = False
        self._auto_btn.setEnabled(True)
        self._auto_note.setText(str(note))
        if exp_us is not None and float(exp_us) > 0:
            # Writing the spin fires the debounce → _apply_now, which is
            # idempotent (the optimizer already left the camera there).
            self._spin.setValue(float(exp_us) / 1000.0)


class _FocusSurveyDialog(QDialog):
    """v7.13 — inspect a well's mosaic focus survey: the CRITICAL SAMPLE
    SURFACE (where the cells are), NOT the plate bottom.

    Reports the within-well tilt, the flatness residual (a domed hydrogel
    shows as residual — never silently flattened), the surface's height above
    the taught plate bottom (convertible only through the verified
    focus↔needle datum), and a diagnostic-only comparison against the v7.11
    plate plane. The operator picks the evaluation model (plane / linear /
    spline); the choice persists with the survey. There are deliberately NO
    plate-bottom or plate-plane install actions here — the plate datum is
    owned by the v7.11 leveling wizard.
    """

    def __init__(self, page, plate_key: str, well: str, parent=None):
        super().__init__(parent)
        self._page = page
        self._plate_key = plate_key
        self._well = well
        self.setWindowTitle(f"Sample surface — {well}")
        self.setModal(True)
        self.setMinimumSize(s(520), s(420))
        lay = QVBoxLayout(self)
        lay.setSpacing(s(8))

        row = QHBoxLayout()
        row.addWidget(QLabel("Surface model:"))
        self._model_combo = QComboBox()
        try:
            from SupportClasses.SampleSurface import SURFACE_MODELS
            for m in SURFACE_MODELS:
                self._model_combo.addItem(m)
        except Exception:
            self._model_combo.addItem("plane")
        row.addWidget(self._model_combo)
        row.addStretch(1)
        lay.addLayout(row)

        self._text = QPlainTextEdit()
        self._text.setReadOnly(True)
        self._text.setStyleSheet("font-family: Consolas, monospace;")
        lay.addWidget(self._text, stretch=1)

        buttons = QDialogButtonBox(QDialogButtonBox.Close, parent=self)
        buttons.rejected.connect(self.reject)
        buttons.accepted.connect(self.accept)
        buttons.clicked.connect(lambda _b: self.accept())
        lay.addWidget(buttons)

        survey = None
        try:
            survey = fms.get_store().get_focus_survey(plate_key, well)
        except Exception:
            survey = None
        self._survey = survey
        if survey:
            model = str(survey.get("model") or "plane")
            i = self._model_combo.findText(model)
            if i >= 0:
                self._model_combo.setCurrentIndex(i)
        self._model_combo.currentTextChanged.connect(self._on_model_changed)
        self._render()

    def _on_model_changed(self, model: str):
        try:
            fms.get_store().set_surface_model(self._plate_key, self._well,
                                              model)
        except Exception:
            pass
        self._render()

    # ── geometry helpers ──────────────────────────────────────────

    def _well_geometry(self):
        summary = (self._survey or {}).get("summary") or {}
        c = summary.get("well_center_um")
        r = summary.get("well_radius_um")
        if not c or not r:
            try:
                c = self._page._well_center_um(self._well)
                r = self._page._well_diameter_mm(self._well) / 2.0 * 1000.0
            except Exception:
                return None, None
        return (float(c[0]), float(c[1])), float(r)

    def _build_model(self):
        from SupportClasses.SampleSurface import SampleSurfaceModel
        center, radius = self._well_geometry()
        if center is None:
            return None
        return SampleSurfaceModel(
            (self._survey or {}).get("samples") or (),
            well_center_um=center, well_radius_um=radius,
            model=self._model_combo.currentText())

    def _render(self):
        L: list[str] = []
        survey = self._survey
        if not survey or not survey.get("samples"):
            self._text.setPlainText(
                "No focus survey stored for this well — run a scan with "
                "autofocus enabled first.")
            return
        summary = survey.get("summary") or {}
        L.append(f"Focus survey for {self._well}  ({survey.get('date', '')})")
        L.append(f"  samples accepted {summary.get('n_accepted', '?')}"
                 f" · sweeps refused {summary.get('n_refused', 0)}"
                 f" (empty tiles) · outliers {summary.get('n_outlier', 0)}")
        L.append("")
        try:
            model = self._build_model()
        except Exception as exc:
            self._text.setPlainText("\n".join(L) + f"\nSurface unusable: {exc}")
            return
        if model is None:
            self._text.setPlainText("\n".join(L) + "\nWell geometry unknown.")
            return
        sx, sy = model.tilt_mm_per_mm()
        L.append(f"Model: {model.model()}   (choice persists with the survey)")
        L.append(f"  Within-well tilt: {sx * 1000:+.1f} / {sy * 1000:+.1f} "
                 f"µm per mm → {model.span_across_well_um():.0f} µm across "
                 f"the well")
        L.append(f"  Flatness (plane residual RMS): "
                 f"{model.rms_plane_residual_um():.1f} µm — a domed hydrogel "
                 f"surface shows up HERE, it is not flattened away")
        L.append("")

        # Height above the taught plate bottom — only through the verified
        # focus↔needle datum. A surface below the plate bottom is a bad datum.
        center, _r = self._well_geometry()
        f_center, conf = model.evaluate(center[0], center[1])
        L.append(f"Surface focus at the well centre: {f_center:.1f} µm "
                 f"({conf} confidence)")
        L.extend(self._height_above_bottom_lines(f_center))
        L.append("")
        L.extend(self._plate_tilt_diagnostic_lines(sx, sy))
        L.append("")
        L.append("⚠ This survey spans ONE well. Its tilt extrapolated across "
                 "the plate amplifies any slope error ~8× — which is why "
                 "nothing here installs a plate-wide datum. The plate bottom "
                 "and plate tilt stay owned by the Plate Bed Level wizard.")
        L.append("This surface is available to Cell Targeting as the removal-"
                 "Z reference (Setup → 'Removal Z from measured sample "
                 "surface').")
        self._text.setPlainText("\n".join(L))

    def _height_above_bottom_lines(self, f_center_um: float) -> list:
        ctrl = getattr(self._page, "_controller", None)
        try:
            from SupportClasses.PlateFocusDatumStore import get_store as datum_store
            cam = self._page._camera_key() or ""
            z_zref = datum_store().needle_z_zref_mm(
                cam, "", self._plate_key, float(f_center_um))
            bottom = ctrl.get_plate_bottom_z() if ctrl is not None else None
            if z_zref is None or bottom is None:
                raise ValueError("no datum or plate bottom")
            height_mm = None
            if hasattr(ctrl, "zref_to_print_height"):
                height_mm = ctrl.zref_to_print_height(float(bottom),
                                                      float(z_zref))
            if height_mm is None:
                raise ValueError("height not convertible")
            lines = [f"  → needle frame: {z_zref:.3f} mm zref = "
                     f"{height_mm * 1000:.0f} µm ABOVE the taught plate bottom"]
            if height_mm < 0:
                lines.append(
                    "  ⚠ the surface converts to BELOW the plate bottom — "
                    "the focus↔needle datum or the taught bottom is wrong; "
                    "do not use this surface for needle Z until re-taught.")
            return lines
        except Exception:
            return ["  (not convertible to needle Z — no verified "
                    "focus↔needle datum for this camera/plate; run the plate "
                    "touch-off with the focus confirmation, or the optical "
                    "plate-bottom calibration)"]

    def _plate_tilt_diagnostic_lines(self, sx: float, sy: float) -> list:
        ctrl = getattr(self._page, "_controller", None)
        try:
            plane = ctrl.get_plate_z_plane() if ctrl is not None else None
            if plane is None:
                raise ValueError("no plate plane")
            px = float(getattr(plane, "sx_mm_per_mm", 0.0))
            py = float(getattr(plane, "sy_mm_per_mm", 0.0))
            return [
                "Diagnostic — whole-plate tilt comparison (report only):",
                f"  this well's surface tilt {sx * 1000:+.1f}/"
                f"{sy * 1000:+.1f} µm/mm vs installed plate plane "
                f"{px * 1000:+.1f}/{py * 1000:+.1f} µm/mm",
            ]
        except Exception:
            return ["Diagnostic: no installed plate Z plane to compare "
                    "against (Plate Bed Level wizard)."]


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
                 parent: QWidget | None = None, *, embedded: bool = False,
                 owns_camera: bool = True):
        """``owns_camera=False``: a HOST page manages the microscope camera.

        Ownership has to be DECLARED, not inferred from who reached the camera
        first. Qt delivers ``showEvent`` to a **child before its parent**
        (verified), so an embedded instance sitting on the host's visible tab
        always won the race — claimed the camera, and then stopped it from its
        own ``hideEvent`` even though the host was still using it. A host that
        wants to keep the feed alive across its own tabs, or across a scan,
        passes False and this page stops touching start/stop entirely.
        """
        super().__init__(parent)
        self._embedded = bool(embedded)
        self._owns_camera = bool(owns_camera)
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

        # v7.19 — camera state this page forces while it is open, and the
        # snapshot it restores on the way out. `_entry_hw` subsumes the older
        # narrower `_entry_exposure_us`: restoring only the exposure left the
        # operator's auto-exposure/auto-levels/gamma changed behind them.
        self._entry_hw: dict | None = None
        self._preset_applied = False
        self._scan_order = "tile"
        # v7.19.3 — debounced persistence for the panel's values. 500 ms: long
        # enough that ⚡Auto's burst of commits is one write, short enough that
        # a kill -9 loses at most the last gesture.
        self._save_timer = QTimer(self)
        self._save_timer.setSingleShot(True)
        self._save_timer.setInterval(500)
        self._save_timer.timeout.connect(self._flush_recipe_save)
        # Which cube the OPERATOR says is fitted, used only when there is no
        # motorised cassette to read. With a body, the body wins.
        self._manual_active: str | None = None

        # Settings popout (scan knobs)
        self._settings_dialog = WorkflowSettingsDialog(
            "fluorescence_mosaic", "Fluorescence Mosaic",
            parent=self, on_change=self._on_settings_changed)
        self._build_settings_dialog(self._settings_dialog)

        # v7.19 — the controls panel. Built BEFORE the layout because it owns
        # the cube pills and the objective combo, which it publishes back onto
        # this page under the names every existing method already uses.
        self._panel = FluorescenceControlsPanel(self)
        # 🐞 v7.19.3 — the two panel controls that were NOT in the saved
        # profile. Every other value on the panel round-trips because its widget
        # is a registered field; the scan order and the camera-preset toggle were
        # plain attributes with hardcoded defaults, so they silently reset to
        # "tile" / "fluorescence" on every launch. ``register_external`` is the
        # v7.7 hook for exactly this: a widget has one parent, so a control
        # promoted onto the panel cannot also live in a settings section.
        #
        # Registered HERE, not in _build_settings_dialog: that runs before the
        # panel exists, and must stay before it (the panel's constructor calls
        # back into the page). Both are before load_last(), which is what
        # restores them.
        self._settings_dialog.register_external(
            "scan_order", self._panel.order_combo(), "tile")
        self._settings_dialog.register_external(
            "camera_preset_fluor", self._panel.preset_button(), True)
        self._channel_checks = self._panel.pills()
        self._objective_combo = self._panel.objective_combo()
        for ch in fms.CHANNELS:
            self._apply_pill(ch)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(10), s(12), s(12))
        outer.setSpacing(s(10))
        if not self._embedded:
            outer.addLayout(self._build_header())
        # Top row: the selected well (+ the controls panel inline when embedded,
        # where there is no left context box of our own to mount it in).
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
        # v7.21: a section moved out of ⚙ Settings lands in the drawer, which is
        # hidden (zero footprint) until something is in it. AFTER the run row on
        # purpose, so Start / Abort never move.
        self._promoted_panel = PromotedSectionsPanel()
        outer.addWidget(self._promoted_panel)
        self._layout_store = wire_section_promotion(
            self, self._settings_dialog, self._promoted_panel.stack,
            settings=self._settings, workflow_id="fluorescence_mosaic")

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
        """Top row: the selected well, plus the controls panel when embedded.

        v7.19: objective, cubes, exposure and the rest moved to the left context
        panel (:class:`FluorescenceControlsPanel`). An EMBEDDED instance has no
        left box of its own — the host page owns it with its own jog panel — so
        the same panel object is mounted here instead. One class, two parents;
        two separately built surfaces onto one set of settings is the
        divergence this codebase has paid for before.
        """
        frame = QFrame(self)
        row = QHBoxLayout(frame)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(8))

        if self._embedded:
            row.addWidget(self._panel, stretch=1)
            row.addSpacing(s(10))

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

    # ── Left context panel (v7.19) ────────────────────────────────

    def get_context_widget(self):
        """The workflow's own left context panel — or None when embedded.

        An embedded instance must return None: its HOST owns the left box (with
        its own jog panel), and handing this panel over would either steal that
        box or mount the same widget in two places at once. The embedded case
        shows the panel inline in the top row instead.
        """
        if self._embedded:
            return None
        return self._panel

    def context_label(self) -> str:
        """Name for the left box's native pill — these are not jog controls."""
        return "Signal"

    # -- the panel's page API (see FluorescenceControlsPanel._PAGE_API) --

    def channels(self) -> tuple:
        return tuple(fms.CHANNELS)

    def camera_manager_for_panel(self):
        return self._camera_manager

    def microscope_cam_idx(self) -> int:
        return self._resolve_microscope_cam_idx()

    def on_panel_pill_toggled(self, channel: str):
        self._on_pill_toggled(channel)

    def on_panel_pick_colour(self, channel: str):
        self._pick_color(channel)

    def on_panel_objective_changed(self, name: str):
        self._on_objective_changed(name)

    def on_panel_scan_order_changed(self, order: str):
        self._scan_order = str(order or "tile")
        self._schedule_recipe_save()

    def scan_order(self) -> str:
        """Read the COMBO, not a mirror of it.

        v7.19.3: ``_scan_order`` was a page attribute updated by the panel's
        change handler. Once the combo is a restored settings field there are
        two ways it can move — an operator pick and ``apply()`` — and only one
        of them was guaranteed to update the mirror. The widget is the value.
        """
        panel = getattr(self, "_panel", None)
        if panel is not None:
            try:
                return panel.scan_order()
            except Exception:
                pass
        return getattr(self, "_scan_order", "tile")

    # -- the per-channel signal recipe -------------------------------

    #: recipe key -> (spin dict attribute, unit conversion spin→recipe).
    #: Exposure is held in µs everywhere in the camera stack but typed in ms.
    _RECIPE_SPINS = {
        "exposure_us": ("_channel_exposure", 1000.0),
        "gain_pct": ("_channel_gain", 1.0),
        "avg_frames": ("_channel_avg", 1.0),
        "display_lo": ("_channel_lo", 1.0),
        "display_hi": ("_channel_hi", 1.0),
    }

    def _recipe_spin(self, channel: str, key: str):
        attr, _scale = self._RECIPE_SPINS.get(key, (None, 1.0))
        if attr is None:
            return None
        return getattr(self, attr, {}).get(channel)

    def channel_recipe(self, channel: str) -> dict:
        """This channel's signal recipe, in the camera stack's own units.

        A zero means "not set": the exposure/gain are left as the camera has
        them, averaging falls back to the shared scan setting, and the display
        levels are measured at the probe. That is the same ``0 = default``
        convention the exposure spin has used since v7.13, extended to the
        controls added beside it.
        """
        out: dict = {}
        for key, (attr, scale) in self._RECIPE_SPINS.items():
            spin = getattr(self, attr, {}).get(channel)
            if spin is None:
                continue
            try:
                val = float(spin.value()) * scale
            except Exception:
                continue
            if val > 0:
                out[key] = val
        return out

    def _schedule_recipe_save(self):
        """Persist the panel's values shortly after the last edit.

        🐞 v7.19.3, operator: *"the values assigned here should be persistant on
        this page"*. They were being WRITTEN into the right widgets — every
        recipe spin is a registered settings field — but nothing ever wrote the
        file. ``WorkflowSettingsDialog.save_last`` is reached only from that
        dialog's own ``hideEvent``/``closeEvent``, and the page hides the popout
        only ``if self._settings_dialog.isVisible()``. So the save ran only for
        an operator who OPENED the ⚙ popout and closed it again — and the whole
        point of the v7.19 panel is that they no longer need to. Every slider
        change was lost on restart.

        Debounced rather than written per commit: the slider already debounces
        at 300 ms, but ⚡Auto and a channel switch can commit several values in a
        burst, and the v7.16 camera-crop incident is the recorded cost of a JSON
        write per widget step.
        """
        try:
            self._save_timer.start()
        except Exception:
            pass

    def _flush_recipe_save(self):
        """Write now. Called on the debounce and on the way off the page."""
        try:
            self._save_timer.stop()
        except Exception:
            pass
        try:
            self._settings_dialog.save_last()
        except Exception as exc:
            logger.debug("fluor: save_last failed: %s", exc)

    def set_channel_recipe(self, channel: str, key: str, value: float):
        """Record an ACHIEVED value into the persisted recipe.

        Called by the panel after the camera has adopted a slider change, so
        the stored number is what the camera ran, not what was asked for.
        Writing under blockSignals keeps the popout's own change handler from
        treating this as an operator edit and re-entering the apply path.
        """
        spin = self._recipe_spin(channel, key)
        if spin is None:
            return
        _attr, scale = self._RECIPE_SPINS[key]
        blocked = spin.blockSignals(True)
        try:
            scaled = float(value) / scale
            # QSpinBox is integral (averaging, raw display counts);
            # QDoubleSpinBox is not (exposure ms, gain %).
            spin.setValue(int(round(scaled)) if isinstance(spin, QSpinBox)
                          else scaled)
        except Exception as exc:
            logger.debug("fluor: recipe write %s/%s failed: %s",
                         channel, key, exc)
        finally:
            spin.blockSignals(blocked)
        # blockSignals above is what stops the popout treating this as an
        # operator edit — and it is also what stops `notify_on_field_change`
        # ever firing, so the save has to be asked for explicitly here.
        self._schedule_recipe_save()

    def apply_panel_control(self, key: str, value: float):
        """Push ONE signal control to the microscope camera; return what it took.

        Returning the ACHIEVED value is the contract the panel's readout relies
        on — a camera that clamps a request must not leave the UI showing the
        request (the v7.13 "exposure resets itself" report was exactly that).
        """
        mgr = self._camera_manager
        if mgr is None:
            return None
        cam = self._resolve_microscope_cam_idx()
        try:
            if key == "exposure_us":
                mgr.set_hw_exposure_us(cam, int(round(float(value))))
            elif key == "gain_pct":
                mgr.set_hw_exposure_gain(cam, float(value))
            elif key == "avg_frames":
                # Averaging is ours, not the camera's — nothing to push.
                return float(value)
            elif key == "display_lo":
                mgr.set_hw_andor_scale_lo(cam, int(round(float(value))))
            elif key == "display_hi":
                mgr.set_hw_andor_scale_hi(cam, int(round(float(value))))
            else:
                return None
        except Exception as exc:
            logger.debug("fluor panel: apply %s failed: %s", key, exc)
            return None
        return self._readback_control(key)

    def _readback_control(self, key: str):
        """Re-read one control FROM the camera; None when it cannot be read."""
        mgr = self._camera_manager
        if mgr is None or not hasattr(mgr, "get_hw_settings"):
            return None
        try:
            st = mgr.get_hw_settings(self._resolve_microscope_cam_idx()) or {}
        except Exception:
            return None
        got = st.get({"exposure_us": "exposure_us",
                      "gain_pct": "exposure_gain_pct",
                      "display_lo": "andor_scale_lo",
                      "display_hi": "andor_scale_hi"}.get(key, ""))
        try:
            return float(got) if got is not None else None
        except (TypeError, ValueError):
            return None

    def run_auto_exposure(self, channel: str):
        """⚡Auto — step the exposure until the signal sits just below clipping.

        Deliberately exposure-ONLY and deliberately the existing, bench-verified
        ``run_signal_optimize``: gain trades signal-to-noise and averaging
        trades time and light dose, so neither should be spent automatically on
        the operator's behalf. ``freeze_display=False`` because the scan freezes
        its own capture levels at the probe.
        """
        mgr = self._camera_manager
        if mgr is None:
            return None
        try:
            from gui.widgets.mono_display import run_signal_optimize
            res = run_signal_optimize(
                mgr, self._resolve_microscope_cam_idx(), freeze_display=False)
        except Exception as exc:
            logger.debug("fluor panel: auto exposure failed: %s", exc)
            return None
        got = None
        if isinstance(res, dict):
            got = res.get("exposure_us")
        return float(got) if got else self._readback_control("exposure_us")

    # -- the camera preset -------------------------------------------

    def _panel_caps(self) -> dict:
        mgr = self._camera_manager
        if mgr is None or not hasattr(mgr, "hardware_capabilities"):
            return {}
        try:
            return mgr.hardware_capabilities(
                self._resolve_microscope_cam_idx()) or {}
        except Exception:
            return {}

    def _capture_entry_hw(self):
        """Snapshot the camera as the operator left it, once per visit.

        Taken BEFORE the preset is applied and never overwritten while the page
        stays open, so toggling the preset back and forth cannot slowly turn the
        operator's own settings into the preset's.
        """
        if self._entry_hw is not None:
            return
        mgr = self._camera_manager
        if mgr is None or not hasattr(mgr, "get_hw_settings"):
            return
        try:
            self._entry_hw = hw_controls_snapshot(
                mgr.get_hw_settings(self._resolve_microscope_cam_idx()) or {})
        except Exception as exc:
            logger.debug("fluor: entry camera snapshot failed: %s", exc)
            self._entry_hw = None

    def _apply_camera_preset(self):
        """Force the camera into the state a quantitative mosaic needs."""
        mgr = self._camera_manager
        if mgr is None:
            return
        preset = fluorescence_preset(self._panel_caps())
        if not preset:
            return
        apply_hw_controls(mgr, self._resolve_microscope_cam_idx(), preset,
                          skip_resolution=True)
        self._preset_applied = True
        logger.info("Fluorescence camera preset applied: %s",
                    ", ".join(sorted(preset)))

    def _restore_entry_hw(self):
        """Put the camera back exactly as this page found it.

        Nothing here is written to CameraCalibrationStore: the preset is a mode
        this workflow runs IN, not a change to the camera's saved configuration.
        """
        mgr = self._camera_manager
        if mgr is None or not self._entry_hw:
            self._preset_applied = False
            return
        apply_hw_controls(mgr, self._resolve_microscope_cam_idx(),
                          self._entry_hw, skip_resolution=True)
        self._preset_applied = False

    def on_panel_camera_preset_changed(self, fluorescence: bool):
        if self.is_scanning():
            return
        if fluorescence:
            self._capture_entry_hw()
            self._apply_camera_preset()
        else:
            self._restore_entry_hw()
        try:
            self._panel.refresh_ranges()
        except Exception:
            pass
        self._schedule_recipe_save()

    # -- which optics are ACTUALLY in the light path -----------------

    def _scope_state(self):
        """The microscope's cached state, or None. Never raises, never blocks.

        Lazy import: this page must keep working on a rig with no microscope
        SDK installed at all.
        """
        try:
            from SupportClasses.MicroscopeControl import get_microscope
            return get_microscope().state()
        except Exception:
            return None

    def _cube_slots(self, state) -> tuple[dict, dict]:
        """``({channel: slot}, {channel: refusal})`` for every channel.

        Resolution goes through ``FluorescenceMosaicStore.channel_slot``, which
        matches exact → normalized → operator alias → the body's own name and
        REFUSES rather than guessing.

        ⚠ Not ``channel_number``: that is an acquisition ORDINAL, and this rig's
        slot 3 holds TxRed while the ordinal calls it mCherry. A TxRed image
        filed as mCherry is a result nothing downstream can detect.
        """
        slots: dict[str, int] = {}
        refusals: dict[str, str] = {}
        try:
            from SupportClasses.MicroscopeConfigStore import get_store as _cfg
            cfg = _cfg()
        except Exception:
            cfg = None
        for ch in fms.CHANNELS:
            try:
                match = fms.channel_slot(ch, scope_state=state, config_store=cfg)
            except Exception:
                match = None
            if match is not None and getattr(match, "ok", False):
                slots[ch] = int(getattr(match, "position", 0))
            else:
                refusals[ch] = (getattr(match, "why_not", "") if match else "")\
                    or f"no filter cube is configured for {ch}"
        return slots, refusals

    def panel_optics_state(self) -> dict:
        """What the panel renders: which cube and objective are really fitted.

        The active channel is a HARDWARE READ, not a flag this page maintains —
        the operator can turn the cassette by hand, and a panel that kept its
        own idea of "active" would enable the wrong channel's sliders and
        record an exposure against the wrong cube.

        With no microscope the operator's own last pick stands (their decision,
        per the "manual turret" case), and the note says so rather than
        pretending a position was read.
        """
        state = self._scope_state()
        slots, refusals = self._cube_slots(state)
        connected = bool(getattr(state, "connected", False))
        has_filter = bool(getattr(state, "has_filter", False))
        note = ""
        active = None
        if connected and has_filter:
            pos = getattr(state, "filter_position", None)
            for ch, slot in slots.items():
                if pos and slot == pos:
                    active = ch
                    break
            if active is None:
                note = (f"Cassette position {pos} is not bound to any channel — "
                        f"bind it on Hardware Setup → Microscope.")
        else:
            active = self._manual_active or (self._selected_channels() or [None])[0]
            # Without a body nothing can be driven, so every cube is "manual".
            refusals = {}
            note = ("No motorised cassette: set the cube by hand, then pick it "
                    "here so its controls apply to the right channel.")
        return {
            "active_channel": active,
            "objective": self._live_objective_name(),
            "cube_refusals": refusals,
            "note": note,
        }

    def _live_objective_name(self) -> str:
        """The objective the BODY has, falling back to the declared one.

        🐞 v7.19.2, operator: *"flourescence mosaic is not autodetecting the
        objective and the current filter. I changed both and it did not
        update."* The cube half was a real hardware read all along
        (``filter_position``); the objective half returned
        ``_current_objective_name()`` — i.e. ``camera_config
        .current_objective_name``, the app's OWN declared value. The panel was
        therefore comparing its combo against a copy of itself and could never
        detect a nosepiece anyone else had moved. It is not that the update was
        slow: nothing was being read.
        """
        try:
            state = self._scope_state()
            if getattr(state, "connected", False):
                from SupportClasses.OpticsRegistry import OBJECTIVE, optic_at
                optic = optic_at(
                    state, getattr(state, "objective_position", None), OBJECTIVE)
                name = str(getattr(optic, "label", "") or "")
                if name:
                    return name
        except Exception:
            pass
        return self._current_objective_name()

    def on_panel_objective_detected(self, name: str):
        """The body reports an objective we were not using. Follow it.

        Called from the panel's poll when the nosepiece has been turned by hand
        or by another surface. This ADOPTS — declares the name and pushes its
        calibrated µm/px — and deliberately does NOT drive: the turret is
        already where it is, and commanding it would be an unasked-for move.

        ⚠ This is the one poll-driven write of ``current_objective_name``, and
        it is legitimate for a reason worth stating: the alternative is to know
        the objective changed and go on scaling every tile by the old one's
        µm/px, which is the silent scale error this whole area exists to
        prevent. What v7.18 forbids is a background write of a DERIVED or
        guessed value; this is a verified read-back of what is physically
        fitted, and the hardware is the authority on that.
        """
        if not name or self.is_scanning():
            return
        if name == self._current_objective_name():
            return
        logger.info("Objective changed on the body to %s — adopting its µm/px.",
                    name)
        self._adopt_objective(name)

    def on_panel_activate_channel(self, channel: str):
        """Put ``channel``'s cube in the light path.

        Rotating the cassette needs no collision guard — it moves no objective
        and changes no height, which is why filter automation is the safe half
        of driving the optics.

        With no motorised cassette this records the operator's word instead:
        they told us what they fitted, and that is the only source there is.
        """
        if self.is_scanning():
            return
        # Held either way: with no body it IS the answer, and with a body it is
        # a harmless fallback if the read later fails.
        self._manual_active = channel
        try:
            from gui.widgets.optics_ensure import ensure_optics_async
            ensure_optics_async(kind="filter", name=str(channel),
                                on_done=self._on_cube_ensured)
        except Exception:
            pass
        try:
            self._panel.refresh_optics()
        except Exception:
            pass

    def _on_cube_ensured(self, result):
        """GUI thread: report the cube switch and re-read the light path."""
        if result is not None:
            if getattr(result, "ok", False):
                logger.info("Filter cube: %s", result.describe())
            else:
                from gui.widgets.optics_ensure import describe_refusal
                why = describe_refusal(result)
                logger.warning("Cube switch refused: %s", why)
                self._status.setText(f"Filter cube NOT changed — {why}")
        try:
            self._panel.refresh_optics()
        except Exception:
            pass

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
        # v7.13 — inspect the per-well sample surface measured by autofocus.
        self._survey_btn = QPushButton("Focus survey…")
        self._survey_btn.setEnabled(False)
        self._survey_btn.setToolTip(
            "Inspect the sample surface (tilt, flatness, height above the "
            "plate bottom) measured by per-tile autofocus for this well.")
        self._survey_btn.clicked.connect(self._open_focus_survey)
        row.addWidget(self._survey_btn)
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
            "Camera orientation, FOV, overlap, settle timing and per-tile "
            "frame averaging are inherited from the full-plate Mosaic scan "
            "settings (Calibration → Plate Location → Mosaic scan → Settings).")
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        sec.add_widget(note)

        # ── v7.13/v7.19: the per-channel SIGNAL RECIPE ────────────
        #
        # These spins are the persistence layer (WorkflowSettingsStore gives
        # profiles, import/export and last-used for free, so a "dim sample"
        # recipe set is saveable). The left-panel sliders are a SECOND VIEW of
        # exactly these widgets — see `_recipe_spin` / `set_channel_recipe`.
        # There is deliberately no third store.
        chan = dlg.add_section("Channels")
        chan.add_note(
            "The signal recipe for each filter cube, applied when that "
            "channel is captured and recorded with the saved mosaic. Editable "
            "here or, live against the histogram, on the Signal panel. "
            "0 = leave the camera as it is.")
        self._channel_exposure: dict[str, QDoubleSpinBox] = {}
        self._channel_gain: dict[str, QDoubleSpinBox] = {}
        self._channel_avg: dict[str, QSpinBox] = {}
        self._channel_lo: dict[str, QSpinBox] = {}
        self._channel_hi: dict[str, QSpinBox] = {}
        for ch in fms.CHANNELS:
            tok = fms._safe_token(ch).lower()
            spin = QDoubleSpinBox()
            spin.setDecimals(2)
            spin.setRange(0.0, 600000.0)
            spin.setSuffix(" ms")
            spin.setSpecialValueText("camera default")
            spin.setValue(0.0)
            chan.add(f"exposure_ms_{tok}", f"{ch} exposure", spin, 0.0)
            self._channel_exposure[ch] = spin

            gain = QDoubleSpinBox()
            gain.setDecimals(1)
            gain.setRange(0.0, 100.0)
            gain.setSuffix(" %")
            gain.setSpecialValueText("camera default")
            chan.add(f"gain_pct_{tok}", f"{ch} gain", gain, 0.0)
            self._channel_gain[ch] = gain

            avg = QSpinBox()
            avg.setRange(0, 64)
            avg.setSpecialValueText("scan default")
            avg.setToolTip(
                "Frames averaged per tile. Real √N signal-to-noise, at N× the "
                "time AND N× the light dose. 0 uses the shared scan setting.")
            chan.add(f"avg_frames_{tok}", f"{ch} averaging", avg, 0)
            self._channel_avg[ch] = avg

            lo = QSpinBox()
            lo.setRange(0, 65535)
            lo.setSpecialValueText("auto (measured at the probe)")
            lo.setToolTip(
                "Display black point in raw counts. Set explicitly and it is "
                "also used as the CAPTURE level, so the saved mosaic looks "
                "like the preview did; left at 0 the scan measures it at the "
                "well centre.")
            chan.add(f"disp_lo_{tok}", f"{ch} black", lo, 0)
            self._channel_lo[ch] = lo

            hi = QSpinBox()
            hi.setRange(0, 65535)
            hi.setSpecialValueText("auto (measured at the probe)")
            chan.add(f"disp_hi_{tok}", f"{ch} white", hi, 0)
            self._channel_hi[ch] = hi

        # ── v7.13: per-tile autofocus (the sample's critical surface) ──
        af = dlg.add_section("Autofocus (per tile)")
        af.add_note(
            "With the microscope connected, the FIRST scanned channel "
            "micro-sweeps the focus drive on a checkerboard of in-well tiles "
            "and every tile is captured at the running surface fit — the "
            "needle never moves. Later channels replay the measured map. The "
            "result is the per-well SAMPLE surface (where the cells are), "
            "inspectable via 'Focus survey…' after the scan.")
        self._af_enabled = QCheckBox("Autofocus during the scan")
        self._af_enabled.setChecked(True)
        af.add_check("af_enabled", self._af_enabled, True)
        self._af_lattice = QSpinBox()
        self._af_lattice.setRange(1, 10)
        self._af_lattice.setValue(2)
        self._af_lattice.setToolTip(
            "Checkerboard lattice spacing: sweep every Nth tile in BOTH grid "
            "axes (staggered). 2 ≈ a quarter of in-well tiles; 1 = literal "
            "per-tile sweeps (slowest, most photobleaching).")
        af.add("af_lattice", "Sweep every Nth tile", self._af_lattice, 2)
        self._af_step = self._dspin(0.0, 200.0, 0.0, " µm", 1, 0.5)
        self._af_step.setSpecialValueText("auto (1× DOF)")
        af.add("af_step_um", "Sweep Z step", self._af_step, 0.0)
        self._af_range = self._dspin(0.0, 2000.0, 0.0, " µm", 1, 5.0)
        self._af_range.setSpecialValueText("auto (±2× DOF)")
        af.add("af_range_um", "Sweep Z range (±)", self._af_range, 0.0)

        # ── v7.13: post-processing (non-destructive) ──────────────
        post = dlg.add_section("Post-processing")
        post.add_note(
            "Optional per-channel processing applied AT SAVE to a copy — the "
            "raw stitch is kept on disk and detection always reads it; only "
            "the display overlays prefer the processed version.")
        self._post_denoise = QComboBox()
        for m in ("off", "median", "gaussian"):
            self._post_denoise.addItem(m, m)
        post.add("post_denoise", "Denoise", self._post_denoise,
                 {"text": "off", "data": "off"})
        self._post_strength = QSpinBox()
        self._post_strength.setRange(1, 3)
        self._post_strength.setValue(1)
        post.add("post_denoise_strength", "Denoise strength",
                 self._post_strength, 1)
        self._post_bg = QCheckBox("Subtract background (rolling-ball)")
        self._post_bg.setChecked(False)
        post.add_check("post_bg_subtract", self._post_bg, False)
        self._post_bg_radius = self._dspin(20.0, 1000.0, 100.0, " µm", 0, 10.0)
        post.add("post_bg_radius_um", "Background radius",
                 self._post_bg_radius, 100.0)
        # v7.13 — retroactive apply: process mosaics that were ALREADY
        # captured (the automatic path runs at save time; this covers data
        # collected before the toggles were enabled, or after changing them).
        self._post_apply_btn = QPushButton("Apply to captured channels now")
        self._post_apply_btn.setCursor(Qt.PointingHandCursor)
        self._post_apply_btn.setToolTip(
            "Re-run the post-processing above over every stored channel of "
            "the selected well, from the RAW stitches (non-destructive: the "
            "raw PNGs are kept; overlays prefer the processed copies).")
        self._post_apply_btn.clicked.connect(self._apply_post_to_captured)
        post.add_widget(self._post_apply_btn)

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

    # v7.19: get_context_widget lives above, with the rest of the panel bridge.
    # It used to return None here — a SECOND definition later in the class,
    # which silently won and left the workflow with no left box at all.

    def on_status_update(self) -> None:
        pass

    def set_settings(self, settings) -> None:
        self._settings = settings

    def showEvent(self, event):
        self._start_camera()
        # v7.19 — take the camera as the operator left it, THEN force the
        # fluorescence preset. Order matters: snapshotting after the preset
        # would make "Camera defaults" restore the preset itself.
        self._capture_entry_hw()
        if self._panel.preset_is_fluorescence():
            self._apply_camera_preset()
        self._panel.refresh_ranges()
        self._panel.load_recipes()
        self._panel.start_polling()
        # Draw the planned-raster preview once now and again shortly after, so it
        # appears as soon as the camera starts delivering frames (needed for the
        # FOV/grid sizing).
        self._refresh_grid_preview()
        try:
            from PySide6.QtCore import QTimer
            QTimer.singleShot(900, self._refresh_grid_preview)
            # The camera may not be delivering (or advertising its ranges) yet.
            QTimer.singleShot(900, self._panel.refresh_ranges)
        except Exception:
            pass
        super().showEvent(event)

    def hideEvent(self, event):
        # Flush any debounced slider edit BEFORE restoring, or the pending
        # write would land on the camera after it had been put back.
        try:
            self._panel.stop_polling()      # flushes any pending slider edit
        except Exception:
            pass
        # ...so the values written by that flush are in the widgets before this
        # writes them out. Reversing these two loses the last edit every time.
        self._flush_recipe_save()
        self._restore_entry_hw()
        self._entry_hw = None
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
        """The shared objectives.json key — v7.16: the DEVICE IDENTITY.

        Was ``camera_spec.name``, which two physical cameras can share; see
        ``MosaicCalibration.objective_camera_key``.
        """
        cam_idx = self._resolve_microscope_cam_idx()
        cfg = self._hw_config
        spec = getattr(getattr(cfg, "camera_config", None), "camera_spec", None)
        spec_name = getattr(spec, "name", None) if spec is not None else None
        try:
            from SupportClasses.MosaicCalibration import objective_camera_key
            key = objective_camera_key(
                getattr(self, "_camera_manager", None), cam_idx, spec_name)
        except Exception:                      # pragma: no cover - import guard
            key = str(spec_name) if spec_name else None
        return key or f"camera_{cam_idx}"

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
                # Only claim it if this page is the owner — otherwise the host's
                # camera would be stopped from this page's hideEvent.
                if self._owns_camera:
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
        """Fill the objective combo, preferring the NOSEPIECE's own slots.

        v7.19: the combo now drives the turret, so it must offer the things the
        turret can actually be driven to — the named positions from Hardware
        Setup → Microscope, in turret order. The calibration store's list is
        kept as the fallback for a rig with no microscope configured, which is
        exactly what it was before this change.
        """
        names = self._nosepiece_objective_names()
        if not names:
            try:
                from SupportClasses.ObjectiveCalibration import (
                    get_store as obj_store)
                names = list(obj_store().objective_names())
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
        #
        # 🔴 v7.19.1 — this used to call ``_on_objective_changed`` and MUST NOT.
        # That was correct while the handler was a software relabel, but v7.19
        # gave it a second job: it now calls ``_drive_objective``, which fires
        # ``ensure_optics_async`` and PHYSICALLY ROTATES THE NOSEPIECE. This
        # function runs from ``set_hardware_config`` — a config load, not an
        # operator click — so loading a saved setup would have commanded a
        # turret rotation nobody asked for.
        #
        # ``_adopt_objective`` is exactly the half this line ever wanted: record
        # the name, push its µm/px, touch no hardware.
        cur = self._objective_combo.currentText()
        if cur:
            self._adopt_objective(cur)

    def _on_objective_changed(self, name: str):
        """Rotate the nosepiece to ``name``, then adopt its calibrated µm/px.

        v7.19, operator: *"when i select a different objective at the top, it
        should change the objective if the microscope is connected"*. Until now
        this was a SOFTWARE RELABEL only — picking 20x on a body sitting at 4x
        silently rescaled every tile of the next mosaic.

        Three cases, and the difference between them is the whole point:

        * **A scan is running** → refused. The run freezes its raster positions
          and µm/px once and replays them for every channel, so changing the
          objective mid-run leaves the tile spacing not matching the field of
          view — the v7.16 seam failure by another route.
        * **A drivable body** → ``ensure_objective``, which retreats the focus
          first (rotation does not move Z, so a height that clears a 4x can be
          inside a 20x's front lens) and verifies the new position by read-back.
          A refusal REVERTS the combo, because a combo showing an objective that
          is not in the light path is the bug this change exists to remove.
        * **No body, or a manual turret** → the operator's pick stands. They
          told us what they fitted, and on a manual turret that is the only
          source there is.

        Only this last, operator-initiated GUI-thread path writes
        ``current_objective_name``; nothing automatic or polled does.
        """
        if not name:
            return
        if self.is_scanning():
            logger.info("Objective change refused: a scan is running.")
            self._revert_objective_combo()
            self._status.setText(
                "Objective unchanged — a capture is running. The raster is "
                "planned for the objective it started with.")
            return
        self._drive_objective(name)
        self._adopt_objective(name)

    def _nosepiece_objective_names(self) -> list[str]:
        """Named nosepiece positions, in turret order. ``[]`` when unconfigured.

        The operator's slot label is what ``OpticsRegistry`` resolves and what
        ``ObjectiveCalibration`` is keyed by — the two already join by string,
        case-insensitively, so a panel labelled ``4X`` finds the ``4x``
        calibration with no store rewrite.
        """
        try:
            from SupportClasses.MicroscopeConfigStore import get_store as _cfg
            from SupportClasses.OpticsRegistry import OBJECTIVE, resolve_slots
            slots = resolve_slots(scope_state=self._scope_state(),
                                  config_store=_cfg(), kind=OBJECTIVE)
        except Exception:
            return []
        return [str(sl.name) for sl in sorted(slots, key=lambda x: x.position)
                if getattr(sl, "name", "")]

    def _drive_objective(self, name: str):
        """Ask the body for ``name``; no-op when it cannot be driven."""
        try:
            from gui.widgets.optics_ensure import ensure_optics_async
        except Exception:
            return
        ensure_optics_async(
            kind="objective", name=str(name),
            glass_focus_um=self._glass_focus_um(),
            needle_retracted=self._needle_is_retracted(),
            on_done=self._on_objective_ensured)

    def _glass_focus_um(self):
        """Focus reading at which the plate glass is sharp, if it is known.

        Comes from the v7.11 optical datum (``PlateFocusDatumStore``), measured
        for THIS camera + objective + plate. ``ensure_objective`` refuses to
        rotate without it rather than rotating hopefully: it is what turns a
        focus position into a gap between the front lens and the glass, and so
        what decides whether a 20x (WD ~1 mm) may be rotated in at all.

        Absent is a legitimate answer — the operator simply gets the refusal
        naming what to measure, which is far better than a guessed clearance.
        """
        try:
            from SupportClasses.PlateFocusDatumStore import get_store
            ident = self._camera_key()
            obj = self._current_objective_name()
            plate = self._plate_key()
            if not (ident and obj and plate):
                return None
            rec = get_store().get(str(ident), str(obj), plate) or {}
            val = rec.get("focus_um_at_bottom")
            return float(val) if val is not None else None
        except Exception:
            return None

    def _needle_is_retracted(self):
        """True/False when it can be judged, None when it cannot.

        ``ensure_objective`` refuses outright on False, so "unknown" must NOT
        be reported as False — that would block every rotation on a rig with no
        Z board connected, where there is no needle to endanger.
        """
        ctrl = self._controller
        if ctrl is None or self._safe_z is None:
            return None
        try:
            if not getattr(ctrl, "is_zp_connected", False):
                return None
            pos = ctrl.get_zp_position_zero_ref() or {}
            cur = pos.get("Z")
            if cur is None:
                return None
            return bool(ctrl.needle_at_or_above(float(cur),
                                                float(self._safe_z)))
        except Exception:
            return None

    def _on_objective_ensured(self, result):
        """GUI thread: report, and revert the combo if nothing moved."""
        if result is None:
            # No body at all — the operator's pick stands (manual turret).
            logger.info("Objective set as a label: no microscope to drive.")
            return
        if getattr(result, "ok", False):
            note = result.describe()
            logger.info("Objective: %s", note)
            if getattr(result, "simulated", False):
                self._status.setText(f"Objective {note}")
            return
        from gui.widgets.optics_ensure import describe_refusal
        why = describe_refusal(result)
        logger.warning("Objective switch refused: %s", why)
        real = self._revert_objective_combo()
        # 🔴 v7.19.1 — reverting the COMBO is not enough, and the gap was the
        # silent-scale-error class this whole area exists to prevent.
        # ``ensure_optics_async`` is asynchronous, so ``_adopt_objective`` has
        # ALREADY run by the time a refusal lands: ``current_objective_name``
        # and the µm/px pushed into CameraManager are both sitting on the
        # objective that never entered the light path. Reverting only the combo
        # left the screen right and the numbers wrong — the worst arrangement,
        # because nothing on screen disagrees. Re-adopt what the body actually
        # has.
        if real:
            self._adopt_objective(real)
        self._status.setText(f"Objective NOT changed — {why}")

    def _revert_objective_combo(self) -> str:
        """Show the objective that is actually in the light path.

        Returns its name so the caller can put the DECLARED objective and the
        pushed µm/px back on it too — see ``_on_objective_ensured``.
        """
        real = ""
        try:
            state = self._scope_state()
            from SupportClasses.OpticsRegistry import OBJECTIVE, optic_at
            optic = optic_at(state, getattr(state, "objective_position", None),
                             OBJECTIVE)
            real = str(getattr(optic, "label", "") or "")
        except Exception:
            real = ""
        if not real:
            return ""
        idx = self._objective_combo.findText(real)
        if idx < 0:
            return real
        blocked = self._objective_combo.blockSignals(True)
        try:
            self._objective_combo.setCurrentIndex(idx)
        finally:
            self._objective_combo.blockSignals(blocked)
        return real

    def _adopt_objective(self, name: str):
        """Record ``name`` as current and push its calibrated µm/px."""
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
            # v7.10: the per-objective ``rotation_deg`` is NOT pushed. Only
            # µm/px is per-objective; rotation belongs to the mount and lives in
            # CameraCalibrationStore. Pushing it here let switching objectives on
            # THIS page silently re-orient every live view and the click→stage
            # map, while the mosaic (which reads the store) disagreed.
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
            # The view still binds to the right slot when a host owns the camera
            # — it just does not start or stop it.
            if not self._owns_camera:
                return
            if not self._camera_manager.is_running(cam_idx):
                self._camera_manager.start(cam_idx)
                self._camera_started_by_us = True
        except Exception as e:
            logger.debug("Fluor mosaic camera start failed: %s", e)

    def _stop_camera(self):
        if not self._owns_camera:
            return
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
        self._update_survey_button()

    def _update_survey_button(self):
        btn = getattr(self, "_survey_btn", None)
        if btn is None:
            return
        has = False
        try:
            plate_key = self._plate_key()
            if plate_key and self._scan_well:
                has = bool(fms.get_store().get_focus_survey(
                    plate_key, self._scan_well))
        except Exception:
            has = False
        btn.setEnabled(has)

    def _open_focus_survey(self):
        plate_key = self._plate_key()
        if not plate_key or not self._scan_well:
            return
        dlg = _FocusSurveyDialog(self, plate_key, self._scan_well, parent=self)
        dlg.exec()

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
        # v7.19 — tile-major needs every selected channel's cube to be
        # resolvable AND drivable. A 400-tile well cannot pause for a human
        # 1200 times, so this is settled BEFORE the run, not discovered at
        # tile 1. Falling back to channel-major is always offered.
        order = self.scan_order()
        if order == "tile":
            blockers = self._tile_major_blockers(channels)
            if blockers:
                order = self._offer_channel_major(blockers)
                if order is None:
                    return
        body, tile_major = self._describe_run_cost(plan, channels, order)
        proceed = QMessageBox.question(
            self, "Fluorescence mosaic", body,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No)
        if proceed != QMessageBox.StandardButton.Yes:
            return

        # v7.14 — optional full-sensor-resolution capture. Deliberately AFTER
        # the confirm: binning does not change the field of view, so the tile
        # count the operator just approved is identical at either resolution,
        # and a declined dialog never leaves the camera switched. The plan is
        # then re-derived because the frame size and µm/px both change (their
        # product, the FOV, does not).
        if self._apply_full_res():
            re_plan = self._compute_raster_plan(well)
            if re_plan is not None:
                plan = re_plan
                grid = plan["grid"]
                bounds = plan["bounds"]
                eff_um_per_px = plan["eff_um_per_px"]
                fw, fh = plan["frame_size"]
            else:
                logger.warning("Fluor mosaic: could not re-plan at full "
                               "resolution — restoring the preview resolution")
                self._restore_full_res()

        # Stash run params and start the per-channel sequence.
        self._scan_positions = grid
        self._scan_bounds = bounds
        self._scan_objective = self._current_objective_name()
        self._scan_um_per_px = eff_um_per_px
        self._scan_frame_size = (fw, fh)
        self._capture_queue = channels
        self._capture_index = 0
        self._aborting = False
        # v7.13 — snapshot the entry exposure (restored when the run ends,
        # aborts or fails) and reset the per-run focus map (measured by the
        # first channel, replayed by the rest).
        self._entry_exposure_us = None
        try:
            st = self._camera_manager.get_hw_settings(cam_idx)
            v = st.get("exposure_us")
            self._entry_exposure_us = float(v) if v else None
        except Exception:
            self._entry_exposure_us = None
        self._run_focus_map = None
        self._run_af_summary = None
        self._scan_exposure_us = 0.0
        self._start_camera()
        if tile_major:
            self._start_tile_major_run(channels)
        else:
            self._prompt_next_channel()

    # ── v7.19: tile-major run planning ────────────────────────────

    def _tile_major_blockers(self, channels: list[str]) -> list[str]:
        """Channels whose cube cannot be driven, each with its reason.

        Empty means tile-major can run unattended.
        """
        state = self._scope_state()
        if not getattr(state, "connected", False) \
                or not getattr(state, "has_filter", False):
            return ["no motorised filter cassette is connected — every cube "
                    "change would need a person"]
        _slots, refusals = self._cube_slots(state)
        return [f"{ch}: {refusals[ch]}" for ch in channels if ch in refusals]

    def _offer_channel_major(self, blockers: list[str]):
        """Ask whether to fall back. Returns ``"channel"`` or None to cancel."""
        detail = "\n".join(f"  • {b}" for b in blockers)
        answer = QMessageBox.question(
            self, "Fluorescence mosaic",
            "Every colour per tile needs the cassette to be switched "
            "automatically at every tile, and these channels cannot be:\n\n"
            f"{detail}\n\n"
            "Capture a full scan per colour instead? You will be prompted to "
            "change the cube between channels.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.Yes)
        return "channel" if answer == QMessageBox.StandardButton.Yes else None

    def _describe_run_cost(self, plan, channels, order) -> tuple[str, bool]:
        """The confirm text, priced honestly. ``(body, tile_major)``."""
        n_tiles = len(plan["grid"])
        n_ch = len(channels)
        tile_major = (order == "tile")
        head = (f"This will raster {n_tiles} tiles "
                f"({plan['cols']}×{plan['rows']}) capturing "
                f"{n_ch} channel(s).\n\n")
        if tile_major:
            rotations = n_tiles * n_ch
            # ~1.5 s per capture as before, plus ~1.5 s per cube rotation.
            est_min = (n_tiles * n_ch * 1.5 + rotations * 1.5) / 60.0
            mem_mb = self._composite_mb(plan) * n_ch
            body = (
                head +
                f"Order: EVERY COLOUR PER TILE — the channels share one focus "
                f"and one registration, so they overlay exactly.\n\n"
                f"Cost: {rotations} cube rotations (~{est_min:.0f} min total), "
                f"and roughly {mem_mb:.0f} MB held while it runs "
                f"({n_ch} composites at once).\n\n"
                f"The needle stays retracted at Safe Z.\n\nStart?")
        else:
            est_min = n_tiles * n_ch * 1.5 / 60.0
            body = (
                head +
                f"Order: FULL SCAN PER COLOUR (~{est_min:.0f} min total). "
                f"Each channel is captured minutes after the last, so they "
                f"register only as well as the stage repeats.\n\n"
                f"The needle stays retracted at Safe Z.\n\nStart?")
        return body, tile_major

    def _composite_mb(self, plan) -> float:
        """Rough float64 accumulator cost of ONE channel's composite, in MB.

        The composite is not final until its last tile, so tile-major genuinely
        holds N of these at once — worth stating before a long run rather than
        discovering as a swap storm.
        """
        try:
            px = float(plan.get("target_px") or 2500)
        except Exception:
            px = 2500.0
        # composite (h × w × 3) + weight_sum (h × w), both float64.
        return (px * px * 4 * 8) / (1024.0 * 1024.0)

    def _start_tile_major_run(self, channels: list[str]):
        """One worker, one raster, every channel captured at each tile."""
        cam_idx = self._resolve_microscope_cam_idx()
        try:
            cam = self._camera_manager.cameras[cam_idx]
        except (AttributeError, IndexError):
            self._status.setText(f"Camera index {cam_idx} not available.")
            return
        try:
            from SupportClasses.MosaicCalibration import (
                build_mosaic_builder, refuse_reason)
        except ImportError:
            self._status.setText("MosaicCalibration unavailable.")
            return
        cal = self._mosaic_calibration(self._scan_frame_size)
        reason = refuse_reason(cal) if cal is not None else "no calibration"
        if reason:
            self._status.setText(reason)
            return
        scan = self._scan_settings() or {}
        target_px = int(getattr(self, "_full_res_canvas", None)
                        or self._target_px.value())
        plans: list[ChannelPlan] = []
        for ch in channels:
            builder = build_mosaic_builder(
                cal, target_mosaic_px=target_px,
                retain_for_reorient=True, retain_frames=False)
            builder.generate_raster_positions(
                self._scan_bounds, overlap=cal.overlap_frac)
            rec = self.channel_recipe(ch) or {}
            lo, hi = rec.get("display_lo"), rec.get("display_hi")
            plans.append(ChannelPlan(
                channel=ch, builder=builder,
                exposure_us=float(rec.get("exposure_us") or 0.0),
                gain_pct=rec.get("gain_pct"),
                # 0 / unset falls back to the shared scan setting.
                avg_frames=int(rec.get("avg_frames")
                               or scan.get("avg_frames") or 1),
                # Explicit operator levels BEAT the probe's measurement: they
                # chose them against the histogram, so the mosaic should look
                # like the preview did. Unset → the probe measures them.
                levels=((float(lo), float(hi))
                        if (lo is not None and hi is not None and hi > lo)
                        else None)))

        af = tracker = predictor = optics = scope = None
        probe_sigma = None
        if self._af_wanted():
            (scope, af, tracker, predictor, optics, probe_sigma,
             note) = self._build_autofocus(cal, cam)
            if note:
                logger.info("Fluor mosaic autofocus: %s", note)
        timing = resolve_grab_timing(scan)
        self._worker = _SingleWellMosaicWorker(
            self._controller, cam, plans[0].builder, self._scan_positions,
            self._safe_z,
            fresh_frames=timing.fresh_frames,
            fresh_timeout_s=timing.fresh_timeout_s,
            settle_ms=timing.settle_ms,
            registration_method=cal.reg_method,
            probe_xy=self._well_center_um(self._scan_well),
            scope=scope, autofocus=af, tracker=tracker,
            focus_predictor=predictor, optics=optics,
            probe_sigma_um=probe_sigma,
            channels=plans)
        self._worker.progress.connect(self._on_channel_progress)
        self._worker.tile.connect(self._on_channel_tile)
        self._worker.channel_done.connect(self._on_tile_major_channel)
        self._worker.finished_all.connect(self._on_tile_major_finished)
        self._worker.failed.connect(self._on_channel_failed)
        self._update_button_state()
        self._status.setText(
            f"Capturing {len(plans)} channel(s), every colour per tile…")
        self._worker.start()

    def _on_tile_major_channel(self, channel, composite, extent, scale,
                               frames, shift_um, meta):
        """Persist ONE finished channel (the worker is still running)."""
        self._save_channel_result(channel, composite, extent, scale, frames,
                                  shift_um, meta)
        self._refresh_channel_status()
        self._refresh_preview()
        self._notify_mosaic_ready()

    def _on_tile_major_finished(self):
        retire_worker(self._worker)
        self._worker = None
        self._finish_run()

    def _prompt_next_channel(self):
        if self._aborting:
            self._restore_entry_exposure()
            self._status.setText("Aborted.")
            self._update_button_state()
            return
        if self._capture_index >= len(self._capture_queue):
            self._finish_run()
            return
        channel = self._capture_queue[self._capture_index]
        # v7.19 — try to SWITCH the cube rather than ask. Only when that
        # refuses does the operator get the prompt, with the reason.
        switched, why_not = self._ensure_cube_for(channel)
        spin = getattr(self, "_channel_exposure", {}).get(channel)
        if switched:
            self._apply_channel_exposure(
                float(spin.value()) if spin is not None else 0.0)
            self._start_channel_scan(channel)
            return

        # ⚠ The label names the CASSETTE SLOT, resolved through
        # `channel_slot`, never `channel_number`. That function returns an
        # acquisition ORDINAL which the store's own comments say is not a
        # turret position: on this rig it calls slot 3 "mCherry" when the
        # cassette holds TxRed, and "Bright Field (5)" when slot 5 is empty.
        ch_label = self._channel_prompt_label(channel)
        # v7.13 — the prompt carries the channel's remembered exposure and
        # applies it LIVE while open, so the operator focuses/checks at the
        # exposure the scan will actually use.
        detail = f"{why_not}\n\n" if why_not else ""
        dlg = _ChannelPromptDialog(
            channel,
            f"{detail}Set the microscope filter / illumination for the "
            f"{ch_label}, focus if needed, then click Start scan.\n\n"
            f"(Cancel stops the capture.)",
            exposure_ms=(float(spin.value()) if spin is not None else 0.0),
            on_apply_exposure=self._apply_channel_exposure,
            camera_manager=self._camera_manager,
            cam_idx=self._resolve_microscope_cam_idx(),
            parent=self)
        if dlg.exec() != QDialog.Accepted:
            self._restore_entry_exposure()
            self._status.setText("Capture stopped by operator.")
            self._update_button_state()
            return
        ms = dlg.exposure_ms()
        if spin is not None:
            spin.setValue(ms)      # write-back → popout persistence
        self._apply_channel_exposure(ms)
        self._start_channel_scan(channel)

    def _channel_prompt_label(self, channel: str) -> str:
        """``"FITC channel (cassette slot 2)"`` — the SLOT, or no number.

        Silence is correct when the cube cannot be resolved: a number that
        names the wrong cube is worse than no number, because a TxRed image
        filed as mCherry is a result nothing downstream can detect.
        """
        slots, _refusals = self._cube_slots(self._scope_state())
        slot = slots.get(channel)
        if slot:
            return f"{channel} channel (cassette slot {slot})"
        return f"{channel} channel"

    def _ensure_cube_for(self, channel: str) -> tuple[bool, str]:
        """Drive the cassette to ``channel``'s cube. ``(switched, why_not)``.

        BLOCKING on purpose — this runs from the GUI thread between channels,
        where the alternative was a modal dialog waiting on a human, so a
        couple of seconds of turret rotation is strictly faster than what it
        replaces. The per-tile path in tile-major mode uses the service
        directly from the worker thread instead.
        """
        try:
            from gui.widgets.optics_ensure import build_service, describe_refusal
        except Exception:
            return False, ""
        service = build_service("fluor_mosaic_cube")
        if service is None:
            return False, ""
        try:
            res = service.ensure_filter(str(channel))
        except Exception as exc:
            logger.debug("ensure_filter(%s) failed: %s", channel, exc)
            return False, ""
        if getattr(res, "ok", False) and not getattr(res, "simulated", False):
            logger.info("Filter cube for %s: %s", channel, res.describe())
            return True, ""
        # A SIMULATED switch is not a real one: the operator still has to move
        # a real cube, so they still get the prompt.
        return False, describe_refusal(res) if not getattr(res, "ok", False) \
            else "the microscope is simulated, so the cube did not really move"

    def _cube_record(self, channel: str) -> dict:
        """``{cube_slot, cube_label}`` for the cube ACTUALLY in the path.

        Read back from the body rather than assumed from the request, and
        omitted entirely when it cannot be read — an absent field is
        recoverable, a wrong one is not.
        """
        state = self._scope_state()
        pos = getattr(state, "filter_position", None)
        if not pos or not getattr(state, "connected", False):
            return {}
        label = ""
        try:
            from SupportClasses.OpticsRegistry import FILTER, optic_at
            optic = optic_at(state, pos, FILTER)
            label = str(getattr(optic, "label", "") or "")
        except Exception:
            label = ""
        return {"cube_slot": int(pos), "cube_label": label}

    def _apply_channel_exposure(self, ms: float):
        """Apply a per-channel exposure to the microscope camera (0 = skip)."""
        if not ms or ms <= 0 or self._camera_manager is None:
            return
        try:
            self._camera_manager.set_hw_exposure_us(
                self._resolve_microscope_cam_idx(), int(round(ms * 1000.0)))
        except Exception:
            logger.debug("fluor mosaic: per-channel exposure apply failed")

    # ── Full-resolution capture (v7.14) ───────────────────────────

    def _apply_full_res(self) -> bool:
        """Switch to full sensor resolution for the run. True if switched.

        The previous resolution is remembered on the page and restored by
        :meth:`_restore_full_res`, which rides the same four exit paths as the
        entry-exposure restore (finish / abort / operator cancel / failure).
        """
        self._full_res_prev = None
        self._full_res_canvas = None
        cfg = self._scan_settings() or {}
        if not cfg.get("full_res_scan"):
            return False
        try:
            from SupportClasses.CaptureResolution import (
                canvas_px_for_full_res, current_resolution, describe_switch,
                switch_to_max)
            cam_idx = self._resolve_microscope_cam_idx()
            prev = switch_to_max(self._camera_manager, cam_idx)
            if prev is None:
                return False
            self._full_res_prev = prev
            new = current_resolution(self._camera_manager, cam_idx)
            canvas = canvas_px_for_full_res(
                int(self._target_px.value()), prev, new)
            self._full_res_canvas = canvas
            logger.info("Fluor mosaic: " + describe_switch(prev, new, canvas))
            return True
        except Exception as exc:
            logger.warning(f"Fluor mosaic: full-resolution switch failed: {exc}")
            self._full_res_prev = None
            return False

    def _restore_full_res(self):
        """Restore the preview resolution. Idempotent; never raises."""
        prev = getattr(self, "_full_res_prev", None)
        if prev is None:
            return
        self._full_res_prev = None
        self._full_res_canvas = None
        try:
            from SupportClasses.CaptureResolution import restore
            restore(self._camera_manager,
                    self._resolve_microscope_cam_idx(), prev)
        except Exception as exc:
            logger.warning(f"Fluor mosaic: resolution restore failed: {exc}")

    def _restore_entry_exposure(self):
        """Best-effort restore of the exposure in force before the run.

        v7.14 — also restores the capture resolution: both are run-scoped
        camera state snapshotted at start, and pairing them here means every
        exit path that already restores one restores the other.
        """
        self._restore_full_res()
        entry = getattr(self, "_entry_exposure_us", None)
        if entry is None or self._camera_manager is None:
            return
        try:
            self._camera_manager.set_hw_exposure_us(
                self._resolve_microscope_cam_idx(), int(round(float(entry))))
            logger.info(f"Fluor mosaic: entry exposure restored ({entry} µs)")
        except Exception:
            logger.debug("fluor mosaic: entry exposure restore failed")
        self._entry_exposure_us = None

    def _finish_run(self):
        """Queue complete: restore exposure, persist the focus survey, report."""
        self._restore_entry_exposure()
        # v7.13 — persist the measured sample surface (raw samples + summary)
        # so the Focus survey dialog and Cell Targeting can use it. This is
        # DATA persistence only — nothing is installed into any datum.
        try:
            fmap = getattr(self, "_run_focus_map", None)
            plate_key = self._plate_key()
            if fmap and plate_key and self._scan_well:
                existing = fms.get_store().get_focus_survey(
                    plate_key, self._scan_well) or {}
                fms.get_store().set_focus_survey(
                    plate_key, self._scan_well, fmap,
                    summary=getattr(self, "_run_af_summary", None),
                    model=str(existing.get("model") or "plane"))
        except Exception:
            logger.exception("fluor mosaic: focus survey persist failed")
        # v7.17: offer the completed well to LabLink — once per RUN, after the
        # store (channels + focus survey) is fully written, so the .nd3 export
        # reads a finished record. Never per channel (that would send N
        # partial files); never raises; a no-op unless the feature is enabled.
        from SupportClasses.LabLinkPublish import publish_fluorescence_well
        publish_fluorescence_well(self._plate_key() or "", self._scan_well or "")
        n_af = len(getattr(self, "_run_focus_map", None) or ())
        af_txt = f" · focus survey: {n_af} samples" if n_af else ""
        self._status.setText(
            f"Done — captured {len(self._capture_queue)} channel(s) for "
            f"{self._scan_well}.{af_txt}")
        self._refresh_channel_status()
        self._refresh_preview()
        self._update_button_state()

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
        # v7.14 — when the run switched to full sensor resolution, raise the
        # stitch canvas with it, or the extra pixels are discarded when each
        # tile is resized into the canvas.
        full_canvas = getattr(self, "_full_res_canvas", None)
        if full_canvas and int(full_canvas) > target_px:
            target_px = int(full_canvas)
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

        # v7.13 — record the exposure ACTUALLY in force (readback, not the
        # spinbox) so the store metadata says what the camera did.
        self._scan_exposure_us = 0.0
        try:
            st = self._camera_manager.get_hw_settings(cam_idx)
            v = st.get("exposure_us")
            self._scan_exposure_us = float(v) if v else 0.0
        except Exception:
            self._scan_exposure_us = 0.0

        # v7.13 — autofocus wiring: tracker on the first channel of the run,
        # replay predictor on later channels. Any refusal degrades to a
        # normal no-AF scan with the reason in the status line.
        scope = af = tracker = predictor = optics = None
        probe_sigma = None
        af_note = ""
        avg_frames = int(getattr(cal, "avg_frames", 1) or 1)
        probe_xy = self._well_center_um(self._scan_well)
        if self._af_wanted():
            (scope, af, tracker, predictor, optics,
             probe_sigma, af_note) = self._build_autofocus(cal, cam)
            if af_note:
                logger.info(f"Fluor mosaic AF: {af_note}")

        status_af = f"  ({af_note})" if af_note else ""
        self._status.setText(
            f"[{self._capture_index + 1}/{len(self._capture_queue)}] "
            f"Scanning {channel}: 0/{len(self._scan_positions)}…{status_af}")
        self._worker = _SingleWellMosaicWorker(
            self._controller, cam, builder, self._scan_positions, safe_z,
            fresh_frames=fresh_frames, fresh_timeout_s=fresh_timeout_s,
            settle_ms=settle_ms, registration_method=cal.reg_method,
            avg_frames=avg_frames, probe_xy=probe_xy,
            exposure_us=self._scan_exposure_us,
            scope=scope, autofocus=af, tracker=tracker,
            focus_predictor=predictor, optics=optics,
            probe_sigma_um=probe_sigma)
        self._worker.progress.connect(self._on_channel_progress)
        self._worker.tile.connect(self._on_channel_tile)
        self._worker.finished_ok.connect(
            lambda comp, ext, scale, frames, shift, meta, ch=channel:
            self._on_channel_finished(ch, comp, ext, scale, frames, shift,
                                      meta))
        self._worker.failed.connect(self._on_channel_failed)
        self._worker.start()
        self._update_button_state()

    # ── v7.13: autofocus construction (GUI thread; lease taken by worker) ──

    def _af_wanted(self) -> bool:
        chk = getattr(self, "_af_enabled", None)
        return bool(chk is not None and chk.isChecked())

    def _build_autofocus(self, cal, cam):
        """Resolve (scope, TileAutofocus, tracker|None, predictor|None,
        optics, probe_sigma, note). Every refusal returns Nones + a note —
        the scan must always be able to run without AF."""
        none = (None, None, None, None, None, None)
        try:
            from SupportClasses.MicroscopeControl import get_microscope
        except Exception:
            return (*none, "microscope control unavailable")
        try:
            scope = get_microscope()
            st = scope.state()
        except Exception:
            return (*none, "microscope not reachable")
        if not getattr(st, "connected", False) or not getattr(st, "has_focus",
                                                              False):
            return (*none, "autofocus off: microscope focus not connected")
        try:
            from SupportClasses.ObjectiveOptics import (
                ObjectiveOptics, depth_of_field_um, refuse_if_incomplete)
            from SupportClasses.TileAutofocus import TileAutofocus
            from SupportClasses.MosaicFocusTracker import (
                MosaicFocusTracker, PlanePredictor)
        except Exception:
            return (*none, "autofocus modules unavailable")

        # The body's own optics for the CURRENT turret position (read-only —
        # this never writes current_objective_name).
        # v7.18: one shared walk (OpticsRegistry.optic_at) instead of a fourth
        # hand-rolled copy, and `label` NOT `name` — `MountedOptic` has no `name`
        # field, so `getattr(optic, "name", "")` was ALWAYS empty and this label
        # silently fell back to `_scan_objective` on every run, never carrying
        # the body's own answer. The sibling lines below read the right fields,
        # which is why nothing looked wrong.
        from SupportClasses.OpticsRegistry import OBJECTIVE, optic_at
        pos = getattr(st, "objective_position", None)
        optic = optic_at(st, pos, OBJECTIVE)
        optics = ObjectiveOptics(
            label=str(getattr(optic, "label", "") or self._scan_objective),
            position=int(pos or 0),
            magnification=getattr(optic, "magnification", None),
            numerical_aperture=getattr(optic, "numerical_aperture", None),
            working_distance_mm=getattr(optic, "working_distance_mm", None),
            um_per_px_sample=float(cal.um_per_px),
            frame_wh=self._scan_frame_size)
        why = refuse_if_incomplete(optics)
        if why:
            return (*none, f"autofocus off: {why}")
        dof = depth_of_field_um(optics)
        if not dof or dof <= 0:
            return (*none, "autofocus off: depth of field unknown")

        fingerprint = ""
        try:
            hw = self._camera_manager.get_hw_settings(
                self._resolve_microscope_cam_idx()) or {}
            fingerprint = "|".join(
                f"{k}={hw.get(k)}" for k in sorted(hw)
                if "expo" in k.lower() or "gain" in k.lower())
        except Exception:
            fingerprint = ""

        af = TileAutofocus(
            scope=scope, cam=cam, dof_um=float(dof),
            step_um=float(self._af_step.value()),
            half_range_um=float(self._af_range.value()),
            fingerprint=fingerprint)

        tracker = predictor = None
        probe_sigma = None
        fmap = getattr(self, "_run_focus_map", None)
        if fmap:
            predictor = PlanePredictor(fmap)
            note = "autofocus: replaying the first channel's focus map"
        else:
            center = self._well_center_um(self._scan_well)
            r_um = self._well_diameter_mm(self._scan_well) / 2.0 * 1000.0
            if center is None or r_um <= 0:
                return (*none, "autofocus off: well geometry unknown")
            tracker = MosaicFocusTracker(
                well_center_um=center, well_radius_um=r_um,
                lattice_spacing=int(self._af_lattice.value()),
                dof_um=float(dof), positions=self._scan_positions)
            note = ""
            # A previous survey of this well narrows the probe search window
            # (narrow-only: it never contributes to the new fit).
            try:
                prev = fms.get_store().get_focus_survey(
                    self._plate_key(), self._scan_well)
                rms = ((prev or {}).get("summary") or {}).get(
                    "rms_residual_um")
                if prev and rms is not None:
                    probe_sigma = max(25.0, float(rms))
            except Exception:
                probe_sigma = None
        return (scope, af, tracker, predictor, optics, probe_sigma, note)

    def _post_settings(self) -> dict:
        return {
            "denoise": str(getattr(self, "_post_denoise", None)
                           and self._post_denoise.currentText() or "off"),
            "denoise_strength": int(getattr(self, "_post_strength", None)
                                    and self._post_strength.value() or 1),
            "bg_subtract": bool(getattr(self, "_post_bg", None)
                                and self._post_bg.isChecked()),
            "bg_radius_um": float(getattr(self, "_post_bg_radius", None)
                                  and self._post_bg_radius.value() or 100.0),
        }

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
                             shift_um=(0.0, 0.0), meta=None):
        # ⚠ finished_ok is emitted from INSIDE the worker's run(), which then
        # goes on to restore the microscope focus, release the scope lease and
        # resume the poller in its finally block. The worker has no Qt parent,
        # so this attribute is the ONLY strong reference: clearing it here used
        # to hand a still-running QThread to the garbage collector and abort the
        # process (0xC0000409, no traceback) right after a scan completed
        # successfully. See gui/worker_retirement.py.
        retire_worker(self._worker)
        self._worker = None
        meta = meta if isinstance(meta, dict) else {}
        # The first channel's measured focus map is replayed by the rest of
        # the queue and persisted at run end.
        if meta.get("focus_map"):
            self._run_focus_map = meta["focus_map"]
            self._run_af_summary = meta.get("af")
        if meta.get("af_note"):
            logger.info(f"Fluor mosaic AF: {meta['af_note']}")
        self._save_channel_result(channel, composite, extent, scale, frames,
                                  shift_um, meta)
        self._capture_index += 1
        self._refresh_channel_status()
        self._refresh_preview()
        self._notify_mosaic_ready()
        self._prompt_next_channel()

    def _save_channel_result(self, channel, composite, extent, scale, frames,
                             shift_um, meta):
        """Persist ONE finished channel. Shared by both acquisition orders.

        v7.19 — factored out so channel-major and tile-major cannot record a
        capture differently; the metadata a mosaic carries must not depend on
        which order it happened to be captured in.
        """
        if composite is None or extent is None:
            return
        meta = meta or {}
        plate_key = self._plate_key() or "plate"
        color = self._channel_colors[channel]
        try:
            fms.get_store().save_channel(
                plate_key, self._scan_well, channel, composite, extent,
                color_rgb=(color.red(), color.green(), color.blue()),
                objective=self._scan_objective,
                um_per_px=self._scan_um_per_px, mosaic_scale=scale,
                frames=frames, shift_um=shift_um,
                exposure_us=float(meta.get("exposure_us")
                                  or self._scan_exposure_us or 0.0),
                display_levels=meta.get("display_levels"),
                avg_frames=int(meta.get("avg_frames", 1) or 1),
                gain_pct=(self.channel_recipe(channel) or {}).get("gain_pct"),
                # v7.19 — read back from the body, so a saved channel says
                # which cube was ACTUALLY in the path rather than which one
                # was asked for.
                **self._cube_record(channel))
        except Exception as exc:
            logger.warning("Fluor mosaic save failed: %s", exc)
        self._maybe_attach_processed(plate_key, channel, composite, scale)

    def _maybe_attach_processed(self, plate_key, channel, composite, scale):
        """v7.13 — bake the optional denoise / background subtraction into a
        SIBLING copy (the raw stitch stays canonical; overlays prefer the
        processed one; detection keeps reading the raw)."""
        try:
            from SupportClasses import FluorescencePostProcess as fpp
            settings = self._post_settings()
            if not fpp.is_active(settings):
                return
            import cv2
            gray = cv2.cvtColor(composite, cv2.COLOR_BGR2GRAY)
            # The composite's own µm/px = 1 / mosaic_scale (scale is px/µm).
            upp = (1.0 / float(scale)) if scale and float(scale) > 0 else 0.0
            out, applied = fpp.process(gray, settings, um_per_px=upp)
            if not applied:
                return
            fms.get_store().attach_processed(
                plate_key, self._scan_well, channel,
                cv2.cvtColor(out, cv2.COLOR_GRAY2BGR), applied)
        except Exception:
            logger.exception("fluor mosaic: post-processing failed")

    def _apply_post_to_captured(self):
        """v7.13 — retroactively post-process every stored channel of the
        selected well from its RAW stitch (the automatic path only runs at
        save time; this covers mosaics captured before the toggles were on,
        or after the settings changed). Non-destructive, same as the save
        path: raw PNGs stay canonical, overlays prefer the processed copies."""
        plate_key = self._plate_key()
        well = self._scan_well
        if not plate_key or not well:
            self._status.setText("Select a scanned well first.")
            return
        try:
            from SupportClasses import FluorescencePostProcess as fpp
        except Exception:
            self._status.setText("Post-processing module unavailable.")
            return
        settings = self._post_settings()
        if not fpp.is_active(settings):
            self._status.setText(
                "Enable a post-processing option (denoise / background "
                "subtraction) first — nothing to apply.")
            return
        st = fms.get_store()
        channels = st.list_channels(plate_key, well)
        if not channels:
            self._status.setText(f"No captured channels for {well} yet.")
            return
        import cv2
        n_done = 0
        for ch in channels:
            try:
                img = st.load_channel_image(plate_key, well, ch)   # RAW
                if img is None:
                    continue
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                scale = st.get_mosaic_scale(plate_key, well, ch)
                upp = (1.0 / float(scale)) if scale else 0.0
                out, applied = fpp.process(gray, settings, um_per_px=upp)
                if applied and st.attach_processed(
                        plate_key, well, ch,
                        cv2.cvtColor(out, cv2.COLOR_GRAY2BGR), applied):
                    n_done += 1
            except Exception:
                logger.exception(
                    f"retroactive post-processing failed for {ch}")
        self._refresh_preview()
        self._status.setText(
            f"Post-processing applied to {n_done}/{len(channels)} captured "
            f"channel(s) of {well}.")

    def _on_channel_failed(self, msg: str):
        # Same hazard as _on_channel_finished: failed is emitted from run() too,
        # and the finally block still has the focus restore + lease release to do.
        retire_worker(self._worker)
        self._worker = None
        self._restore_entry_exposure()
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
