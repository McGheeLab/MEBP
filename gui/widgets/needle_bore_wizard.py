"""
needle_bore_wizard.py — the microscope half of the needle/bore calibration.

v7.10. The Needle Location tab historically measured bore offsets by jogging
each bore's tip onto BOTH side-camera crosshairs and subtracting the two stage
positions. That works, but it is slow, needs both needle cams µm/px-calibrated,
and — the reason this module exists — it **cannot measure a per-bore Z**: the
operator jogs only XY between bores, so both captures read the same stage
height and every ``z_offset_mm`` comes out exactly ``0.0``. Downstream,
``PickAndPlaceManager._descend_z_mm`` is then an identity function and the
assembly's coplanarity is *assumed* rather than verified.

THE MICROSCOPE METHOD, in one sentence: park once at a safe height above the
glass, click each bore's tip in a single frame (XY offsets, with the stage
stationary so the microscope↔needle registration cancels exactly), then jog Z
and record each bore's own best focus (the per-bore Z that was missing).

WHAT THIS WIDGET DOES **NOT** DO
--------------------------------
It does not own any datum. ``needle_origin_um``, the reference Z heights and the
plate-bottom Z all keep their existing single writers on ``CalibrationPage``;
this widget calls them through an injected host. Duplicating a datum writer is
how two sources of truth are born.

It also commands **no XY at all** except in the one optional park (step 3's
"Go to needle at survey height"), and no Z except through the host's own bounded
helpers — see ``_on_park`` and the safety notes on each step below.

⚠ THE BORE OFFSETS ARE A ZERO-MOTION MEASUREMENT. They are differences between
clicks in ONE camera frame, so the stage position cancels exactly and *where*
the needle sits is irrelevant — only that it does not move between clicks (the
drift guard). The park is therefore convenience, never calibration: put the
needle wherever the bores are visible, by any means, and press "Start measuring
here". Its gate (:meth:`park_gate`) is deliberately separate from the
measurement's (:meth:`gate`) so an un-taught reference height cannot block a
procedure that never moves the stage.

Layout (v7.13): the wizard renders as a compact CONTROL COLUMN — step strip,
Back/Next, one instruction line, one status line, stacked step pages. The
camera video lives in the HOST's large pane: the host mounts this widget's
``microscope_pane()`` into its own camera stack and listens to ``step_changed``
to switch that stack between the needle side cams (steps 1-2) and the
microscope (steps 3-4), so the step's relevant video is always the biggest
thing on screen.
"""

from __future__ import annotations

import logging
from typing import Optional

from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtWidgets import (
    QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMessageBox, QProgressBar,
    QPushButton, QSizePolicy, QSplitter, QStackedWidget, QTextEdit,
    QVBoxLayout, QWidget,
)

from gui.scaling import s, sp
from gui.styles import COLORS, SECTION_TITLE_STYLE

logger = logging.getLogger(__name__)

try:
    from SupportClasses.NeedleBoreCalibrationStore import (
        get_store as _get_bore_store,
        offset_from_frame_clicks,
        needle_camera_offset_from_click,
        z_offset_from_centred_heights,
    )
    BORE_STORE_AVAILABLE = True
except ImportError:                                  # pragma: no cover
    BORE_STORE_AVAILABLE = False
    _get_bore_store = None
    offset_from_frame_clicks = None
    needle_camera_offset_from_click = None
    z_offset_from_centred_heights = None

try:
    from SupportClasses.BoreFocusROI import rois_for_clicks, view_px_to_frame_px
    ROI_AVAILABLE = True
except ImportError:                                  # pragma: no cover
    ROI_AVAILABLE = False
    rois_for_clicks = None
    view_px_to_frame_px = None

try:
    from SupportClasses.PlateBottomOptical import DEFAULT_OFFSETS_UM
    OPTICAL_AVAILABLE = True
except ImportError:                                  # pragma: no cover
    OPTICAL_AVAILABLE = False
    DEFAULT_OFFSETS_UM = (1000.0, 500.0, 200.0, 100.0)


#: Steps, in order. ``key`` is what the strip and the gates address.
STEP_NEEDLE_ZERO = "needle_zero"
STEP_Z_REFS = "z_refs"
STEP_BORES = "bores"
STEP_TOUCHOFF = "touchoff"
STEP_ORDER = (STEP_NEEDLE_ZERO, STEP_Z_REFS, STEP_BORES, STEP_TOUCHOFF)
STEP_TITLES = {
    STEP_NEEDLE_ZERO: "1 · Needle zero",
    STEP_Z_REFS: "2 · Reference Z",
    STEP_BORES: "3 · Bore offsets",
    STEP_TOUCHOFF: "4 · Plate touch-off",
}

#: Survey height above the taught plate bottom for step 3, in mm. The bores are
#: measured here, NOT at the glass: only DIFFERENCES matter for both the XY
#: offsets and dz, so there is no reason to approach the plate — and doing so
#: would be actively unsafe, because the longest bore reaches lower by exactly
#: the amount being measured and is therefore unknown until afterwards.
DEFAULT_SURVEY_CLEARANCE_MM = 0.5
#: Never park closer than this to the plate bottom, whatever the operator types.
MIN_SURVEY_CLEARANCE_MM = 0.25
#: Step 4 approaches to this above the LONGEST bore's reach.
DEFAULT_TOUCH_CLEARANCE_MM = 0.10
#: Live focus-score refresh. Fast enough to follow a Z jog by eye, and on the
#: widget's own timer rather than the 300 ms page tick so the lifecycle is
#: self-contained (the pattern `_auto_z_timer` already uses).
FOCUS_TICK_MS = 150
#: Refuse a click/record if the stage drifted this far since the park — the
#: one-frame method's whole validity rests on the stage being stationary.
MAX_STAGE_DRIFT_UM = 5.0

#: Bore-dot overlay palette, by bore index (datum green, then cycling).
#: Keys into ``gui.styles.COLORS`` so the dots follow the app theme.
BORE_DOT_COLOR_KEYS = ("green", "blue", "yellow", "pink", "teal")


def bore_dot_color(bore_index: int) -> str:
    """Hex colour for bore ``k``'s overlay dot (datum green, then cycling)."""
    keys = BORE_DOT_COLOR_KEYS
    key = keys[0] if bore_index <= 0 else keys[1 + (bore_index - 1) % (len(keys) - 1)]
    return COLORS.get(key, "#a6e3a1")


def plate_bottom_zref_from_top(top_zref_mm: float, offset_below_top_mm: float,
                               z_up_sign: float) -> float:
    """Plate bottom (zero-ref mm) from the plate TOP and a typed offset.

    ``offset_below_top_mm`` is a physical distance (mm, positive): how far the
    plate's inner bottom surface sits BELOW its top surface — the number on the
    plate's datasheet, auto-fillable from the plate type's stored z_offsets.

    The polarity comes from ``StageController.z_up_sign()`` (+height = up in
    the user frame maps to ``z_up_sign``×raw), NOT from ``print_z_dir()`` —
    that one is *derived from* the taught top/bottom pair, so using it here
    would read a stale sign from a previous plate's references (circular).
    With ``z_up_sign=+1`` the bottom is numerically below the top; on a
    ``z_up_sign=-1`` machine (needle descends as raw Z increases) it is above.
    After applying, ``derive_z_up_sign(top, bottom)`` reproduces ``z_up_sign``
    by construction, so ``print_z_dir()`` comes out self-consistent.
    """
    return float(top_zref_mm) - float(z_up_sign) * float(offset_below_top_mm)


# ════════════════════════════════════════════════════════════════════
#  Pure measurement state — no Qt, so it is directly testable
# ════════════════════════════════════════════════════════════════════

class BoreMeasurement:
    """Accumulates the clicks and Z records for one measurement session.

    Deliberately Qt-free: every number that reaches the calibration store is
    computed here, so the arithmetic can be tested without building a widget.
    """

    def __init__(self, bore_count: int):
        self.bore_count = max(1, int(bore_count))
        #: bore index → click pixel in RAW frame coords
        self.clicks: dict[int, tuple[float, float]] = {}
        #: bore index → ``pixel_to_stage_offset`` for that click
        self.pto: dict[int, tuple[float, float]] = {}
        #: bore index → recorded tip height (user/height frame, mm)
        self.z_user: dict[int, float] = {}
        #: bore index → (best_score, z_at_best) seen live, advisory only
        self.peak: dict[int, tuple[float, float]] = {}

    # ── recording ────────────────────────────────────────────────

    def record_click(self, bore_index: int, click_px, pto_um) -> None:
        k = int(bore_index)
        self.clicks[k] = (float(click_px[0]), float(click_px[1]))
        self.pto[k] = (float(pto_um[0]), float(pto_um[1]))

    def record_z(self, bore_index: int, z_user_mm: float) -> None:
        self.z_user[int(bore_index)] = float(z_user_mm)

    def note_score(self, bore_index: int, score: float, z_user_mm) -> None:
        """Track the sharpest frame seen for a bore. Advisory: the operator's
        Record click is authoritative, this only says 'you passed the peak'."""
        k = int(bore_index)
        if z_user_mm is None:
            return
        best = self.peak.get(k)
        if best is None or float(score) > best[0]:
            self.peak[k] = (float(score), float(z_user_mm))

    def forget(self, bore_index: int) -> None:
        for d in (self.clicks, self.pto, self.z_user, self.peak):
            d.pop(int(bore_index), None)

    # ── derived ──────────────────────────────────────────────────

    @property
    def datum_clicked(self) -> bool:
        return 0 in self.pto

    def clicked_bores(self) -> list:
        return sorted(self.pto)

    def missing_clicks(self) -> list:
        return [k for k in range(self.bore_count) if k not in self.pto]

    def missing_z(self) -> list:
        """Which bores still need a Z record. Single-bore needles need none —
        there is nothing to be coplanar WITH."""
        if self.bore_count <= 1:
            return []
        return [k for k in range(self.bore_count) if k not in self.z_user]

    def offset_for(self, bore_index: int):
        """Datum-relative XY offset (µm), or None when not measurable yet."""
        k = int(bore_index)
        if k == 0:
            return (0.0, 0.0)
        if 0 not in self.pto or k not in self.pto:
            return None
        return offset_from_frame_clicks(self.pto[0], self.pto[k])

    def dz_for(self, bore_index: int) -> float:
        """Datum-relative height offset (mm, + ⇒ reaches lower). 0.0 when either
        end is unrecorded — the same fail-safe the side-camera path uses."""
        k = int(bore_index)
        if k == 0 or 0 not in self.z_user or k not in self.z_user:
            return 0.0
        return z_offset_from_centred_heights(self.z_user[0], self.z_user[k])

    def needle_camera_offset(self):
        """The datum bore's offset from the microscope centre, or None.

        This is the single-bore needle's reason to run the wizard: it is what
        makes a live-view click-to-PICK land on the needle rather than under the
        crosshair.
        """
        if 0 not in self.pto:
            return None
        return needle_camera_offset_from_click(self.pto[0])

    def is_complete(self) -> bool:
        return not self.missing_clicks() and not self.missing_z()


# ════════════════════════════════════════════════════════════════════
#  Optical plate-bottom: one rung, measured off the GUI thread
# ════════════════════════════════════════════════════════════════════

class _Cancelled(Exception):
    """Cooperative stop. Emits nothing — the caller already knows."""


class _RungAbort(RuntimeError):
    """Carries an operator-actionable sentence."""


class PlateBottomRungWorker(QThread):
    """Measure ONE margin: park the needle, then sweep the FOCUS through it.

    WHY THE FOCUS MOVES AND THE NEEDLE DOES NOT
    -------------------------------------------
    Sweeping the needle through a fixed focal plane is the obvious reading of the
    gesture, and it is fine at 10x (DOF 10 µm) or 20x (4 µm). At 4x the depth of
    field is 57 µm, so a curve worth fitting spans roughly ±150 µm — which at a
    100 µm margin puts the tip 50 µm THROUGH the glass. Sweeping the focus
    instead moves only the objective, bounded by 25 % of a 16 mm working
    distance, and the needle makes exactly one bounded descent and then holds
    still. It also makes the answer a MEASUREMENT rather than a command: the tip
    height comes out as ``f_tip − f0``, so a needle that lands short of its
    target because the geometric guess was wrong costs nothing.

    WHY ONE RUNG PER RUN
    --------------------
    The operator confirms or overrides between rungs, and overriding means
    jogging by hand — which needs the microscope panel, which the lease would
    block. Rather than release and re-acquire around a pause of unknown length
    (and fail obscurely if the re-acquire loses), each run holds the lease only
    for its own sweep and exits. The ladder is driven by the widget.

    ⚠ Only the FIRST rung's descent is bounded by the geometric guess; every
    later one is bounded by the measurement before it. That is why the margins
    must be ordered largest first, and why the largest should comfortably exceed
    the guess's uncertainty. The armed print floor is the backstop, but it is
    computed from the same guess, so it does not protect against the guess being
    wrong — the operator watching the live view does.
    """

    #: (focus_um, score) for the live plot/readout.
    sample = Signal(float, float)
    progress = Signal(str)
    measured = Signal(object)          # RungMeasurement
    failed = Signal(str)

    #: Frames to discard after a focus move before believing the image.
    SETTLE_S = 0.30
    FRESH_FRAMES = 2
    FRESH_TIMEOUT_S = 3.0
    OP_TIMEOUT_S = 10.0
    LEASE = "plate_bottom"

    def __init__(self, *, controller, scope, cam, margin_um, focus_zero_um,
                 plate_bottom_zref_mm, zdir, focus_up_sign, optics,
                 roi_rect=None, focus_limits_um=None, soft_limits_um=None,
                 parent=None):
        super().__init__(parent)
        self._ctrl = controller
        self._scope = scope
        self._cam = cam
        self._margin = float(margin_um)
        self._f0 = float(focus_zero_um)
        self._bottom = float(plate_bottom_zref_mm)
        self._zdir = float(zdir)
        self._fus = float(focus_up_sign)
        self._optics = optics
        self._roi = roi_rect
        self._focus_limits = focus_limits_um
        self._soft_limits = soft_limits_um
        self._stop = False
        #: Every needle Z this run commanded. The safety property under test is
        #: that it has exactly one entry — the needle must not move mid-sweep.
        self.needle_moves: list = []

    def stop(self) -> None:
        self._stop = True

    # ── plumbing ─────────────────────────────────────────────────

    def _check(self) -> None:
        if self._stop:
            raise _Cancelled()

    def _op(self, op, what: str, timeout: float | None = None):
        """Await one microscope op and CHECK ITS ERROR.

        ``done`` being set is not success: a op queued behind >20 s of work is
        dropped with ``error="dropped (stale)"``, and retrying only re-queues it
        behind the same backlog — so that one aborts rather than loops.
        """
        if op is None:
            return
        if not op.done.wait(timeout or self.OP_TIMEOUT_S):
            raise _RungAbort(f"The microscope did not respond while {what}.")
        err = getattr(op, "error", None)
        if err:
            if "stale" in str(err):
                raise _RungAbort(
                    f"The microscope dropped the command while {what} — it is "
                    f"busy with something else. Close other microscope panels "
                    f"and retry.")
            raise _RungAbort(f"The microscope refused while {what}: {err}")

    def _goto_focus(self, z_um: float, lead_in_um: float) -> float:
        """Approach from one side (backlash), then READ BACK.

        ``set_focus_um`` clamps to the soft limits *before* queueing and the
        backend clamps again — both silently. A sample taken at a clamped
        position is a guess about where it was, so the caller discards it.
        """
        self._check()
        self._op(self._scope.set_focus_um(z_um - lead_in_um), "moving the focus")
        self._op(self._scope.set_focus_um(z_um), "moving the focus")
        back = getattr(self._scope.state(), "focus_um", None)
        return float(z_um if back is None else back)

    def _grab(self):
        """The only worker-safe frame grab: counter + snapshot copy.

        ``capture_fresh_frame`` touches the backend directly and its own
        docstring forbids this use.
        """
        import time
        time.sleep(self.SETTLE_S)
        start = self._cam.frame_count_value()
        deadline = time.monotonic() + self.FRESH_TIMEOUT_S
        while self._cam.frame_count_value() - start < self.FRESH_FRAMES:
            self._check()
            if time.monotonic() > deadline:
                break
            time.sleep(0.02)
        return self._cam.get_current_frame()

    def _score(self, frame) -> tuple:
        from SupportClasses.VisionDetector import NeedleDetector
        res = NeedleDetector.compute_focus_score(frame, self._roi)
        sat = 0.0
        try:
            import numpy as np
            g = frame if frame.ndim == 2 else frame[..., 0]
            if self._roi:
                x, y, w, h = self._roi
                g = g[y:y + h, x:x + w]
            if g.size and int(g.max()) >= 254:
                sat = float(np.count_nonzero(g >= 254)) / float(g.size)
        except Exception:
            pass
        return float(res.score), sat

    def _fingerprint(self) -> str:
        """Exposure/gain identity. Any change mid-sweep invalidates the curve —
        ``compute_focus_score`` is unnormalised, so scores taken under different
        settings are not comparable at all."""
        try:
            hw = self._cam.get_hw_settings() or {}
            return "|".join(f"{k}={hw[k]}" for k in sorted(hw)
                            if "expo" in k.lower() or "gain" in k.lower())
        except Exception:
            return ""

    # ── the run ──────────────────────────────────────────────────

    def run(self) -> None:                            # noqa: C901
        from SupportClasses.PlateBottomOptical import (
            RungMeasurement, needle_target_zref)
        entry_focus = None
        entry_z = None
        leased = False
        suspended = False
        try:
            acq = getattr(self._scope, "try_acquire", None)
            if callable(acq) and not acq(self.LEASE, 5.0):
                owner = getattr(self._scope, "lease_owner", lambda: "?")()
                raise _RungAbort(
                    f"The microscope is reserved by {owner}. Close that "
                    f"operation and retry.")
            leased = callable(acq)

            entry_focus = getattr(self._scope.state(), "focus_um", None)
            entry_z = self._read_z()
            fn = getattr(self._ctrl, "suspend_position_poller", None)
            if callable(fn):
                fn()
                suspended = True

            # 1 ── the focal plane goes to the margin above the glass.
            f_target = self._f0 + self._margin * self._fus
            self.progress.emit(f"Focusing {self._margin:.0f} µm above the glass…")
            self._goto_focus(f_target, 0.0)

            # 2 ── ONE needle descent, Z only at the current XY, then it holds.
            z_target = needle_target_zref(self._bottom, self._margin, self._zdir)
            self.progress.emit(f"Lowering the needle to {self._margin:.0f} µm…")
            self._move_needle(z_target)
            z_actual = self._read_z()
            if z_actual is None:
                raise _RungAbort("Could not read the needle Z back.")

            # 3 ── sweep the FOCUS through the stationary tip, coarse to fine.
            from SupportClasses.FocusCurve import peak_focus_um
            from SupportClasses.FocusSweepPlanner import next_rung
            from SupportClasses.ObjectiveOptics import depth_of_field_um
            plan, why = self._plan(f_target)
            if plan is None:
                raise _RungAbort(why)
            dof = depth_of_field_um(self._optics)

            peak = reason = None
            rung = plan.rungs[0] if plan.rungs else None
            idx = 0
            while rung is not None:
                samples = self._sweep(rung)
                peak, reason = peak_focus_um(samples, dof_um=dof)
                if peak is None:
                    break
                rung = next_rung(
                    plan=plan, rung_index=idx, measured_peak_um=peak.z_um,
                    focus_limits_um=self._focus_limits,
                    soft_limits_um=self._soft_limits)
                idx += 1

            # 4 ── report.
            if peak is None:
                self.measured.emit(RungMeasurement(
                    margin_um=self._margin, needle_z_zref_mm=z_actual,
                    focus_tip_um=f_target, refusal=reason or "NO_PEAK"))
                return
            # Park ON the peak so the operator is shown a sharp tip when asked
            # whether to accept it. Leaving the focus at the sweep's last
            # position would blur it by up to a half-range.
            self._goto_focus(peak.z_um, 0.0)
            self.measured.emit(RungMeasurement(
                margin_um=self._margin, needle_z_zref_mm=z_actual,
                focus_tip_um=peak.z_um, focus_sigma_um=peak.sigma_z_um,
                fwhm_um=peak.fwhm_um, adopted_by="auto"))
        except _Cancelled:
            self._restore(entry_focus, entry_z)
        except _RungAbort as e:
            self._restore(entry_focus, entry_z)
            self.failed.emit(str(e))
        except Exception as e:                        # pragma: no cover
            logger.exception("plate-bottom rung failed")
            self._restore(entry_focus, entry_z)
            self.failed.emit(f"The measurement failed: {e}")
        finally:
            if suspended:
                try:
                    self._ctrl.resume_position_poller()
                except Exception:
                    pass
            if leased:
                try:
                    self._scope.release(self.LEASE)
                except Exception:
                    pass

    #: How far the tip may plausibly be from where it was SENT, as a fraction of
    #: the margin. The needle is positioned from an estimate of the plate bottom,
    #: so the search must cover that estimate's error — a few DOF wide would
    #: find nothing and report "no peak", an optics diagnosis for a positioning
    #: problem. Proportional to the margin because the estimate improves as the
    #: ladder descends: the first rung searches widely, the last barely at all.
    SEARCH_FRACTION = 0.30

    def _plan(self, center_um: float):
        from SupportClasses.FocusSweepPlanner import plan_sweep
        from SupportClasses.ObjectiveOptics import depth_of_field_um
        dof = depth_of_field_um(self._optics) or 20.0
        want = max(4.0 * dof, self.SEARCH_FRACTION * abs(self._margin))
        return plan_sweep(
            optics=self._optics, center_um=center_um,
            requested_half_range_um=want,
            focus_limits_um=self._focus_limits,
            soft_limits_um=self._soft_limits)

    def _sweep(self, rung) -> list:
        from SupportClasses.FocusCurve import FocusSample
        fp = self._fingerprint()
        out = []
        n = len(rung.z_targets_um)
        for i, z in enumerate(rung.z_targets_um):
            self._check()
            back = self._goto_focus(z, rung.lead_in_um)
            if abs(back - z) > max(1e-6, rung.step_um / 2.0):
                continue                    # clamped: a guess, not a measurement
            frame = self._grab()
            if frame is None:
                continue
            score, sat = self._score(frame)
            out.append(FocusSample(z_um=back, score=score, saturated_frac=sat,
                                   settings_fingerprint=fp,
                                   focus_readback_um=back,
                                   roi_rect=self._roi))
            self.sample.emit(back, score)
            self.progress.emit(f"Sweeping the focus… {i + 1}/{n}")
        return out

    # ── needle ───────────────────────────────────────────────────

    def _move_needle(self, z_zref_mm: float) -> None:
        """Z ONLY, at the current XY — so there is no XY move to retract for.

        Recorded in ``needle_moves`` because "exactly one descent per rung, and
        none once the sweep starts" is the safety property, not an implementation
        detail.
        """
        self.needle_moves.append(float(z_zref_mm))
        self._ctrl.move_z_absolute(float(z_zref_mm), from_zero_ref=True)
        fn = getattr(self._ctrl, "wait_for_z_arrival", None)
        if callable(fn):
            try:
                fn(float(z_zref_mm))
            except Exception:
                pass

    def _read_z(self):
        fn = getattr(self._ctrl, "capture_current_z_raw", None)
        try:
            return None if not callable(fn) else float(fn())
        except Exception:
            return None

    def _restore(self, entry_focus, entry_z) -> None:
        """Put back what we moved — never lower than we found it.

        On SUCCESS the focus is deliberately LEFT at the measured peak: the
        operator is about to be asked whether the tip looks sharp, and restoring
        would blur it first. Only cancel and failure restore.
        """
        if entry_focus is not None:
            try:
                self._op(self._scope.set_focus_um(float(entry_focus)),
                         "restoring the focus", 8.0)
            except Exception:
                pass
        if entry_z is not None:
            try:
                cur = self._read_z()
                h = getattr(self._ctrl, "z_height_of", None)
                if cur is not None and callable(h) and h(cur) < h(entry_z):
                    self.needle_moves.append(float(entry_z))
                    self._ctrl.move_z_absolute(float(entry_z), from_zero_ref=True)
            except Exception:
                pass


# ════════════════════════════════════════════════════════════════════
#  Step strip
# ════════════════════════════════════════════════════════════════════

class _StepStrip(QWidget):
    """Clickable breadcrumb. Steps are revisitable — the gating that matters is
    on the two buttons that move the needle, not on navigation."""

    step_clicked = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, s(4))
        lay.setSpacing(s(4))
        self._buttons: dict[str, QPushButton] = {}
        for key in STEP_ORDER:
            btn = QPushButton(STEP_TITLES[key])
            btn.setCheckable(True)
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(
                lambda _c=False, k=key: self.step_clicked.emit(k))
            self._buttons[key] = btn
            lay.addWidget(btn)
        lay.addStretch()

    def render(self, current: str, states: dict) -> None:
        """``states`` maps step key → 'ok' | 'warn' | 'todo'."""
        for key, btn in self._buttons.items():
            state = states.get(key, "todo")
            mark = {"ok": "✓ ", "warn": "⚠ ", "todo": ""}[state]
            btn.setText(mark + STEP_TITLES[key])
            btn.setChecked(key == current)
            colour = {"ok": COLORS["green"], "warn": COLORS["yellow"],
                      "todo": COLORS["subtext0"]}[state]
            weight = "700" if key == current else "500"
            btn.setStyleSheet(
                f"QPushButton {{ color: {colour}; font-weight: {weight};"
                f" padding: {sp(6)} {sp(10)}; text-align: left; }}")


# ════════════════════════════════════════════════════════════════════
#  The wizard
# ════════════════════════════════════════════════════════════════════

class NeedleBoreWizard(QWidget):
    """The microscope bore-offset + plate-touch-off steps.

    The host is the ``CalibrationPage``; every datum write and every stage move
    goes back through it, so this widget owns presentation and arithmetic only.
    """

    #: Something was written that the page should persist / re-broadcast.
    calibration_changed = Signal()
    #: Bore offsets changed — the host re-applies them to the live needle.
    bore_offsets_changed = Signal()
    #: v7.13: the current step changed — the host switches its big camera pane
    #: (side cams for steps 1-2, microscope for steps 3-4) on this.
    step_changed = Signal(str)

    def __init__(self, host, parent=None):
        super().__init__(parent)
        self._host = host
        self._meas: Optional[BoreMeasurement] = None
        self._park_xy: Optional[tuple] = None
        self._survey_z_zref: Optional[float] = None
        self._floor_armed = False
        self._active_bore = 0
        self._session_committed = False
        self._live_scores: dict[int, float] = {}
        #: v7.13: sticky inline refusal shown on the status line until the next
        #: step change / successful action — modals are gone from refusals.
        self._refusal_text: Optional[str] = None
        # ── step 2 state ──
        self._s2_offset_user_edited = False       # a typed offset wins autofill
        self._s2_offset_setting = False           # programmatic-set guard
        # ── optical plate-bottom state ──
        self._opt_f0: Optional[float] = None      # on-glass focus, µm
        self._opt_rungs: list = []                # accepted RungMeasurements
        self._opt_pending = None                  # awaiting confirm/override
        self._opt_worker = None
        #: (best_score, focus_um) seen live while the operator jogs. A running
        #: max, NOT a fitted peak — reported separately and never given a σ.
        self._opt_live: Optional[tuple] = None
        self._build_ui()
        # Open on the first step that still needs doing (not mid-procedure).
        # _enter_step deliberately does NOT run here: arming the floor and
        # starting the focus timer belong to showEvent, so a wizard built on a
        # hidden page commands nothing.
        self._current = self._initial_step()
        self._stack.setCurrentWidget(self._pages[self._current])
        self._timer = QTimer(self)
        self._timer.setInterval(FOCUS_TICK_MS)
        self._timer.timeout.connect(self._focus_tick)

    # ── construction ─────────────────────────────────────────────

    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(8), s(8), s(8), s(8))
        outer.setSpacing(s(6))

        self._strip = _StepStrip()
        self._strip.step_clicked.connect(self.go_to_step)
        outer.addWidget(self._strip)

        # Back / Next. Navigation stays free (the strip philosophy): Next is
        # never gated — a blocked DESTINATION renders its refusal inline, which
        # beats a dead button with no explanation.
        nav = QHBoxLayout()
        self._btn_back = QPushButton("← Back")
        self._btn_back.clicked.connect(self._on_back)
        nav.addWidget(self._btn_back)
        self._btn_next = QPushButton("Next →")
        self._btn_next.setObjectName("accentBtn")
        self._btn_next.clicked.connect(self._on_next)
        nav.addWidget(self._btn_next)
        nav.addStretch()
        outer.addLayout(nav)

        # ONE instruction (bold: what to do) + ONE status (how it's going /
        # why it's blocked). Every other text surface ranks below these two.
        self._instruction = QLabel("—")
        self._instruction.setWordWrap(True)
        self._instruction.setStyleSheet(
            f"color: {COLORS['text']}; font-weight: 600;")
        outer.addWidget(self._instruction)

        self._status = QLabel("—")
        self._status.setWordWrap(True)
        self._status.setStyleSheet(f"color: {COLORS['subtext0']};")
        outer.addWidget(self._status)

        self._stack = QStackedWidget()
        self._pages = {
            STEP_NEEDLE_ZERO: self._build_step1(),
            STEP_Z_REFS: self._build_step2(),
            STEP_BORES: self._build_step3(),
            STEP_TOUCHOFF: self._build_step4(),
        }
        for key in STEP_ORDER:
            self._stack.addWidget(self._pages[key])
        outer.addWidget(self._stack, stretch=1)

        # The microscope pane is BUILT here but MOUNTED by the host (into its
        # big camera stack, via microscope_pane()). Parent it to the wizard
        # hidden so a host that never mounts it leaks nothing and no top-level
        # window can float; addWidget() on the host's stack reparents it.
        pane = self._build_microscope_feed()
        self._mic_pane = pane
        if pane is not None:
            pane.setParent(self)
            pane.hide()

    def _build_step1(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(6))
        # The host's needle-centring controls mount HERE via set_step_panel(),
        # so the step's working controls are where the operator is looking
        # (they used to live 2000 lines up the page with a note pointing at
        # them). Index 0 of this layout is the slot.
        self._step_panel_slots: dict[str, QVBoxLayout] = getattr(
            self, "_step_panel_slots", {})
        self._step_panel_slots[STEP_NEEDLE_ZERO] = lay
        self._s1_state = QLabel("—")
        self._s1_state.setWordWrap(True)
        lay.addWidget(self._s1_state)
        lay.addStretch()
        return w

    def set_step_panel(self, step: str, panel: QWidget) -> None:
        """Mount a host-built control panel at the top of a step page.

        Optional API — a standalone wizard (unit tests, headless builds) never
        calls it and each page keeps its own state labels. The panel keeps its
        host-side attribute names, so every existing test that binds them by
        name is untouched; only WHERE they render changed.
        """
        slots = getattr(self, "_step_panel_slots", None)
        lay = slots.get(step) if slots else None
        if lay is None or panel is None:
            return
        lay.insertWidget(0, panel)

    def _build_step2(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(6))

        # Row 1 — the plate TOP is what the operator can actually SEE and
        # verify (the tip touching the top surface); the contested bottom is
        # then DERIVED from a known plate dimension instead of taught blind.
        top_row = QHBoxLayout()
        self._s2_set_top = QPushButton("Set plate top = current Z")
        self._s2_set_top.setToolTip(
            "Jog the needle tip down until it just touches the plate's TOP "
            "surface (watch the side cameras), then press this to capture the "
            "current Z as the Plate Top reference.")
        self._s2_set_top.clicked.connect(self._on_set_plate_top)
        top_row.addWidget(self._s2_set_top)
        self._s2_top_lbl = QLabel("—")
        top_row.addWidget(self._s2_top_lbl, 1)
        lay.addLayout(top_row)

        # Row 2 — typed offset, auto-filled from the selected plate type.
        off_row = QHBoxLayout()
        off_row.addWidget(QLabel("Plate bottom is"))
        self._s2_bottom_offset = QDoubleSpinBox()
        self._s2_bottom_offset.setRange(0.05, 30.0)
        self._s2_bottom_offset.setDecimals(3)
        self._s2_bottom_offset.setSingleStep(0.05)
        self._s2_bottom_offset.setSuffix(" mm")
        self._s2_bottom_offset.setValue(1.0)
        self._s2_bottom_offset.setToolTip(
            "How far the plate's inner bottom surface sits BELOW its top "
            "surface — from the plate's datasheet. Auto-filled from the plate "
            "selected on Hardware Setup → Plate when it stores both offsets.")
        self._s2_bottom_offset.valueChanged.connect(self._on_offset_edited)
        off_row.addWidget(self._s2_bottom_offset)
        off_row.addWidget(QLabel("below the plate top"))
        self._s2_offset_refresh = QPushButton("↻ from plate type")
        self._s2_offset_refresh.setToolTip(
            "Re-fill the offset from the selected plate's stored z_offsets "
            "(bottom − top), replacing anything typed here.")
        self._s2_offset_refresh.clicked.connect(
            lambda: self._autofill_bottom_offset(force=True))
        off_row.addWidget(self._s2_offset_refresh)
        off_row.addStretch()
        lay.addLayout(off_row)

        self._s2_offset_hint = QLabel("—")
        self._s2_offset_hint.setWordWrap(True)
        self._s2_offset_hint.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        lay.addWidget(self._s2_offset_hint)

        # Row 3 — derived bottom, applied explicitly (never silently).
        apply_row = QHBoxLayout()
        self._s2_apply_bottom = QPushButton("Apply plate bottom")
        self._s2_apply_bottom.setObjectName("successBtn")
        self._s2_apply_bottom.setToolTip(
            "Set the Plate Bottom Z reference to plate-top − offset (tagged "
            "'estimated' — the optical touch-off on step 4 refines it).")
        self._s2_apply_bottom.clicked.connect(self._on_apply_bottom)
        apply_row.addWidget(self._s2_apply_bottom)
        self._s2_bottom_preview = QLabel("—")
        self._s2_bottom_preview.setWordWrap(True)
        apply_row.addWidget(self._s2_bottom_preview, 1)
        lay.addLayout(apply_row)

        # Row 4 — the safe/travel height, needed by step 3's park gate.
        safe_row = QHBoxLayout()
        self._s2_set_safe = QPushButton("Set fast-move (safe) Z = current Z")
        self._s2_set_safe.setToolTip(
            "Retract the needle to a comfortable travel height, then capture "
            "it. Every cross-position XY move retracts here first.")
        self._s2_set_safe.clicked.connect(self._on_set_safe)
        safe_row.addWidget(self._s2_set_safe)
        safe_row.addStretch()
        lay.addLayout(safe_row)

        row = QHBoxLayout()
        self._s2_autofill = QPushButton("Auto-fill all from plate type")
        self._s2_autofill.setToolTip(
            "Derive ALL the plate Z references (top / bottom / safe / max) "
            "from the needle-cam fiducial and this plate type's stored "
            "offsets. Asks before overwriting a value you taught by hand.")
        self._s2_autofill.clicked.connect(self._on_autofill_z)
        row.addWidget(self._s2_autofill)
        row.addStretch()
        lay.addLayout(row)

        self._s2_state = QLabel("—")
        self._s2_state.setWordWrap(True)
        lay.addWidget(self._s2_state)
        lay.addStretch()
        self._autofill_bottom_offset()
        return w

    def _build_step3(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(6))

        # The offsets are relative to the CAMERA field of view: with the stage
        # still, each bore's click IS its position in the frame, and the
        # differences are the offsets — NO stage move is part of the
        # measurement. So 'Start measuring here' is the primary action: put the
        # needle wherever the bores are visible, by any means (hand jog, Xbox,
        # a previous operation), and click them. The park below is pure
        # convenience for a rig whose needle location is already taught.
        start_row = QHBoxLayout()
        self._s3_start_btn = QPushButton("Start measuring here")
        self._s3_start_btn.setObjectName("successBtn")
        self._s3_start_btn.setToolTip(
            "Begin a measurement session at the CURRENT stage position — no "
            "stage motion is commanded. Put the needle wherever you like; all "
            "that matters is that every bore is visible in one frame and the "
            "stage does not move until the offsets are saved.")
        self._s3_start_btn.clicked.connect(self._on_start_session)
        start_row.addWidget(self._s3_start_btn)
        start_row.addStretch()
        lay.addLayout(start_row)

        park = QHBoxLayout()
        park.addWidget(QLabel("Optional park —"))
        park.addWidget(QLabel("survey height:"))
        self._s3_clearance = QDoubleSpinBox()
        self._s3_clearance.setRange(MIN_SURVEY_CLEARANCE_MM, 10.0)
        self._s3_clearance.setSingleStep(0.05)
        self._s3_clearance.setDecimals(3)
        self._s3_clearance.setSuffix(" mm")
        self._s3_clearance.setValue(DEFAULT_SURVEY_CLEARANCE_MM)
        self._s3_clearance.setToolTip(
            "Height above the plate bottom for the optional park. The bores "
            "are measured here, not at the glass: only DIFFERENCES matter, and "
            "the longest bore reaches lower by exactly the amount being "
            "measured, which is unknown until afterwards.")
        park.addWidget(self._s3_clearance)
        self._s3_park_btn = QPushButton("Go to needle at survey height")
        self._s3_park_btn.setToolTip(
            "Convenience only: safe-travel to the SAVED needle location at the "
            "survey height (retract → XY → lower), then start a session. This "
            "is the one part of step 3 that moves the stage — you never have to "
            "use it.")
        self._s3_park_btn.clicked.connect(self._on_park)
        park.addWidget(self._s3_park_btn)
        park.addStretch()
        lay.addLayout(park)

        # Live drift readout — the stationarity guarantee, visible instead of
        # a modal ambush on the click that violated it.
        self._s3_drift_lbl = QLabel("—")
        self._s3_drift_lbl.setWordWrap(True)
        self._s3_drift_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        lay.addWidget(self._s3_drift_lbl)

        self._s3_rows_host = QWidget()
        self._s3_rows_lay = QVBoxLayout(self._s3_rows_host)
        self._s3_rows_lay.setContentsMargins(0, 0, 0, 0)
        self._s3_rows_lay.setSpacing(s(3))
        lay.addWidget(self._s3_rows_host)
        self._s3_row_widgets: list = []

        btns = QHBoxLayout()
        self._s3_commit = QPushButton("Save bore offsets")
        self._s3_commit.setObjectName("successBtn")
        self._s3_commit.clicked.connect(self._on_commit_bores)
        btns.addWidget(self._s3_commit)
        self._s3_restart = QPushButton("Start over")
        self._s3_restart.clicked.connect(self._on_restart_bores)
        btns.addWidget(self._s3_restart)
        btns.addStretch()
        lay.addLayout(btns)

        self._s3_state = QLabel("—")
        self._s3_state.setWordWrap(True)
        lay.addWidget(self._s3_state)
        lay.addStretch()
        return w

    def _build_step4(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(6))

        mrow = QHBoxLayout()
        mrow.addWidget(QLabel("Method:"))
        self._s4_mode = QComboBox()
        self._s4_mode.addItem("Optical — the tip never touches the glass", "optical")
        self._s4_mode.addItem("Contact touch-off (fallback)", "contact")
        self._s4_mode.setToolTip(
            "Optical measures the plate bottom by focusing on the glass, "
            "raising the focus by a known margin and finding the needle tip at "
            "that plane. The tip never comes closer than the smallest margin.\n\n"
            "Contact is the original gesture: drive the tip down until it "
            "touches. Kept for rigs with no motorised focus or no microscope.")
        self._s4_mode.currentIndexChanged.connect(self._on_s4_mode_changed)
        mrow.addWidget(self._s4_mode, 1)
        lay.addLayout(mrow)

        self._s4_optical_box = self._build_step4_optical()
        lay.addWidget(self._s4_optical_box)

        self._s4_contact_box = self._build_step4_contact()
        lay.addWidget(self._s4_contact_box)

        self._s4_state = QLabel("—")
        self._s4_state.setWordWrap(True)
        lay.addWidget(self._s4_state)
        lay.addStretch()
        self._on_s4_mode_changed()
        return w

    def _build_step4_contact(self) -> QWidget:
        """The original gesture, unchanged — the fallback path must stay
        byte-identical for rigs with no motorised focus."""
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(6))

        note = QLabel(
            "With the microscope focused on the plate bottom, bring the needle "
            "tip down to that plane. The approach is planned against the "
            "LONGEST bore, which step 3 has just measured — that is why this "
            "step comes last.")
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 9pt;")
        lay.addWidget(note)

        row = QHBoxLayout()
        row.addWidget(QLabel("Stop above the longest bore's reach:"))
        self._s4_clearance = QDoubleSpinBox()
        self._s4_clearance.setRange(0.005, 2.0)
        self._s4_clearance.setSingleStep(0.01)
        self._s4_clearance.setDecimals(3)
        self._s4_clearance.setSuffix(" mm")
        self._s4_clearance.setValue(DEFAULT_TOUCH_CLEARANCE_MM)
        row.addWidget(self._s4_clearance)
        self._s4_goto = QPushButton("Go to approx Z bottom")
        self._s4_goto.clicked.connect(self._on_goto_approx_bottom)
        row.addWidget(self._s4_goto)
        row.addStretch()
        lay.addLayout(row)

        self._s4_offset_lbl = QLabel("—")
        self._s4_offset_lbl.setWordWrap(True)
        self._s4_offset_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        lay.addWidget(self._s4_offset_lbl)

        self._s4_update_bottom = QCheckBox(
            "Also update the Plate Bottom Z from this touch-off")
        self._s4_update_bottom.setToolTip(
            "Off by default: this may be a re-touch at a well whose bottom "
            "differs from the taught reference.")
        lay.addWidget(self._s4_update_bottom)

        # THE ONLY PLACE THE OPTICAL Z DATUM CAN BE TAKEN.
        #
        # The datum pairs a microscope focus reading with a needle Z, so it is
        # only meaningful when BOTH are at the same physical plane — which is
        # true here and nowhere else in the calibration. In particular it can
        # NOT be taken while centring the needle in the side cameras: those are
        # mounted far above the focal plane, so the needle tip sits well outside
        # anything the objective can focus on and the focus axis is reading an
        # unrelated position.
        #
        # Ticked by default because this step's own instruction is to focus on
        # the plate bottom first. Untick it if you did not, and the datum is
        # recorded as absent rather than as a number that means nothing.
        self._s4_focus_ok = QCheckBox(
            "The microscope is focused on the plate bottom "
            "(records the optical Z datum)")
        self._s4_focus_ok.setChecked(True)
        self._s4_focus_ok.setToolTip(
            "Pairs the focus-axis reading with this needle Z. Only valid when "
            "the microscope is focused on the same plane the needle tip is "
            "touching. The needle side cameras sit far above the focal plane, "
            "so this datum cannot come from the needle-centring step.")
        lay.addWidget(self._s4_focus_ok)

        cbtn = QHBoxLayout()
        self._s4_confirm = QPushButton("Needle tip is at the plate bottom")
        self._s4_confirm.setObjectName("successBtn")
        self._s4_confirm.clicked.connect(self._on_confirm_touchoff)
        cbtn.addWidget(self._s4_confirm)
        cbtn.addStretch()
        lay.addLayout(cbtn)
        return w

    # ── step 4, optical ──────────────────────────────────────────

    def _build_step4_optical(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(6))

        note = QLabel(
            "Focus on the glass, then let the software raise the focus by each "
            "margin in turn and find the tip at that plane. The tip's height "
            "comes out MEASURED (from where the focus found it), not commanded, "
            "so a wrong starting guess costs nothing — and the tip never comes "
            "closer to the glass than the smallest margin.")
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 9pt;")
        lay.addWidget(note)

        self._s4_guess_lbl = QLabel("—")
        self._s4_guess_lbl.setWordWrap(True)
        self._s4_guess_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 9pt;")
        lay.addWidget(self._s4_guess_lbl)

        frow = QHBoxLayout()
        self._s4_setf0 = QPushButton("Set plate-bottom focus")
        self._s4_setf0.setToolTip(
            "Records the focus-axis reading with the microscope focused on the "
            "glass. Everything else is measured relative to this, so an error "
            "here shifts every margin equally and the agreement check between "
            "margins cannot see it — focus carefully.")
        self._s4_setf0.clicked.connect(self._on_set_focus_zero)
        frow.addWidget(self._s4_setf0)
        self._s4_f0_lbl = QLabel("not set")
        self._s4_f0_lbl.setStyleSheet(f"color: {COLORS['yellow']};")
        frow.addWidget(self._s4_f0_lbl, 1)
        lay.addLayout(frow)

        lrow = QHBoxLayout()
        lrow.addWidget(QLabel("Margins (µm, largest first):"))
        self._s4_margins = QLineEdit()
        self._s4_margins.setToolTip(
            "Each margin is an independent measurement of the same number; "
            "their agreement is the verification. Largest first, because only "
            "the first descent trusts the geometric guess — every later one is "
            "bounded by the measurement before it.")
        self._s4_margins.setText(
            ", ".join(f"{v:.0f}" for v in DEFAULT_OFFSETS_UM)
            if OPTICAL_AVAILABLE else "1000, 500, 200, 100")
        self._s4_margins.editingFinished.connect(self._refresh_optical)
        lrow.addWidget(self._s4_margins, 1)
        lay.addLayout(lrow)

        brow = QHBoxLayout()
        self._s4_measure = QPushButton("Measure next margin")
        self._s4_measure.clicked.connect(self._on_measure_rung)
        brow.addWidget(self._s4_measure)
        self._s4_cancel = QPushButton("Cancel")
        self._s4_cancel.clicked.connect(self._on_cancel_rung)
        brow.addWidget(self._s4_cancel)
        brow.addStretch()
        lay.addLayout(brow)

        # THE RESULT — the constant focal-plane ↔ needle-tip offset. The whole
        # point of the focal scan: pair f (the objective focal drive, µm) with
        # the needle Z when the focal plane is ON the tip, and the offset
        # between the two frames falls out, once per rung. Constancy across
        # rungs is the verification.
        self._s4_result = QLabel("—")
        self._s4_result.setWordWrap(True)
        self._s4_result.setStyleSheet(
            f"color: {COLORS['text']}; font-weight: 600;")
        self._s4_result.setToolTip(
            "needle_z_zref = focal_sign × (focus_µm / 1000) + offset. Every "
            "rung pairs a fitted tip focus with the needle Z read back at that "
            "moment, so each is an independent estimate of the same constant; "
            "the median is reported with the spread across rungs.")
        lay.addWidget(self._s4_result)

        # Live "best so far" while the operator jogs by hand. A running max over
        # whatever heights they happened to visit — accurate to about half a jog
        # step, which is why it is reported separately from the fitted peak.
        self._s4_live = QLabel("—")
        self._s4_live.setWordWrap(True)
        self._s4_live.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 9pt;")
        lay.addWidget(self._s4_live)

        self._s4_log = QTextEdit()
        self._s4_log.setReadOnly(True)
        self._s4_log.setMaximumHeight(s(110))
        lay.addWidget(self._s4_log)

        arow = QHBoxLayout()
        self._s4_use_live = QPushButton("Use best-so-far instead")
        self._s4_use_live.setToolTip(
            "Override the fitted peak with the sharpest frame you have jogged "
            "through. Records a training image labelled with this needle type "
            "and the difference from the software's pick.")
        self._s4_use_live.clicked.connect(self._on_use_live_focus)
        arow.addWidget(self._s4_use_live)
        self._s4_accept = QPushButton("Accept plate bottom")
        self._s4_accept.setObjectName("successBtn")
        self._s4_accept.clicked.connect(self._on_accept_optical)
        arow.addWidget(self._s4_accept)
        arow.addStretch()
        lay.addLayout(arow)
        return w

    def _on_s4_mode_changed(self, *_a) -> None:
        optical = self._s4_optical_mode()
        if getattr(self, "_s4_optical_box", None) is not None:
            self._s4_optical_box.setVisible(optical)
        if getattr(self, "_s4_contact_box", None) is not None:
            self._s4_contact_box.setVisible(not optical)
        # This also runs from _build_step4, i.e. part-way through _build_ui.
        # `_pages` is assigned only once every page has been built, so it is the
        # accurate "construction finished" signal — refreshing before that
        # renders half a widget and asks the gates for a controller the caller
        # has not necessarily injected yet.
        if getattr(self, "_pages", None):
            self.refresh()

    def _s4_optical_mode(self) -> bool:
        box = getattr(self, "_s4_mode", None)
        return True if box is None else box.currentData() == "optical"

    # ── host access, all getattr-guarded so a partial host is usable ──

    def _ctrl(self):
        return getattr(self._host, "controller", None)

    def _needle(self):
        hw = getattr(self._host, "_hardware_config", None)
        return getattr(hw, "needle", None) if hw is not None else None

    def _bore_count(self) -> int:
        n = self._needle()
        try:
            return max(1, int(n.bore_count))
        except (AttributeError, TypeError, ValueError):
            return 0 if n is None else 1

    def _store(self):
        if not BORE_STORE_AVAILABLE:
            return None
        try:
            return _get_bore_store()
        except Exception as e:                       # pragma: no cover
            logger.debug(f"bore store unavailable: {e}")
            return None

    def _mic_cam_idx(self):
        fn = getattr(self._host, "_ploc_microscope_cam_idx", None)
        try:
            return fn() if callable(fn) else None
        except Exception:
            return None

    def _build_microscope_feed(self):
        """The wizard's own microscope feed, or None in a headless/import-poor
        build (the arithmetic and the gates are still exercisable without it)."""
        try:
            from gui.widgets.camera_feed_view import CameraFeedView
        except ImportError:                           # pragma: no cover
            self._mic_feed = None
            return None
        box = QGroupBox("Microscope — click each bore tip")
        box.setStyleSheet(SECTION_TITLE_STYLE)
        blay = QVBoxLayout(box)
        blay.setContentsMargins(s(4), s(4), s(4), s(4))
        # v7.13: bore-dots overlay toggle. Hidden until a dot source exists (a
        # session in progress, or stored offsets that can be projected); turned
        # ON automatically when offsets are committed.
        head = QHBoxLayout()
        self._bore_dots_chk = QCheckBox("Show bore dots")
        self._bore_dots_chk.setToolTip(
            "Overlay a colored dot per bore at its measured position in the "
            "camera view (datum green, then one colour per bore).")
        self._bore_dots_chk.setChecked(True)
        self._bore_dots_chk.setVisible(False)
        self._bore_dots_chk.toggled.connect(
            lambda _c=False: self._push_bore_markers())
        head.addWidget(self._bore_dots_chk)
        head.addStretch()
        blay.addLayout(head)
        idx = self._mic_cam_idx()
        self._mic_feed = CameraFeedView(
            camera_manager=self._camera_manager(),
            cam_idx=0 if idx is None else int(idx),
            show_crosshair=True,
            # Apply the camera's calibrated mirror/rotation to the DISPLAY.
            # Clicks are still reported in raw frame coords, so
            # pixel_to_stage_offset stays valid — that separation is what makes
            # a rotated scope safe to click on.
            auto_orient=True,
            label="Microscope feed — starts on steps 3-4",
        )
        self._mic_feed.clicked.connect(self.on_view_clicked)
        blay.addWidget(self._mic_feed, stretch=1)
        # No minimum height here (v7.13): the host mounts this pane into its
        # large camera stack, which provides the height — a floor would only
        # fight the splitter.
        return box

    def microscope_pane(self) -> Optional[QWidget]:
        """The microscope groupbox for the HOST to mount into its big camera
        pane (addWidget reparents it). None in a headless build."""
        return getattr(self, "_mic_pane", None)

    def _mic_view(self):
        """The live microscope feed this wizard clicks on."""
        return getattr(self, "_mic_feed", None)

    def _camera_manager(self):
        return getattr(self._host, "_camera_manager", None)

    # ── gates ────────────────────────────────────────────────────

    def gate(self, step: str) -> tuple:
        """``(ok, why_not)`` for the step's own action.

        ⚠ For step 3 this is the gate on a MEASUREMENT, which commands no
        motion at all — so it asks only for what turns a click into a distance:
        a µm/px-calibrated microscope, a configured needle, and a readable stage
        position (the drift guard). It deliberately does NOT require the plate
        bottom or the safe Z. Those are needed only by the optional park
        *travel*, and are gated separately by :meth:`park_gate`; requiring them
        here made an un-taught reference height block a procedure that never
        moves the stage, forcing the operator through step 2 to click two dots
        in one camera frame.
        """
        ctrl = self._ctrl()
        if step == STEP_NEEDLE_ZERO:
            if ctrl is None or not ctrl.is_xy_connected:
                return False, "Connect the XY stage."
            return True, ""
        if step == STEP_Z_REFS:
            return True, ""
        if step in (STEP_BORES, STEP_TOUCHOFF):
            if not BORE_STORE_AVAILABLE:
                return False, "The bore-offset calibration store is unavailable."
            if ctrl is None or not ctrl.is_xy_connected:
                return False, ("Connect the XY stage — the offsets are "
                               "referenced to the stage position.")
            if self._needle() is None:
                return False, ("No needle is configured. Set it on Hardware "
                               "Setup → Needle first.")
            mic = self._mic_cam_idx()
            if mic is None:
                return False, ("No camera has the Microscope role — assign it "
                               "on Hardware Setup → Cameras.")
            mgr = self._camera_manager()
            if mgr is None or not mgr.is_um_per_px_calibrated(mic):
                return False, ("The microscope camera is not µm/px-calibrated — "
                               "run 'Calibrate µm/px' for it. Without a scale a "
                               "click cannot become a distance.")
        if step == STEP_TOUCHOFF:
            # Step 4 genuinely descends toward the glass, so it keeps both
            # reference heights as hard requirements.
            if getattr(self._host, "_plate_bottom_z", None) is None:
                return False, ("Teach the Plate Bottom Z on step 2 — the "
                               "approach height is measured from it.")
            if getattr(self._host, "_safe_z", None) is None:
                return False, ("Set the Fast Move (Safe) Z on step 2 — travel "
                               "retracts to it first.")
        return True, ""

    def park_gate(self) -> tuple:
        """``(ok, why_not)`` for step 3's OPTIONAL park travel.

        Separate from :meth:`gate` because this one moves the stage: it needs a
        destination (the saved needle location), a height measured from the
        plate bottom, and a safe Z to retract to first. A failure here disables
        the park button and nothing else — the click session stays available,
        because measuring bore offsets never required any of this.
        """
        ok, why = self.gate(STEP_BORES)
        if not ok:
            return False, why
        if getattr(self._host, "_plate_bottom_z", None) is None:
            return False, ("Teach the Plate Bottom Z on step 2 — the park "
                           "height is measured from it. (Not needed to measure: "
                           "jog the bores into view and press "
                           "'Start measuring here'.)")
        if getattr(self._host, "_safe_z", None) is None:
            return False, ("Set the Fast Move (Safe) Z on step 2 — the park "
                           "retracts to it first. (Not needed to measure: jog "
                           "the bores into view and press 'Start measuring here'.)")
        if not getattr(self._host, "_needle_loc_xy_um", None):
            return False, ("No saved needle location — complete step 1, or use "
                           "'Set current as needle center', so the park knows "
                           "where to go. (Not needed to measure: jog the bores "
                           "into view and press 'Start measuring here'.)")
        if not callable(getattr(self._host, "_safe_navigate_to", None)):
            return False, "This build cannot travel safely."
        return True, ""

    def step_state(self, step: str) -> str:
        """'ok' | 'warn' | 'todo' for the strip."""
        if step == STEP_NEEDLE_ZERO:
            return "ok" if getattr(self._host, "_needle_origin_um", None) else "todo"
        if step == STEP_Z_REFS:
            have = (getattr(self._host, "_plate_bottom_z", None) is not None
                    and getattr(self._host, "_safe_z", None) is not None)
            return "ok" if have else "todo"
        if step == STEP_BORES:
            store = self._store()
            if store is None:
                return "todo"
            # An in-progress, uncommitted session shows as ⚠ so the strip
            # tracks the clicks as they land (the store only changes on save).
            if (self._meas is not None and self._meas.pto
                    and not getattr(self, "_session_committed", False)):
                return "warn"
            n = self._bore_count()
            if n <= 1:
                ctrl = self._ctrl()
                have = (ctrl is not None
                        and getattr(ctrl, "get_needle_camera_offset_um", None)
                        and ctrl.get_needle_camera_offset_um())
                return "ok" if have else "todo"
            measured = set(store.measured_bore_indices())
            if all(k in measured for k in range(n)):
                return "ok"
            return "warn" if measured else "todo"
        if step == STEP_TOUCHOFF:
            return "ok" if getattr(self, "_touchoff_done", False) else "todo"
        return "todo"

    # ── navigation ───────────────────────────────────────────────

    #: Steps during which the needle may be jogged toward the glass.
    _FLOOR_STEPS = (STEP_BORES, STEP_TOUCHOFF)

    @property
    def current_step(self) -> str:
        """The step key the wizard is on (for the host's camera-pane sync)."""
        return self._current

    def _initial_step(self) -> str:
        """The first step that still needs doing — where the wizard opens.

        Falls back to the last step when everything reads complete (the
        operator is probably back to re-verify).
        """
        for key in STEP_ORDER:
            try:
                if self.step_state(key) != "ok":
                    return key
            except Exception:                         # pragma: no cover
                return key
        return STEP_ORDER[-1]

    def go_to_step(self, step: str) -> None:
        if step not in STEP_ORDER:
            return
        self._refusal_text = None
        self._current = step
        self._stack.setCurrentWidget(self._pages[step])
        self._enter_step(step)
        self.step_changed.emit(step)
        self.refresh()

    def _on_back(self) -> None:
        i = STEP_ORDER.index(self._current)
        if i > 0:
            self.go_to_step(STEP_ORDER[i - 1])

    def _on_next(self) -> None:
        i = STEP_ORDER.index(self._current)
        if i < len(STEP_ORDER) - 1:
            self.go_to_step(STEP_ORDER[i + 1])

    def _refuse(self, text: str) -> None:
        """Inline refusal: sticky yellow status text, never a modal.

        Cleared on the next step change / session start / successful action —
        until then ``refresh()`` keeps showing it, so a 300 ms tick cannot
        blink it away before it is read.
        """
        self._refusal_text = str(text)
        self._status.setText(f"⚠ {self._refusal_text}")
        self._status.setStyleSheet(f"color: {COLORS['yellow']};")

    def showEvent(self, event):                       # noqa: N802 (Qt override)
        """Re-enter the current step on show.

        ``hideEvent`` disarms the print floor; without this, re-showing the
        page left the floor OFF until the operator happened to click a strip
        button — a real safety gap, since steps 3/4 jog the needle toward the
        glass. Also (re)announces the step so the host's camera pane syncs.
        """
        super().showEvent(event)
        self._enter_step(self._current)
        self.step_changed.emit(self._current)
        self.refresh()

    def _enter_step(self, step: str) -> None:
        """Set the armed state from the DESTINATION step, in one place.

        Deliberately not a leave-then-enter pair: moving between the two armed
        steps would disarm and immediately re-arm, and however brief that gap
        is, it is a window in which the plate-bottom clamp is off. Computing the
        desired state once means the floor simply stays on across step 3 → 4.
        """
        armed = step in self._FLOOR_STEPS
        if armed:
            self._ensure_microscope()
            self._arm_floor(True)
            self._timer.start()
        else:
            self._timer.stop()
            self._arm_floor(False)

    def hideEvent(self, event):                       # noqa: N802 (Qt override)
        """Never leave the floor armed behind a hidden page.

        The refcount added in v7.10 means an unbalanced disarm here cannot
        disable a concurrent print's protection, but leaving it ARMED would
        silently clamp an unrelated later operation — so disarm unconditionally.

        The optical worker is torn down first: it holds the microscope lease and
        moves the focus, and a hidden page must not leave either in flight.
        """
        self._teardown_rung_worker()
        self._timer.stop()
        self._arm_floor(False)
        super().hideEvent(event)

    def _arm_floor(self, on: bool) -> None:
        """Arm/disarm the plate-bottom clamp, exactly once per state change."""
        if bool(on) == self._floor_armed:
            return
        ctrl = self._ctrl()
        fn = getattr(ctrl, "set_print_floor_active", None) if ctrl else None
        if not callable(fn):
            return
        try:
            fn(bool(on))
            self._floor_armed = bool(on)
        except Exception as e:                        # pragma: no cover
            logger.debug(f"print floor {on} failed: {e}")

    def _ensure_microscope(self) -> None:
        """Point this wizard's feed at the microscope slot and start it.

        Mirrors ``CalibrationPage._zoff_ensure_live_camera``: ``set_camera`` is
        re-issued only on a real change, so the feed does not flicker, and
        ``start`` is a no-op when the camera is already running.

        Nothing is ever STOPPED here — the camera is shared with other surfaces
        (the Plate Z Auto-Cal feed, the Rosettes tab, the mosaic scans), and
        stopping it on the way out would break whichever of those is live.
        """
        view, mgr, idx = self._mic_view(), self._camera_manager(), self._mic_cam_idx()
        if view is None or mgr is None or idx is None:
            return
        try:
            if view.cam_idx != idx:
                view.set_camera(idx)
            mgr.start(idx)
        except Exception as e:                        # pragma: no cover
            logger.debug(f"microscope start skipped: {e}")

    # ── step 2 ───────────────────────────────────────────────────

    def _on_autofill_z(self) -> None:
        host = self._host
        taught = [n for n, a in (("Plate Top Z", "_top_z"),
                                 ("Plate Bottom Z", "_plate_bottom_z"),
                                 ("Fast Move Z", "_safe_z"),
                                 ("Max Z", "_max_z"))
                  if getattr(host, a, None) is not None]
        if taught:
            resp = QMessageBox.question(
                self, "Auto-fill reference Z",
                "Overwrite the reference heights from this plate type's saved "
                "offsets?\n\nAlready set: " + ", ".join(taught) +
                "\n\nA value you taught by hand is more trustworthy than a "
                "stored guess — this replaces it.",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No)
            if resp != QMessageBox.StandardButton.Yes:
                return
        fn = getattr(host, "_apply_plate_type_z_estimates", None)
        if not callable(fn):
            self._refuse("This build cannot auto-fill the Z references.")
            return
        try:
            fn(force=True)
        except Exception as e:
            self._refuse(f"Auto-fill failed: {e}")
            return
        self._refusal_text = None
        self.calibration_changed.emit()
        self.refresh()

    def _z_up_sign(self) -> float:
        """The per-machine up sign — NOT print_z_dir(), which is derived from
        the very top/bottom pair step 2 is establishing (circular)."""
        ctrl = self._ctrl()
        fn = getattr(ctrl, "z_up_sign", None) if ctrl else None
        try:
            return float(fn()) if callable(fn) else 1.0
        except Exception:
            return 1.0

    def _on_set_plate_top(self) -> None:
        """Capture the current Z as the plate TOP reference.

        Emits ``calibration_changed`` explicitly: the host's ``_set_top_z``
        does not emit on its own, and without the emit the controller never
        receives the new top (app.py pushes it on the changed signal) — so
        ``print_z_dir()`` would keep reading a stale pair.
        """
        host = self._host
        fn = getattr(host, "_zoff_set_top_z", None)
        if not callable(fn):
            self._refuse("This build cannot capture the plate top.")
            return
        try:
            fn()
        except Exception as e:
            self._refuse(f"Could not capture the plate top: {e}")
            return
        if getattr(host, "_top_z", None) is None:
            self._refuse("Could not read the needle Z — is the Z board "
                         "connected?")
            return
        self._refusal_text = None
        self.calibration_changed.emit()
        self.refresh()

    def _on_set_safe(self) -> None:
        host = self._host
        fn = getattr(host, "_zoff_set_safe_z", None)
        if not callable(fn):
            self._refuse("This build cannot capture the fast-move Z.")
            return
        try:
            fn()
        except Exception as e:
            self._refuse(f"Could not capture the fast-move Z: {e}")
            return
        if getattr(host, "_safe_z", None) is None:
            self._refuse("Could not read the needle Z — is the Z board "
                         "connected?")
            return
        self._refusal_text = None
        self.refresh()

    def _plate_offsets(self) -> dict:
        """The selected plate's stored z_offsets (mm below the fiducial), {}
        when none resolve."""
        hw = getattr(self._host, "_hardware_config", None)
        fn = getattr(hw, "plate_z_offsets", None) if hw is not None else None
        try:
            return dict(fn() or {}) if callable(fn) else {}
        except Exception:
            return {}

    def _plate_top_bottom_offset_mm(self):
        """Plate top→bottom distance from the selected plate type, or None.

        Both stored offsets are mm BELOW the same needle-cam fiducial, so their
        difference is the physical top→bottom distance — fiducial-independent,
        which is what makes it safe to auto-fill without the fiducial being
        captured.
        """
        offs = self._plate_offsets()
        top, bottom = offs.get("top"), offs.get("bottom")
        if top is None or bottom is None:
            return None
        try:
            d = float(bottom) - float(top)
        except (TypeError, ValueError):
            return None
        return d if d > 0.0 else None

    def _on_offset_edited(self, _value=None) -> None:
        """A typed offset wins: autofill never silently overwrites it."""
        if not self._s2_offset_setting:
            self._s2_offset_user_edited = True
        self._render_step2()

    def _autofill_bottom_offset(self, force: bool = False) -> None:
        """Fill the bottom-offset spin from the plate type's (bottom − top).

        ``force`` (the ↻ button) replaces a typed value; otherwise a user edit
        is never overwritten. Missing/degenerate stored offsets leave the field
        editable with a hint saying to enter the datasheet number.
        """
        spin = getattr(self, "_s2_bottom_offset", None)
        hint = getattr(self, "_s2_offset_hint", None)
        if spin is None:
            return
        d = self._plate_top_bottom_offset_mm()
        if d is None:
            if hint is not None:
                hint.setText(
                    "No stored top→bottom offset for this plate — enter the "
                    "distance from the plate's datasheet.")
                hint.setStyleSheet(
                    f"color: {COLORS['yellow']}; font-size: 9pt;")
            return
        if self._s2_offset_user_edited and not force:
            return
        self._s2_offset_setting = True
        try:
            spin.setValue(float(d))
        finally:
            self._s2_offset_setting = False
        if force:
            self._s2_offset_user_edited = False
        if hint is not None:
            hint.setText(
                f"Auto-filled from the selected plate's stored offsets "
                f"(bottom − top = {d:.3f} mm). Edit if the datasheet says "
                f"otherwise.")
            hint.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: 9pt;")

    def _computed_bottom_zref(self):
        """Plate bottom implied by the captured top + typed offset, or None."""
        top = getattr(self._host, "_top_z", None)
        spin = getattr(self, "_s2_bottom_offset", None)
        if top is None or spin is None:
            return None
        return plate_bottom_zref_from_top(
            float(top), float(spin.value()), self._z_up_sign())

    def _on_apply_bottom(self) -> None:
        z = self._computed_bottom_zref()
        if z is None:
            self._refuse("Capture the plate top first — the bottom is derived "
                         "from it.")
            return
        fn = getattr(self._host, "_zoff_apply_plate_bottom_z", None)
        if not callable(fn):
            self._refuse("This build cannot apply a derived plate bottom.")
            return
        try:
            fn(float(z), source="estimated")
        except TypeError:                     # older host without source=
            fn(float(z))
        except Exception as e:
            self._refuse(f"Could not apply the plate bottom: {e}")
            return
        self._refusal_text = None
        self.refresh()

    # ── step 3 ───────────────────────────────────────────────────

    def _survey_target_zref(self):
        """Zero-ref Z for the park, floored so it can never approach the glass."""
        host = self._host
        bottom = getattr(host, "_plate_bottom_z", None)
        if bottom is None:
            return None
        clearance = max(MIN_SURVEY_CLEARANCE_MM, float(self._s3_clearance.value()))
        ctrl = self._ctrl()
        fn = getattr(ctrl, "print_height_to_zref", None) if ctrl else None
        if callable(fn):
            try:
                return float(fn(clearance))
            except Exception:
                pass
        # Polarity-safe fallback: never a raw literal, always through the sign.
        sign = 1.0
        try:
            sign = float(ctrl.print_z_dir())
        except Exception:
            pass
        return float(bottom) + sign * clearance

    def _on_park(self) -> None:
        """OPTIONAL convenience: drive to the saved needle location.

        Gated by :meth:`park_gate`, not :meth:`gate` — this is the only part of
        step 3 that moves the stage, so it is the only part that needs a
        destination and the reference heights.
        """
        ok, why = self.park_gate()
        if not ok:
            self._refuse(why)
            return
        target = self._survey_target_zref()
        if target is None:
            self._refuse("The plate bottom Z is not taught — step 2.")
            return
        xy = getattr(self._host, "_needle_loc_xy_um", None)
        nav = getattr(self._host, "_safe_navigate_to", None)
        # safe_travel_to: retract → wait → XY → wait → lower. Never a bare
        # move_z followed by a move_xy; that is the dragged-needle failure.
        self._s3_park_btn.setEnabled(False)
        try:
            moved = nav(float(xy[0]), float(xy[1]), target_z_mm=target,
                        lower_z=True)
        finally:
            self._s3_park_btn.setEnabled(True)
        # v7.9.1: a refused/unconfirmed travel used to be swallowed — the park
        # reported nothing, then armed a measurement session whose whole
        # validity rests on the stage being where it thinks it is. The most
        # common cause is a stale Fast-Move Z (captured against a previous Z
        # datum), which is why that is named here.
        if moved is False:
            self._refuse(
                "The stage did not reach the survey height — the travel was "
                "refused or could not be confirmed (see the log). The usual "
                "cause is a stale Fast Move / Safe Z: re-teach it on "
                "Calibration → Needle Location → Advanced Z references, then "
                "try again.")
            return
        self._survey_z_zref = target
        self._on_start_session()

    def _on_start_session(self) -> None:
        """Begin a measurement session at the CURRENT stage position.

        The offsets are camera-frame differences, so WHERE the stage sits is
        irrelevant — what matters is that it does not move between clicks. The
        session records the reference XY here; the drift readout and the click
        gate are both measured against it. Reached from the park (after
        arriving) or directly ('Start measuring here') when the operator jogged the
        bores into view themselves.
        """
        ok, why = self.gate(STEP_BORES)
        if not ok:
            self._refuse(why)
            return
        self._refusal_text = None
        self._park_xy = self._read_xy()
        self._meas = BoreMeasurement(self._bore_count())
        self._active_bore = 0
        self._session_committed = False
        self.refresh()

    def _read_xy(self):
        ctrl = self._ctrl()
        try:
            xy = ctrl.get_xy_position(cached=False)
            if xy and xy[0] is not None and xy[1] is not None:
                return (float(xy[0]), float(xy[1]))
        except Exception:
            pass
        return None

    def _stage_drift_um(self):
        """How far the stage has moved since the park, or None if unknown."""
        if self._park_xy is None:
            return None
        now = self._read_xy()
        if now is None:
            return None
        return max(abs(now[0] - self._park_xy[0]),
                   abs(now[1] - self._park_xy[1]))

    def _read_z_user(self):
        ctrl = self._ctrl()
        try:
            raw = ctrl.capture_current_z_raw()
            if raw is None:
                return None
            return float(ctrl.raw_to_user_z(raw))
        except Exception:
            return None

    def on_view_clicked(self, px: float, py: float) -> None:
        """A click on the microscope feed — record the active bore's tip.

        The one-frame method's validity rests on the stage being stationary, so
        a drift since the session started is refused rather than silently
        folded in — inline (status + drift readout), never a modal.
        """
        if self._current != STEP_BORES or self._meas is None:
            return
        drift = self._stage_drift_um()
        if drift is not None and drift > MAX_STAGE_DRIFT_UM:
            self._refuse(
                f"Click ignored — the stage has moved {drift:.0f} µm since "
                f"the session started. The offsets are differences taken in "
                f"ONE frame, so every click must be at the same stage "
                f"position. Press 'Start measuring here' to re-anchor and re-click "
                f"every bore.")
            self._update_drift_label()
            return
        view = self._mic_view()
        mgr = self._camera_manager()
        mic = self._mic_cam_idx()
        if view is None or mgr is None or mic is None:
            return
        img_w, img_h = view.image_size
        if not img_w or not img_h:
            return
        try:
            pto = mgr.pixel_to_stage_offset(mic, px, py, img_w, img_h)
        except Exception as e:
            logger.debug(f"pixel_to_stage_offset failed: {e}")
            return
        self._refusal_text = None
        self._session_committed = False
        self._meas.record_click(self._active_bore, (px, py), pto)
        # Advance to the next unclicked bore so the operator can click straight
        # down the list without touching the radio buttons.
        for k in range(self._bore_count()):
            if k not in self._meas.pto:
                self._active_bore = k
                break
        self.refresh()

    def _on_record_z(self, bore_index: int) -> None:
        if self._meas is None:
            return
        z = self._read_z_user()
        if z is None:
            self._refuse("Could not read the needle Z — is the Z board "
                         "connected?")
            return
        self._meas.record_z(bore_index, z)
        self.refresh()

    def _on_use_peak(self, bore_index: int) -> None:
        """Adopt the sharpest height seen live. A readout, not an autofocus —
        no stage motion is commanded."""
        if self._meas is None:
            return
        peak = self._meas.peak.get(int(bore_index))
        if peak is None:
            return
        self._meas.record_z(bore_index, peak[1])
        self.refresh()

    def _on_redo_bore(self, bore_index: int) -> None:
        if self._meas is None:
            return
        self._meas.forget(bore_index)
        self._active_bore = int(bore_index)
        self.refresh()

    def _on_restart_bores(self) -> None:
        self._meas = BoreMeasurement(self._bore_count())
        self._active_bore = 0
        self._session_committed = False
        self.refresh()

    def _commit_blockers(self) -> list:
        """Why 'Save bore offsets' would refuse right now — [] when ready.

        Computed by ``refresh()`` to DISABLE the button with the first reason
        as its tooltip, so the operator never has to click to find out.
        """
        meas, store, needle = self._meas, self._store(), self._needle()
        if store is None or needle is None:
            return ["The bore-offset calibration store is unavailable."]
        if meas is None:
            return ["Start a measurement session first."]
        if not meas.datum_clicked:
            return ["Click bore 1 first — it is the datum every other bore's "
                    "offset is measured from."]
        missing = meas.missing_clicks()
        if missing:
            return ["Still to click: bore "
                    + ", ".join(str(k + 1) for k in missing)
                    + ". An unmeasured bore is treated as sitting exactly on "
                    "the datum, so it would miss its target by the real "
                    "spacing."]
        # Plausibility BEFORE writing: a clamped offset is a wrong move that
        # looks right, so refuse and name the value.
        try:
            from SupportClasses.PhysicalModels import (
                MAX_BORE_OFFSET_UM, MAX_BORE_Z_OFFSET_MM)
        except ImportError:                           # pragma: no cover
            MAX_BORE_OFFSET_UM, MAX_BORE_Z_OFFSET_MM = 2000.0, 1.0
        out = []
        for k in range(1, meas.bore_count):
            off = meas.offset_for(k) or (0.0, 0.0)
            if max(abs(off[0]), abs(off[1])) > MAX_BORE_OFFSET_UM:
                out.append(
                    f"Bore {k + 1}'s measured offset is ({off[0]:+.0f}, "
                    f"{off[1]:+.0f}) µm, beyond the "
                    f"{MAX_BORE_OFFSET_UM:.0f} µm limit for a fused assembly. "
                    f"Check the clicks landed on the right tips.")
            if abs(meas.dz_for(k)) > MAX_BORE_Z_OFFSET_MM:
                out.append(
                    f"Bore {k + 1}'s measured Z offset is "
                    f"{meas.dz_for(k):+.3f} mm, beyond the "
                    f"{MAX_BORE_Z_OFFSET_MM:.1f} mm limit. Check that each Z "
                    f"was recorded on its OWN bore's focus peak.")
        return out

    def _on_commit_bores(self) -> None:
        meas, store, needle = self._meas, self._store(), self._needle()
        if meas is None or store is None or needle is None:
            return
        blockers = self._commit_blockers()
        if blockers:
            self._refuse(blockers[0])
            return

        # Re-measuring the datum re-bases every other bore, so the old set is
        # meaningless — same reasoning as the side-camera path.
        store.clear()
        store.set_bore(0, (0.0, 0.0), 0.0, stage_um=meas.pto.get(0),
                       z_user_mm=meas.z_user.get(0), needle=needle)
        for k in range(1, meas.bore_count):
            store.set_bore(k, meas.offset_for(k) or (0.0, 0.0), meas.dz_for(k),
                           stage_um=meas.pto.get(k),
                           z_user_mm=meas.z_user.get(k), needle=needle)

        # Push onto the live needle HERE rather than relying on a listener.
        # The store is the authority and it has just been written, so the needle
        # the motion path reads must agree immediately — whether or not a host
        # happens to be connected to `bore_offsets_changed`. `apply_to_needle`
        # is idempotent, so the host re-applying afterwards is harmless.
        try:
            store.apply_to_needle(needle)
        except Exception as e:                       # pragma: no cover
            logger.debug(f"apply_to_needle after commit failed: {e}")

        cam_off = meas.needle_camera_offset()
        ctrl = self._ctrl()
        setter = getattr(ctrl, "set_needle_camera_offset_um", None) if ctrl else None
        if cam_off is not None and callable(setter):
            setter(cam_off[0], cam_off[1])

        logger.info("[bore-wizard] committed %d bore(s); needle↔camera offset "
                    "(%.1f, %.1f) µm",
                    meas.bore_count, *(cam_off or (0.0, 0.0)))
        self._refusal_text = None
        self._session_committed = True
        # The dots are the visual proof of what was just committed — turn the
        # overlay on so the operator sees them land on the bores immediately.
        chk = getattr(self, "_bore_dots_chk", None)
        if chk is not None:
            chk.setChecked(True)
        self.bore_offsets_changed.emit()
        self.calibration_changed.emit()
        self.refresh()

    # ── step 4 ───────────────────────────────────────────────────

    def _longest_bore_mm(self) -> float:
        n = self._needle()
        try:
            return max(0.0, float(n.max_bore_z_offset_mm))
        except (AttributeError, TypeError, ValueError):
            return 0.0

    def _touch_target_zref(self):
        """Approach height, planned against the LOWEST-reaching bore.

        Without the longest-bore term a bore protruding further than the datum
        ends up below the glass while the datum sits at its nominal clearance —
        the mistake ``PickAndPlaceManager._descend_z_mm`` exists to prevent.
        """
        bottom = getattr(self._host, "_plate_bottom_z", None)
        if bottom is None:
            return None
        h = self._longest_bore_mm() + float(self._s4_clearance.value())
        ctrl = self._ctrl()
        fn = getattr(ctrl, "print_height_to_zref", None) if ctrl else None
        if callable(fn):
            try:
                return float(fn(h))
            except Exception:
                pass
        sign = 1.0
        try:
            sign = float(ctrl.print_z_dir())
        except Exception:
            pass
        return float(bottom) + sign * h

    def _on_goto_approx_bottom(self) -> None:
        ok, why = self.gate(STEP_TOUCHOFF)
        if not ok:
            self._refuse(why)
            return
        target = self._touch_target_zref()
        if target is None:
            self._refuse("The plate bottom Z is not taught — step 2.")
            return
        ctrl = self._ctrl()
        self._arm_floor(True)
        try:
            # Z ONLY, at the current XY — so there is no XY move to retract for.
            ctrl.move_z_absolute(float(target), from_zero_ref=True)
        except Exception as e:
            self._refuse(str(e))
            return
        self._refusal_text = None
        self.refresh()

    def _on_confirm_touchoff(self) -> None:
        host = self._host
        z_zref = None
        fn = getattr(host, "_zoff_capture_current_z", None)
        if callable(fn):
            try:
                z_zref = fn()
            except Exception:
                z_zref = None
        if z_zref is None:
            self._refuse("Could not read the needle Z — is the Z board "
                         "connected?")
            return

        if self._s4_update_bottom.isChecked():
            setter = getattr(host, "_zoff_set_plate_bottom_z", None)
            if callable(setter):
                setter()

        self._write_touchoff_capture(z_zref)
        self._touchoff_done = True
        self.calibration_changed.emit()
        self.refresh()

    # ── step 4, optical: the ladder ──────────────────────────────

    def _margins_um(self) -> list:
        """Parse the operator's margin list. Never raises — the gate reports."""
        txt = (self._s4_margins.text() if getattr(self, "_s4_margins", None)
               else "")
        out = []
        for part in str(txt).replace(";", ",").split(","):
            part = part.strip()
            if not part:
                continue
            try:
                out.append(float(part))
            except ValueError:
                return []
        return out

    def _optical_gate(self) -> tuple:
        """``(ok, why)`` for starting the next rung. Cumulative."""
        try:
            from SupportClasses.PlateBottomOptical import ladder_gate
        except ImportError:                          # pragma: no cover
            return False, "The optical plate-bottom module is unavailable."
        scope = self._scope()
        if scope is None or not getattr(scope.state(), "has_focus", False):
            return (False,
                    "This needs a motorised microscope focus. Connect the "
                    "microscope, or switch to the contact touch-off.")
        if self._optics() is None:
            return (False,
                    "The objective in use has no µm/px calibration, or the body "
                    "does not report its NA and working distance. Calibrate it "
                    "on Hardware Setup → Cameras first — the sweep step and the "
                    "collision bound are both sized from those numbers.")
        margins = self._margins_um()
        ok, why = ladder_gate(margins, self._longest_bore_mm())
        if not ok:
            return False, why
        if self._opt_f0 is None:
            return (False,
                    "Focus the microscope on the plate bottom, then press "
                    "\"Set plate-bottom focus\".")
        if self._plate_bottom_guess() is None:
            return (False,
                    "No starting plate-bottom estimate. Auto-fill it on step 2 "
                    "— the first descent is bounded by it.")
        if len(self._opt_rungs) >= len(margins):
            return False, "Every margin has been measured. Accept, or re-run."
        return True, ""

    def _plate_bottom_guess(self):
        """Best current estimate: the last measurement, else step 2's guess.

        Only the FIRST rung ever falls back to the guess.
        """
        if self._opt_rungs:
            try:
                from SupportClasses.PlateBottomOptical import reconcile_rungs
                r = reconcile_rungs(
                    self._opt_rungs, focus_zero_um=self._opt_f0,
                    focus_up_sign=self._focus_up_sign(), zdir=self._zdir(),
                    spread_tol_um=float("inf"))
                if r.plate_bottom_zref_mm is not None:
                    return r.plate_bottom_zref_mm
            except Exception:
                pass
        return getattr(self._host, "_plate_bottom_z", None)

    def _scope(self):
        try:
            from SupportClasses.MicroscopeControl import get_microscope
            return get_microscope()
        except Exception:
            return None

    def _zdir(self) -> float:
        ctrl = self._ctrl()
        try:
            return float(ctrl.print_z_dir())
        except Exception:
            return 1.0

    def _focus_up_sign(self) -> float:
        try:
            from SupportClasses.MicroscopeConfigStore import get_store
            return 1.0 if get_store().focus_up_is_positive() else -1.0
        except Exception:
            return 1.0

    def _optics(self):
        """``ObjectiveOptics`` for the objective ACTUALLY IN THE TURRET, or None.

        Resolved from the live turret position, read-only, for the duration of
        this one measurement — never from ``current_objective_name``, and never
        falling back to the live manager's µm/px. That fallback is exactly how
        "I literally just calibrated it" happens: it silently supplies a number
        from a different objective and every derived bound is then wrong.
        Refuse instead; the gate says which objective needs calibrating.
        """
        try:
            from SupportClasses.ObjectiveLadder import resolve_ladder
            from SupportClasses.ObjectiveOptics import from_mounted_optic
            from SupportClasses.ObjectiveCalibration import get_store as obj_store
            from SupportClasses.MicroscopeConfigStore import get_store as cfg
        except ImportError:                          # pragma: no cover
            return None
        scope = self._scope()
        mgr, mic = self._camera_manager(), self._mic_cam_idx()
        if scope is None or mgr is None or mic is None:
            return None
        try:
            st = scope.state()
            pos = int(getattr(st, "objective_position", 0) or 0)
            if pos <= 0:
                return None
            frame = None
            try:
                f = mgr.cameras[mic].get_current_frame()
                if f is not None:
                    frame = (int(f.shape[1]), int(f.shape[0]))
            except Exception:
                frame = None
            cam_name = ""
            try:
                ident = mgr.camera_identity(mic)
                cam_name = ident[0] if ident else ""
            except Exception:
                pass
            rungs = resolve_ladder(
                scope_state=st, config_store=cfg(), objective_store=obj_store(),
                camera_name=cam_name, live_resolution=frame, positions=[pos])
            rung = rungs[0] if rungs else None
            if rung is None or not rung.calibrated:
                return None
            optic = next(
                (o for o in (getattr(st, "mounted_objectives", ()) or ())
                 if getattr(o, "position", 0) == pos), None)
            if optic is None:
                return None
            optics, _why = from_mounted_optic(
                optic, um_per_px_sample=rung.um_per_px, frame_wh=frame)
            return optics
        except Exception as e:
            logger.debug(f"optics resolve failed: {e}")
            return None

    def _tip_roi(self):
        """The datum bore's focus ROI in RAW frame px, or None for centred."""
        if not ROI_AVAILABLE or self._meas is None or 0 not in self._meas.clicks:
            return None
        mgr, mic, view = self._camera_manager(), self._mic_cam_idx(), self._mic_view()
        if mgr is None or mic is None or view is None:
            return None
        try:
            frame = mgr.cameras[mic].get_current_frame()
            fh, fw = frame.shape[:2]
            img_w, img_h = view.image_size
            clicks = {k: view_px_to_frame_px(p, (img_w, img_h), (fw, fh))
                      for k, p in self._meas.clicks.items()}
            od = float(getattr(self._needle().bore(0), "od_um", 0.0) or 0.0)
            return rois_for_clicks(clicks, (fw, fh),
                                   mgr.effective_um_per_px(mic, fw), od).get(0)
        except Exception:
            return None

    def _on_set_focus_zero(self) -> None:
        f = self._microscope_focus_um()
        if f is None:
            self._refuse(
                "No motorised focus reading is available. Connect the "
                "microscope, or use the contact touch-off.")
            return
        self._refusal_text = None
        self._opt_f0 = float(f)
        self._opt_rungs = []
        self._opt_pending = None
        self._s4_f0_lbl.setText(f"glass focus = {self._opt_f0:.1f} µm")
        self._s4_f0_lbl.setStyleSheet(f"color: {COLORS['green']};")
        self._s4_log.clear()
        self._log_opt(f"Glass focus set to {self._opt_f0:.1f} µm. "
                      f"Every margin is measured relative to this.")
        self.refresh()

    def _on_measure_rung(self) -> None:
        ok, why = self._optical_gate()
        if not ok:
            self._refuse(why)
            return
        if self._opt_worker is not None:
            return
        margins = self._margins_um()
        margin = margins[len(self._opt_rungs)]
        cam = None
        mgr, mic = self._camera_manager(), self._mic_cam_idx()
        try:
            cam = mgr.cameras[mic]
        except Exception:
            pass
        if cam is None:
            self._refuse("The microscope camera is not running.")
            return
        self._refusal_text = None
        self._arm_floor(True)
        self._opt_live = None
        soft = None
        try:
            from SupportClasses.MicroscopeConfigStore import get_store
            soft = get_store().focus_soft_limits_um()
        except Exception:
            pass
        st = self._scope().state()
        self._opt_worker = PlateBottomRungWorker(
            controller=self._ctrl(), scope=self._scope(), cam=cam,
            margin_um=margin, focus_zero_um=self._opt_f0,
            plate_bottom_zref_mm=self._plate_bottom_guess(), zdir=self._zdir(),
            focus_up_sign=self._focus_up_sign(), optics=self._optics(),
            roi_rect=self._tip_roi(),
            focus_limits_um=(getattr(st, "focus_min_um", None),
                             getattr(st, "focus_max_um", None)),
            soft_limits_um=soft, parent=self)
        self._opt_worker.progress.connect(self._on_rung_progress)
        self._opt_worker.sample.connect(self._on_rung_sample)
        self._opt_worker.measured.connect(self._on_rung_measured)
        self._opt_worker.failed.connect(self._on_rung_failed)
        self._log_opt(f"── margin {margin:.0f} µm ──")
        self._opt_worker.start()
        self.refresh()

    def _teardown_rung_worker(self) -> None:
        """Disconnect BEFORE stopping so a late queued signal cannot reach a
        torn-down page — the `plate_level_wizard` precedent."""
        w, self._opt_worker = self._opt_worker, None
        if w is None:
            return
        for sig in (w.progress, w.sample, w.measured, w.failed):
            try:
                sig.disconnect()
            except (RuntimeError, TypeError):
                pass
        try:
            w.stop()
            w.wait(8000)
        except Exception:                             # pragma: no cover
            pass

    def _on_cancel_rung(self) -> None:
        self._teardown_rung_worker()
        self._log_opt("Cancelled — the focus and needle were put back.")
        self.refresh()

    def _on_rung_progress(self, text) -> None:
        self._s4_state.setText(text)

    def _on_rung_sample(self, focus_um, score) -> None:
        self._note_live_focus(float(focus_um), float(score))

    def _on_rung_measured(self, m) -> None:
        self._teardown_rung_worker()
        self._opt_pending = m
        if m.refusal:
            self._log_opt(f"  refused: {m.refusal}")
            self._s4_state.setText(
                f"The {m.margin_um:.0f} µm margin found no usable focus peak. "
                f"Jog to the sharpest tip you can see and press \"Use "
                f"best-so-far instead\", or re-run this margin.")
        else:
            h = (m.focus_tip_um - self._opt_f0) * self._focus_up_sign()
            self._log_opt(
                f"  measured height {h:.1f} µm (asked {m.margin_um:.0f}), "
                f"σ {m.focus_sigma_um:.2f}, FWHM {m.fwhm_um:.1f} µm")
            self._adopt_rung(m)
        self.refresh()

    def _on_rung_failed(self, why) -> None:
        self._teardown_rung_worker()
        self._log_opt(f"  failed: {why}")
        self._s4_state.setText(why)
        self.refresh()

    def _adopt_rung(self, m, ground_truth: bool = False,
                    auto_focus_um=None) -> None:
        """Accept a rung and record the ONE capture it teaches."""
        self._opt_rungs.append(m)
        self._opt_pending = None
        self._write_focus_training(m, ground_truth=ground_truth,
                                   auto_focus_um=auto_focus_um)
        self._report_optical_progress()

    def _on_use_live_focus(self) -> None:
        """Operator override — and the only place ground truth is created."""
        if self._opt_live is None:
            self._refuse(
                "No live focus has been seen yet. Jog the focus or the needle "
                "with the tip in view, then try again.")
            return
        pend = self._opt_pending
        if pend is None:
            self._refuse(
                "Measure a margin first — the override replaces its result.")
            return
        _score, focus_um = self._opt_live
        auto = None if pend.refusal else pend.focus_tip_um
        delta = "" if auto is None else f"\n\nThe software picked {auto:.1f} µm; " \
                                        f"you are choosing {focus_um:.1f} µm " \
                                        f"({focus_um - auto:+.1f} µm)."
        if QMessageBox.question(
                self, "Use this focus as ground truth?",
                f"Record {focus_um:.1f} µm as the tip's true focus at the "
                f"{pend.margin_um:.0f} µm margin?{delta}\n\n"
                f"This is stored as a labelled training image for this needle "
                f"type. Only confirmed captures are used.",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No) != QMessageBox.Yes:
            return
        try:
            from SupportClasses.PlateBottomOptical import RungMeasurement
        except ImportError:                          # pragma: no cover
            return
        m = RungMeasurement(
            margin_um=pend.margin_um, needle_z_zref_mm=pend.needle_z_zref_mm,
            focus_tip_um=float(focus_um),
            # A running max resolves to about half a jog step, so it does not
            # get the fitted peak's sigma. Zero here means "not fitted".
            focus_sigma_um=0.0, fwhm_um=0.0, adopted_by="operator")
        self._log_opt(f"  overridden by operator: {focus_um:.1f} µm")
        self._adopt_rung(m, ground_truth=True, auto_focus_um=auto)
        self.refresh()

    def _note_live_focus(self, focus_um: float, score: float) -> None:
        best = self._opt_live
        if best is None or score > best[0]:
            self._opt_live = (float(score), float(focus_um))
        b = self._opt_live
        self._s4_live.setText(
            f"best focus so far: {b[1]:.1f} µm  (score {b[0]:.0f})  —  a running "
            f"max over the frames seen, not a fitted peak")

    def _fz_sign(self) -> float:
        """The datum's ±1: relates focus mm to needle zref mm."""
        return 1.0 if self._zdir() * self._focus_up_sign() >= 0 else -1.0

    def _fz_offset(self) -> tuple:
        """``(offset_mm, spread_mm, n)`` — the constant focal-plane ↔
        needle-tip offset from the adopted rungs (None, 0, 0 when none)."""
        try:
            from SupportClasses.PlateBottomOptical import focus_needle_offset_mm
        except ImportError:                          # pragma: no cover
            return None, 0.0, 0
        return focus_needle_offset_mm(self._opt_rungs, self._fz_sign())

    def _render_fz_result(self) -> None:
        """The headline: one constant, verified by its constancy."""
        lbl = getattr(self, "_s4_result", None)
        if lbl is None:
            return
        off, spread_mm, n = self._fz_offset()
        if off is None:
            lbl.setText("Focal-plane ↔ needle-tip offset: not yet measured.")
            lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-weight: 600;")
            return
        spread_um = spread_mm * 1000.0
        try:
            from SupportClasses.PlateBottomOptical import DEFAULT_SPREAD_TOL_UM
            tol = DEFAULT_SPREAD_TOL_UM
        except ImportError:                          # pragma: no cover
            tol = 25.0
        sign = "+" if self._fz_sign() >= 0 else "−"
        txt = (f"Focal-plane ↔ needle-tip offset: {off:+.3f} mm "
               f"(z = {sign}f/1000 {off:+.3f};  spread {spread_um:.0f} µm "
               f"over {n} margin{'s' if n != 1 else ''})")
        if n >= 2 and spread_um > tol:
            lbl.setText(txt + " — spread above tolerance; re-run a margin.")
            lbl.setStyleSheet(
                f"color: {COLORS['yellow']}; font-weight: 600;")
        else:
            lbl.setText(txt)
            lbl.setStyleSheet(
                f"color: {COLORS['green']}; font-weight: 600;")

    def _report_optical_progress(self) -> None:
        self._render_fz_result()
        margins = self._margins_um()
        done = len(self._opt_rungs)
        if done < len(margins):
            self._s4_state.setText(
                f"{done}/{len(margins)} margins measured. Next: "
                f"{margins[done]:.0f} µm.")
            return
        rec = self._reconcile()
        if rec is None:
            return
        if rec.ok:
            self._s4_state.setText(
                f"All {done} margins agree to {rec.spread_um:.0f} µm. "
                f"{rec.scale_note}")
        else:
            self._s4_state.setText(rec.refusal)

    def _reconcile(self):
        if not self._opt_rungs or self._opt_f0 is None:
            return None
        try:
            from SupportClasses.PlateBottomOptical import reconcile_rungs
            return reconcile_rungs(
                self._opt_rungs, focus_zero_um=self._opt_f0,
                focus_up_sign=self._focus_up_sign(), zdir=self._zdir())
        except Exception as e:                        # pragma: no cover
            logger.debug(f"reconcile failed: {e}")
            return None

    def _on_accept_optical(self) -> None:
        rec = self._reconcile()
        if rec is None:
            self._refuse("Nothing measured yet — run at least one margin.")
            return
        if not rec.ok:
            self._refuse(rec.refusal)
            return
        if rec.n_used < 2:
            if QMessageBox.question(
                    self, "Accept with one margin?",
                    "Only one margin was measured, so there is nothing to check "
                    "it against — the offset is measured but the scale is "
                    "assumed. Accept anyway?",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No) != QMessageBox.Yes:
                return
        host = self._host
        b = float(rec.plate_bottom_zref_mm)
        host._plate_bottom_z = b
        ctrl = self._ctrl()
        try:
            ctrl.set_plate_bottom_z(b, at_xy_um=self._read_xy(), source="optical")
        except TypeError:                             # older controller
            try:
                ctrl.set_plate_bottom_z(b)
            except Exception:
                pass
        except Exception:
            pass
        self._write_focus_datum(rec)
        fn = getattr(host, "_emit_calibration_data_changed", None)
        if callable(fn):
            try:
                fn()
            except Exception:
                pass
        self._refusal_text = None
        self._touchoff_done = True
        self._render_fz_result()
        self._s4_state.setText(
            f"Plate bottom set optically to {b:.3f} mm from {rec.n_used} "
            f"margin(s), spread {rec.spread_um:.0f} µm. The tip never came "
            f"closer than {min(self._margins_um() or [0]):.0f} µm to the glass.")
        self.calibration_changed.emit()
        self.refresh()

    def _write_focus_training(self, m, ground_truth: bool = False,
                              auto_focus_um=None) -> None:
        """One labelled tip image per adopted rung.

        This is what "training the focus model" means concretely, and it is only
        possible because the capture now carries the needle type and the focus
        score — the two fields the touch-off writer dropped, which is why
        ``reference_focus_score`` returned None for every capture ever written.

        The pair (what the software picked, what the operator adopted) is the
        whole signal: a consistent difference for a needle type is a measurable,
        correctable bias in the gradient metric. Storing only the adopted value
        would leave nothing to measure it from.
        """
        try:
            from SupportClasses.NeedleFocusTemplateStore import (
                get_store as _tpl_store, template_key)
        except ImportError:                          # pragma: no cover
            return
        mgr, mic, view = self._camera_manager(), self._mic_cam_idx(), self._mic_view()
        if mgr is None or mic is None or view is None:
            return
        try:
            frame = mgr.cameras[mic].get_current_frame()
            if frame is None:
                return
            fh, fw = frame.shape[:2]
            roi = self._tip_roi()
            if roi:
                x, y, ww, hh = roi
            else:
                half = max(48, int(fw * 0.125))
                x, y = max(0, fw // 2 - half), max(0, fh // 2 - half)
                ww = hh = 2 * half
            patch = frame[y:y + hh, x:x + ww].copy()
            cx, cy = x + ww / 2.0, y + hh / 2.0
            score = None
            try:
                from SupportClasses.VisionDetector import NeedleDetector
                score = float(
                    NeedleDetector.compute_focus_score(frame, roi).score)
            except Exception:
                pass
            needle = self._needle()
            bore0 = needle.bore(0)
            _tpl_store().add_capture(
                template_key(self._host._ploc_camera_objective_key(),
                             getattr(needle, "needle_type", ""),
                             getattr(bore0, "orifice_id_um", 0.0)),
                patch,
                center_offset_px=(cx - fw / 2.0, cy - fh / 2.0),
                center_offset_um=(self._meas.needle_camera_offset()
                                  if self._meas else None) or (0.0, 0.0),
                um_per_px=mgr.effective_um_per_px(mic, fw),
                frame_wh=(fw, fh),
                focus_score=score,
                z_zref_mm=m.needle_z_zref_mm,
                stage_um=self._read_xy(),
                needle_type=getattr(needle, "needle_type", None),
                needle_bore_um=getattr(bore0, "orifice_id_um", None),
                needle_tip_length_mm=getattr(needle, "tip_length_mm", None),
                microscope_focus_um=m.focus_tip_um,
                needle_z_user_mm=self._read_z_user(),
                ground_truth=ground_truth,
                adopted_by=m.adopted_by,
                auto_focus_um=auto_focus_um,
                margin_um=m.margin_um)
        except Exception as e:
            logger.debug(f"focus training capture skipped: {e}")

    def _datum_camera_identity(self) -> str:
        """BARE camera identity for the focus-datum key.

        v7.13 — this used to pass ``_ploc_camera_objective_key()`` (a composite
        ``"cam|objective"``) as the CAMERA half of the key while the reader
        (``plate_level_wizard._focal_sign``) passed the bare identity — so the
        written and read keys could never match and the datum was dead on
        arrival. Both callers now use (bare identity, objective name).
        """
        mgr, mic = self._camera_manager(), self._mic_cam_idx()
        if mgr is None or mic is None:
            return ""
        try:
            ident = mgr.camera_identity(mic)
            return ident[0] if ident else ""
        except Exception:
            return ""

    def _datum_objective_name(self) -> str:
        hw = getattr(self._host, "_hardware_config", None)
        cam_cfg = getattr(hw, "camera_config", None) if hw is not None else None
        return str(getattr(cam_cfg, "current_objective_name", "") or "")

    def _write_focus_datum(self, rec) -> None:
        """Persist the focus↔needle bridge — THE constant offset between the
        objective focal plane and the needle tip.

        v7.13: the offset is the MEDIAN over the tip pairs (each rung pairs a
        fitted tip focus with the needle Z read back at that moment — n
        independent estimates of the one constant) rather than the single
        hand-focused on-glass pair; ``focal_sign_and_offset`` still supplies
        the ±1 and serves as the fallback when no tip pair is usable. Also
        stamps the zero-Z epoch (making ``is_stale`` live) and the turret
        position. Best-effort — a store failure must not cost the operator the
        plate bottom, which is the part that matters.
        """
        try:
            from SupportClasses.PlateBottomOptical import (
                focal_sign_and_offset, focus_needle_offset_mm)
            from SupportClasses.PlateFocusDatumStore import get_store
            sign, off_glass = focal_sign_and_offset(
                rec.plate_bottom_zref_mm, self._opt_f0,
                self._focus_up_sign(), self._zdir())
            off_tip, _spread, n_tip = focus_needle_offset_mm(
                self._opt_rungs, sign)
            off = off_glass if off_tip is None else off_tip
            zero_z = None
            turret = None
            ctrl = self._ctrl()
            try:
                zero_z = float((getattr(ctrl, "zero_position", None) or {})
                               .get("Z", 0.0))
            except Exception:
                zero_z = None
            try:
                scope = self._scope()
                st = scope.state() if scope is not None else None
                pos = int(getattr(st, "objective_position", 0) or 0)
                turret = pos if pos > 0 else None
            except Exception:
                turret = None
            get_store().save(
                self._datum_camera_identity(), self._datum_objective_name(),
                self._plate_key(), focal_sign=sign, offset_mm=off,
                focus_um_at_bottom=self._opt_f0,
                plate_bottom_z_zref_mm=rec.plate_bottom_zref_mm,
                anchor_xy_um=self._read_xy(),
                zero_z_mm=zero_z, turret_position=turret,
                measured_scale=rec.scale, n_points=max(rec.n_used, n_tip),
                span_um=rec.span_um)
        except Exception as e:
            logger.debug(f"focus datum not written: {e}")

    def _plate_key(self) -> str:
        fn = getattr(self._host, "_ploc_plate_key", None)
        try:
            return fn() if callable(fn) else "plate"
        except Exception:
            return "plate"

    def _log_opt(self, text: str) -> None:
        if getattr(self, "_s4_log", None) is not None:
            self._s4_log.append(text)

    def _refresh_optical(self) -> None:
        self._render_fz_result()
        guess = self._plate_bottom_guess()
        src = ""
        try:
            src = self._ctrl().get_plate_bottom_z_source() or ""
        except Exception:
            pass
        if guess is None:
            self._s4_guess_lbl.setText(
                "No starting estimate — auto-fill the plate bottom on step 2.")
        else:
            tag = f" ({src})" if src and not self._opt_rungs else ""
            what = "measured" if self._opt_rungs else "starting estimate"
            self._s4_guess_lbl.setText(
                f"{what}: plate bottom {guess:.3f} mm{tag}")
        ok, why = self._optical_gate()
        busy = self._opt_worker is not None
        self._s4_measure.setEnabled(ok and not busy)
        self._s4_cancel.setEnabled(busy)
        self._s4_setf0.setEnabled(not busy)
        self._s4_use_live.setEnabled(not busy and self._opt_pending is not None)
        rec = self._reconcile()
        self._s4_accept.setEnabled(not busy and rec is not None and rec.ok)
        if not busy and not ok and why:
            self._s4_state.setText(why)

    def _microscope_focus_um(self):
        """The scope's focus-axis reading, or None with no motorised focus.

        This is the RAW axis position. It only becomes an optical Z DATUM when
        paired with a needle Z at the same physical plane — see
        :meth:`_optical_datum_focus_um`, which is the only caller that stores it.

        Reads the POLLED immutable snapshot rather than calling the backend: the
        controller serialises on its own COM-owning worker, and a blocking call
        from the GUI thread is the freeze class this repo has fixed repeatedly.
        """
        try:
            from SupportClasses.MicroscopeControl import get_microscope
            # `state` is a METHOD, not a property. Calling it without () yields a
            # bound method, whose `has_focus` is always absent -> the getattr
            # below returned False forever and this function returned None on
            # every call, silently, because the whole block sits in a broad
            # except. That meant NeedleFocusTemplateStore never received a
            # microscope_focus_um and the optical Z datum was never captured.
            state = get_microscope().state()
            if getattr(state, "has_focus", False):
                return state.focus_um
        except Exception as e:
            logger.debug(f"microscope focus unavailable: {e}")
        return None

    def _write_touchoff_capture(self, z_zref) -> None:
        """Template patch + needle↔camera offset + the optical Z datum.

        Best-effort throughout: a failure here must not cost the operator the
        touch-off itself, which is the part that matters.
        """
        try:
            from SupportClasses.NeedleFocusTemplateStore import (
                get_store as _tpl_store, template_key)
        except ImportError:
            return
        meas = self._meas
        if meas is None or 0 not in meas.clicks:
            self._s4_state.setText(
                "Plate bottom recorded. No template captured — the bore tips "
                "were not clicked in step 3.")
            return
        view, mgr, mic = self._mic_view(), self._camera_manager(), self._mic_cam_idx()
        if view is None or mgr is None or mic is None:
            return
        try:
            cam = mgr.cameras[mic]
            frame = cam.get_current_frame()
            if frame is None:
                return
            fh, fw = frame.shape[:2]
            img_w, img_h = view.image_size
            cx, cy = view_px_to_frame_px(meas.clicks[0], (img_w, img_h), (fw, fh))
            half = max(48, int(fw * 0.125))
            x0, x1 = max(0, int(cx - half)), min(fw, int(cx + half))
            y0, y1 = max(0, int(cy - half)), min(fh, int(cy + half))
            patch = frame[y0:y1, x0:x1].copy()
            needle = self._needle()
            bore0 = needle.bore(0)
            key = template_key(
                self._host._ploc_camera_objective_key(),
                getattr(needle, "needle_type", ""),
                getattr(bore0, "orifice_id_um", 0.0))
            # D3 — needle_type / needle_bore_um / focus_score were never passed.
            # The needle type then survived only as a substring of the key (so
            # `store.get(key)["needle_type"]` was None for every capture on
            # disk), and `reference_focus_score` — the whole point of storing an
            # in-focus reference — could never return anything but None.
            score = None
            try:
                from SupportClasses.VisionDetector import NeedleDetector
                score = float(NeedleDetector.compute_focus_score(
                    frame, rois_for_clicks(
                        {0: (cx, cy)}, (fw, fh),
                        mgr.effective_um_per_px(mic, fw),
                        float(getattr(bore0, "od_um", 0.0) or 0.0)).get(0)
                    if ROI_AVAILABLE else None).score)
            except Exception:
                pass
            _tpl_store().add_capture(
                key, patch,
                center_offset_px=(cx - fw / 2.0, cy - fh / 2.0),
                center_offset_um=meas.needle_camera_offset() or (0.0, 0.0),
                um_per_px=mgr.effective_um_per_px(mic, fw),
                frame_wh=(fw, fh),
                focus_score=score,
                z_zref_mm=z_zref,
                stage_um=self._read_xy(),
                needle_type=getattr(needle, "needle_type", None),
                needle_bore_um=getattr(bore0, "orifice_id_um", None),
                needle_tip_length_mm=getattr(needle, "tip_length_mm", None),
                microscope_focus_um=self._optical_datum_focus_um(),
                needle_z_user_mm=self._read_z_user())
            self._report_datum()
        except Exception as e:
            logger.debug(f"touch-off capture skipped: {e}")

    def _optical_datum_focus_um(self):
        """The focus reading to pair with this needle Z, or None.

        Returns None unless the operator has confirmed the microscope is
        focused on the plate bottom. Storing the focus axis' position when it is
        NOT on the same plane as the needle tip produces a datum that is wrong
        by however far the focus happens to be — and, because every downstream
        consumer treats the datum as ground truth for converting focus to needle
        Z, that error would propagate into the plate-bottom height everywhere.
        Absent is recoverable; wrong is not.
        """
        box = getattr(self, "_s4_focus_ok", None)
        if box is not None and not box.isChecked():
            return None
        return self._microscope_focus_um()

    def _report_datum(self) -> None:
        """Say whether the optical datum was recorded — it is easy to miss."""
        box = getattr(self, "_s4_focus_ok", None)
        if box is not None and not box.isChecked():
            self._s4_state.setText(
                "Plate bottom recorded. Optical Z datum NOT captured — tick "
                "\"The microscope is focused on the plate bottom\" and re-confirm "
                "to record it.")
            return
        if self._microscope_focus_um() is None:
            self._s4_state.setText(
                "Plate bottom recorded. Optical Z datum NOT captured — the "
                "microscope reports no motorised focus position.")
            return
        self._s4_state.setText(
            f"Plate bottom recorded, with the optical Z datum at "
            f"{self._microscope_focus_um():.1f} µm on the focus axis.")

    # ── live focus scores ────────────────────────────────────────

    def _focus_tick(self) -> None:
        """One frame, one score per bore ROI. Independent ROIs mean a single
        frame resolves every bore at once, which is what lets the operator jog
        Z and watch them all respond."""
        if not ROI_AVAILABLE or self._meas is None or not self._meas.clicks:
            return
        mgr, mic, view = self._camera_manager(), self._mic_cam_idx(), self._mic_view()
        if mgr is None or mic is None or view is None:
            return
        try:
            from SupportClasses.VisionDetector import NeedleDetector
            frame = mgr.cameras[mic].get_current_frame()
            if frame is None:
                return
            fh, fw = frame.shape[:2]
            img_w, img_h = view.image_size
            clicks = {k: view_px_to_frame_px(p, (img_w, img_h), (fw, fh))
                      for k, p in self._meas.clicks.items()}
            needle = self._needle()
            od = float(getattr(needle.bore(0), "od_um", 0.0) or 0.0)
            rois = rois_for_clicks(clicks, (fw, fh),
                                   mgr.effective_um_per_px(mic, fw), od)
            z_now = self._read_z_user()
            for k, roi in rois.items():
                score = float(NeedleDetector.compute_focus_score(frame, roi).score)
                self._live_scores[k] = score
                self._meas.note_score(k, score, z_now)
            # Step 4 optical: the same frame also feeds the "best focus so far"
            # the operator jogs against. Keyed on the FOCUS axis, not needle Z,
            # because either may be the axis they are moving.
            if (self._current == STEP_TOUCHOFF and self._s4_optical_mode()
                    and self._opt_worker is None and 0 in self._live_scores):
                f_now = self._microscope_focus_um()
                if f_now is not None:
                    self._note_live_focus(f_now, self._live_scores[0])
        except Exception as e:                        # pragma: no cover
            logger.debug(f"focus tick skipped: {e}")
            return
        self._render_bore_rows()
        self._update_drift_label()

    # ── rendering ────────────────────────────────────────────────

    def refresh(self) -> None:
        states = {k: self.step_state(k) for k in STEP_ORDER}
        self._strip.render(self._current, states)
        self._instruction.setText(self._instruction_for(self._current))
        ok, why = self.gate(self._current)
        if self._refusal_text:
            # A sticky inline refusal outranks everything until the next step
            # change / successful action — a 300 ms tick must not blink it away.
            self._status.setText(f"⚠ {self._refusal_text}")
            self._status.setStyleSheet(f"color: {COLORS['yellow']};")
        elif not ok:
            self._status.setText(f"⚠ {why}")
            self._status.setStyleSheet(f"color: {COLORS['yellow']};")
        else:
            self._status.setText(self._status_for(self._current))
            self._status.setStyleSheet(f"color: {COLORS['subtext0']};")
        # Back/Next reflect position only; the destination gates itself.
        i = STEP_ORDER.index(self._current)
        self._btn_back.setEnabled(i > 0)
        self._btn_next.setEnabled(i < len(STEP_ORDER) - 1)
        self._render_step1()
        self._render_step2()
        self._render_step3_buttons()
        self._render_bore_rows()
        self._update_drift_label()
        self._push_bore_markers()
        self._render_step4()

    def _instruction_for(self, step: str) -> str:
        """The ONE bold sentence: what the operator does on this step."""
        if step == STEP_NEEDLE_ZERO:
            return ("Centre the needle tip on both side-camera crosshairs, "
                    "then save the needle origin.")
        if step == STEP_Z_REFS:
            return ("Jog the tip to the plate TOP and capture it — the plate "
                    "bottom is derived from the typed offset below it.")
        if step == STEP_BORES:
            return ("Put the needle anywhere every bore is visible in the "
                    "microscope, press 'Start measuring here', then click each "
                    "bore's tip — no stage motion is needed.")
        if step == STEP_TOUCHOFF:
            return ("Find the needle tip by focus — this measures the plate "
                    "bottom and the focal-plane ↔ needle-tip offset.")
        return ""

    def _status_for(self, step: str) -> str:
        if step == STEP_BORES:
            n = self._bore_count()
            if self._meas is None:
                return ("Jog the bores into the microscope view — anywhere is "
                        "fine — then press 'Start measuring here'. "
                        + ("This needle has one bore, so only the "
                           "needle↔camera offset is measured." if n <= 1 else
                           f"{n} bores to measure.")
                        + self._floor_advisory())
            miss_c = self._meas.missing_clicks()
            miss_z = self._meas.missing_z()
            if miss_c:
                return ("Click the tip of bore "
                        + ", ".join(str(k + 1) for k in miss_c) + ".")
            if miss_z:
                return ("Jog Z and record the focus peak for bore "
                        + ", ".join(str(k + 1) for k in miss_z) + "."
                        + self._floor_advisory())
            return "All bores measured — save the offsets."
        if step == STEP_TOUCHOFF:
            return ("Focus the microscope on the plate bottom, bring the tip to "
                    "that plane, then confirm.")
        return ""

    def _floor_advisory(self) -> str:
        """Advisory (never a refusal) when the plate-bottom clamp has no datum.

        Step 3 is now reachable without a taught plate bottom, which is correct —
        clicking two dots in one frame does not need one. But recording each
        bore's focus Z means jogging the needle DOWN by hand, and
        ``_arm_floor(True)`` is a no-op without the datum
        (``StageController._apply_print_floor_raw`` returns early on
        ``_plate_bottom_z_zref is None``), so the clamp the operator may assume
        is protecting them is not armed. Say so rather than either blocking the
        measurement or staying silent about an inactive guard.
        """
        if getattr(self._host, "_plate_bottom_z", None) is not None:
            return ""
        return (" ⚠ Plate Bottom Z is not taught, so the plate-bottom clamp is "
                "inactive — jog Z by hand with care, or teach it on step 2.")

    def _render_step3_buttons(self) -> None:
        """Enable/disable step 3's two buttons with the first reason as tooltip.

        The two gates are independent on purpose: an un-taught reference height
        disables only the PARK, while 'Start measuring here' stays live because
        measuring commands no motion. Disabling-with-a-reason follows
        ``_commit_blockers`` — the operator never has to click to find out.
        """
        start = getattr(self, "_s3_start_btn", None)
        park = getattr(self, "_s3_park_btn", None)
        if start is None or park is None:
            return
        ok, why = self.gate(STEP_BORES)
        start.setEnabled(ok)
        start.setToolTip(why if not ok else (
            "Begin a measurement session at the CURRENT stage position — no "
            "stage motion is commanded. Every bore must be visible in one "
            "frame, and the stage must not move until the offsets are saved."))
        pok, pwhy = self.park_gate()
        park.setEnabled(pok)
        park.setToolTip(pwhy if not pok else (
            "Convenience only: safe-travel to the saved needle location at the "
            "survey height, then start a session."))

    def _update_drift_label(self) -> None:
        """Live stage-drift readout for step 3 — cached position, so the ~3 Hz
        refresh costs no serial round-trip (the CLICK gate re-reads fresh)."""
        lbl = getattr(self, "_s3_drift_lbl", None)
        if lbl is None:
            return
        if self._meas is None or self._park_xy is None:
            lbl.setText("No measurement session — press 'Start measuring here' "
                        "once the bores are in view.")
            lbl.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 9pt;")
            return
        drift = None
        ctrl = self._ctrl()
        try:
            xy = ctrl.get_xy_position(cached=True)
            if xy and xy[0] is not None and xy[1] is not None:
                drift = max(abs(float(xy[0]) - self._park_xy[0]),
                            abs(float(xy[1]) - self._park_xy[1]))
        except Exception:
            drift = None
        if drift is None:
            lbl.setText("Stage drift: unknown (no position read).")
            lbl.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 9pt;")
        elif drift > MAX_STAGE_DRIFT_UM:
            lbl.setText(
                f"Stage drift: {drift:.1f} µm — clicks paused (limit "
                f"{MAX_STAGE_DRIFT_UM:.0f} µm). Press 'Start measuring here' to "
                f"re-anchor and re-click every bore.")
            lbl.setStyleSheet(f"color: {COLORS['red']}; font-size: 9pt;")
        else:
            lbl.setText(f"Stage drift: {drift:.1f} µm "
                        f"(limit {MAX_STAGE_DRIFT_UM:.0f} µm) — OK to click.")
            lbl.setStyleSheet(f"color: {COLORS['green']}; font-size: 9pt;")

    # ── bore-dot overlay ─────────────────────────────────────────

    def _session_bore_fov_offsets(self):
        """Dots from the IN-PROGRESS session's clicks (pto = FOV µm)."""
        if self._meas is None or not self._meas.pto:
            return None
        return [(f"B{k + 1}", v[0], v[1], bore_dot_color(k))
                for k, v in sorted(self._meas.pto.items())]

    def _stored_bore_fov_offsets(self):
        """Dots from the COMMITTED calibration, or None when underivable.

        Preferred derivation: ``needle_camera_offset + offset_um(k)`` — exact
        by ``pto(P_k) = pto(P_0) + offset_um(k)``. Fallback: each bore's
        ``stage_um`` provenance, but ONLY when the invariant
        ``stage_um(k) − stage_um(0) ≈ offset_um(k)`` holds — the legacy
        side-camera path wrote ABSOLUTE stage positions into the same field
        (with the opposite difference sign), and projecting those would draw
        dots kilometres off-frame or, worse, plausibly wrong.
        """
        store = self._store()
        if store is None:
            return None
        try:
            bores = store.all_bores()
        except Exception:
            return None
        if not bores:
            return None
        ncam = None
        ctrl = self._ctrl()
        fn = getattr(ctrl, "get_needle_camera_offset_um", None) if ctrl else None
        if callable(fn):
            try:
                ncam = fn()
            except Exception:
                ncam = None
        if ncam is not None:
            out = []
            for b in bores:
                off = b.offset_um or (0.0, 0.0)
                out.append((f"B{b.bore_index + 1}",
                            float(ncam[0]) + float(off[0]),
                            float(ncam[1]) + float(off[1]),
                            bore_dot_color(b.bore_index)))
            return out
        datum = next((b for b in bores if b.bore_index == 0), None)
        if datum is None or datum.stage_um is None:
            return None
        d = datum.stage_um
        for b in bores:
            if b.stage_um is None:
                return None
            off = b.offset_um or (0.0, 0.0)
            if (abs((b.stage_um[0] - d[0]) - off[0]) > 1.0
                    or abs((b.stage_um[1] - d[1]) - off[1]) > 1.0):
                return None     # absolute-µm provenance: not FOV space
        return [(f"B{b.bore_index + 1}", float(b.stage_um[0]),
                 float(b.stage_um[1]), bore_dot_color(b.bore_index))
                for b in bores]

    def _push_bore_markers(self) -> None:
        """Feed the overlay: session clicks while measuring, stored offsets
        otherwise. Change-gated inside ``set_bore_markers``, so calling this
        from every refresh costs nothing when nothing changed."""
        view = self._mic_view()
        setter = getattr(view, "set_bore_markers", None) if view else None
        if not callable(setter):
            return
        markers = self._session_bore_fov_offsets()
        if markers is None:
            markers = self._stored_bore_fov_offsets()
        chk = getattr(self, "_bore_dots_chk", None)
        if chk is not None:
            chk.setVisible(markers is not None)
            if markers is not None and not chk.isChecked():
                markers = None
        setter(markers)

    def _render_step1(self) -> None:
        origin = getattr(self._host, "_needle_origin_um", None)
        if origin:
            self._s1_state.setText(
                f"✓ needle_origin_um: ({origin[0]:.1f}, {origin[1]:.1f}) µm")
            self._s1_state.setStyleSheet(f"color: {COLORS['green']};")
        else:
            self._s1_state.setText("Not set — centre the needle and save.")
            self._s1_state.setStyleSheet(f"color: {COLORS['yellow']};")

    def _render_step2(self) -> None:
        host = self._host
        top = getattr(host, "_top_z", None)
        lbl = getattr(self, "_s2_top_lbl", None)
        if lbl is not None:
            if top is None:
                lbl.setText("not captured")
                lbl.setStyleSheet(f"color: {COLORS['yellow']};")
            else:
                lbl.setText(f"plate top = {top:.3f} mm")
                lbl.setStyleSheet(f"color: {COLORS['green']};")
        prev = getattr(self, "_s2_bottom_preview", None)
        if prev is not None:
            z = self._computed_bottom_zref()
            prev.setText("→ capture the plate top first" if z is None
                         else f"→ plate bottom ≈ {z:.3f} mm")
            prev.setStyleSheet(
                f"color: {COLORS['subtext0'] if z is None else COLORS['text']};"
                f" font-size: 9pt;")
        btn = getattr(self, "_s2_apply_bottom", None)
        if btn is not None:
            can = (self._computed_bottom_zref() is not None
                   and callable(getattr(host, "_zoff_apply_plate_bottom_z",
                                        None)))
            btn.setEnabled(can)
        bits = []
        for label, attr in (("Plate top", "_top_z"),
                            ("Plate bottom", "_plate_bottom_z"),
                            ("Fast-move", "_safe_z")):
            v = getattr(host, attr, None)
            bits.append(f"{label}: " + ("—" if v is None else f"{v:.3f} mm"))
        src = ""
        try:
            src = self._ctrl().get_plate_bottom_z_source() or ""
        except Exception:
            src = ""
        if src:
            bits.append(f"bottom source: {src}")
        self._s2_state.setText(" · ".join(bits))
        ready = (getattr(host, "_plate_bottom_z", None) is not None
                 and getattr(host, "_safe_z", None) is not None)
        self._s2_state.setStyleSheet(
            f"color: {COLORS['green'] if ready else COLORS['subtext0']};")

    def _build_bore_row(self, k: int, multi: bool) -> dict:
        """One per-bore row. Built ONCE per bore count; afterwards only its
        text/enabled/value are updated — the focus timer runs at ~7 Hz and
        tearing down widgets at that rate is exactly the churn this app watches
        for as a leak signature."""
        row_w = QWidget()
        row = QHBoxLayout(row_w)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(6))

        pick = QPushButton(f"Bore {k + 1}")
        pick.setCheckable(True)
        pick.setToolTip("Select, then click this bore's tip in the view.")
        pick.clicked.connect(
            lambda _c=False, idx=k: (setattr(self, "_active_bore", idx),
                                     self.refresh()))
        row.addWidget(pick)

        lbl = QLabel("—")
        lbl.setMinimumWidth(s(150))
        row.addWidget(lbl, stretch=1)

        bar = QProgressBar()
        bar.setTextVisible(False)
        bar.setFixedWidth(s(80))
        bar.setRange(0, 1000)
        bar.setVisible(False)
        row.addWidget(bar)

        rec = up = None
        if multi:
            rec = QPushButton("Record Z")
            rec.setToolTip("Store this bore's height at its sharpest focus.")
            rec.clicked.connect(lambda _c=False, idx=k: self._on_record_z(idx))
            row.addWidget(rec)
            up = QPushButton("Use peak")
            up.setToolTip("Adopt the sharpest height seen — a readout, not an "
                          "autofocus: no stage motion is commanded.")
            up.setVisible(False)
            up.clicked.connect(lambda _c=False, idx=k: self._on_use_peak(idx))
            row.addWidget(up)

        redo = QPushButton("Redo")
        redo.clicked.connect(lambda _c=False, idx=k: self._on_redo_bore(idx))
        row.addWidget(redo)

        self._s3_rows_lay.addWidget(row_w)
        return {"w": row_w, "pick": pick, "lbl": lbl, "bar": bar,
                "rec": rec, "up": up, "redo": redo}

    def _render_bore_rows(self) -> None:
        meas = self._meas
        want = 0 if meas is None else meas.bore_count
        # Rebuild only when the row COUNT changes (a different assembly).
        if len(self._s3_row_widgets) != want:
            for r in self._s3_row_widgets:
                try:
                    self._s3_rows_lay.removeWidget(r["w"])
                    r["w"].setParent(None)
                    r["w"].deleteLater()
                except Exception:
                    pass
            self._s3_row_widgets = [
                self._build_bore_row(k, want > 1) for k in range(want)]
        if meas is None:
            return

        multi = meas.bore_count > 1
        for k, r in enumerate(self._s3_row_widgets):
            r["pick"].setChecked(k == self._active_bore)

            off = meas.offset_for(k)
            if k == 0:
                txt = "datum"
            elif off is None:
                txt = "not clicked"
            else:
                txt = f"({off[0]:+.0f}, {off[1]:+.0f}) µm"
            if multi and k in meas.z_user:
                txt += f" · Z {meas.dz_for(k):+.3f} mm"
            r["lbl"].setText(txt)
            done = (k in meas.pto) and (not multi or k in meas.z_user)
            r["lbl"].setStyleSheet(
                f"color: {COLORS['green'] if done else COLORS['subtext0']};"
                f" font-size: 9pt;")

            score = self._live_scores.get(k)
            peak = meas.peak.get(k)
            if score is None:
                r["bar"].setVisible(False)
            else:
                top = max(score, peak[0] if peak else score, 1e-9)
                r["bar"].setValue(int(1000 * score / top))
                r["bar"].setToolTip(
                    f"focus {score:.0f}" +
                    (f" · peak {peak[0]:.0f} @ {peak[1]:.3f} mm" if peak else ""))
                r["bar"].setVisible(True)

            if r["rec"] is not None:
                r["rec"].setEnabled(k in meas.pto)
            if r["up"] is not None:
                r["up"].setVisible(peak is not None and k in meas.pto)
            r["redo"].setEnabled(k in meas.pto)

        # Commit affordance: disabled with the first blocker as its tooltip —
        # the operator never has to click to discover what is missing.
        btn = getattr(self, "_s3_commit", None)
        if btn is not None:
            blockers = self._commit_blockers()
            btn.setEnabled(not blockers)
            btn.setToolTip(blockers[0] if blockers
                           else "Write the measured offsets to the "
                                "calibration store and the live needle.")

    def _render_step4(self) -> None:
        if self._s4_optical_mode():
            self._refresh_optical()
            return
        longest = self._longest_bore_mm()
        target = self._touch_target_zref()
        bottom = getattr(self._host, "_plate_bottom_z", None)
        if target is None or bottom is None:
            self._s4_offset_lbl.setText(
                "Teach the Plate Bottom Z before approaching.")
            return
        clearance = float(self._s4_clearance.value())
        msg = (f"Approach stops {clearance:.3f} mm above the longest bore's "
               f"reach. Longest bore protrudes {longest:.3f} mm past the datum, "
               f"so the datum tip stops {longest + clearance:.3f} mm above the "
               f"plate bottom.")
        if longest <= 0.0 and self._bore_count() > 1:
            msg += ("  ⚠ No per-bore Z has been measured, so the approach "
                    "assumes the bores are coplanar.")
        self._s4_offset_lbl.setText(msg)

    # ── host hooks ───────────────────────────────────────────────

    def on_status_update(self) -> None:
        """Cheap periodic hook — keeps the gates and readouts live."""
        try:
            self.refresh()
        except Exception as e:                        # pragma: no cover
            logger.debug(f"wizard refresh skipped: {e}")

    def set_hardware_config(self, config) -> None:
        """A new assembly invalidates an in-progress measurement; a new plate
        re-fills the step-2 bottom offset (a typed value still wins)."""
        n = self._bore_count()
        if self._meas is not None and self._meas.bore_count != n:
            self._meas = None
        self._autofill_bottom_offset()
        self.refresh()
