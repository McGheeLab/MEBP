"""
plate_level_wizard.py — automated optical plate-bed leveling.

The plate bottom is flat but TILTED. On this machine the measured gradient is
0.0199 mm/mm — about 1.15 mm of Z across a 24-well plate's row span, several
times a typical 0.1–0.5 mm print height. That tilt was previously measured by the
calibration page and then thrown away: prints resolved plate bottom from a single
scalar.

This wizard measures it **optically**. The operator teaches a feature once; the
software then focuses on that feature at several sites, fits a plane through the
focus heights, proves the fit against a site it never saw, and installs it.

THE NEEDLE NEVER APPROACHES THE GLASS
-------------------------------------
Focus readings give the plate's SHAPE. The absolute datum comes from the taught
plate-bottom scalar (or, once measured, the focus↔needle datum) — so this run
retracts the needle and leaves it retracted. Every site is reached with
``safe_travel_to(..., target_z_mm=None)``, which only ever raises.

Consequently ``set_print_floor_active`` is **never called**. Arming a refcount
this workflow cannot need would risk an unbalanced decrement against a concurrent
print, for no benefit. A test pins that it stays untouched.

THE NEW HAZARD IS THE OBJECTIVE, NOT THE NEEDLE
-----------------------------------------------
The focus drive moves the objective toward the specimen, so a sweep is bounded by
working distance. That bound lives in ``FocusSweepPlanner`` (pure, testable), is
layered under the SDK's declared travel and the operator's soft limits, and is
verified by reading the focus position BACK after every move — ``set_focus_um``
clamps silently, and a clamped sample is a guess, not a measurement.

THE EXIT GUARANTEE
------------------
On every exit path — Accept, Cancel, error, page hidden, app close — the needle
has not moved toward the plate, the objective and focus drive are back where they
started, the position poller is running, the microscope lease is released, and
the installed plane is either the one just accepted or exactly the one that was
there before.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field, replace
from typing import Optional

from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QMessageBox, QProgressBar, QPushButton,
    QSizePolicy, QSpinBox, QSplitter, QStackedWidget, QTextEdit, QVBoxLayout,
    QWidget,
)

from gui.scaling import s, sp
from gui.styles import COLORS, SECTION_TITLE_STYLE

logger = logging.getLogger(__name__)

try:
    from SupportClasses.ObjectiveOptics import (
        ObjectiveOptics, depth_of_field_um, fov_um, from_mounted_optic)
    from SupportClasses.FocusCurve import FocusSample, peak_focus_um
    from SupportClasses.FocusSweepPlanner import (
        plan_sweep, plan_handoff, next_rung, DEFAULT_TURRET_S)
    from SupportClasses.ObjectiveLadder import (
        resolve_ladder, ladder_gate, parfocal_offsets_um)
    from SupportClasses.PlateLeveling import (
        MIN_SITES, SiteMeasurement, sites_gate, sites_geometry, propose_sites,
        solve, update_prior, DEFAULT_TOLERANCE_MM)
    LEVEL_AVAILABLE = True
except ImportError:                                   # pragma: no cover
    LEVEL_AVAILABLE = False


# ── steps ─────────────────────────────────────────────────────────────────

STEP_PREFLIGHT = "preflight"
STEP_OPTICS = "optics"
STEP_SITES = "sites"
STEP_SURVEY = "survey"
STEP_VERIFY = "verify"
STEP_ORDER = (STEP_PREFLIGHT, STEP_OPTICS, STEP_SITES, STEP_SURVEY, STEP_VERIFY)
STEP_TITLES = {
    STEP_PREFLIGHT: "1 · Preflight",
    STEP_OPTICS: "2 · Objectives",
    STEP_SITES: "3 · Sites",
    STEP_SURVEY: "4 · Survey",
    STEP_VERIFY: "5 · Verify & accept",
}

#: Per-microscope-op wait. Generous: a turret rotation is multi-hundred ms and a
#: body under load can take seconds.
OP_TIMEOUT_S = 20.0
#: Settle after a focus move before sampling the frame counter.
DEFAULT_SETTLE_MS = 300
#: Fresh frames to wait for after a move (the only legal off-GUI-thread grab).
DEFAULT_FRESH_FRAMES = 2
FRESH_TIMEOUT_S = 3.0
#: Anchor focus may drift this far during a run before the result is suspect.
MAX_ANCHOR_DRIFT_UM = 10.0
#: The operator has this long to confirm a feature before the survey gives up.
FEATURE_TIMEOUT_S = 600.0
#: Template match confidence floor — the auto-re-anchor precedent.
MIN_MATCH_CONF = 0.5

_LEASE = "plate_level"


class _Cancelled(RuntimeError):
    pass


class _Abort(RuntimeError):
    """Carries operator-actionable text."""


# ════════════════════════════════════════════════════════════════════
#  Qt-free state — directly testable
# ════════════════════════════════════════════════════════════════════

@dataclass(frozen=True, kw_only=True)
class SiteSpec:
    label: str
    x_um: float
    y_um: float


@dataclass
class SurveyState:
    """Everything one run accumulates. No Qt, so tests drive it directly."""
    sites: list = field(default_factory=list)
    measurements: dict = field(default_factory=dict)     # label -> SiteMeasurement
    per_objective_peaks: dict = field(default_factory=dict)
    anchor_focus_um: Optional[float] = None
    anchor_focus_closing_um: Optional[float] = None
    run_offset_um: Optional[float] = None

    def ordered(self) -> list:
        return [self.measurements[s.label] for s in self.sites
                if s.label in self.measurements]

    def complete(self) -> bool:
        return len(self.measurements) >= len(self.sites) > 0

    def anchor_drift_um(self) -> Optional[float]:
        if self.anchor_focus_um is None or self.anchor_focus_closing_um is None:
            return None
        return abs(self.anchor_focus_closing_um - self.anchor_focus_um)


# ════════════════════════════════════════════════════════════════════
#  Step strip
# ════════════════════════════════════════════════════════════════════

class _StepStrip(QWidget):
    step_clicked = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, s(4))
        lay.setSpacing(s(4))
        self._buttons: dict = {}
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
#  Survey worker
# ════════════════════════════════════════════════════════════════════

class PlateLevelSurveyWorker(QThread):
    """Drives stage + microscope + camera through the survey.

    Constructed with PLAIN DATA only — no widgets, no host — so every failure
    branch is reachable from a test with hand-written fakes.
    """

    progress = Signal(int, int, str)
    site_started = Signal(int, object)
    awaiting_feature = Signal(int, object)
    focus_sample = Signal(int, float, float, str)
    site_measured = Signal(int, object)
    finished_ok = Signal(object)
    failed = Signal(str)

    def __init__(self, *, controller, scope, cam, cam_mgr, cam_idx,
                 sites, ladder, safe_z_zref_mm,
                 focus_limits_um=None, soft_limits_um=None,
                 settle_ms: int = DEFAULT_SETTLE_MS,
                 fresh_frames: int = DEFAULT_FRESH_FRAMES,
                 prior=None, banked=None, teach_all: bool = False,
                 parent=None):
        super().__init__(parent)
        self._ctrl = controller
        self._scope = scope
        self._cam = cam
        self._mgr = cam_mgr
        self._cam_idx = cam_idx
        self._sites = list(sites or ())
        self._ladder = list(ladder or ())
        self._safe_z = safe_z_zref_mm
        self._focus_limits = focus_limits_um
        self._soft_limits = soft_limits_um
        self._settle_ms = int(settle_ms)
        self._fresh = int(fresh_frames)
        self._prior = prior
        self._banked = banked or {}
        self._teach_all = bool(teach_all)

        self._stop = False
        self._state = SurveyState(sites=list(self._sites))
        self._feature_px: Optional[tuple] = None
        self._feature_ready = False
        self._parfocal: dict = {}

    # ── control ──────────────────────────────────────────────────

    def stop(self) -> None:
        self._stop = True

    def set_feature(self, cx, cy) -> None:
        """Called from the GUI thread when the operator confirms a patch."""
        self._feature_px = None if cx is None else (float(cx), float(cy))
        self._feature_ready = True

    # ── microscope plumbing ──────────────────────────────────────

    def _scope_op(self, op, what: str, timeout_s: float = OP_TIMEOUT_S) -> None:
        """Submit-and-wait for ONE op. ``done`` alone is NOT success.

        The controller drops any op that waited longer than ``STALE_OP_S`` and
        sets ``done`` with ``error='dropped (stale)'``. A dropped set_objective
        means every later sample is taken through the WRONG objective — silently,
        and the resulting plane looks entirely well-formed. So the error is
        checked on every call, and a stale drop ABORTS rather than retries: a
        retry re-queues behind exactly the same backlog.
        """
        if op is None:
            raise _Abort(f"the microscope refused {what}")
        done = getattr(op, "done", None)
        if done is not None and not done.wait(timeout_s):
            raise _Abort(
                f"The microscope did not finish {what} within {timeout_s:.0f} s "
                f"— the body may be wedged. Reconnect it and re-run.")
        err = getattr(op, "error", None)
        if err == "dropped (stale)":
            raise _Abort(
                "Another part of the app is driving the microscope (the jog "
                "panel's Microscope card, or Hardware Setup → Microscope). The "
                "survey cannot share the body — close those and re-run.")
        if err:
            raise _Abort(f"The microscope refused {what}: {err}")

    def _focus_now_um(self) -> Optional[float]:
        try:
            st = self._scope.state()
            return getattr(st, "focus_um", None)
        except Exception:
            return None

    def _goto_focus(self, z_um: float, lead_in_um: float) -> float:
        """Move to ``z_um``, always approached from below. Returns the READBACK.

        The readback is not a formality: ``set_focus_um`` clamps to the operator
        soft limits BEFORE queueing, so a commanded value that hits a limit
        arrives silently short.
        """
        if lead_in_um and lead_in_um > 0:
            self._scope_op(self._scope.set_focus_um(z_um - lead_in_um),
                           "moving the focus", 10.0)
        self._scope_op(self._scope.set_focus_um(z_um), "moving the focus", 10.0)
        back = self._focus_now_um()
        return z_um if back is None else float(back)

    # ── frames ───────────────────────────────────────────────────

    def _grab(self):
        """Fresh frame via the counter pattern — the only legal off-thread grab.

        ``capture_fresh_frame`` touches the backend directly and is documented as
        unsafe to call concurrently with the display timer from another thread.
        """
        try:
            time.sleep(self._settle_ms / 1000.0)
            start = int(self._cam.frame_count_value())
            deadline = time.monotonic() + FRESH_TIMEOUT_S
            while time.monotonic() < deadline:
                if self._stop:
                    raise _Cancelled
                if int(self._cam.frame_count_value()) - start >= self._fresh:
                    break
                time.sleep(0.02)
            return self._cam.get_current_frame()
        except _Cancelled:
            raise
        except Exception as e:
            logger.debug(f"plate level: frame grab failed: {e}")
            return None

    # ── the run ──────────────────────────────────────────────────

    def run(self):                                    # noqa: C901
        if not self._scope.try_acquire(_LEASE, timeout=2.0):
            who = self._scope.lease_owner()
            self.failed.emit(
                f"Another part of the app holds the microscope ({who}). "
                f"Close it and re-run.")
            return

        entry_focus = self._focus_now_um()
        entry_obj = None
        try:
            entry_obj = self._scope.state().objective_position
        except Exception:
            pass

        self._suspend_poller()
        err = None
        try:
            self._survey()
        except _Cancelled:
            err = "cancelled"
        except _Abort as e:
            err = str(e)
        except Exception as e:                        # pragma: no cover
            logger.exception("plate level survey raised")
            err = f"The survey stopped: {e}"
        finally:
            note = self._restore(entry_obj, entry_focus)
            self._resume_poller()
            try:
                self._scope.release(_LEASE)
            except Exception:
                pass

        if err == "cancelled":
            return                                    # no signal on cancel
        if err:
            self.failed.emit(err + note)
        else:
            self.finished_ok.emit(self._state)

    def _restore(self, entry_obj, entry_focus) -> str:
        """Put the body back. Never raises; reports what it could not do."""
        problems = []
        try:
            if entry_obj:
                op = self._scope.set_objective(int(entry_obj))
                if getattr(op, "done", None):
                    op.done.wait(OP_TIMEOUT_S)
        except Exception:
            problems.append("the objective did not return to its start position")
        try:
            if entry_focus is not None:
                op = self._scope.set_focus_um(float(entry_focus))
                if getattr(op, "done", None):
                    op.done.wait(OP_TIMEOUT_S)
                back = self._focus_now_um()
                if back is not None and abs(back - float(entry_focus)) > 5.0:
                    problems.append(
                        f"the focus drive is at {back:.0f} µm and did not return "
                        f"to {float(entry_focus):.0f} µm")
        except Exception:
            problems.append("the focus drive did not return to its start value")
        return ("  ⚠ " + "; ".join(problems) +
                " — check the body before moving the objective.") \
            if problems else ""

    def _suspend_poller(self) -> None:
        try:
            self._ctrl.suspend_position_poller()
        except Exception:
            pass

    def _resume_poller(self) -> None:
        try:
            self._ctrl.resume_position_poller()
        except Exception:
            pass

    def _survey(self) -> None:
        total = max(1, len(self._sites))
        for idx, site in enumerate(self._sites):
            if self._stop:
                raise _Cancelled
            self.progress.emit(idx, total, f"travelling to {site.label}")
            self.site_started.emit(idx, (site.x_um, site.y_um))
            self._travel(site)
            m = self._measure_site(idx, site)
            self._state.measurements[site.label] = m
            self.site_measured.emit(idx, m)
            if idx == 0 and m.focus_um is not None:
                self._state.anchor_focus_um = m.focus_um
            self.progress.emit(idx + 1, total, f"{site.label} done")

        # Closing drift check: re-measure site 1. Ti focus drives creep
        # 0.1-1 µm/min, which over a ten-minute survey is a real contributor to
        # the residuals — and it is invisible unless it is measured.
        if self._sites and self._state.anchor_focus_um is not None:
            if self._stop:
                raise _Cancelled
            self.progress.emit(total, total, "re-checking the first site")
            self._travel(self._sites[0])
            closing = self._measure_site(0, self._sites[0], closing=True)
            self._state.anchor_focus_closing_um = closing.focus_um

    def _travel(self, site: SiteSpec) -> None:
        """Retract → XY → wait. NEVER lowers: target_z_mm is None."""
        try:
            ok = self._ctrl.safe_travel_to(
                float(site.x_um), float(site.y_um),
                safe_z_mm=float(self._safe_z), target_z_mm=None,
                apply_insert_floor=False)
        except Exception as e:
            raise _Abort(f"Travel to {site.label} failed: {e}")
        if ok is False:
            raise _Abort(
                f"The stage refused to travel to {site.label}. The needle was "
                f"not retracted, or the Z board is disconnected — the survey "
                f"stopped rather than move XY with the needle down.")

    # ── one site ─────────────────────────────────────────────────

    def _measure_site(self, idx: int, site: SiteSpec,
                      closing: bool = False) -> SiteMeasurement:
        peak_um = None
        sigma = fwhm = prom = None
        refusal = ""
        clamped = False
        at_edge = False
        obj_name = ""
        prev_peak = None
        prev_sigma = None

        for rung_i, rung in enumerate(self._ladder):
            if self._stop:
                raise _Cancelled
            obj_name = rung.objective_name
            self._scope_op(self._scope.set_objective(int(rung.turret_position)),
                           f"switching to {obj_name}")

            optics = rung.optics
            if rung_i == 0:
                centre = self._first_centre(site)
                half = self._first_half_range(site)
                plan, why = plan_sweep(
                    optics=optics, center_um=centre,
                    requested_half_range_um=half,
                    focus_limits_um=self._focus_limits,
                    soft_limits_um=self._soft_limits)
            else:
                delta = self._parfocal.get(obj_name)
                plan, why = plan_handoff(
                    to_optics=optics, prior_peak_um=prev_peak,
                    prior_sigma_um=prev_sigma,
                    parfocal_delta_um=(delta or 0.0),
                    focus_limits_um=self._focus_limits,
                    soft_limits_um=self._soft_limits)
            if plan is None:
                return SiteMeasurement(
                    label=site.label, x_stage_um=site.x_um, y_stage_um=site.y_um,
                    objective_name=obj_name, refusal=why)
            if plan.clamped:
                clamped = True

            pk, why = self._run_plan(idx, site, plan, optics, obj_name,
                                     teach=(idx == 0 or self._teach_all))
            if pk is None:
                refusal = why
                at_edge = "PEAK_AT_EDGE" in why
                break
            prev_peak, prev_sigma = pk.z_um, pk.sigma_z_um
            peak_um, sigma, fwhm, prom = (pk.z_um, pk.sigma_z_um, pk.fwhm_um,
                                          pk.prominence)
            self._state.per_objective_peaks.setdefault(obj_name, []).append(
                pk.z_um)

        if not closing and len(self._ladder) > 1:
            self._update_parfocal()

        return SiteMeasurement(
            label=site.label, x_stage_um=site.x_um, y_stage_um=site.y_um,
            focus_um=peak_um, focus_sigma_um=sigma, fwhm_um=fwhm,
            prominence=prom, objective_name=obj_name, clamped=clamped,
            peak_at_edge=at_edge, refusal=refusal)

    def _first_centre(self, site: SiteSpec) -> float:
        """Where to start looking, from the prior — a SEARCH HINT only."""
        cur = self._focus_now_um() or 0.0
        if self._state.run_offset_um is not None and self._prior is not None:
            pred = self._prior.predict_focus_um(site.x_um, site.y_um,
                                                self._state.run_offset_um)
            if pred is not None:
                return pred
        # After three sites, this run's own fit predicts better than any prior.
        live = self._predict_from_run(site)
        if live is not None:
            return live
        if self._prior is not None and self._prior.ref_focus_um is not None:
            pred = self._prior.predict_focus_um(site.x_um, site.y_um, 0.0)
            if pred is not None:
                return pred
        return cur

    def _first_half_range(self, site: SiteSpec) -> float:
        if self._predict_from_run(site) is not None:
            return 60.0
        if self._prior is not None:
            return self._prior.first_site_half_range_um()
        return 1000.0

    def _predict_from_run(self, site: SiteSpec) -> Optional[float]:
        """Predict this site's focus from THIS plate's own measured sites.

        Only ever narrows the search window; the fit that gets accepted is
        recomputed from the measurements at the end, never from this.
        """
        done = [m for m in self._state.measurements.values() if m.usable]
        if len(done) < 3:
            return None
        try:
            import numpy as np
            A = np.array([[m.x_stage_um / 1000.0, m.y_stage_um / 1000.0, 1.0]
                          for m in done], dtype=float)
            b = np.array([m.focus_um for m in done], dtype=float)
            sol, *_ = np.linalg.lstsq(A, b, rcond=None)
            return float(sol[0] * site.x_um / 1000.0
                         + sol[1] * site.y_um / 1000.0 + sol[2])
        except Exception:
            return None

    def _run_plan(self, idx, site, plan, optics, obj_name, teach: bool):
        """Walk a plan's rungs, re-centring each on the measured peak."""
        peak = None
        rung = plan.rungs[0] if plan.rungs else None
        dof = depth_of_field_um(optics)
        for r_i in range(len(plan.rungs)):
            if rung is None:
                break
            samples = []
            for z in rung.z_targets_um:
                if self._stop:
                    raise _Cancelled
                back = self._goto_focus(z, rung.lead_in_um)
                if abs(back - z) > max(1e-6, rung.step_um * 0.5):
                    # Hit a limit. A clamped sample is a guess, not data.
                    break
                frame = self._grab()
                if frame is None:
                    continue
                score, roi, sat = self._score(frame, teach and r_i == 0
                                              and not samples, idx)
                if score is None:
                    continue
                samples.append(FocusSample(
                    z_um=back, score=float(score), roi_rect=roi,
                    saturated_frac=float(sat),
                    settings_fingerprint=self._fingerprint(),
                    focus_readback_um=back))
                self.focus_sample.emit(idx, float(back), float(score), obj_name)
            peak, why = peak_focus_um(samples, dof_um=dof)
            if peak is None:
                return (None, why)
            rung = next_rung(plan=plan, rung_index=r_i,
                             measured_peak_um=peak.z_um,
                             focus_limits_um=self._focus_limits,
                             soft_limits_um=self._soft_limits)
        return (peak, "")

    def _fingerprint(self) -> str:
        try:
            st = self._mgr.get_hw_settings(self._cam_idx) or {}
            return "|".join(f"{k}={st.get(k)}" for k in sorted(st)
                            if "expo" in k.lower() or "gain" in k.lower())
        except Exception:
            return ""

    def _score(self, frame, want_feature: bool, idx: int):
        """Focus score inside the feature ROI. ``(score, roi, saturated_frac)``."""
        try:
            from SupportClasses.VisionDetector import NeedleDetector
            import numpy as np
        except ImportError:
            return (None, None, 0.0)
        h, w = frame.shape[:2]
        if want_feature and self._feature_px is None:
            self.awaiting_feature.emit(idx, frame)
            self._await_feature()
        cx, cy = (self._feature_px or (w / 2.0, h / 2.0))
        side = max(48, int(min(w, h) * 0.15))
        x0 = int(max(0, min(w - side, cx - side / 2)))
        y0 = int(max(0, min(h - side, cy - side / 2)))
        roi = (x0, y0, side, side)
        try:
            sub = frame[y0:y0 + side, x0:x0 + side]
            mx = float(np.max(sub)) if sub.size else 0.0
            sat = (float(np.count_nonzero(sub >= 254)) / max(1, sub.size)
                   if mx >= 254 else 0.0)
            res = NeedleDetector.compute_focus_score(frame, roi)
            return (float(res.score), roi, sat)
        except Exception as e:
            logger.debug(f"focus score failed: {e}")
            return (None, roi, 0.0)

    def _await_feature(self) -> None:
        """Wait for the operator's patch WITHOUT deadlocking cancel.

        A bare ``Event.wait()`` here is the classic bug: the operator walks away,
        the page is hidden, cancel calls stop() then wait() — and the worker is
        parked on something nobody will ever set.
        """
        deadline = time.monotonic() + FEATURE_TIMEOUT_S
        while not self._feature_ready:
            if self._stop:
                raise _Cancelled
            if time.monotonic() > deadline:
                raise _Abort(
                    f"No feature was confirmed within "
                    f"{FEATURE_TIMEOUT_S / 60:.0f} minutes — the survey stopped.")
            time.sleep(0.1)

    def _update_parfocal(self) -> None:
        try:
            offsets, _spread, _ref = parfocal_offsets_um(
                self._state.per_objective_peaks)
            self._parfocal = offsets or {}
        except Exception:
            pass


# ════════════════════════════════════════════════════════════════════
#  The wizard
# ════════════════════════════════════════════════════════════════════

class PlateLevelWizard(QWidget):
    """Optical plate-bed leveling. The host is ``CalibrationPage``."""

    calibration_changed = Signal()

    def __init__(self, host, parent=None):
        super().__init__(parent)
        self._host = host
        self._current = STEP_PREFLIGHT
        self._worker: Optional[PlateLevelSurveyWorker] = None
        self._state: Optional[SurveyState] = None
        self._solution = None
        self._sites: list = []
        self._ladder_positions: list = []
        self._tolerance_mm = DEFAULT_TOLERANCE_MM
        self._build_ui()
        self.refresh()

    # ── host accessors (all guarded: the host is duck-typed) ─────

    @property
    def _ctrl(self):
        return getattr(self._host, "controller", None)

    @property
    def _scope(self):
        try:
            from SupportClasses.MicroscopeControl import get_microscope
            return get_microscope()
        except Exception:
            return None

    def _cam_idx(self):
        fn = getattr(self._host, "_ploc_microscope_cam_idx", None)
        try:
            return fn() if callable(fn) else None
        except Exception:
            return None

    def _mgr(self):
        """The shared CameraManager.

        The page stores it as ``_camera_manager``; there is no public
        ``camera_manager``. Reading the public spelling returned None on every
        real run while a test stub that happened to define it stayed green, so
        the survey was built with ``cam=None`` and could not grab a frame.
        ``_ploc_*`` and the bore wizard both use this spelling — match them.
        """
        return getattr(self._host, "_camera_manager", None)

    def _cam(self):
        mgr = self._mgr()
        idx = self._cam_idx()
        if mgr is None or idx is None:
            return None
        try:
            # `cameras` is a PROPERTY returning a list. Calling it raises
            # TypeError, which the except below would have swallowed silently.
            cams = mgr.cameras
            return cams[idx] if 0 <= idx < len(cams) else None
        except Exception:
            return None

    def _plate_key(self):
        fn = getattr(self._host, "_ploc_plate_key", None)
        try:
            return fn() if callable(fn) else "plate"
        except Exception:
            return "plate"

    def _well_positions(self) -> dict:
        return dict(getattr(self._host, "_calibrated_positions", None)
                    or getattr(self._host, "_predicted_positions", None) or {})

    def _camera_name(self) -> str:
        mgr = self._mgr()
        idx = self._cam_idx()
        if mgr is None or idx is None:
            return ""
        try:
            ident = mgr.camera_identity(idx)
            return ident[0] if ident else ""
        except Exception:
            return ""

    # ── gates ────────────────────────────────────────────────────

    def gate(self, step: str) -> tuple[bool, str]:
        """``(ok, why)`` — cumulative, operator-actionable sentences."""
        if not LEVEL_AVAILABLE:
            return (False, "The plate-leveling modules are unavailable.")
        c = self._ctrl
        if c is None:
            return (False, "No stage controller.")

        if not getattr(c, "is_xy_connected", False):
            return (False, "Connect the XY stage — every site is reached by an "
                           "absolute XY move.")
        if not getattr(c, "is_zp_connected", False):
            return (False, "Connect the Z board — the wizard reads the needle "
                           "height to confirm it is retracted before the "
                           "objective moves.")
        scope = self._scope
        st = None
        try:
            st = scope.state() if scope else None
        except Exception:
            st = None
        if st is None or not getattr(st, "connected", False):
            return (False, "Connect the microscope body (the Connect Hardware "
                           "card, or the jog panel's Microscope card). This "
                           "calibration drives the nosepiece and the focus "
                           "drive.")
        if not getattr(st, "has_focus", False):
            return (False, "This microscope reports no motorised focus drive. "
                           "Plate leveling is optical — it needs the focus axis.")
        if self._cam_idx() is None:
            return (False, "No camera has the Microscope role — assign it on "
                           "Hardware Setup → Cameras.")
        if getattr(self._host, "_safe_z", None) is None:
            return (False, "Set the Fast Move (Safe) Z — every site is reached "
                           "by a retract-then-travel.")
        try:
            lo, hi = self._focus_soft_limits()
        except Exception:
            lo = hi = None
        if lo is None or hi is None:
            return (False, "Set the microscope focus soft limits on Hardware "
                           "Setup → Microscope. The focus drive raises the "
                           "objective toward the glass; without limits an "
                           "autofocus sweep can push the front lens into the "
                           "plate.")
        if getattr(c, "get_plate_bottom_z", lambda: None)() is None:
            return (False, "Teach the Plate Bottom Z first (Needle Location → "
                           "plate touch-off). The optical survey measures the "
                           "plate's TILT; only the needle touch-off knows how "
                           "deep the glass actually is.")
        if getattr(c, "get_plate_bottom_anchor_xy_um", lambda: None)() is None:
            return (False, "The taught Plate Bottom Z has no recorded XY. Re-do "
                           "the touch-off so the wizard knows WHERE that height "
                           "was measured — a plane anchored at an unknown "
                           "position is not a plane.")
        try:
            if c.plate_bottom_z_is_stale():
                return (False, "Set Z Zero has run since the Plate Bottom Z was "
                               "taught. Re-touch the plate bottom before "
                               "leveling — the whole plane hangs off that "
                               "number.")
        except Exception:
            pass
        if not self._well_positions():
            return (False, "Finish Plate Location first — the site proposals "
                           "and the travel come from the calibrated well map.")
        if step == STEP_PREFLIGHT:
            return (True, "")

        rungs = self._resolve_ladder()
        ok, why = ladder_gate(
            [r for r in rungs if r.turret_position in self._ladder_positions],
            live_turret_position=getattr(st, "objective_position", None),
            current_objective_name=self._current_objective_name())
        if not ok:
            return (False, why)
        if step == STEP_OPTICS:
            return (True, "")

        ok, why = sites_gate(
            [(s_.x_um, s_.y_um) for s_ in self._sites],
            footprint_um=self._footprint_um(), fov_um=self._fov_um())
        if not ok:
            return (False, why)
        if step == STEP_SITES:
            return (True, "")

        if self._state is None or not self._state.complete():
            done = len(self._state.measurements) if self._state else 0
            return (False, f"{done} of {len(self._sites)} sites measured. "
                           f"Finish the survey, or drop the unmeasured sites.")
        return (True, "")

    def step_state(self, step: str) -> str:
        ok, _ = self.gate(step)
        if not ok:
            return "todo"
        if step == STEP_SURVEY and self._state and self._state.complete():
            return "ok"
        if step == STEP_VERIFY:
            if self._solution is None:
                return "todo"
            return "ok" if self._solution.ok else "warn"
        return "ok"

    # ── helpers ──────────────────────────────────────────────────

    def _focus_soft_limits(self):
        from SupportClasses.MicroscopeConfigStore import get_store
        return get_store().focus_soft_limits_um()

    def _focus_hw_limits(self):
        try:
            st = self._scope.state()
            return (getattr(st, "focus_min_um", None),
                    getattr(st, "focus_max_um", None))
        except Exception:
            return (None, None)

    def _current_objective_name(self):
        hw = getattr(self._host, "_hardware_config", None)
        cc = getattr(hw, "camera_config", None) if hw else None
        return getattr(cc, "current_objective_name", None) if cc else None

    def _footprint_um(self):
        pos = self._well_positions()
        if not pos:
            return None
        xs = [p[0] for p in pos.values()]
        ys = [p[1] for p in pos.values()]
        return (min(xs), min(ys), max(xs), max(ys))

    def _fov_um(self):
        rungs = [r for r in self._resolve_ladder()
                 if r.turret_position in self._ladder_positions]
        for r in rungs:
            o = getattr(r, "optics", None)
            if o is not None:
                f = fov_um(o)
                if f:
                    return f[0]
        return None

    def _resolve_ladder(self):
        """Every nosepiece position, with optics attached where resolvable."""
        try:
            from SupportClasses.ObjectiveCalibration import get_store as obj_store
            from SupportClasses.MicroscopeConfigStore import get_store as cfg
            st = self._scope.state()
            frame = None
            cam = self._cam()
            if cam is not None:
                try:
                    f = cam.get_current_frame()
                    if f is not None:
                        frame = (int(f.shape[1]), int(f.shape[0]))
                except Exception:
                    frame = None
            rungs = resolve_ladder(
                scope_state=st, config_store=cfg(), objective_store=obj_store(),
                camera_name=self._camera_name(), live_resolution=frame)
            out = []
            for r in rungs:
                optic = next((o for o in (getattr(st, "mounted_objectives", ())
                                          or ())
                              if getattr(o, "position", 0) == r.turret_position),
                             None)
                optics = None
                if r.calibrated and optic is not None:
                    optics, _why = from_mounted_optic(
                        optic, um_per_px_sample=r.um_per_px, frame_wh=frame)
                out.append(_RungWithOptics(r, optics))
            return out
        except Exception as e:
            logger.debug(f"ladder resolve failed: {e}")
            return []

    # ── UI ───────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(8), s(8), s(8), s(8))
        outer.setSpacing(s(6))

        self._strip = _StepStrip()
        self._strip.step_clicked.connect(self.go_to_step)
        outer.addWidget(self._strip)

        self._status = QLabel("—")
        self._status.setWordWrap(True)
        self._status.setStyleSheet(f"color: {COLORS['subtext0']};")
        outer.addWidget(self._status)

        self._stack = QStackedWidget()
        self._pages = {
            STEP_PREFLIGHT: self._build_preflight(),
            STEP_OPTICS: self._build_optics(),
            STEP_SITES: self._build_sites(),
            STEP_SURVEY: self._build_survey(),
            STEP_VERIFY: self._build_verify(),
        }
        for key in STEP_ORDER:
            self._stack.addWidget(self._pages[key])
        outer.addWidget(self._stack, 1)

    def _build_preflight(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addWidget(_title("Preflight"))
        self._pre_text = QLabel()
        self._pre_text.setWordWrap(True)
        lay.addWidget(self._pre_text)
        lay.addStretch()
        return w

    def _build_optics(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addWidget(_title("Objective ladder"))
        lay.addWidget(QLabel(
            "Pick the objectives the survey climbs, lowest magnification first. "
            "The wide objective acquires the feature; each later one narrows the "
            "Z window. Parfocal offsets are measured for every objective you "
            "include."))
        self._obj_list = QListWidget()
        self._obj_list.itemChanged.connect(lambda *_: self._on_ladder_changed())
        lay.addWidget(self._obj_list, 1)
        self._obj_note = QLabel()
        self._obj_note.setWordWrap(True)
        lay.addWidget(self._obj_note)
        return w

    def _build_sites(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addWidget(_title("Survey sites"))
        row = QHBoxLayout()
        row.addWidget(QLabel("Sites:"))
        self._n_sites = QSpinBox()
        self._n_sites.setRange(MIN_SITES, 12)
        self._n_sites.setValue(5)
        row.addWidget(self._n_sites)
        btn = QPushButton("Propose sites")
        btn.clicked.connect(self._propose)
        row.addWidget(btn)
        row.addStretch()
        lay.addLayout(row)
        self._site_list = QListWidget()
        lay.addWidget(self._site_list, 1)
        self._site_note = QLabel()
        self._site_note.setWordWrap(True)
        lay.addWidget(self._site_note)
        return w

    def _build_survey(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addWidget(_title("Survey"))
        self._bar = QProgressBar()
        lay.addWidget(self._bar)
        self._survey_note = QLabel("Not started.")
        self._survey_note.setWordWrap(True)
        lay.addWidget(self._survey_note)
        row = QHBoxLayout()
        self._btn_start = QPushButton("Start survey")
        self._btn_start.clicked.connect(self._start)
        row.addWidget(self._btn_start)
        self._btn_cancel = QPushButton("Cancel")
        self._btn_cancel.clicked.connect(self._cancel)
        self._btn_cancel.setEnabled(False)
        row.addWidget(self._btn_cancel)
        row.addStretch()
        lay.addLayout(row)
        self._survey_log = QTextEdit()
        self._survey_log.setReadOnly(True)
        lay.addWidget(self._survey_log, 1)
        return w

    def _build_verify(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addWidget(_title("Verify & accept"))
        self._verify_text = QTextEdit()
        self._verify_text.setReadOnly(True)
        lay.addWidget(self._verify_text, 1)
        row = QHBoxLayout()
        self._btn_accept = QPushButton("Accept and install plane")
        self._btn_accept.clicked.connect(self._accept)
        self._btn_accept.setEnabled(False)
        row.addWidget(self._btn_accept)
        row.addStretch()
        lay.addLayout(row)
        return w

    # ── navigation / render ──────────────────────────────────────

    def go_to_step(self, key: str) -> None:
        if key not in STEP_ORDER:
            return
        self._current = key
        self._stack.setCurrentWidget(self._pages[key])
        self.refresh()

    def refresh(self) -> None:
        states = {k: self.step_state(k) for k in STEP_ORDER}
        self._strip.render(self._current, states)
        ok, why = self.gate(self._current)
        self._status.setText(why or "Ready.")
        self._status.setStyleSheet(
            f"color: {COLORS['green'] if ok else COLORS['yellow']};")
        if self._current == STEP_PREFLIGHT:
            self._render_preflight()
        elif self._current == STEP_OPTICS:
            self._render_optics()
        elif self._current == STEP_SITES:
            self._render_sites()
        self._btn_start.setEnabled(
            self.gate(STEP_SITES)[0] and self._worker is None)

    def _render_preflight(self) -> None:
        c = self._ctrl
        bits = []
        try:
            pb = c.get_plate_bottom_z()
            src = c.get_plate_bottom_z_source() or "unknown"
            bits.append(f"Plate bottom Z: {pb:.3f} mm ({src})")
            if src == "estimated":
                bits.append("⚠ That is an ESTIMATE, not a taught touch-off. "
                            "Anchoring a plane to it spreads its error across "
                            "the whole plate — re-touch before accepting.")
        except Exception:
            pass
        self._pre_text.setText("\n".join(bits) or "—")

    def _render_optics(self) -> None:
        self._obj_list.blockSignals(True)
        self._obj_list.clear()
        notes = []
        for r in self._resolve_ladder():
            item = QListWidgetItem(
                f"{r.turret_position}: {r.describe()}")
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(
                Qt.Checked if r.turret_position in self._ladder_positions
                else Qt.Unchecked)
            item.setData(Qt.UserRole, r.turret_position)
            if not r.calibrated:
                item.setFlags(item.flags() & ~Qt.ItemIsEnabled)
                notes.append(r.why_not)
            self._obj_list.addItem(item)
        self._obj_list.blockSignals(False)
        self._obj_note.setText("\n\n".join(notes))

    def _on_ladder_changed(self) -> None:
        picked = []
        for i in range(self._obj_list.count()):
            it = self._obj_list.item(i)
            if it.checkState() == Qt.Checked:
                picked.append(int(it.data(Qt.UserRole)))
        self._ladder_positions = picked
        self.refresh()

    def _propose(self) -> None:
        anchor = None
        try:
            anchor = self._ctrl.get_plate_bottom_anchor_xy_um()
        except Exception:
            pass
        names = propose_sites(well_positions_um=self._well_positions(),
                              anchor_xy_um=anchor, n=self._n_sites.value(),
                              footprint_um=self._footprint_um())
        pos = self._well_positions()
        self._sites = [SiteSpec(label=n, x_um=pos[n][0], y_um=pos[n][1])
                       for n in names if n in pos]
        self.refresh()

    def _render_sites(self) -> None:
        self._site_list.clear()
        for i, sp_ in enumerate(self._sites):
            tag = "  (anchor)" if i == 0 else ""
            self._site_list.addItem(
                f"{sp_.label}{tag}   {sp_.x_um / 1000:.1f}, "
                f"{sp_.y_um / 1000:.1f} mm")
        if not self._sites:
            self._site_note.setText("No sites yet — press Propose sites.")
            return
        g = sites_geometry([(x.x_um, x.y_um) for x in self._sites],
                           footprint_um=self._footprint_um())
        n_obj = max(1, len(self._ladder_positions))
        est = len(self._sites) * n_obj * 40 * 0.3 / 60.0
        self._site_note.setText(
            f"{g.count} sites · largest triangle {g.triangle_max_area_mm2:.0f} "
            f"mm² · span {g.span_mm:.0f} mm · about {est:.0f} min")

    # ── run ──────────────────────────────────────────────────────

    def _start(self) -> None:
        ok, why = self.gate(STEP_SITES)
        if not ok:
            QMessageBox.warning(self, "Cannot start", why)
            return
        rungs = [r for r in self._resolve_ladder()
                 if r.turret_position in self._ladder_positions]
        rungs.sort(key=lambda r: (r.magnification or 0))
        self._state = None
        self._solution = None
        self._survey_log.clear()
        hw_lo, hw_hi = self._focus_hw_limits()
        soft = self._focus_soft_limits()
        prior = None
        try:
            from SupportClasses.PlateLevelSiteStore import get_store
            prior = get_store().get_prior(self._plate_key(),
                                          self._camera_name())
        except Exception:
            pass
        self._worker = PlateLevelSurveyWorker(
            controller=self._ctrl, scope=self._scope, cam=self._cam(),
            cam_mgr=self._mgr(),
            cam_idx=self._cam_idx(), sites=self._sites, ladder=rungs,
            safe_z_zref_mm=getattr(self._host, "_safe_z", None),
            focus_limits_um=(hw_lo, hw_hi), soft_limits_um=soft, prior=prior)
        self._worker.progress.connect(self._on_progress)
        self._worker.focus_sample.connect(self._on_focus_sample)
        self._worker.site_measured.connect(self._on_site)
        self._worker.finished_ok.connect(self._on_done)
        self._worker.failed.connect(self._on_failed)
        self._worker.awaiting_feature.connect(self._on_feature_request)
        self._btn_cancel.setEnabled(True)
        self._btn_start.setEnabled(False)
        self._worker.start()

    def _on_progress(self, done, total, phase) -> None:
        self._bar.setMaximum(total)
        self._bar.setValue(done)
        self._survey_note.setText(phase)

    def _on_focus_sample(self, idx, focus_um, score, obj_name) -> None:
        """Live per-frame feedback during a multi-minute survey.

        Deliberately overwrites one status line rather than appending: a rung is
        ~39 frames and appending each would bury the per-site verdicts the log
        exists to show. This signal was emitted and disconnected but never
        connected, so the operator saw nothing move between sites.
        """
        self._survey_note.setText(
            f"{obj_name}: focus {focus_um:.1f} µm  ·  score {score:.0f}")

    def _on_site(self, idx, m) -> None:
        if m.focus_um is None:
            self._survey_log.append(f"{m.label}: {m.refusal}")
        else:
            self._survey_log.append(
                f"{m.label}: focus {m.focus_um:.1f} µm  ±{m.focus_sigma_um:.2f} "
                f"(FWHM {m.fwhm_um:.1f})")

    def _on_feature_request(self, idx, frame) -> None:
        """Auto-propose the most textured patch; the operator may override."""
        try:
            from SupportClasses.VisionDetector import select_trackable_patch
            hit = select_trackable_patch(frame)
        except Exception:
            hit = None
        if self._worker is not None:
            if hit:
                self._worker.set_feature(hit[0], hit[1])
            else:
                self._worker.set_feature(None, None)

    def _on_failed(self, why) -> None:
        self._teardown_worker()
        self._survey_note.setText(why)
        self.refresh()

    def _on_done(self, state) -> None:
        self._teardown_worker()
        self._state = state
        self._solve_and_show()
        self.go_to_step(STEP_VERIFY)

    def _cancel(self) -> None:
        self._teardown_worker()
        self._survey_note.setText("Cancelled.")
        self.refresh()

    def _teardown_worker(self) -> None:
        w, self._worker = self._worker, None
        self._btn_cancel.setEnabled(False)
        if w is None:
            return
        # Disconnect FIRST so a late queued signal cannot re-apply results into
        # a torn-down UI, then stop and join.
        for sig in (w.progress, w.site_started, w.awaiting_feature,
                    w.focus_sample, w.site_measured, w.finished_ok, w.failed):
            try:
                sig.disconnect()
            except Exception:
                pass
        try:
            w.stop()
            w.set_feature(None, None)
            if not w.wait(8000):
                logger.error("plate-level worker did not stop in 8 s — leaking "
                             "the thread rather than terminating it mid-COM-call")
        except Exception:
            pass

    # ── solve + verify ───────────────────────────────────────────

    def _solve_and_show(self) -> None:
        if self._state is None:
            return
        c = self._ctrl
        anchor_xy = c.get_plate_bottom_anchor_xy_um()
        anchor_z = c.get_plate_bottom_z()
        prov = {}
        try:
            prov = {
                "zero_z_mm_at_fit": float((c.zero_position or {}).get("Z", 0.0)),
                "plate_flip_180": bool(c.plate_flip_180()),
                "z_up_sign_at_fit": float(c.z_up_sign()),
                "print_z_dir_at_fit": float(c.print_z_dir()),
                "plate_bottom_z_source": c.get_plate_bottom_z_source(),
            }
        except Exception:
            pass
        self._solution = solve(
            self._state.ordered(), anchor_xy_um=anchor_xy,
            anchor_z_zref_mm=anchor_z, focal_sign=self._focal_sign(),
            tolerance_mm=self._tolerance_mm, provenance=prov,
            plate_key=self._plate_key())
        self._render_verify()

    def _objective_name(self) -> str:
        """The datum key's objective half — resolved the SAME way the bore
        wizard's writer resolves it, so the two keys can actually match.

        v7.13 — the writer used to pass a composite ``"cam|objective"`` as the
        camera half with an empty objective, while this reader passed the bare
        camera with an empty objective: the keys could never be equal and the
        stored datum was unreadable. Both now use (bare identity, this name).
        """
        hw = getattr(self._host, "_hardware_config", None)
        cam_cfg = getattr(hw, "camera_config", None) if hw is not None else None
        return str(getattr(cam_cfg, "current_objective_name", "") or "")

    def _focal_sign(self) -> int:
        """Sign relating focus mm to needle zref mm — measured if available."""
        try:
            from SupportClasses.PlateFocusDatumStore import get_store
            rec = get_store().get(self._camera_name(), self._objective_name(),
                                  self._plate_key())
            if rec:
                return int(rec.get("focal_sign", 1))
        except Exception:
            pass
        try:
            from SupportClasses.MicroscopeConfigStore import get_store as cfg
            up = 1.0 if cfg().focus_up_is_positive() else -1.0
            return 1 if (self._ctrl.z_up_sign() * up) >= 0 else -1
        except Exception:
            return 1

    def _render_verify(self) -> None:
        sol = self._solution
        if sol is None:
            self._verify_text.setPlainText("No solution.")
            return
        L = []
        if sol.plane is not None:
            fp = self._footprint_um()
            try:
                span = abs(sol.plane.span_mm(fp)) if fp else 0.0
            except Exception:
                span = 0.0
            L.append(f"Plate bottom varies {span * 1000:.0f} µm across this "
                     f"plate.")
            L.append(f"Tilt: {sol.plane.sx_mm_per_mm:+.5f} / "
                     f"{sol.plane.sy_mm_per_mm:+.5f} mm per mm")
            L.append("")
        L.append("Per-site measurements")
        for i, m in enumerate(self._state.ordered() if self._state else []):
            tag = "  ← anchor (exact by construction)" if i == 0 else ""
            if m.focus_um is None:
                L.append(f"  {m.label}: {m.refusal}")
            else:
                L.append(f"  {m.label}: focus {m.focus_um:9.1f} µm  "
                         f"±{m.focus_sigma_um:.2f}  FWHM {m.fwhm_um:6.1f}{tag}")
        L.append("")
        if sol.holdout_um is not None:
            L.append(f"Hold-out (worst leave-one-out): {sol.holdout_um:.0f} µm "
                     f"— most demanding site {sol.holdout_site}")
        if sol.residual_max_um is not None:
            L.append(f"Largest residual: {sol.residual_max_um:.0f} µm")
        L.append(f"Target: better than {self._tolerance_mm * 1000:.0f} µm")
        drift = self._state.anchor_drift_um() if self._state else None
        if drift is not None:
            L.append(f"First-site drift over the run: {drift:.1f} µm")
        L.append("")
        ok_ctrl, why_ctrl = (True, "")
        if sol.plane is not None:
            try:
                ok_ctrl, why_ctrl = self._ctrl.would_accept_plate_z_plane(
                    sol.plane)
            except Exception as e:
                ok_ctrl, why_ctrl = (False, str(e))
        blockers = list(sol.blockers)
        if drift is not None and drift > MAX_ANCHOR_DRIFT_UM:
            blockers.append(
                f"The first site's focus moved {drift:.1f} µm during the run "
                f"(limit {MAX_ANCHOR_DRIFT_UM:.0f} µm). Something drifted — the "
                f"measurements were not all taken against the same reference.")
        if not ok_ctrl:
            blockers.append(f"The controller would reject this plane: {why_ctrl}")
        for wmsg in sol.warnings:
            L.append(f"⚠ {wmsg}")
        if blockers:
            L.append("")
            L.append("BLOCKED:")
            for b in blockers:
                L.append(f"  • {b}")
        self._verify_text.setPlainText("\n".join(L))
        self._btn_accept.setEnabled(bool(sol.plane is not None and not blockers))

    def _accept(self) -> None:
        """Install the plane and persist everything. GUI thread only."""
        sol = self._solution
        if sol is None or sol.plane is None:
            return
        ok, why = self._ctrl.set_plate_z_plane(sol.plane)
        if not ok:
            QMessageBox.warning(self, "Plane rejected", why)
            self.refresh()
            return
        # Stores are written AFTER the plane is accepted and in a fixed order,
        # so "plane present" always implies its supporting data is present too.
        try:
            self._persist_prior()
        except Exception as e:
            logger.debug(f"prior persist failed: {e}")
        self.calibration_changed.emit()
        QMessageBox.information(
            self, "Plane installed",
            f"Plate tilt is now active.\n\n{sol.plane.describe()}")
        self.refresh()

    def _persist_prior(self) -> None:
        from SupportClasses.PlateLevelSiteStore import get_store
        st = self._state
        sol = self._solution
        if st is None or sol is None or sol.plane is None or not st.sites:
            return
        store = get_store()
        key_cam = self._camera_name()
        prior = store.get_prior(self._plate_key(), key_cam)
        first = st.measurements.get(st.sites[0].label)
        if first is None or first.focus_um is None:
            return
        pred = prior.predict_focus_um(first.x_stage_um, first.y_stage_um, 0.0)
        run_off = 0.0 if pred is None else (first.focus_um - pred)
        store.set_prior(self._plate_key(), key_cam, update_prior(
            prior, run_offset_um=run_off,
            sx=sol.plane.sx_mm_per_mm, sy=sol.plane.sy_mm_per_mm,
            ref_xy_um=(first.x_stage_um, first.y_stage_um),
            ref_focus_um=first.focus_um))

    # ── lifecycle ────────────────────────────────────────────────

    def hideEvent(self, e):
        self._teardown_worker()
        super().hideEvent(e)

    def closeEvent(self, e):
        self._teardown_worker()
        super().closeEvent(e)


@dataclass
class _RungWithOptics:
    """A ladder rung plus its resolved optics, forwarding rung attributes."""
    rung: object
    optics: object = None

    def __getattr__(self, name):
        return getattr(self.rung, name)


def _title(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(SECTION_TITLE_STYLE)
    return lbl
