"""
TileAutofocus.py — focus-drive executor for per-tile mosaic autofocus (v7.13).

Encapsulates the PROVEN v7.11 microscope-focus mechanics (see
gui/widgets/plate_level_wizard.py, the reference implementation) so the
fluorescence mosaic worker stays thin:

* submit-and-wait ops with the STALE-DROP check — ``done`` alone is not
  success; ``error == "dropped (stale)"`` means another surface is driving
  the body and the run must abort loudly, never retry (B4 lesson);
* focus moves approached from below (backlash lead-in) with the READBACK
  verified — ``set_focus_um`` clamps silently, so a commanded value that hit
  a limit arrives short and must be treated as "no data", not data;
* light fresh-frame grabs via the counter pattern (the only legal off-thread
  grab);
* focus scoring on a central ROI with the raw ≥254 saturation fraction, so
  ``FocusCurve``'s SATURATED refusal can fire;
* micro-sweeps (one rung around a predicted centre) for per-tile tracking and
  a coarse-to-fine ladder (``FocusSweepPlanner.plan_sweep`` + ``next_rung``)
  for the once-per-run solve at the probe position.

Duck-typed ``scope`` (MicroscopeController-shaped) and ``cam`` (CameraWidget-
shaped) so the whole thing is testable with the v7.11 fakes. The CALLER owns
the microscope lease, the entry-focus snapshot/restore and the poller — this
class only moves the focus drive and scores frames. The needle NEVER moves
here; the focus drive moves the objective only.
"""

from __future__ import annotations

import logging
import time

logger = logging.getLogger(__name__)

#: Central ROI fraction of the frame used for scoring. Bigger than the plate-
#: level wizard's operator-picked patch — a mosaic tile has no operator to
#: point at a feature, so score a generous centre crop.
ROI_FRAC = 0.25

#: Default per-op timeout (matches the plate-level wizard's).
OP_TIMEOUT_S = 30.0

#: Settle + fresh-frame parameters for AF sample grabs (lighter than tile
#: grabs — AF frames only feed the metric, not the stitch).
AF_SETTLE_S = 0.06
AF_FRESH_FRAMES = 1
AF_FRESH_TIMEOUT_S = 2.0

#: Micro-sweep geometry defaults, in DOF multiples (used when the operator
#: leaves step/range at 0 = auto).
AUTO_STEP_DOF = 1.0
AUTO_HALF_RANGE_DOF = 2.0

#: Micro-sweep sample floor — FocusCurve.MIN_SAMPLES is 5.
MIN_SWEEP_SAMPLES = 5


class AfAbort(RuntimeError):
    """The microscope cannot be driven (wedged / claimed by another surface)."""


class AfCancelled(RuntimeError):
    """The caller's should_stop fired mid-sweep."""


class TileAutofocus:
    """Focus micro-sweeps + coarse solve against a scope/cam pair."""

    def __init__(self, *, scope, cam, dof_um: float,
                 step_um: float = 0.0, half_range_um: float = 0.0,
                 soft_limits_um: tuple | None = None,
                 focus_limits_um: tuple | None = None,
                 settle_s: float = AF_SETTLE_S,
                 fingerprint: str = ""):
        """``step_um`` / ``half_range_um`` of 0 mean auto (DOF-derived)."""
        self._scope = scope
        self._cam = cam
        self._dof = max(0.1, float(dof_um))
        self._step = (float(step_um) if step_um and step_um > 0
                      else AUTO_STEP_DOF * self._dof)
        self._half = (float(half_range_um) if half_range_um and half_range_um > 0
                      else AUTO_HALF_RANGE_DOF * self._dof)
        # A sweep needs >= MIN_SWEEP_SAMPLES points: widen the half-range to
        # fit them rather than silently under-sampling (FocusCurve refuses
        # under 5 samples, which would read as "AF broken").
        min_half = self._step * (MIN_SWEEP_SAMPLES - 1) / 2.0
        if self._half < min_half:
            self._half = min_half
        self._soft = soft_limits_um
        self._hard = focus_limits_um
        self._settle_s = max(0.0, float(settle_s))
        self._fingerprint = str(fingerprint or "")
        from SupportClasses.FocusSweepPlanner import BACKLASH_LEADIN_UM
        self._lead_in = BACKLASH_LEADIN_UM

    # ── scope plumbing (mirrors plate_level_wizard) ───────────────

    def _scope_op(self, op, what: str, timeout_s: float = OP_TIMEOUT_S):
        if op is None:
            raise AfAbort(f"the microscope refused {what}")
        done = getattr(op, "done", None)
        if done is not None and not done.wait(timeout_s):
            raise AfAbort(
                f"the microscope did not finish {what} within {timeout_s:.0f} s")
        err = getattr(op, "error", None)
        if err == "dropped (stale)":
            raise AfAbort(
                "another part of the app is driving the microscope — the scan "
                "cannot share the body")
        if err:
            raise AfAbort(f"the microscope refused {what}: {err}")

    def focus_now_um(self):
        try:
            st = self._scope.state()
            return getattr(st, "focus_um", None)
        except Exception:
            return None

    def goto_focus(self, z_um: float, lead_in: bool = True) -> float:
        """Move the focus drive; approach from below; return the READBACK."""
        z = float(z_um)
        if lead_in and self._lead_in > 0:
            self._scope_op(self._scope.set_focus_um(z - self._lead_in),
                           "moving the focus", 10.0)
        self._scope_op(self._scope.set_focus_um(z), "moving the focus", 10.0)
        back = self.focus_now_um()
        return z if back is None else float(back)

    # ── frames + scoring ──────────────────────────────────────────

    def grab_af_frame(self, should_stop=None):
        """Fresh frame via the counter pattern (light: 1 fresh frame)."""
        cam = self._cam
        try:
            if self._settle_s > 0:
                time.sleep(self._settle_s)
            start = int(cam.frame_count_value())
            deadline = time.monotonic() + AF_FRESH_TIMEOUT_S
            while time.monotonic() < deadline:
                if should_stop is not None and should_stop():
                    raise AfCancelled()
                if int(cam.frame_count_value()) - start >= AF_FRESH_FRAMES:
                    break
                time.sleep(0.02)
            return cam.get_current_frame()
        except AfCancelled:
            raise
        except Exception as exc:
            logger.debug(f"TileAutofocus: frame grab failed: {exc}")
            return None

    def score(self, frame):
        """``(score, saturated_frac)`` on a central ROI; (None, 0) on failure."""
        try:
            from SupportClasses.VisionDetector import NeedleDetector
            import numpy as np
        except ImportError:              # pragma: no cover
            return (None, 0.0)
        try:
            h, w = frame.shape[:2]
            side = max(48, int(min(w, h) * ROI_FRAC))
            x0 = int(max(0, (w - side) / 2))
            y0 = int(max(0, (h - side) / 2))
            roi = (x0, y0, side, side)
            sub = frame[y0:y0 + side, x0:x0 + side]
            mx = float(np.max(sub)) if sub.size else 0.0
            sat = (float(np.count_nonzero(sub >= 254)) / max(1, sub.size)
                   if mx >= 254 else 0.0)
            res = NeedleDetector.compute_focus_score(frame, roi)
            return (float(res.score), sat)
        except Exception as exc:
            logger.debug(f"TileAutofocus: score failed: {exc}")
            return (None, 0.0)

    # ── sweeps ────────────────────────────────────────────────────

    def _collect(self, targets, should_stop):
        """Drive the targets, score each; returns FocusCurve samples."""
        from SupportClasses.FocusCurve import FocusSample
        samples = []
        first = True
        for z in targets:
            if should_stop is not None and should_stop():
                raise AfCancelled()
            back = self.goto_focus(z, lead_in=first)
            first = False
            if abs(back - z) > max(1e-6, self._step * 0.5):
                # Hit a limit — a clamped sample is a guess, not data; the
                # remaining targets sit past the same limit.
                break
            frame = self.grab_af_frame(should_stop)
            if frame is None:
                continue
            score, sat = self.score(frame)
            if score is None:
                continue
            samples.append(FocusSample(
                z_um=back, score=float(score), saturated_frac=float(sat),
                settings_fingerprint=self._fingerprint,
                focus_readback_um=back))
        return samples

    def micro_sweep(self, center_um: float, should_stop=None):
        """One rung around ``center_um``; ``(FocusPeak | None, reason)``."""
        from SupportClasses.FocusCurve import peak_focus_um
        from SupportClasses.FocusSweepPlanner import sweep_targets_um
        lo, hi = self._merged_limits()
        try:
            targets = sweep_targets_um(
                center_um=float(center_um), half_range_um=self._half,
                step_um=self._step, soft_lo=lo, soft_hi=hi,
                lead_in_um=self._lead_in)
        except ValueError as exc:
            return (None, str(exc))
        samples = self._collect(targets, should_stop)
        return peak_focus_um(samples, dof_um=self._dof)

    def coarse_solve(self, optics, center_um: float, half_range_um: float,
                     prior_sigma_um=None, should_stop=None):
        """Coarse-to-fine ladder solve (once per run, at the probe position).

        ``(FocusPeak | None, reason)``. The prior only ever NARROWS the
        window (FocusSweepPlanner's contract); it never contributes to the
        measured result.
        """
        from SupportClasses.FocusCurve import peak_focus_um
        from SupportClasses.FocusSweepPlanner import plan_sweep, next_rung
        plan, why = plan_sweep(
            optics=optics, center_um=float(center_um),
            requested_half_range_um=float(half_range_um),
            focus_limits_um=self._hard, soft_limits_um=self._soft,
            prior_sigma_um=prior_sigma_um)
        if plan is None:
            return (None, why)
        peak = None
        rung = plan.rungs[0] if plan.rungs else None
        for r_i in range(len(plan.rungs)):
            if rung is None:
                break
            samples = []
            first = True
            for z in rung.z_targets_um:
                if should_stop is not None and should_stop():
                    raise AfCancelled()
                back = self.goto_focus(z, lead_in=first)
                first = False
                if abs(back - z) > max(1e-6, rung.step_um * 0.5):
                    break
                frame = self.grab_af_frame(should_stop)
                if frame is None:
                    continue
                score, sat = self.score(frame)
                if score is None:
                    continue
                from SupportClasses.FocusCurve import FocusSample
                samples.append(FocusSample(
                    z_um=back, score=float(score), saturated_frac=float(sat),
                    settings_fingerprint=self._fingerprint,
                    focus_readback_um=back))
            peak, why = peak_focus_um(samples, dof_um=plan.dof_um)
            if peak is None:
                return (None, why)
            rung = next_rung(plan=plan, rung_index=r_i,
                             measured_peak_um=peak.z_um,
                             focus_limits_um=self._hard,
                             soft_limits_um=self._soft)
        return (peak, "")

    # ── helpers ───────────────────────────────────────────────────

    def _merged_limits(self):
        lo = hi = None
        for lim in (self._hard, self._soft):
            if not lim:
                continue
            try:
                l0 = None if lim[0] is None else float(lim[0])
                l1 = None if lim[1] is None else float(lim[1])
            except (TypeError, ValueError, IndexError):
                continue
            if l0 is not None:
                lo = l0 if lo is None else max(lo, l0)
            if l1 is not None:
                hi = l1 if hi is None else min(hi, l1)
        return lo, hi

    def dof_um(self) -> float:
        return self._dof

    def step_um(self) -> float:
        return self._step

    def half_range_um(self) -> float:
        return self._half
