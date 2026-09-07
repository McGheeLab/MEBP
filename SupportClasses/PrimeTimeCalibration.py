"""Prime-time calibration — the arithmetic, with no hardware and no Qt.

THE PHYSICS
-----------
``build_well_plate_job`` dispenses a PRIME just before every print path::

    prime_amounts_uL[pump] = flow_uL_s * HardwareConfig.pump_prime_time_s

The pump command and the XY command then start at the same instant, but ink does
not emerge at that instant: the syringe/tubing/needle compliance and the dead
volume must be taken up first. During that delay ``tau`` the stage has already
travelled away from the commanded start ``A`` toward ``B``, so the printed line
does not begin at A — it begins at some point ``C`` along it::

    tau = prime_used_in_run + (distance A->C along the line) / velocity

which is exactly the prime time that WOULD have put ink at A. That is the whole
measurement: print one straight line, look at where the ink actually starts.

WHY ``prime_used_in_run`` IS IN THE SUM
--------------------------------------
It makes the formula work for BOTH kinds of run with no special cases:

* a **baseline** run forces the prime to 0, so ``tau`` is the absolute answer;
* a **verify** run uses the prime already applied, and any residual A->C is the
  amount still MISSING — so the corrected prime is ``used + residual``, and
  re-running converges instead of restarting from zero.

It also makes over-priming expressible. If the applied prime is too large, ink
appears BEFORE A and the operator clicks behind it: ``along_mm`` is negative and
the sum correctly REDUCES the prime. A negative ``along_mm`` on a run that used
no prime is physically impossible, so there it is a refusal (the click is wrong)
rather than a number — see :data:`REFUSALS`.

TWO PRIMES, ONE KNOB
--------------------
The calibration prints TWO lines, because there are two different primes:

* **line 1 — the cold start**, after the wash and clean steps. The column is
  relaxed and may hold buffer or air, so getting flow going takes the LONG time.
* **line 2 — the restart**, after the pump merely PAUSED between segments. The
  column is still largely pressurised (less whatever the quick-move pressure
  relief pulled back), so restarting takes the SHORT time.

Expect ``initial >= restart``. But ``build_well_plate_job`` emits
``prime_amounts_uL[pump]`` before EVERY segment, first one included, so ONE value
has to serve both — which is what :func:`compare_primes` exists to price. A
restart that needs MORE than a cold start is the unexpected case and points at
the pressure relief / pause length, not at the prime.

WHY VELOCITY IS RESOLVED, NOT ASSUMED
-------------------------------------
``tau`` is a distance divided by a speed, so a wrong speed is a wrong prime. The
commanded speed is NOT what the stage delivers (see
``PrintTimingCalibrationStore``: this class of machine sustains well under the
commanded speed on short segments), so :func:`resolve_velocity` prefers the
run's own ``vel_sample`` rows — which project the ACTUAL measured position onto
the path — and falls back to the commanded speed only when there are none. The
source travels with the number so the operator can judge it.

Deliberately NOT used: ``path_end.t - path_start.t``. On the discrete path
``path_start`` is logged BEFORE the move-to-``points[0]`` and its 5 s settle, so
that window over-states the traverse duration, which would UNDER-state the
velocity and OVER-state the prime — a blob at A, the failure this calibration
exists to remove.

Deliberately NOT done (recorded, not overlooked): interpolating the sample time
at ``s = along_mm`` instead of dividing by an average velocity. That would be
strictly more accurate across the initial accel ramp — which is exactly where C
lands — but it is a second, differently-derived number for the same quantity,
and one authority beats two. If the ramp turns out to matter on the bench, this
is the refinement to make.

Pure: stdlib + a duck-typed ``PrintLog``. No Qt, no numpy, no cv2 — so the
arithmetic is testable headless and cannot be quietly changed by a GUI edit.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Optional, Sequence

logger = logging.getLogger(__name__)

#: A line shorter than this cannot define a direction, so it cannot be measured.
MIN_LINE_MM = 0.05

#: Default tolerance for "the click is not on the printed line" (mm). Scaled up
#: for long lines by :func:`off_line_limit_mm` — a 0.5 mm miss on a 1 mm line is
#: a different kind of mistake from a 0.5 mm miss on a 10 mm line.
OFF_LINE_TOL_MM = 0.5

#: Fraction of the line length that also counts as "on the line".
OFF_LINE_FRAC = 0.15

#: An endpoint tolerance, so a click a hair before A / after B is not refused.
END_TOL_MM = 0.05

#: Beyond this fraction of the line, tau is probably clipped by the line length.
NEAR_END_FRAC = 0.9

# Refusal codes. Each one exists because the alternative is returning a
# plausible-looking prime time that is wrong.
REFUSAL_DEGENERATE_LINE = "DEGENERATE_LINE"
REFUSAL_NO_VELOCITY = "NO_VELOCITY"
REFUSAL_OFF_LINE = "OFF_LINE"
REFUSAL_BEFORE_START = "BEFORE_START"
REFUSAL_BEYOND_END = "BEYOND_END"

REFUSALS = (
    REFUSAL_DEGENERATE_LINE,
    REFUSAL_NO_VELOCITY,
    REFUSAL_OFF_LINE,
    REFUSAL_BEFORE_START,
    REFUSAL_BEYOND_END,
)

# Velocity provenance. "measured" is the only one derived from where the stage
# actually was; the other two describe what it was ASKED to do.
VEL_MEASURED = "measured"
VEL_COMMANDED_SCHEDULE = "commanded-schedule"
VEL_COMMANDED = "commanded"


# ════════════════════════════════════════════════════════════════════
#  Geometry
# ════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class LineFit:
    """Where C sits relative to the commanded line A->B (all mm)."""

    length_mm: float
    #: SIGNED distance along the line from A. Negative = behind A.
    along_mm: float
    #: Unsigned perpendicular distance from the infinite line through A,B.
    perp_mm: float
    #: ``along_mm / length_mm``. 0 at A, 1 at B.
    frac: float


def project_onto_line(a_mm, b_mm, c_mm) -> Optional[LineFit]:
    """Project C onto the line A->B.

    Returns ``None`` when A and B are closer than :data:`MIN_LINE_MM` (no
    direction to project onto). Projecting — rather than taking ``|C - A|`` — is
    what makes the measurement tolerant of a click that is a little off the
    bead: the along-track component is the one that maps to elapsed time, and
    the perpendicular component is reported separately so a click that is not on
    the line at all can be refused instead of silently inflating the distance.
    """
    try:
        ax, ay = float(a_mm[0]), float(a_mm[1])
        bx, by = float(b_mm[0]), float(b_mm[1])
        cx, cy = float(c_mm[0]), float(c_mm[1])
    except (TypeError, ValueError, IndexError):
        return None
    if not all(math.isfinite(v) for v in (ax, ay, bx, by, cx, cy)):
        return None

    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length < MIN_LINE_MM:
        return None
    ux, uy = dx / length, dy / length

    vx, vy = cx - ax, cy - ay
    along = vx * ux + vy * uy
    # 2-D cross product magnitude = perpendicular distance from the line.
    perp = abs(vx * uy - vy * ux)
    return LineFit(length_mm=length, along_mm=along, perp_mm=perp,
                   frac=along / length)


def off_line_limit_mm(length_mm: float,
                      off_line_tol_mm: float = OFF_LINE_TOL_MM) -> float:
    """How far off the line a click may land and still be accepted."""
    return max(float(off_line_tol_mm), OFF_LINE_FRAC * float(length_mm))


# ════════════════════════════════════════════════════════════════════
#  The result
# ════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class PrimeTimeResult:
    """One prime-time measurement, or a refusal explaining why there is none."""

    prime_time_s: Optional[float]
    #: ``along_mm / velocity`` — the correction this run measured, on its own.
    added_s: Optional[float]
    prime_used_s: float
    #: ``flow * prime_time_s`` — the volume the DISPENSE prime will actually be.
    prime_volume_uL: Optional[float]
    velocity_mm_s: float
    velocity_source: str
    fit: Optional[LineFit]
    refusal: Optional[str]
    warning: Optional[str]
    message: str

    @property
    def ok(self) -> bool:
        return self.refusal is None and self.prime_time_s is not None


def _velocity_phrase(source: str) -> str:
    if source == VEL_MEASURED:
        return "measured"
    if source == VEL_COMMANDED_SCHEDULE:
        return "commanded schedule"
    return "commanded"


def evaluate(a_mm, b_mm, c_mm, *,
             velocity_mm_s: float,
             velocity_source: str = VEL_COMMANDED,
             prime_used_s: float = 0.0,
             flow_uL_s: float = 0.0,
             off_line_tol_mm: float = OFF_LINE_TOL_MM) -> PrimeTimeResult:
    """Turn a clicked ink-start point into a prime time.

    ``a_mm`` / ``b_mm`` are the COMMANDED line endpoints and ``c_mm`` the point
    the operator clicked, all in the same frame (any consistent frame — the
    result depends only on distances). ``prime_used_s`` is the prime that
    actually ran; see the module docstring for why it is added rather than
    ignored.
    """
    try:
        v = float(velocity_mm_s)
    except (TypeError, ValueError):
        v = 0.0
    try:
        used = max(0.0, float(prime_used_s))
    except (TypeError, ValueError):
        used = 0.0
    try:
        flow = max(0.0, float(flow_uL_s))
    except (TypeError, ValueError):
        flow = 0.0

    def _refuse(code: str, msg: str, fit=None) -> PrimeTimeResult:
        return PrimeTimeResult(
            prime_time_s=None, added_s=None, prime_used_s=used,
            prime_volume_uL=None, velocity_mm_s=v,
            velocity_source=str(velocity_source), fit=fit,
            refusal=code, warning=None, message=msg)

    fit = project_onto_line(a_mm, b_mm, c_mm)
    if fit is None:
        return _refuse(
            REFUSAL_DEGENERATE_LINE,
            "The calibration line is too short to measure — it needs a "
            f"length of at least {MIN_LINE_MM:.2f} mm.")

    if not math.isfinite(v) or v <= 0:
        return _refuse(
            REFUSAL_NO_VELOCITY,
            "No print velocity is known for that run, so the distance cannot "
            "be converted to a time.", fit)

    limit = off_line_limit_mm(fit.length_mm, off_line_tol_mm)
    if fit.perp_mm > limit:
        return _refuse(
            REFUSAL_OFF_LINE,
            f"That point is {fit.perp_mm:.2f} mm off the printed line (limit "
            f"{limit:.2f} mm) — click on the bead itself.", fit)

    if fit.along_mm < -END_TOL_MM and used <= 0.0:
        return _refuse(
            REFUSAL_BEFORE_START,
            "That point is behind the line start, but this run used no prime "
            "at all — ink cannot appear before the pump moves. Click where the "
            "bead actually begins.", fit)

    if fit.along_mm > fit.length_mm + END_TOL_MM:
        return _refuse(
            REFUSAL_BEYOND_END,
            "That point is past the end of the line — ink never appeared "
            "within it. Print a longer line or reduce the print speed.", fit)

    added = fit.along_mm / v
    prime = used + added
    warning = None
    if prime < 0.0:
        # A verify run that was over-primed. The clamp is not a fudge: a
        # negative prime has no meaning, and 0 is the correct instruction.
        prime = 0.0
        warning = ("This run was over-primed — ink appeared before the line "
                   "start. The prime should be reduced to 0 and re-measured.")
    elif fit.frac > NEAR_END_FRAC:
        warning = (f"Ink started {fit.frac * 100:.0f}% of the way along the "
                   "line, so the prime time may be clipped by the line "
                   "length — print a longer line to confirm.")

    vol = flow * prime if flow > 0 else None
    src = _velocity_phrase(str(velocity_source))
    msg = (f"A→C = {fit.along_mm:.3f} mm along the line "
           f"({fit.perp_mm:.3f} mm off) ÷ {v:.3f} mm/s ({src}) "
           f"= {added:.3f} s;  prime used {used:.3f} s "
           f"→ prime time {prime:.3f} s")
    if vol is not None:
        msg += f"  (≈ {vol:.4f} µL)"

    return PrimeTimeResult(
        prime_time_s=prime, added_s=added, prime_used_s=used,
        prime_volume_uL=vol, velocity_mm_s=v,
        velocity_source=str(velocity_source), fit=fit,
        refusal=None, warning=warning, message=msg)


# ════════════════════════════════════════════════════════════════════
#  Reading the run back out of its execution log
# ════════════════════════════════════════════════════════════════════

def _f(value, default: float = 0.0) -> float:
    try:
        if value is None or isinstance(value, bool):
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _velocity_from_samples(rows: Sequence[dict]) -> Optional[float]:
    """Average speed from arc-length/time sample rows, or ``None``.

    Uses the first and last row that carry ``s_mm``: the samples are emitted at
    a fixed cadence over the whole path, so the endpoints give the mean speed
    without any curve fitting.
    """
    pts = [(_f(r.get("t"), -1.0), _f(r.get("s_mm"), -1.0))
           for r in rows if r.get("s_mm") is not None]
    pts = [(t, s) for (t, s) in pts if t >= 0.0 and s >= 0.0]
    if len(pts) < 2:
        return None
    t0, s0 = pts[0]
    t1, s1 = pts[-1]
    dt, ds = t1 - t0, s1 - s0
    if dt <= 0 or ds <= 0:
        return None
    return ds / dt


def _settings_of(log) -> dict:
    """The run's stamped ``PrintSettings`` as a dict, ``{}`` on anything else.

    ``PrintLog.settings`` is ``manifest.get("settings") or {}``, which passes a
    corrupt log's non-dict straight through — so the type is checked here rather
    than assumed at three call sites."""
    if log is None:
        return {}
    try:
        settings = log.settings
    except Exception:
        return {}
    return settings if isinstance(settings, dict) else {}


@dataclass(frozen=True)
class PrimeComparison:
    """The two lines measure TWO DIFFERENT PRIMES, and the code has one knob.

    * **Line 1 — the cold start.** The needle has just been washed and cleaned,
      so the column is relaxed (and may hold buffer or air). Getting flow going
      from there is the LONG prime.
    * **Line 2 — the restart.** The pump merely PAUSED between segments; the
      column is still largely pressurised, less whatever
      ``PrintManager._print_pump_suckback("quick_move")`` relieved on the way
      out. Restarting from there is the SHORT prime.

    So ``initial_s >= restart_s`` is the EXPECTED result, and ``difference_s``
    (initial − restart) is how much the two disagree.

    ``build_well_plate_job`` emits ``prime_amounts_uL[pump]`` before EVERY
    segment, first one included, so a single value has to serve both:

    * set it to the initial prime ⇒ every mid-print restart OVER-primes by
      ``difference_s`` (a small blob where each line resumes);
    * set it to the restart prime ⇒ the first line UNDER-primes (missing start).

    ``required_s`` is therefore the larger — nothing missing is the safe error —
    and the cost of that choice is stated rather than left to be discovered.

    ``restart_binds`` marks the UNEXPECTED case (a restart needing more than a
    clean start): that points at the pressure relief / suck-back volume or the
    hop, not at the prime, and is worth investigating rather than papering over.
    """

    initial_s: float
    restart_s: float
    #: initial − restart. Positive is the expected direction.
    difference_s: float
    required_s: float
    restart_binds: bool
    message: str


def compare_primes(initial: PrimeTimeResult,
                   restart: PrimeTimeResult) -> Optional[PrimeComparison]:
    """Compare the cold-start prime with the after-a-pause restart prime.

    ``None`` unless BOTH lines produced a usable measurement — half a comparison
    is not a comparison.
    """
    if (initial is None or restart is None
            or not initial.ok or not restart.ok):
        return None
    a = float(initial.prime_time_s)
    b = float(restart.prime_time_s)
    diff = a - b
    required = max(a, b)
    binds = b > a
    if diff > 0:
        msg = (f"The cold start after wash & clean needs {diff:.3f} s MORE "
               f"prime than a restart after a pump pause ({a:.3f} s vs "
               f"{b:.3f} s). One prime serves both, so at {required:.3f} s "
               f"every mid-print restart over-primes by {diff:.3f} s — expect a "
               f"small blob where each line resumes. Lower it toward "
               f"{b:.3f} s only if a short first line is acceptable.")
    elif diff < 0:
        msg = (f"⚠ Unexpected: the pump restart needs MORE prime than the cold "
               f"start ({b:.3f} s vs {a:.3f} s). The pause is losing more "
               f"pressure than the wash & clean left behind — look at the "
               f"pressure relief / suck-back volume and the hop settings rather "
               f"than at the prime. {required:.3f} s keeps anything from being "
               f"missing.")
    else:
        msg = (f"The cold start and the restart need the same {required:.3f} s, "
               f"so one prime covers both with nothing wasted.")
    return PrimeComparison(initial_s=a, restart_s=b, difference_s=diff,
                           required_s=required, restart_binds=binds,
                           message=msg)


def _rows_of(log, event: str) -> list:
    try:
        rows = log.of(event)
    except Exception:
        return []
    return list(rows) if rows else []


def _commanded_speed(log) -> float:
    return max(0.0, _f(_settings_of(log).get("print_speed_mm_s"), 0.0))


def _velocity_from(log, rows_vel: Sequence[dict],
                   rows_openvel: Sequence[dict]) -> tuple[float, str]:
    """Resolve a velocity from one window's samples, then the commanded speed."""
    v = _velocity_from_samples(rows_vel)
    if v is not None:
        return (v, VEL_MEASURED)
    v = _velocity_from_samples(rows_openvel)
    if v is not None:
        return (v, VEL_COMMANDED_SCHEDULE)
    return (_commanded_speed(log), VEL_COMMANDED)


def resolve_velocity(log) -> tuple[float, str]:
    """Best available print velocity for ``log`` (mm/s) and its provenance.

    Priority — actual position first, commanded last:

    1. ``vel_sample`` rows (closed-loop velocity / feed-plan modes): ``s_mm`` is
       the ACTUAL measured position projected onto the path → :data:`VEL_MEASURED`.
    2. ``openvel_sample`` rows (open-loop velocity streaming): ``s_mm`` is the
       time-paced COMMANDED target, not where the stage got to, so this reduces
       to the commanded speed → :data:`VEL_COMMANDED_SCHEDULE`.
    3. ``settings["print_speed_mm_s"]`` → :data:`VEL_COMMANDED`.

    Returns ``(0.0, VEL_COMMANDED)`` when nothing is available; :func:`evaluate`
    then refuses with :data:`REFUSAL_NO_VELOCITY` rather than inventing a speed.

    ⚠ WHOLE-LOG. Correct for a one-segment run only: each ``PRINT_PATH`` restarts
    ``s_mm`` at 0, so on a multi-segment run the first-to-last span crosses a
    reset AND includes the inter-segment hop. Use :func:`segments_from_log` there.
    """
    if log is None:
        return (0.0, VEL_COMMANDED)
    return _velocity_from(log, _rows_of(log, "vel_sample"),
                          _rows_of(log, "openvel_sample"))


@dataclass(frozen=True)
class SegmentRun:
    """One executed ``PRINT_PATH`` — a printed line — from an exec log."""

    index: int
    #: The COMMANDED endpoints in zero-ref mm, or None when the log (pre-v7.7)
    #: recorded only ``n_points``.
    a_mm: Optional[tuple]
    b_mm: Optional[tuple]
    t0: float
    t1: float
    velocity_mm_s: float
    velocity_source: str


def segments_from_log(log) -> list:
    """Every printed segment in ``log``, each with its OWN velocity.

    A multi-segment print (``build_well_plate_job(path_segments=…)``) emits one
    ``path_start``/``path_end`` pair per line, and each line's samples restart
    ``s_mm`` at 0. So a per-segment velocity must be resolved from only the
    samples inside that segment's ``[t0, t1]`` window — taking the whole log's
    first-to-last span would cross the reset and also swallow the inter-segment
    lift → hop → lower, badly under-stating the speed and therefore over-stating
    every prime time derived from it.

    ``path_end`` may be missing on an aborted run, so a segment's window closes
    at the next ``path_start`` (or is left open) rather than being dropped.
    """
    starts = _rows_of(log, "path_start")
    if not starts:
        return []
    ends = _rows_of(log, "path_end")
    vel = _rows_of(log, "vel_sample")
    openvel = _rows_of(log, "openvel_sample")

    out = []
    for i, st in enumerate(starts):
        t0 = _f(st.get("t"), 0.0)
        if i < len(ends):
            t1 = _f(ends[i].get("t"), float("inf"))
        elif i + 1 < len(starts):
            t1 = _f(starts[i + 1].get("t"), float("inf"))
        else:
            t1 = float("inf")
        if t1 < t0:
            t1 = float("inf")

        def _win(rows):
            return [r for r in rows
                    if t0 <= _f(r.get("t"), -1.0) <= t1]

        v, src = _velocity_from(log, _win(vel), _win(openvel))
        a = b = None
        pts = st.get("points")
        if isinstance(pts, list) and len(pts) >= 2:
            try:
                a = (float(pts[0][0]), float(pts[0][1]))
                b = (float(pts[-1][0]), float(pts[-1][1]))
            except (TypeError, ValueError, IndexError):
                a = b = None
        out.append(SegmentRun(index=i, a_mm=a, b_mm=b, t0=t0, t1=t1,
                              velocity_mm_s=v, velocity_source=src))
    return out


def line_endpoints_from_log(log) -> Optional[tuple[tuple[float, float],
                                                   tuple[float, float]]]:
    """The COMMANDED line endpoints ``(A, B)`` in zero-ref mm, or ``None``.

    Read from the log rather than recomputed from the calibration row's own
    length/angle/offset arithmetic: ``path_start.points`` IS the path that was
    handed to the executor, so taking A and B from it makes a desync between
    "what printed" and "what is being measured" structurally impossible. Older
    logs recorded only ``n_points`` and yield ``None``, so the caller must be
    able to fall back.
    """
    if log is None:
        return None
    try:
        pts = log.ideal_points()
    except Exception:
        return None
    if not pts or len(pts) < 2:
        return None
    return (pts[0], pts[-1])


def prime_used_s_from_log(log, pump: str) -> float:
    """The prime time that actually ran, recovered from the stamped settings.

    The job records a prime VOLUME and a rate, and ``prime = flow * time`` by
    construction (see the module docstring), so ``time = volume / rate``. This
    is what makes a verify run additive instead of a fresh guess. Returns 0.0
    when the run carried no prime or the settings are unreadable.
    """
    settings = _settings_of(log)
    primes = settings.get("prime_amounts_uL") or {}
    rates = settings.get("pump_rates_uL_s") or {}
    if not isinstance(primes, dict) or not isinstance(rates, dict):
        return 0.0
    vol = _f(primes.get(pump), 0.0)
    rate = _f(rates.get(pump), 0.0)
    if vol <= 0 or rate <= 0:
        return 0.0
    return vol / rate


def hop_settings_from_log(log) -> dict:
    """The settings that governed the PUMP PAUSE between the two lines.

    ``{hop_z_mm, hop_z_speed_mm_s, hop_xy_speed_mm_s}``, read from the stamped
    ``PrintSettings`` in the log rather than from the live widgets: these are the
    numbers that produced the measured restart, and the operator may well have
    changed a spin since. 0.0 means "not stamped / use the executor default".

    Between two print segments the pump STOPS while
    ``build_well_plate_job`` lifts to ``print_z + intra_well_hop_z_mm``, moves XY
    at ``line_move_xy_speed_mm_s`` and lowers at ``line_move_z_speed_mm_s``. So
    these three set how LONG the pump is paused, and therefore how much pressure
    is lost before the restart :func:`compare_primes` measures — they are the
    knobs to reach for when a restart needs more prime than it should.
    """
    settings = _settings_of(log)
    return {
        "hop_z_mm": max(0.0, _f(settings.get("intra_well_hop_z_mm"), 0.0)),
        "hop_z_speed_mm_s": max(
            0.0, _f(settings.get("line_move_z_speed_mm_s"), 0.0)),
        "hop_xy_speed_mm_s": max(
            0.0, _f(settings.get("line_move_xy_speed_mm_s"), 0.0)),
    }


def describe_hop_settings(hop: dict) -> str:
    """One-line summary of what the pump was paused FOR, between the lines."""
    z = _f((hop or {}).get("hop_z_mm"), 0.0)
    vz = _f((hop or {}).get("hop_z_speed_mm_s"), 0.0)
    vxy = _f((hop or {}).get("hop_xy_speed_mm_s"), 0.0)
    parts = [f"lift {z:.2f} mm"]
    parts.append(f"lift/lower {vz:.1f} mm/s" if vz > 0 else
                 "lift/lower at the default speed")
    parts.append(f"reposition {vxy:.1f} mm/s" if vxy > 0 else
                 "reposition at the travel speed")
    return "pump paused for: " + ", ".join(parts)


def flow_uL_s_from_log(log, pump: str) -> float:
    """The print flow rate the run used (µL/s), or 0.0 when unknown."""
    settings = _settings_of(log)
    rates = settings.get("pump_rates_uL_s") or {}
    if isinstance(rates, dict):
        v = _f(rates.get(pump), 0.0)
        if v > 0:
            return v
    return max(0.0, _f(settings.get("pump_rate_uL_s"), 0.0))
