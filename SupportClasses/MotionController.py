"""
MotionController.py — Real-time trajectory tracking for MEBP v7.1.

Provides two controller strategies for keeping the needle on the
planned trajectory:

1. KalmanMotionController (PRIMARY)
   - 6-state Kalman filter: [x, y, vx, vy, ax, ay]
   - Feedforward velocity from trajectory derivative
   - Lookahead for latency compensation
   - Rate-limited output for stage safety

2. PIDMotionController (SECONDARY)
   - Classical PID with feedforward
   - Anti-windup integral limiting
   - Ziegler-Nichols auto-tune parameter computation

Also includes:
- StageRateTester: measure actual stage responsiveness
- TrackingErrorMonitor: real-time error statistics

Control Architecture:
    ┌──────────────────────────────────────────────┐
    │  Feedforward    │  State        │  Correction │ → Stage
    │  Velocity Calc  │  Estimation   │  Output     │   Commands
    └───────┬─────────┴───────┬───────┴─────────────┘
            │                 │
       Planned velocity   Position feedback
       from trajectory    from position poller
"""

from __future__ import annotations

import logging
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_DT = 0.05                # 50 ms control loop interval
MAX_VELOCITY_UM_S = 10_000.0     # Max stage velocity (µm/s safety limit)
MIN_VELOCITY_UM_S = 1.0          # Below this we send zero
MM_TO_UM = 1000.0                # mm → µm conversion


# ---------------------------------------------------------------------------
# Tracking Error Monitor
# ---------------------------------------------------------------------------

@dataclass
class TrackingError:
    """Snapshot of tracking error at a single time step."""
    t: float                    # Timestamp (s)
    error_x_mm: float = 0.0    # Position error in X (mm)
    error_y_mm: float = 0.0    # Position error in Y (mm)
    error_mag_mm: float = 0.0  # Magnitude of XY error (mm)
    desired_x: float = 0.0     # Target position
    desired_y: float = 0.0
    actual_x: float = 0.0      # Measured position
    actual_y: float = 0.0
    cmd_vx: float = 0.0        # Commanded velocity (µm/s)
    cmd_vy: float = 0.0


class TrackingErrorMonitor:
    """
    Accumulates tracking error history during a print for analysis.

    Provides running statistics (mean, max, RMS) and recent history
    for display in the monitor page.
    """

    def __init__(self, max_history: int = 2000):
        self._history: deque[TrackingError] = deque(maxlen=max_history)
        self._error_sum_sq: float = 0.0
        self._error_count: int = 0
        self._max_error: float = 0.0

    def record(self, err: TrackingError):
        self._history.append(err)
        self._error_sum_sq += err.error_mag_mm ** 2
        self._error_count += 1
        self._max_error = max(self._max_error, err.error_mag_mm)

    def reset(self):
        self._history.clear()
        self._error_sum_sq = 0.0
        self._error_count = 0
        self._max_error = 0.0

    @property
    def rms_error_mm(self) -> float:
        if self._error_count == 0:
            return 0.0
        return math.sqrt(self._error_sum_sq / self._error_count)

    @property
    def max_error_mm(self) -> float:
        return self._max_error

    @property
    def mean_error_mm(self) -> float:
        if not self._history:
            return 0.0
        return sum(e.error_mag_mm for e in self._history) / len(self._history)

    @property
    def recent_errors(self) -> list[TrackingError]:
        return list(self._history)

    @property
    def count(self) -> int:
        return self._error_count

    def summary(self) -> dict:
        return {
            "count": self._error_count,
            "rms_error_mm": self.rms_error_mm,
            "max_error_mm": self._max_error,
            "mean_error_mm": self.mean_error_mm,
        }


# ---------------------------------------------------------------------------
# Kalman Motion Controller (PRIMARY)
# ---------------------------------------------------------------------------

class KalmanMotionController:
    """
    Kalman-filtered predictive controller for XY trajectory tracking.

    State vector: [x, y, vx, vy, ax, ay]  (position, velocity, acceleration)
    Measurement:  [x, y] from position poller
    Output:       (vx_cmd, vy_cmd) velocity commands in µm/s

    Accounts for:
    - Communication latency via lookahead
    - Noisy position measurements via Kalman filtering
    - Stage dynamics via process noise model

    All internal coordinates are in mm. Output velocities are in µm/s.
    """

    def __init__(
        self,
        dt: float = DEFAULT_DT,
        process_noise: float = 0.5,
        measurement_noise: float = 0.05,
        lookahead_steps: int = 5,
        max_velocity_um_s: float = MAX_VELOCITY_UM_S,
        gain: float = 2.0,
    ):
        """
        Args:
            dt: Control loop timestep (seconds)
            process_noise: Process noise magnitude (mm²/s⁴)
            measurement_noise: Measurement noise (mm²)
            lookahead_steps: How many timesteps ahead to target
            max_velocity_um_s: Safety limit on velocity commands
            gain: Proportional correction gain
        """
        self.dt = dt
        self.lookahead_steps = lookahead_steps
        self.max_velocity_um_s = max_velocity_um_s
        self.gain = gain

        # State: [x, y, vx, vy, ax, ay]
        self.state = np.zeros(6)
        self.P = np.eye(6) * 1000.0      # Initial covariance (high uncertainty)

        # State transition matrix (constant acceleration model)
        #   x'  = x + vx*dt + 0.5*ax*dt²
        #   vx' = vx + ax*dt
        #   ax' = ax  (constant)
        self.F = np.eye(6)
        self.F[0, 2] = dt           # x += vx * dt
        self.F[0, 4] = 0.5 * dt**2  # x += 0.5 * ax * dt²
        self.F[1, 3] = dt           # y += vy * dt
        self.F[1, 5] = 0.5 * dt**2  # y += 0.5 * ay * dt²
        self.F[2, 4] = dt           # vx += ax * dt
        self.F[3, 5] = dt           # vy += ay * dt

        # Measurement matrix: observe [x, y]
        self.H = np.zeros((2, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0

        # Process noise covariance
        self.Q = np.eye(6) * process_noise
        # Position has less process noise than acceleration
        self.Q[0, 0] = process_noise * 0.01
        self.Q[1, 1] = process_noise * 0.01
        self.Q[2, 2] = process_noise * 0.1
        self.Q[3, 3] = process_noise * 0.1

        # Measurement noise covariance
        self.R = np.eye(2) * measurement_noise

        # Error monitor
        self.error_monitor = TrackingErrorMonitor()

        # Command history for diagnostics
        self.command_history: deque[tuple[float, float, float]] = deque(maxlen=100)

        self._initialized = False

    def reset(self, initial_pos: tuple[float, float] | None = None):
        """Reset filter state. Call when starting a new print."""
        self.state = np.zeros(6)
        if initial_pos:
            self.state[0] = initial_pos[0]
            self.state[1] = initial_pos[1]
        self.P = np.eye(6) * 1000.0
        self.error_monitor.reset()
        self.command_history.clear()
        self._initialized = bool(initial_pos)

    def predict(self):
        """Kalman predict step — propagate state forward."""
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_measurement(self, measured_x: float, measured_y: float):
        """Kalman update step — correct state with measurement."""
        z = np.array([measured_x, measured_y])
        y = z - self.H @ self.state               # Innovation
        S = self.H @ self.P @ self.H.T + self.R   # Innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

        if not self._initialized:
            self.state[0] = measured_x
            self.state[1] = measured_y
            self._initialized = True

    def compute_command(
        self,
        desired_trajectory: np.ndarray,
        current_time: float,
    ) -> tuple[float, float]:
        """
        Compute velocity command from filtered state and trajectory.

        Args:
            desired_trajectory: Nx7 array of waypoints [x,y,z,p1,p2,p3,t]
            current_time: Current time in seconds

        Returns:
            (vx_cmd, vy_cmd) in µm/s
        """
        if len(desired_trajectory) == 0:
            return (0.0, 0.0)

        # Find the lookahead target on the trajectory
        target_time = current_time + self.lookahead_steps * self.dt
        target = self._interpolate_target(desired_trajectory, target_time)

        # Current target (no lookahead) for error recording
        current_target = self._interpolate_target(desired_trajectory, current_time)

        # Feedforward: velocity from trajectory derivative at current time
        ff_vx, ff_vy = self._trajectory_velocity(desired_trajectory, current_time)

        # Position error (estimated position vs lookahead target)
        est_x, est_y = self.state[0], self.state[1]
        err_x = target[0] - est_x
        err_y = target[1] - est_y

        # Correction = gain × error
        corr_vx = self.gain * err_x / max(self.dt, 0.001)
        corr_vy = self.gain * err_y / max(self.dt, 0.001)

        # Total command = feedforward + correction (in mm/s)
        vx_mm = ff_vx + corr_vx
        vy_mm = ff_vy + corr_vy

        # Convert to µm/s and rate-limit
        vx_um = vx_mm * MM_TO_UM
        vy_um = vy_mm * MM_TO_UM
        vx_um, vy_um = self._rate_limit(vx_um, vy_um)

        # Record tracking error (against current target, not lookahead)
        err = TrackingError(
            t=current_time,
            error_x_mm=current_target[0] - est_x,
            error_y_mm=current_target[1] - est_y,
            error_mag_mm=math.sqrt(
                (current_target[0] - est_x)**2 + (current_target[1] - est_y)**2),
            desired_x=current_target[0], desired_y=current_target[1],
            actual_x=est_x, actual_y=est_y,
            cmd_vx=vx_um, cmd_vy=vy_um,
        )
        self.error_monitor.record(err)
        self.command_history.append((current_time, vx_um, vy_um))

        return (vx_um, vy_um)

    def step(
        self,
        measured_pos: tuple[float, float],
        desired_trajectory: np.ndarray,
        current_time: float,
    ) -> tuple[float, float]:
        """
        Full control loop step: predict → update → compute command.

        Args:
            measured_pos: (x, y) in mm from position poller
            desired_trajectory: Nx7 waypoint array
            current_time: Current time in seconds

        Returns:
            (vx_cmd, vy_cmd) in µm/s
        """
        self.predict()
        self.update_measurement(measured_pos[0], measured_pos[1])
        return self.compute_command(desired_trajectory, current_time)

    @property
    def estimated_position(self) -> tuple[float, float]:
        """Current filtered position estimate (mm)."""
        return (float(self.state[0]), float(self.state[1]))

    @property
    def estimated_velocity(self) -> tuple[float, float]:
        """Current filtered velocity estimate (mm/s)."""
        return (float(self.state[2]), float(self.state[3]))

    # ── Internal helpers ───────────────────────────────────────────

    def _interpolate_target(
        self, trajectory: np.ndarray, target_time: float,
    ) -> tuple[float, float]:
        """Linearly interpolate XY position at target_time."""
        times = trajectory[:, 6]  # COL_T
        if target_time <= times[0]:
            return (float(trajectory[0, 0]), float(trajectory[0, 1]))
        if target_time >= times[-1]:
            return (float(trajectory[-1, 0]), float(trajectory[-1, 1]))

        idx = np.searchsorted(times, target_time, side='right') - 1
        idx = max(0, min(idx, len(times) - 2))

        t0, t1 = times[idx], times[idx + 1]
        dt = t1 - t0
        if dt < 1e-9:
            frac = 0.0
        else:
            frac = (target_time - t0) / dt

        x = trajectory[idx, 0] + frac * (trajectory[idx + 1, 0] - trajectory[idx, 0])
        y = trajectory[idx, 1] + frac * (trajectory[idx + 1, 1] - trajectory[idx, 1])
        return (float(x), float(y))

    def _trajectory_velocity(
        self, trajectory: np.ndarray, t: float,
    ) -> tuple[float, float]:
        """Compute instantaneous velocity from trajectory at time t (mm/s)."""
        times = trajectory[:, 6]
        idx = np.searchsorted(times, t, side='right') - 1
        idx = max(0, min(idx, len(times) - 2))

        dt = times[idx + 1] - times[idx]
        if dt < 1e-9:
            return (0.0, 0.0)

        vx = (trajectory[idx + 1, 0] - trajectory[idx, 0]) / dt
        vy = (trajectory[idx + 1, 1] - trajectory[idx, 1]) / dt
        return (float(vx), float(vy))

    def _rate_limit(self, vx: float, vy: float) -> tuple[float, float]:
        """Apply velocity magnitude limit."""
        mag = math.sqrt(vx**2 + vy**2)
        if mag > self.max_velocity_um_s:
            scale = self.max_velocity_um_s / mag
            vx *= scale
            vy *= scale
        if mag < MIN_VELOCITY_UM_S:
            return (0.0, 0.0)
        return (vx, vy)

    def to_dict(self) -> dict:
        """Serialize tuning parameters."""
        return {
            "dt": self.dt,
            "lookahead_steps": self.lookahead_steps,
            "max_velocity_um_s": self.max_velocity_um_s,
            "gain": self.gain,
            "process_noise": float(self.Q[4, 4]),
            "measurement_noise": float(self.R[0, 0]),
        }

    @classmethod
    def from_dict(cls, d: dict) -> KalmanMotionController:
        return cls(
            dt=d.get("dt", DEFAULT_DT),
            process_noise=d.get("process_noise", 0.5),
            measurement_noise=d.get("measurement_noise", 0.05),
            lookahead_steps=d.get("lookahead_steps", 5),
            max_velocity_um_s=d.get("max_velocity_um_s", MAX_VELOCITY_UM_S),
            gain=d.get("gain", 2.0),
        )


# ---------------------------------------------------------------------------
# PID Motion Controller (SECONDARY)
# ---------------------------------------------------------------------------

class PIDMotionController:
    """
    Classical PID with feedforward for XY trajectory tracking.

    Feedforward: velocity directly from trajectory derivative
    PID: corrects for position error
    Anti-windup: integral clamping

    Output: (vx_cmd, vy_cmd) in µm/s

    All internal coordinates in mm; output in µm/s.
    """

    def __init__(
        self,
        kp: float = 0.5,
        ki: float = 0.01,
        kd: float = 0.02,
        dt: float = DEFAULT_DT,
        max_output_um_s: float = MAX_VELOCITY_UM_S,
        integral_limit_mm: float = 5.0,
        feedforward_gain: float = 1.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_output_um_s = max_output_um_s
        self.integral_limit_mm = integral_limit_mm
        self.feedforward_gain = feedforward_gain

        # PID state (per axis)
        self._integral_x: float = 0.0
        self._integral_y: float = 0.0
        self._prev_error_x: float = 0.0
        self._prev_error_y: float = 0.0

        self.error_monitor = TrackingErrorMonitor()
        self.command_history: deque[tuple[float, float, float]] = deque(maxlen=100)

    def reset(self):
        """Reset PID state for new print."""
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._prev_error_x = 0.0
        self._prev_error_y = 0.0
        self.error_monitor.reset()
        self.command_history.clear()

    def step(
        self,
        measured_pos: tuple[float, float],
        desired_trajectory: np.ndarray,
        current_time: float,
    ) -> tuple[float, float]:
        """
        PID control step.

        Args:
            measured_pos: (x, y) in mm
            desired_trajectory: Nx7 waypoint array
            current_time: Current time in seconds

        Returns:
            (vx_cmd, vy_cmd) in µm/s
        """
        if len(desired_trajectory) == 0:
            return (0.0, 0.0)

        # Get desired position at current time
        target = self._interpolate_target(desired_trajectory, current_time)

        # Position error
        error_x = target[0] - measured_pos[0]
        error_y = target[1] - measured_pos[1]

        # Integral (with anti-windup clamping)
        self._integral_x += error_x * self.dt
        self._integral_y += error_y * self.dt
        self._integral_x = np.clip(
            self._integral_x, -self.integral_limit_mm, self.integral_limit_mm)
        self._integral_y = np.clip(
            self._integral_y, -self.integral_limit_mm, self.integral_limit_mm)

        # Derivative
        deriv_x = (error_x - self._prev_error_x) / self.dt if self.dt > 0 else 0
        deriv_y = (error_y - self._prev_error_y) / self.dt if self.dt > 0 else 0
        self._prev_error_x = error_x
        self._prev_error_y = error_y

        # PID output (mm/s)
        pid_vx = self.kp * error_x + self.ki * self._integral_x + self.kd * deriv_x
        pid_vy = self.kp * error_y + self.ki * self._integral_y + self.kd * deriv_y

        # Feedforward from trajectory
        ff_vx, ff_vy = self._trajectory_velocity(desired_trajectory, current_time)

        # Total command (mm/s → µm/s)
        vx_um = (self.feedforward_gain * ff_vx + pid_vx) * MM_TO_UM
        vy_um = (self.feedforward_gain * ff_vy + pid_vy) * MM_TO_UM

        # Rate limit
        vx_um, vy_um = self._rate_limit(vx_um, vy_um)

        # Record error
        err_mag = math.sqrt(error_x**2 + error_y**2)
        err = TrackingError(
            t=current_time,
            error_x_mm=error_x, error_y_mm=error_y,
            error_mag_mm=err_mag,
            desired_x=target[0], desired_y=target[1],
            actual_x=measured_pos[0], actual_y=measured_pos[1],
            cmd_vx=vx_um, cmd_vy=vy_um,
        )
        self.error_monitor.record(err)
        self.command_history.append((current_time, vx_um, vy_um))

        return (vx_um, vy_um)

    def _interpolate_target(
        self, trajectory: np.ndarray, t: float,
    ) -> tuple[float, float]:
        """Linearly interpolate XY target position at time t."""
        times = trajectory[:, 6]
        if t <= times[0]:
            return (float(trajectory[0, 0]), float(trajectory[0, 1]))
        if t >= times[-1]:
            return (float(trajectory[-1, 0]), float(trajectory[-1, 1]))
        idx = np.searchsorted(times, t, side='right') - 1
        idx = max(0, min(idx, len(times) - 2))
        t0, t1 = times[idx], times[idx + 1]
        dt = t1 - t0
        frac = (t - t0) / dt if dt > 1e-9 else 0.0
        x = trajectory[idx, 0] + frac * (trajectory[idx + 1, 0] - trajectory[idx, 0])
        y = trajectory[idx, 1] + frac * (trajectory[idx + 1, 1] - trajectory[idx, 1])
        return (float(x), float(y))

    def _trajectory_velocity(
        self, trajectory: np.ndarray, t: float,
    ) -> tuple[float, float]:
        """Instantaneous velocity from trajectory derivative (mm/s)."""
        times = trajectory[:, 6]
        idx = np.searchsorted(times, t, side='right') - 1
        idx = max(0, min(idx, len(times) - 2))
        dt = times[idx + 1] - times[idx]
        if dt < 1e-9:
            return (0.0, 0.0)
        vx = (trajectory[idx + 1, 0] - trajectory[idx, 0]) / dt
        vy = (trajectory[idx + 1, 1] - trajectory[idx, 1]) / dt
        return (float(vx), float(vy))

    def _rate_limit(self, vx: float, vy: float) -> tuple[float, float]:
        mag = math.sqrt(vx**2 + vy**2)
        if mag > self.max_output_um_s:
            scale = self.max_output_um_s / mag
            vx *= scale
            vy *= scale
        if mag < MIN_VELOCITY_UM_S:
            return (0.0, 0.0)
        return (vx, vy)

    def to_dict(self) -> dict:
        return {
            "kp": self.kp, "ki": self.ki, "kd": self.kd,
            "dt": self.dt,
            "max_output_um_s": self.max_output_um_s,
            "integral_limit_mm": self.integral_limit_mm,
            "feedforward_gain": self.feedforward_gain,
        }

    @classmethod
    def from_dict(cls, d: dict) -> PIDMotionController:
        return cls(**{k: d[k] for k in d if k in cls.__init__.__code__.co_varnames})


# ---------------------------------------------------------------------------
# Ziegler-Nichols Auto-Tune Parameter Calculator
# ---------------------------------------------------------------------------

def compute_zn_pid_gains(
    ultimate_gain: float,
    ultimate_period_s: float,
    method: str = "classic",
) -> dict[str, float]:
    """
    Compute PID gains from Ziegler-Nichols ultimate gain/period.

    The caller must experimentally determine Ku and Tu by running
    a relay feedback oscillation test on the stage.

    Args:
        ultimate_gain: Ku — gain at which system oscillates
        ultimate_period_s: Tu — oscillation period in seconds
        method: "classic", "some_overshoot", or "no_overshoot"

    Returns:
        {"kp": float, "ki": float, "kd": float}
    """
    Ku = ultimate_gain
    Tu = ultimate_period_s

    if method == "no_overshoot":
        kp = 0.20 * Ku
        ki = 0.40 * Ku / Tu
        kd = 0.066 * Ku * Tu
    elif method == "some_overshoot":
        kp = 0.33 * Ku
        ki = 0.66 * Ku / Tu
        kd = 0.11 * Ku * Tu
    else:  # classic
        kp = 0.60 * Ku
        ki = 1.20 * Ku / Tu
        kd = 0.075 * Ku * Tu

    return {"kp": kp, "ki": ki, "kd": kd}


# ---------------------------------------------------------------------------
# Stage Rate Tester
# ---------------------------------------------------------------------------

@dataclass
class StageRateTestResult:
    """Results from stage command rate testing."""
    command_response_ms: float = 0.0   # Time until stage starts moving
    settle_time_ms: float = 0.0        # Time until position stabilizes
    min_reliable_poll_ms: int = 333    # Fastest poll that gives stable reads
    max_command_hz: float = 2.0        # Effective command rate
    position_accuracy_um: float = 50.0 # Typical overshoot/undershoot
    avg_round_trip_ms: float = 500.0   # Send command + read position
    raw_data: list[dict] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "command_response_ms": self.command_response_ms,
            "settle_time_ms": self.settle_time_ms,
            "min_reliable_poll_ms": self.min_reliable_poll_ms,
            "max_command_hz": self.max_command_hz,
            "position_accuracy_um": self.position_accuracy_um,
            "avg_round_trip_ms": self.avg_round_trip_ms,
        }

    @classmethod
    def from_dict(cls, d: dict) -> StageRateTestResult:
        return cls(**{k: d[k] for k in d if k in cls.__dataclass_fields__
                      and k != "raw_data"})


def test_stage_command_rate(
    move_fn,
    position_fn,
    displacement_um: float = 500.0,
    num_trials: int = 5,
    poll_intervals_ms: tuple[int, ...] = (100, 200, 333, 500, 1000),
    settle_threshold_um: float = 10.0,
    max_wait_ms: int = 5000,
    on_progress=None,
) -> StageRateTestResult:
    """
    Measure actual stage responsiveness.

    SAFETY: Uses LOW velocity and SMALL displacements only.
    The stage must be connected and clear of obstacles.

    This function is hardware-agnostic — it takes callable
    `move_fn(dx, dy)` and `position_fn() → (x, y)` so it
    can work with real or simulated stages.

    Args:
        move_fn: Callable(dx_um, dy_um) to command relative moves
        position_fn: Callable() → (x_um, y_um) to read position
        displacement_um: Size of test moves (µm)
        num_trials: Number of test moves per direction
        poll_intervals_ms: Intervals to test polling at
        settle_threshold_um: Position change below this = settled
        max_wait_ms: Max time to wait for settle
        on_progress: Optional callback(step, total, message)

    Returns:
        StageRateTestResult with measured characteristics
    """
    raw_data = []
    response_times = []
    settle_times = []
    round_trips = []
    accuracies = []

    total_steps = num_trials * 2  # Forward + backward
    step = 0

    for trial in range(num_trials):
        for direction in [1, -1]:
            dx = displacement_um * direction
            step += 1
            if on_progress:
                on_progress(step, total_steps, f"Trial {trial+1}/{num_trials}")

            # Record start position
            start_pos = position_fn()
            if start_pos[0] is None:
                continue

            start_x = start_pos[0]
            target_x = start_x + dx

            # Send move command and measure round-trip
            t_cmd = time.monotonic()
            move_fn(dx, 0)
            t_sent = time.monotonic()

            # Poll position at intervals to measure response
            t_first_move = None
            t_settled = None
            prev_x = start_x
            trial_data = {"dx": dx, "polls": []}

            elapsed_ms = 0
            while elapsed_ms < max_wait_ms:
                time.sleep(0.05)  # 50ms min poll
                elapsed_ms = (time.monotonic() - t_cmd) * 1000

                pos = position_fn()
                if pos[0] is None:
                    continue
                curr_x = pos[0]
                rt_ms = (time.monotonic() - t_cmd) * 1000

                trial_data["polls"].append({
                    "t_ms": rt_ms, "x": curr_x,
                })

                moved = abs(curr_x - start_x)
                if t_first_move is None and moved > settle_threshold_um:
                    t_first_move = rt_ms

                if t_settled is None and abs(curr_x - target_x) < settle_threshold_um:
                    t_settled = rt_ms

                prev_x = curr_x

                if t_settled is not None:
                    break

            # Record measurements
            final_pos = position_fn()
            final_x = final_pos[0] if final_pos[0] is not None else start_x
            accuracy = abs(final_x - target_x)

            rt = (time.monotonic() - t_cmd) * 1000
            round_trips.append(rt)

            if t_first_move is not None:
                response_times.append(t_first_move)
            if t_settled is not None:
                settle_times.append(t_settled)
            accuracies.append(accuracy)

            trial_data["response_ms"] = t_first_move
            trial_data["settle_ms"] = t_settled
            trial_data["accuracy_um"] = accuracy
            raw_data.append(trial_data)

    # Compute summary
    result = StageRateTestResult(
        command_response_ms=float(np.mean(response_times)) if response_times else 500.0,
        settle_time_ms=float(np.mean(settle_times)) if settle_times else 1000.0,
        avg_round_trip_ms=float(np.mean(round_trips)) if round_trips else 500.0,
        position_accuracy_um=float(np.mean(accuracies)) if accuracies else 50.0,
        raw_data=raw_data,
    )

    # Compute max command Hz from settle time
    if result.settle_time_ms > 0:
        result.max_command_hz = 1000.0 / result.settle_time_ms
    else:
        result.max_command_hz = 2.0

    # Find minimum reliable poll interval
    if response_times:
        min_response = min(response_times)
        for interval in sorted(poll_intervals_ms):
            if interval >= min_response * 0.8:
                result.min_reliable_poll_ms = interval
                break
        else:
            result.min_reliable_poll_ms = max(poll_intervals_ms)

    logger.info(f"Stage rate test: response={result.command_response_ms:.0f}ms, "
                f"settle={result.settle_time_ms:.0f}ms, "
                f"accuracy={result.position_accuracy_um:.1f}µm")
    return result


# ---------------------------------------------------------------------------
# Controller Factory
# ---------------------------------------------------------------------------

def create_controller(
    strategy: str = "kalman",
    params: dict | None = None,
) -> KalmanMotionController | PIDMotionController:
    """
    Factory function to create a motion controller.

    Args:
        strategy: "kalman" or "pid"
        params: Optional dict of tuning parameters

    Returns:
        Controller instance
    """
    params = params or {}
    if strategy == "pid":
        return PIDMotionController.from_dict(params) if params else PIDMotionController()
    else:
        return KalmanMotionController.from_dict(params) if params else KalmanMotionController()
