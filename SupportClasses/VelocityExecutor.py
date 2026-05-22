"""
VelocityExecutor.py — Velocity-based trajectory execution for MEBP v7.3.

Runs a real-time control loop that sends velocity commands to the XY stage
via the KalmanMotionController or PIDMotionController, while commanding
Z and pump axes with position-based moves.

Architecture:
    Trajectory (Waypoints)
         │
         ▼
    Nx7 numpy array ──► MotionController.step(pos, traj, t)
         │                       │
         │                  (vx, vy) µm/s
         │                       │
         ▼                       ▼
    Z + Pump position    send_velocity_xy(vx, vy)
    moves (Marlin)       (Prior ProScan)
         │                       │
         └───── PrintRecorder ◄──┘
              (planned vs actual)

Control loop runs at 50ms (20Hz), matching the controller dt.
Position feedback comes from get_position_with_timestamp() for
accurate Kalman filtering.

Usage:
    executor = VelocityExecutor(controller, strategy="kalman")
    executor.execute(waypoints, pause_event, on_progress)
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Callable, Optional

import numpy as np

logger = logging.getLogger(__name__)

# Column indices matching MotionController's expected format
COL_X, COL_Y, COL_Z = 0, 1, 2
COL_P1, COL_P2, COL_P3 = 3, 4, 5
COL_T = 6

# Control loop timing
CONTROL_DT = 0.05          # 50ms control loop interval
POSITION_STALE_S = 1.0     # If no position update for this long, stop
Z_COMMAND_INTERVAL = 0.2   # Only send Z commands every 200ms (Marlin is slow)
PUMP_COMMAND_INTERVAL = 0.2


class VelocityExecutor:
    """
    Velocity-based trajectory executor using MotionController feedback.

    XY axes: velocity commands via send_velocity_xy()
    Z axis: position commands via move_z_absolute()
    Pumps: position commands via move_pump_relative()
    """

    def __init__(
        self,
        controller,
        strategy: str = "kalman",
        controller_params: dict | None = None,
        recorder=None,
    ):
        """
        Args:
            controller: StageController instance
            strategy: "kalman" or "pid"
            controller_params: Optional tuning parameters
            recorder: Optional PrintRecorder
        """
        self.controller = controller
        self.recorder = recorder
        self._abort_flag = threading.Event()
        self._strategy = strategy

        # Create motion controller
        try:
            from SupportClasses.MotionController import create_controller
            self._motion_ctrl = create_controller(strategy, controller_params)
        except ImportError:
            logger.warning("MotionController not available — falling back to position mode")
            self._motion_ctrl = None

        # Tracking state
        self._traj_array: np.ndarray | None = None
        self._last_z_cmd_time = 0.0
        self._last_pump_cmd_time = 0.0
        self._last_z_sent = None
        self._last_pumps_sent = {"P1": None, "P2": None, "P3": None}

    def execute(
        self,
        waypoints: list,
        pause_event: threading.Event | None = None,
        on_progress: Callable | None = None,
    ) -> bool:
        """
        Execute a trajectory using velocity-based XY control.

        Args:
            waypoints: List of Waypoint objects (from PrintTrajectoryPlanner)
            pause_event: Event that blocks when cleared (for pause)
            on_progress: Callback(current_idx, total, message)

        Returns:
            True if completed, False if aborted
        """
        if not waypoints:
            logger.warning("VelocityExecutor: empty waypoint list")
            return True

        if self._motion_ctrl is None:
            logger.warning("No MotionController — falling back to position execution")
            return self._execute_position_fallback(waypoints, pause_event, on_progress)

        total = len(waypoints)
        ctrl = self.controller

        # Convert waypoints to Nx7 numpy array
        self._traj_array = self._waypoints_to_array(waypoints)
        traj_duration = self._traj_array[-1, COL_T]

        # Initialize motion controller with current position
        try:
            pos = ctrl.get_position_with_timestamp()
            xy = pos.get("xy", (None, None, None))
            if xy[0] is not None:
                zero = getattr(ctrl, 'zero_position', {})
                init_x = xy[0] - zero.get('x', 0)
                init_y = xy[1] - zero.get('y', 0)
                self._motion_ctrl.reset(initial_pos=(init_x, init_y))
            else:
                self._motion_ctrl.reset()
        except Exception:
            self._motion_ctrl.reset()

        logger.info(
            f"VelocityExecutor: starting {total} waypoints, "
            f"duration={traj_duration:.1f}s, strategy={self._strategy}")

        t_start = time.monotonic()
        last_progress_idx = 0
        last_position_time = t_start

        try:
            while True:
                # Check abort
                if self._abort_flag.is_set():
                    self._stop_xy()
                    logger.info("VelocityExecutor: aborted")
                    return False

                # Wait if paused
                if pause_event is not None:
                    if not pause_event.is_set():
                        self._stop_xy()  # Stop XY while paused
                        pause_event.wait()
                        if self._abort_flag.is_set():
                            return False
                        # Recalibrate time after pause
                        t_start = time.monotonic() - elapsed
                        logger.info("VelocityExecutor: resumed")

                # Current elapsed time
                t_now = time.monotonic()
                elapsed = t_now - t_start

                # Check if trajectory is complete
                if elapsed >= traj_duration + 0.5:  # 500ms grace period
                    self._stop_xy()
                    logger.info("VelocityExecutor: trajectory complete")
                    break

                # ── Read position ─────────────────────────────────
                measured_x, measured_y = None, None
                measured_z = None
                try:
                    if getattr(ctrl, 'is_xy_connected', False):
                        pos_data = ctrl.get_position_with_timestamp()
                        xy = pos_data.get("xy", (None, None, None))
                        zp = pos_data.get("zp", (None, None, None, None))
                        zero = getattr(ctrl, 'zero_position', {})

                        if xy[0] is not None:
                            # v7.3: Convert µm → mm (waypoints are in mm)
                            measured_x = (xy[0] - zero.get('x', 0)) / 1000.0
                            measured_y = (xy[1] - zero.get('y', 0)) / 1000.0
                            last_position_time = t_now

                        # v7.4.2 hotfix: route Z read through axis_map.
                        z_val = ctrl.zp_logical_value(zp, "Z") if zp else None
                        if z_val is not None:
                            measured_z = z_val - zero.get('Z', 0)
                except Exception as e:
                    logger.debug(f"Position read error: {e}")

                # Safety: if no position for too long, stop
                if t_now - last_position_time > POSITION_STALE_S:
                    logger.warning("Position stale — stopping XY")
                    self._stop_xy()
                    time.sleep(CONTROL_DT)
                    continue

                # ── XY velocity control ───────────────────────────
                if measured_x is not None and measured_y is not None:
                    try:
                        vx, vy = self._motion_ctrl.step(
                            measured_pos=(measured_x, measured_y),
                            desired_trajectory=self._traj_array,
                            current_time=elapsed,
                        )
                        ctrl.send_velocity_xy(vx, vy)
                    except Exception as e:
                        logger.debug(f"Velocity command error: {e}")

                # ── Z position control (throttled) ────────────────
                if t_now - self._last_z_cmd_time >= Z_COMMAND_INTERVAL:
                    target_z = self._interpolate_axis(elapsed, COL_Z)
                    if target_z is not None and target_z != self._last_z_sent:
                        try:
                            ctrl.move_z_absolute(target_z, from_zero_ref=True)
                            self._last_z_sent = target_z
                        except Exception as e:
                            logger.debug(f"Z command error: {e}")
                    self._last_z_cmd_time = t_now

                # ── Pump position control (throttled) ─────────────
                if t_now - self._last_pump_cmd_time >= PUMP_COMMAND_INTERVAL:
                    self._update_pumps(elapsed, ctrl)
                    self._last_pump_cmd_time = t_now

                # ── Recording ─────────────────────────────────────
                if self.recorder and self.recorder.is_recording:
                    self._record_sample(elapsed, waypoints, measured_x,
                                       measured_y, measured_z, ctrl)

                # ── Progress reporting ────────────────────────────
                if on_progress:
                    progress_frac = min(1.0, elapsed / max(traj_duration, 0.01))
                    progress_idx = int(progress_frac * total)
                    if progress_idx != last_progress_idx or progress_idx % 10 == 0:
                        # Find current well name from nearest waypoint
                        wp_idx = min(progress_idx, total - 1)
                        well = getattr(waypoints[wp_idx], 'well', '')
                        segment = getattr(waypoints[wp_idx], 'segment', '')
                        msg = f"Well {well}: {segment}" if well else segment
                        on_progress(progress_idx, total, msg)
                        last_progress_idx = progress_idx

                # ── Sleep to maintain loop rate ───────────────────
                loop_end = time.monotonic()
                sleep_time = CONTROL_DT - (loop_end - t_now)
                if sleep_time > 0.001:
                    time.sleep(sleep_time)

        except Exception as exc:
            logger.error(f"VelocityExecutor error: {exc}", exc_info=True)
            self._stop_xy()
            return False

        # Final: stop XY and move to last waypoint position
        self._stop_xy()
        last_wp = waypoints[-1]
        try:
            ctrl.move_xy_absolute(last_wp.x, last_wp.y, from_zero_ref=True)
            ctrl.move_z_absolute(last_wp.z, from_zero_ref=True)
        except Exception:
            pass

        if on_progress:
            on_progress(total, total, "Complete!")
        return True

    # ── Helpers ───────────────────────────────────────────────────

    def _waypoints_to_array(self, waypoints) -> np.ndarray:
        """Convert Waypoint list to Nx7 numpy array [x,y,z,p1,p2,p3,t]."""
        rows = []
        for wp in waypoints:
            rows.append([wp.x, wp.y, wp.z, wp.p1, wp.p2, wp.p3, wp.t])
        return np.array(rows, dtype=np.float64)

    def _interpolate_axis(self, t: float, col: int) -> float | None:
        """Interpolate a single axis value at time t."""
        if self._traj_array is None or len(self._traj_array) == 0:
            return None
        times = self._traj_array[:, COL_T]
        vals = self._traj_array[:, col]
        if t <= times[0]:
            return float(vals[0])
        if t >= times[-1]:
            return float(vals[-1])
        # Linear interpolation
        idx = np.searchsorted(times, t) - 1
        idx = max(0, min(idx, len(times) - 2))
        t0, t1 = times[idx], times[idx + 1]
        dt = t1 - t0
        if dt < 1e-9:
            return float(vals[idx])
        frac = (t - t0) / dt
        return float(vals[idx] + (vals[idx + 1] - vals[idx]) * frac)

    def _update_pumps(self, elapsed: float, ctrl):
        """Send pump position commands."""
        from SupportClasses.ZPStage import AXIS_MAP
        zero = getattr(ctrl, 'zero_position', {})
        # v7.4.2: honor configurable per-machine axis_map
        _axis_map = getattr(ctrl.zp_stage, 'axis_map', AXIS_MAP) \
            if ctrl.zp_stage else AXIS_MAP

        for pump_id, col in [("P1", COL_P1), ("P2", COL_P2), ("P3", COL_P3)]:
            target = self._interpolate_axis(elapsed, col)
            if target is None:
                continue
            if target == self._last_pumps_sent.get(pump_id):
                continue

            mapped = _axis_map.get(pump_id, AXIS_MAP.get(pump_id))
            if mapped and ctrl.zp_stage:
                try:
                    abs_pos = target + zero.get(pump_id, 0)
                    ctrl.zp_stage.move_absolute({mapped: abs_pos}, fast=False)
                    self._last_pumps_sent[pump_id] = target
                except Exception as e:
                    logger.debug(f"Pump {pump_id} command error: {e}")

    def _stop_xy(self):
        """Send zero velocity to stop XY stage."""
        try:
            self.controller.send_velocity_xy(0.0, 0.0)
        except Exception:
            pass

    def _record_sample(self, elapsed, waypoints, mx, my, mz, ctrl):
        """Record planned vs actual position."""
        if not self.recorder:
            return
        wp_idx = min(
            int(elapsed / max(waypoints[-1].t, 0.01) * len(waypoints)),
            len(waypoints) - 1)
        wp = waypoints[wp_idx]
        zero = getattr(ctrl, 'zero_position', {})

        actual_xy = (mx or 0, my or 0)
        try:
            zp = ctrl.get_zp_position(cached=True)
            # v7.4.2 hotfix: route each logical axis through axis_map so
            # the recorded actual_zp matches the logical (Z, P1, P2, P3)
            # ordering of the planned waypoints — independent of the
            # physical-tuple order Marlin happens to emit.
            def _logical(ax: str, key: str) -> float:
                v = ctrl.zp_logical_value(zp, ax) if zp else None
                return (v - zero.get(key, 0)) if v is not None else 0
            actual_zp = (
                _logical("Z", "Z"),
                _logical("P1", "P1"),
                _logical("P2", "P2"),
                _logical("P3", "P3"),
            )
        except Exception:
            actual_zp = (0, 0, 0, 0)

        try:
            self.recorder.record_sample(
                t=elapsed,
                planned=(wp.x, wp.y, wp.z, wp.p1, wp.p2, wp.p3),
                actual_xy=actual_xy,
                actual_zp=actual_zp,
                segment_id=getattr(wp, 'segment_id', 0),
                is_travel=getattr(wp, 'is_travel', False),
                is_retract=getattr(wp, 'is_retract', False),
            )
        except Exception as e:
            logger.debug(f"Recording error: {e}")

    # ── Fallback: position-based execution ────────────────────────

    def _execute_position_fallback(self, waypoints, pause_event, on_progress):
        """Fallback using position commands (same as TrajectoryExecutor)."""
        logger.info("Using position-based fallback execution")
        total = len(waypoints)
        ctrl = self.controller
        t_start = time.monotonic()

        for i, wp in enumerate(waypoints):
            if self._abort_flag.is_set():
                return False
            if pause_event is not None:
                pause_event.wait()
                if self._abort_flag.is_set():
                    return False

            # Wait until waypoint time
            t_target = t_start + wp.t
            t_now = time.monotonic()
            if t_target > t_now:
                time.sleep(t_target - t_now)

            # Command axes
            try:
                if getattr(ctrl, 'is_xy_connected', False):
                    ctrl.move_xy_absolute(wp.x, wp.y, from_zero_ref=True)
                if getattr(ctrl, 'is_zp_connected', False):
                    ctrl.move_z_absolute(wp.z, from_zero_ref=True)
                    # Pumps
                    from SupportClasses.ZPStage import AXIS_MAP
                    # v7.4.2: honor configurable per-machine axis_map
                    _axis_map = getattr(ctrl.zp_stage, 'axis_map', AXIS_MAP) \
                        if ctrl.zp_stage else AXIS_MAP
                    zero = getattr(ctrl, 'zero_position', {})
                    for pid, val in [("P1", wp.p1), ("P2", wp.p2), ("P3", wp.p3)]:
                        mapped = _axis_map.get(pid, AXIS_MAP.get(pid))
                        if mapped and ctrl.zp_stage and val != 0.0:
                            ctrl.zp_stage.move_absolute(
                                {mapped: val + zero.get(pid, 0)}, fast=False)
            except Exception as e:
                logger.debug(f"Position fallback error at wp {i}: {e}")

            if on_progress and i % 10 == 0:
                well = getattr(wp, 'well', '')
                on_progress(i, total, f"Well {well}" if well else f"Waypoint {i}/{total}")

        if on_progress:
            on_progress(total, total, "Complete!")
        return True

    def abort(self):
        """Signal executor to stop."""
        self._abort_flag.set()
        self._stop_xy()

    def reset(self):
        """Reset for reuse."""
        self._abort_flag.clear()
        self._last_z_sent = None
        self._last_pumps_sent = {"P1": None, "P2": None, "P3": None}

    @property
    def error_monitor(self):
        """Access the motion controller's tracking error monitor."""
        if self._motion_ctrl and hasattr(self._motion_ctrl, 'error_monitor'):
            return self._motion_ctrl.error_monitor
        return None
