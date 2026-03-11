#!/usr/bin/env python3
"""
Diagnostic test: Full hybrid execution with simulated hardware.

Setup:
  - 24-well plate
  - Print in A1 (spiral path)
  - Ink in A2, Buffer in A3, Wash in B2, Waste in B3

Runs the FULL hybrid executor pipeline and logs every position change,
timing discrepancy, and Z anomaly so we can diagnose issues with:
  1. Z position sequencing (incorrect Z during travel)
  2. Trajectory display timing (estimate vs actual)
  3. Settle dwells and axis-change ordering

Run:  python3 tests/test_hybrid_execution.py
"""

import sys
import os
import time
import math
import threading
import logging

# Project root
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ["QT_QPA_PLATFORM"] = "offscreen"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d [%(name)-22s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("test_hybrid")

# ═══════════════════════════════════════════════════════════════════
# Helper: Position tracker — polls stages and logs every change
# ═══════════════════════════════════════════════════════════════════

class PositionTracker:
    """Background thread that polls XY/Z position and logs changes."""

    def __init__(self, controller):
        self.ctrl = controller
        self._stop = threading.Event()
        self._thread = None
        self.samples = []   # [(t, x_mm, y_mm, z_mm)]
        self._t0 = None

    def start(self):
        self._t0 = time.monotonic()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def _run(self):
        prev_x = prev_y = prev_z = None
        while not self._stop.is_set():
            t = time.monotonic() - self._t0
            try:
                xy = self.ctrl.get_xy_position_mm(cached=False)
                x_mm = xy[0] if xy[0] is not None else 0.0
                y_mm = xy[1] if xy[1] is not None else 0.0
            except Exception:
                x_mm = y_mm = 0.0

            try:
                zp = self.ctrl.get_zp_position(cached=False)
                z_raw = zp[0] if zp[0] is not None else 0.0
                z_zero = self.ctrl.zero_position.get("Z", 0)
                z_mm = z_raw - z_zero
            except Exception:
                z_mm = 0.0

            self.samples.append((t, x_mm, y_mm, z_mm))

            # Log significant moves
            if prev_x is not None:
                dx = abs(x_mm - prev_x)
                dy = abs(y_mm - prev_y)
                dz = abs(z_mm - prev_z)
                if dx > 0.1 or dy > 0.1 or dz > 0.05:
                    logger.info(
                        f"POS  t={t:6.2f}s  XY=({x_mm:8.2f}, {y_mm:8.2f})  "
                        f"Z={z_mm:6.2f}  Δ=({dx:.2f}, {dy:.2f}, {dz:.2f})")

            prev_x, prev_y, prev_z = x_mm, y_mm, z_mm
            time.sleep(0.1)


# ═══════════════════════════════════════════════════════════════════
# Setup: Create plate, well assignments, plan, settings
# ═══════════════════════════════════════════════════════════════════

def setup():
    """Create all objects needed for hybrid execution."""
    from SupportClasses.WellPlate import WellPlate, generate_spiral_path
    from SupportClasses.WellSetup import WellSetupModel
    from SupportClasses.PhysicalModels import WellRole, SyringeSpec, InkSpec
    from SupportClasses.PrintManager import PrintSettings
    from SupportClasses.StageController import StageController
    from SupportClasses.HardwareConfig import HardwareConfig, PumpChannelConfig

    # ── Plate ──
    plate = WellPlate.from_format(24)
    logger.info(f"Plate: 24-well, spacing={plate.well_spacing_x:.1f}mm")

    # Print well positions for reference
    for name in ["A1", "A2", "A3", "B2", "B3"]:
        x, y = plate.get_well_position(name)
        logger.info(f"  {name}: ({x:.2f}, {y:.2f}) mm")

    # ── Well setup model ──
    well_model = WellSetupModel(plate_format=24)
    well_model.set_role(["A1"], WellRole.PRINT)
    well_model.set_role(["A2"], WellRole.INK)
    well_model.set_role(["A3"], WellRole.BUFFER)
    well_model.set_role(["B2"], WellRole.WASH)
    well_model.set_role(["B3"], WellRole.WASTE)

    # Verify assignments
    for name in ["A1", "A2", "A3", "B2", "B3"]:
        wa = well_model.assignments[name]
        logger.info(f"  {name} → {wa.role.value}")

    # ── Path: spiral in-well ──
    path_points = generate_spiral_path(
        diameter=10.0,     # 10mm spiral
        line_spacing=1.0,  # 1mm between turns
        turns=4,
        center_x=0.0,
        center_y=0.0,
        points_per_turn=20,
    )
    logger.info(f"Spiral path: {len(path_points)} points, "
                f"diameter=10mm, 4 turns")

    # ── Settings ──
    settings = PrintSettings(
        travel_z_height=5.0,        # Safe Z for travel
        print_z_height=0.5,         # Z height for printing
        top_z_height=2.0,           # Well top (for 2-phase Z entry)
        num_layers=1,
        layer_height=0.1,
        print_feedrate=200.0,       # 200 mm/min = 3.33 mm/s
        z_feedrate=60.0,            # 60 mm/min = 1 mm/s
        pump_feedrate=30.0,
        xy_feedrate=1000.0,
        print_speed_mm_s=3.0,
        travel_speed_mm_s=10.0,
        service_xy_speed_mm_s=20.0,
        fast_z_feedrate_mm_min=120.0,
        entry_z_feedrate_mm_min=12.0,
        pump_rate_uL_s=0.25,
        service_pump_rate_uL_s=5.0,
        dwell_after_move=0.0,
        prime_amount=0.0,
        retract_amount=0.0,
    )

    # ── Hardware config (needed for pump µL→mm conversion) ──
    syringe = SyringeSpec(volume_uL=250, stroke_length_mm=30.0)
    ink = InkSpec(name="Test Ink", color="#89b4fa")
    p1 = PumpChannelConfig(pump_id="P1", syringe=syringe, inks=[ink], enabled=True)
    hw_config = HardwareConfig(
        pumps={"P1": p1,
               "P2": PumpChannelConfig(pump_id="P2"),
               "P3": PumpChannelConfig(pump_id="P3")},
        plate_format=24,
    )

    # ── StageController (simulated) ──
    ctrl = StageController(simulate_xy=True, simulate_zp=True)
    ctrl.connect_stages(xy=True, zp=True)
    ctrl.set_hardware_config(hw_config)

    # Set a calibrated zero position (simulates having calibrated A1)
    # A1 is at plate offset from corner
    ctrl.zero_position = {"x": 5000, "y": 5000, "Z": 10.0,
                          "P1": 0, "P2": 0, "P3": 0}
    logger.info(f"Zero position: {ctrl.zero_position}")

    return plate, well_model, path_points, settings, ctrl


# ═══════════════════════════════════════════════════════════════════
# Build Plan of Action
# ═══════════════════════════════════════════════════════════════════

def build_plan():
    """Build a realistic plan: gather ink → print A1 → waste → return home."""
    from SupportClasses.PrintPlanOfAction import PlanStepType, PlanStep

    steps = [
        # 1. Load ink from A2
        PlanStep(
            step_type=PlanStepType.LOAD_INK,
            description="Load ink from A2",
            target_wells=["A2"],
            pump_id="P1",
            volume_uL=50.0,
            ink_name="Test Ink",
        ),
        # 2. Print in A1
        PlanStep(
            step_type=PlanStepType.PRINT,
            description="Print spiral in A1",
            target_wells=["A1"],
            pump_id="P1",
            volume_uL=0.0,
        ),
        # 3. Waste eject in B3
        PlanStep(
            step_type=PlanStepType.WASTE,
            description="Eject waste in B3",
            target_wells=["B3"],
            pump_id="P1",
            volume_uL=50.0,
        ),
        # 4. Wash in B2
        PlanStep(
            step_type=PlanStepType.WASH,
            description="Wash in B2",
            target_wells=["B2"],
            pump_id="P1",
            volume_uL=0.0,
        ),
        # 5. Return home
        PlanStep(
            step_type=PlanStepType.RETURN_HOME,
            description="Return to home position",
            target_wells=[],
            pump_id="P1",
            volume_uL=0.0,
        ),
    ]

    # Wrap in a plan-like object
    class SimplePlan:
        def __init__(self, steps):
            self.steps = steps

    return SimplePlan(steps)


# ═══════════════════════════════════════════════════════════════════
# TEST 1: Time estimation accuracy
# ═══════════════════════════════════════════════════════════════════

def test_estimate_time(plate, well_model, path_points, settings, ctrl):
    """Run estimate_time() and print the breakdown."""
    print("\n" + "=" * 70)
    print("TEST 1: Time Estimation")
    print("=" * 70)

    from SupportClasses.PrintManager import HybridPlanExecutor

    plan = build_plan()
    executor = HybridPlanExecutor(
        controller=ctrl,
        plan=plan,
        well_model=well_model,
        plate=plate,
        path_points=path_points,
        settings=settings,
    )

    estimated = executor.estimate_time()
    logger.info(f"Estimated total time: {estimated:.1f}s ({estimated/60:.1f}min)")
    return estimated


# ═══════════════════════════════════════════════════════════════════
# TEST 2: Full hybrid execution with position tracking
# ═══════════════════════════════════════════════════════════════════

def test_full_execution(plate, well_model, path_points, settings, ctrl, estimated_time):
    """Execute the full plan and compare actual vs estimated timing."""
    print("\n" + "=" * 70)
    print("TEST 2: Full Hybrid Execution")
    print("=" * 70)

    from SupportClasses.PrintManager import HybridPlanExecutor

    plan = build_plan()
    executor = HybridPlanExecutor(
        controller=ctrl,
        plan=plan,
        well_model=well_model,
        plate=plate,
        path_points=path_points,
        settings=settings,
    )

    # Start position tracker
    tracker = PositionTracker(ctrl)
    tracker.start()

    # Progress log
    progress_log = []

    def on_progress(step, total, msg):
        t = time.monotonic()
        progress_log.append((t, step, total, msg))
        logger.info(f"PROGRESS: [{step}/{total}] {msg}")

    # Execute
    t_start = time.monotonic()
    logger.info(">>> Starting hybrid execution...")

    success = executor.execute(
        pause_event=None,
        on_progress=on_progress,
    )

    t_end = time.monotonic()
    actual_time = t_end - t_start
    tracker.stop()

    # ── Results ──
    print("\n" + "-" * 70)
    print("EXECUTION RESULTS")
    print("-" * 70)
    print(f"  Success:        {success}")
    print(f"  Estimated time: {estimated_time:.1f}s")
    print(f"  Actual time:    {actual_time:.1f}s")
    if estimated_time > 0:
        error_pct = abs(actual_time - estimated_time) / estimated_time * 100
        print(f"  Error:          {error_pct:.1f}%")
    print(f"  Position samples: {len(tracker.samples)}")

    # ── Z position analysis ──
    print("\n" + "-" * 70)
    print("Z POSITION ANALYSIS")
    print("-" * 70)

    z_changes = []
    prev_z = None
    for t, x, y, z in tracker.samples:
        if prev_z is not None and abs(z - prev_z) > 0.05:
            z_changes.append((t, z, prev_z, x, y))
        prev_z = z

    print(f"  Z changes detected: {len(z_changes)}")
    for t, z_new, z_old, x, y in z_changes:
        direction = "DOWN" if z_new < z_old else "UP"
        print(f"    t={t:6.2f}s  Z: {z_old:.2f} → {z_new:.2f}  "
              f"({direction} {abs(z_new-z_old):.2f}mm)  "
              f"XY=({x:.1f}, {y:.1f})")

    # ── XY position at Z changes — check safety ──
    print("\n" + "-" * 70)
    print("Z SAFETY CHECK: XY position when Z descends")
    print("-" * 70)

    safe_z = settings.travel_z_height
    print_z = settings.print_z_height
    top_z = settings.top_z_height

    for t, z_new, z_old, x, y in z_changes:
        if z_new < z_old:  # Z descending
            # Check: is XY at a valid well position?
            well_found = False
            for wname in ["A1", "A2", "A3", "B2", "B3"]:
                wx, wy = plate.get_well_position(wname)
                dist = math.sqrt((x - wx)**2 + (y - wy)**2)
                if dist < 10.0:  # within 10mm of a well
                    well_found = True
                    print(f"    t={t:.2f}s  Z↓ to {z_new:.2f}  "
                          f"XY near {wname} (dist={dist:.1f}mm)  ✓")
                    break
            if not well_found:
                print(f"    t={t:.2f}s  Z↓ to {z_new:.2f}  "
                      f"XY=({x:.1f}, {y:.1f}) — NOT NEAR ANY WELL  ✗")

    # ── Trajectory playback analysis ──
    print("\n" + "-" * 70)
    print("TRAJECTORY MOTION ANALYSIS")
    print("-" * 70)

    # Find the printing phase: when Z is at print_z and XY is moving
    print_samples = [(t, x, y, z) for t, x, y, z in tracker.samples
                     if abs(z - print_z) < 0.3]
    if print_samples:
        t0 = print_samples[0][0]
        t1 = print_samples[-1][0]
        print(f"  Print phase: t={t0:.2f}s to t={t1:.2f}s "
              f"({t1-t0:.1f}s, {len(print_samples)} samples)")

        # Check XY motion during print
        moving_count = 0
        for i in range(1, len(print_samples)):
            dx = print_samples[i][1] - print_samples[i-1][1]
            dy = print_samples[i][2] - print_samples[i-1][2]
            if abs(dx) > 0.01 or abs(dy) > 0.01:
                moving_count += 1
        print(f"  XY moving during print: {moving_count}/{len(print_samples)} samples")
    else:
        print(f"  WARNING: No samples found at print Z={print_z:.2f}mm!")

    # ── Full timeline ──
    print("\n" + "-" * 70)
    print("FULL POSITION TIMELINE (sampled every 0.5s)")
    print("-" * 70)
    print(f"  {'Time':>7s}  {'X':>8s}  {'Y':>8s}  {'Z':>6s}")

    last_print_t = -1
    for t, x, y, z in tracker.samples:
        if t - last_print_t >= 0.5:
            print(f"  {t:7.2f}s  {x:8.2f}  {y:8.2f}  {z:6.2f}")
            last_print_t = t

    return actual_time, tracker.samples


# ═══════════════════════════════════════════════════════════════════
# TEST 3: DirectCommandExecutor Z sequencing
# ═══════════════════════════════════════════════════════════════════

def test_z_sequencing(ctrl, settings, plate):
    """Test the Z up → XY → Z down sequence in isolation."""
    print("\n" + "=" * 70)
    print("TEST 3: DirectCommandExecutor Z Sequencing")
    print("=" * 70)

    from SupportClasses.PrintManager import DirectCommandExecutor

    direct = DirectCommandExecutor(ctrl)
    safe_z = settings.travel_z_height
    print_z = settings.print_z_height
    top_z = settings.top_z_height

    # Start at home
    direct.move_z(safe_z)
    direct.move_xy(0, 0, timeout_s=10)
    time.sleep(0.5)

    # Get A1 position
    wx, wy = plate.get_well_position("A1")
    logger.info(f"A1 position: ({wx:.2f}, {wy:.2f})")

    # travel_to_well sequence
    print("\n  Testing travel_to_well(A1)...")
    t0 = time.monotonic()

    # Phase 1: Z up
    logger.info("Phase 1: Z up to safe_z")
    ok = direct.move_z(safe_z)
    z_pos = ctrl.get_zp_position(cached=False)
    z_ref = ctrl.zero_position.get("Z", 0)
    z_mm = (z_pos[0] - z_ref) if z_pos[0] is not None else -999
    print(f"    Z up: ok={ok}, Z={z_mm:.2f}mm (target={safe_z:.2f})")

    direct.dwell(1.0)

    # Phase 2: XY travel
    logger.info("Phase 2: XY to A1")
    ok = direct.move_xy(wx, wy, timeout_s=10)
    xy = ctrl.get_xy_position_mm(cached=False)
    print(f"    XY move: ok={ok}, pos=({xy[0]:.2f}, {xy[1]:.2f}) "
          f"(target=({wx:.2f}, {wy:.2f}))")

    direct.dwell(1.0)

    # Phase 3: Z down (2-phase)
    logger.info("Phase 3: Z down to print_z (2-phase)")
    if top_z > 0:
        ok1 = direct.move_z(top_z + 0.5)
        z_pos = ctrl.get_zp_position(cached=False)
        z_mm = (z_pos[0] - z_ref) if z_pos[0] is not None else -999
        print(f"    Z approach: ok={ok1}, Z={z_mm:.2f}mm (target={top_z+0.5:.2f})")

        direct.dwell(1.0)

    ok2 = direct.move_z(print_z)
    z_pos = ctrl.get_zp_position(cached=False)
    z_mm = (z_pos[0] - z_ref) if z_pos[0] is not None else -999
    print(f"    Z final: ok={ok2}, Z={z_mm:.2f}mm (target={print_z:.2f})")

    elapsed = time.monotonic() - t0
    print(f"\n  Total travel_to_well time: {elapsed:.1f}s")

    # Phase 4: raise
    logger.info("Phase 4: Z raise")
    direct.dwell(1.0)
    ok = direct.move_z(safe_z)
    z_pos = ctrl.get_zp_position(cached=False)
    z_mm = (z_pos[0] - z_ref) if z_pos[0] is not None else -999
    print(f"    Z raise: ok={ok}, Z={z_mm:.2f}mm (target={safe_z:.2f})")


# ═══════════════════════════════════════════════════════════════════
# TEST 4: generate_inwell_print trajectory inspection
# ═══════════════════════════════════════════════════════════════════

def test_inwell_trajectory(plate, path_points, settings):
    """Generate an in-well trajectory and inspect its waypoints."""
    print("\n" + "=" * 70)
    print("TEST 4: In-Well Trajectory Inspection")
    print("=" * 70)

    from SupportClasses.PrintTrajectoryPlanner import PrintTrajectoryPlanner

    planner = PrintTrajectoryPlanner()
    wx, wy = plate.get_well_position("A1")

    result = planner.generate_inwell_print(
        well_x=wx, well_y=wy,
        pump_id="P1",
        path_points=path_points,
        settings=settings,
        well_name="A1",
    )

    print(f"  Valid: {result.valid}")
    print(f"  Issues: {result.issues}")
    print(f"  Waypoints: {len(result.waypoints)}")
    print(f"  Duration: {result.total_duration_s:.2f}s")

    if result.waypoints:
        # Check Z values in trajectory
        z_values = set()
        for wp in result.waypoints:
            z_values.add(round(wp.z, 3))
        print(f"  Z values in trajectory: {sorted(z_values)}")

        # Check: does trajectory move Z away from print_z?
        bad_z = [wp for wp in result.waypoints
                 if abs(wp.z - settings.print_z_height) > 0.5]
        if bad_z:
            print(f"  WARNING: {len(bad_z)} waypoints have Z far from "
                  f"print_z ({settings.print_z_height:.2f})!")
            for wp in bad_z[:5]:
                print(f"    t={wp.t:.3f}s Z={wp.z:.3f} "
                      f"segment={wp.segment}")
        else:
            print(f"  ✓ All waypoints Z ≈ {settings.print_z_height:.2f}mm")

        # XY range
        xs = [wp.x for wp in result.waypoints]
        ys = [wp.y for wp in result.waypoints]
        print(f"  XY range: x=[{min(xs):.2f}, {max(xs):.2f}]  "
              f"y=[{min(ys):.2f}, {max(ys):.2f}]")

        # First and last waypoints
        wp0 = result.waypoints[0]
        wpN = result.waypoints[-1]
        print(f"  First: t={wp0.t:.3f}s XY=({wp0.x:.2f}, {wp0.y:.2f}) "
              f"Z={wp0.z:.2f} seg={wp0.segment}")
        print(f"  Last:  t={wpN.t:.3f}s XY=({wpN.x:.2f}, {wpN.y:.2f}) "
              f"Z={wpN.z:.2f} seg={wpN.segment}")

    return result


# ═══════════════════════════════════════════════════════════════════
# Position Predictor — mirrors estimate_time() logic with positions
# ═══════════════════════════════════════════════════════════════════

class PositionPredictor:
    """Build predicted (x,y,z) timeline from the same logic as estimate_time().

    Each event is (time, x, y, z, phase_label).  Between events, position
    is linearly interpolated.
    """

    def __init__(self, plate, well_model, path_points, settings, plan):
        self.events = []  # [(t, x, y, z, phase)]
        self._build(plate, well_model, path_points, settings, plan)

    # ── Find well helper (mirrors HybridPlanExecutor._find_well) ──

    @staticmethod
    def _find_well(well_model, plate, role):
        """Find first well with given role. Returns (name, x, y) or None."""
        from SupportClasses.PhysicalModels import WellRole
        role_map = {
            "waste": WellRole.WASTE, "wash": WellRole.WASH,
            "buffer": WellRole.BUFFER, "ink": WellRole.INK,
            "print": WellRole.PRINT,
        }
        target = role_map.get(role)
        if not target:
            return None
        for name, wa in well_model.assignments.items():
            if wa.role == target:
                x, y = plate.get_well_position(name)
                return (name, x, y)
        return None

    # ── Build timeline ──

    def _build(self, plate, well_model, path_points, settings, plan):
        from SupportClasses.PrintPlanOfAction import PlanStepType

        s = settings
        safe_z = s.travel_z_height
        print_z = s.print_z_height
        top_z = getattr(s, 'top_z_height', 0.0)
        z_fast = getattr(s, 'fast_z_feedrate_mm_min', s.z_feedrate) / 60.0
        z_entry = getattr(s, 'entry_z_feedrate_mm_min', s.z_feedrate) / 60.0
        xy_travel = getattr(s, 'service_xy_speed_mm_s', 10.0)
        xy_print = getattr(s, 'print_speed_mm_s', 5.0)
        pump_rate = getattr(s, 'service_pump_rate_uL_s', 5.0)
        SETTLE = 1.0

        cur_x, cur_y, cur_z = 0.0, 0.0, safe_z
        t = 0.0
        ev = self.events

        def add(x, y, z, phase):
            ev.append((t, x, y, z, phase))

        def z_time(from_z, to_z):
            dist = abs(to_z - from_z)
            if dist < 0.01:
                return 0.0
            if to_z > from_z:
                return dist / max(z_fast, 0.1)
            if top_z > 0 and from_z > top_z:
                fast_d = max(0, from_z - top_z)
                slow_d = max(0, top_z - to_z)
                return fast_d / max(z_fast, 0.1) + slow_d / max(z_entry, 0.1)
            if top_z > 0 and from_z <= top_z:
                return dist / max(z_entry, 0.1)
            return dist / max(z_fast, 0.1)

        def xy_time(x1, y1, x2, y2, speed):
            d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            return d / max(speed, 0.1) if d > 0.01 else 0.0

        def service_time(role, step):
            if role == "waste":
                vol = getattr(step, 'volume_uL', 50.0) or 50.0
                return vol / max(pump_rate, 0.1) + 0.5
            elif role == "wash":
                return 5.0
            elif role == "buffer":
                vol = getattr(step, 'volume_uL', 5.0) or 5.0
                return vol / max(pump_rate, 0.1) + 1.0
            elif role == "ink":
                vol = getattr(step, 'volume_uL', 50.0) or 50.0
                return vol / max(pump_rate, 0.1) + 1.0
            return 0.0

        def travel_to_well(wx, wy, phase_prefix):
            nonlocal cur_x, cur_y, cur_z, t
            # Z up
            add(cur_x, cur_y, cur_z, f"{phase_prefix}:z_up_start")
            dt = z_time(cur_z, safe_z)
            t += dt
            cur_z = safe_z
            add(cur_x, cur_y, cur_z, f"{phase_prefix}:z_up_done")
            # Settle
            t += SETTLE
            add(cur_x, cur_y, cur_z, f"{phase_prefix}:settle_1")
            # XY
            dt = xy_time(cur_x, cur_y, wx, wy, xy_travel)
            t += dt
            cur_x, cur_y = wx, wy
            add(cur_x, cur_y, cur_z, f"{phase_prefix}:xy_done")
            # Settle
            t += SETTLE
            add(cur_x, cur_y, cur_z, f"{phase_prefix}:settle_2")
            # Z down
            dt = z_time(safe_z, print_z)
            t += dt
            cur_z = print_z
            add(cur_x, cur_y, cur_z, f"{phase_prefix}:z_down_done")
            # Dwell
            dwell = getattr(s, 'dwell_after_move', 0)
            if dwell > 0:
                t += dwell
                add(cur_x, cur_y, cur_z, f"{phase_prefix}:dwell")

        def raise_from_well(phase_prefix):
            nonlocal cur_z, t
            add(cur_x, cur_y, cur_z, f"{phase_prefix}:raise_start")
            dt = z_time(cur_z, safe_z)
            t += dt
            cur_z = safe_z
            add(cur_x, cur_y, cur_z, f"{phase_prefix}:raise_done")

        # Initial position
        add(cur_x, cur_y, cur_z, "start")

        pts = path_points
        # Precompute cumulative path distances for trajectory interpolation
        path_cum = [0.0]
        for i in range(1, len(pts)):
            dx = pts[i][0] - pts[i - 1][0]
            dy = pts[i][1] - pts[i - 1][1]
            path_cum.append(path_cum[-1] + math.sqrt(dx * dx + dy * dy))
        path_len = path_cum[-1] if path_cum else 0.0

        for step in plan.steps:
            stype = getattr(step, 'step_type', None)
            if stype is None:
                continue

            if stype == PlanStepType.PRINT:
                for wn in getattr(step, 'target_wells', []):
                    try:
                        wx, wy = plate.get_well_position(wn)
                    except Exception:
                        continue
                    fx = wx + (pts[0][0] if pts else 0.0)
                    fy = wy + (pts[0][1] if pts else 0.0)

                    # Z up
                    add(cur_x, cur_y, cur_z, f"PRINT({wn}):z_up_start")
                    t += z_time(cur_z, safe_z) + SETTLE
                    cur_z = safe_z
                    add(cur_x, cur_y, cur_z, f"PRINT({wn}):z_up+settle")

                    # XY to first point
                    t += xy_time(cur_x, cur_y, fx, fy, xy_travel) + SETTLE
                    cur_x, cur_y = fx, fy
                    add(cur_x, cur_y, cur_z, f"PRINT({wn}):xy+settle")

                    # Z down
                    t += z_time(safe_z, print_z)
                    cur_z = print_z
                    add(cur_x, cur_y, cur_z, f"PRINT({wn}):z_down_done")

                    # Trajectory: use blocking-time model (matches estimate_time)
                    SETTLE_OVERHEAD_S = 0.25
                    traj_duration = 0.0
                    try:
                        from SupportClasses.PrintTrajectoryPlanner import (
                            PrintTrajectoryPlanner)
                        _pl = PrintTrajectoryPlanner()
                        _r = _pl.generate_inwell_print(
                            well_x=wx, well_y=wy,
                            pump_id=getattr(step, 'pump_id', 'P1') or 'P1',
                            path_points=pts, settings=s, well_name=wn)
                        if _r.valid and _r.waypoints:
                            wps = _r.waypoints
                            _ms = 0.0
                            for _j in range(1, min(len(wps), 100)):
                                _dtw = wps[_j].t - wps[_j-1].t
                                if _dtw > 1e-6:
                                    _dxw = wps[_j].x - wps[_j-1].x
                                    _dyw = wps[_j].y - wps[_j-1].y
                                    _ms = max(_ms, math.sqrt(_dxw**2+_dyw**2)/_dtw)
                            eff_spd = max(_ms * 1.5, 0.1)
                            bt = 0.0
                            for _j in range(1, len(wps)):
                                _dxw = wps[_j].x - wps[_j-1].x
                                _dyw = wps[_j].y - wps[_j-1].y
                                sd = math.sqrt(_dxw**2 + _dyw**2)
                                dt_p = wps[_j].t - wps[_j-1].t
                                bt += max(sd / eff_spd + SETTLE_OVERHEAD_S, dt_p)
                            traj_duration = bt
                    except Exception:
                        pass
                    if traj_duration <= 0 and xy_print > 0 and path_len > 0:
                        traj_duration = path_len / xy_print * s.num_layers
                    if traj_duration > 0:
                        # Add ~20 intermediate events along the trajectory
                        n_samples = min(len(pts), 20)
                        for k in range(n_samples):
                            frac = (k + 1) / n_samples
                            # Find point along path at this fraction
                            target_dist = frac * path_len
                            # Binary search in path_cum
                            lo, hi = 0, len(path_cum) - 1
                            while lo < hi:
                                mid = (lo + hi) // 2
                                if path_cum[mid] < target_dist:
                                    lo = mid + 1
                                else:
                                    hi = mid
                            idx = lo
                            if idx > 0 and idx < len(pts):
                                seg_len = path_cum[idx] - path_cum[idx - 1]
                                if seg_len > 0:
                                    alpha = (target_dist - path_cum[idx - 1]) / seg_len
                                else:
                                    alpha = 0.0
                                px = pts[idx - 1][0] + alpha * (pts[idx][0] - pts[idx - 1][0])
                                py = pts[idx - 1][1] + alpha * (pts[idx][1] - pts[idx - 1][1])
                            else:
                                px, py = pts[-1][0], pts[-1][1]

                            t += traj_duration / n_samples
                            cur_x = wx + px
                            cur_y = wy + py
                            add(cur_x, cur_y, cur_z,
                                f"PRINT({wn}):traj_{k+1}/{n_samples}")

                    # Raise: settle + Z up
                    t += SETTLE
                    add(cur_x, cur_y, cur_z, f"PRINT({wn}):pre_raise")
                    t += z_time(print_z, safe_z)
                    cur_z = safe_z
                    add(cur_x, cur_y, cur_z, f"PRINT({wn}):raised")

            elif stype in (PlanStepType.WASTE, PlanStepType.WASH,
                           PlanStepType.REFILL_BUFFER,
                           PlanStepType.LOAD_INK, PlanStepType.GATHER_INK):
                role_map = {
                    PlanStepType.WASTE: "waste", PlanStepType.WASH: "wash",
                    PlanStepType.REFILL_BUFFER: "buffer",
                    PlanStepType.LOAD_INK: "ink",
                    PlanStepType.GATHER_INK: "ink",
                }
                role = role_map.get(stype, "waste")
                well = self._find_well(well_model, plate, role)
                if well:
                    _, wx, wy = well
                    travel_to_well(wx, wy, f"{stype.name}")
                    # Service action (position stays same)
                    dt = service_time(role, step)
                    t += dt
                    add(cur_x, cur_y, cur_z, f"{stype.name}:action_done")
                    raise_from_well(f"{stype.name}")

            elif stype == PlanStepType.MOVE_SAFE_Z:
                t += z_time(cur_z, safe_z)
                cur_z = safe_z
                add(cur_x, cur_y, cur_z, "MOVE_SAFE_Z")

            elif stype == PlanStepType.RETURN_HOME:
                add(cur_x, cur_y, cur_z, "HOME:z_up_start")
                t += z_time(cur_z, safe_z) + SETTLE
                cur_z = safe_z
                add(cur_x, cur_y, cur_z, "HOME:z_up+settle")
                t += xy_time(cur_x, cur_y, 0, 0, xy_travel)
                cur_x, cur_y = 0.0, 0.0
                add(cur_x, cur_y, cur_z, "HOME:done")

    def predict(self, t_query):
        """Return predicted (x, y, z, phase) at elapsed time t_query."""
        if not self.events:
            return 0.0, 0.0, 0.0, "empty"

        # Before first event
        if t_query <= self.events[0][0]:
            e = self.events[0]
            return e[1], e[2], e[3], e[4]

        # After last event
        if t_query >= self.events[-1][0]:
            e = self.events[-1]
            return e[1], e[2], e[3], e[4] + " (past_end)"

        # Binary search for bracketing events
        lo, hi = 0, len(self.events) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if self.events[mid][0] <= t_query:
                lo = mid
            else:
                hi = mid

        e0 = self.events[lo]
        e1 = self.events[hi]
        dt = e1[0] - e0[0]
        if dt < 1e-6:
            return e1[1], e1[2], e1[3], e1[4]

        alpha = (t_query - e0[0]) / dt
        x = e0[1] + alpha * (e1[1] - e0[1])
        y = e0[2] + alpha * (e1[2] - e0[2])
        z = e0[3] + alpha * (e1[3] - e0[3])
        return x, y, z, e1[4]


# ═══════════════════════════════════════════════════════════════════
# TEST 5: Position prediction vs actual — time-polling error log
# ═══════════════════════════════════════════════════════════════════

def test_position_prediction(plate, well_model, path_points, settings, ctrl):
    """Execute with position prediction and log the error at every poll."""
    print("\n" + "=" * 70)
    print("TEST 5: Position Prediction vs Actual (Time-Polling Error Log)")
    print("=" * 70)

    from SupportClasses.PrintManager import HybridPlanExecutor

    plan = build_plan()
    predictor = PositionPredictor(plate, well_model, path_points, settings, plan)

    # Show predicted timeline
    print("\n  PREDICTED TIMELINE (from estimate_time logic):")
    print(f"  {'Time':>7s}  {'X':>8s}  {'Y':>8s}  {'Z':>6s}  Phase")
    for t_ev, x_ev, y_ev, z_ev, phase in predictor.events:
        print(f"  {t_ev:7.2f}s  {x_ev:8.2f}  {y_ev:8.2f}  {z_ev:6.2f}  {phase}")

    # Create executor
    executor = HybridPlanExecutor(
        controller=ctrl,
        plan=plan,
        well_model=well_model,
        plate=plate,
        path_points=path_points,
        settings=settings,
    )

    # Start position tracker
    tracker = PositionTracker(ctrl)
    tracker.start()

    # Execute
    t_start = time.monotonic()
    logger.info(">>> TEST 5: Starting hybrid execution with prediction tracking...")
    success = executor.execute(pause_event=None, on_progress=None)
    t_end = time.monotonic()
    actual_duration = t_end - t_start
    tracker.stop()

    # ── Compare predicted vs actual at each sample ──
    print("\n" + "-" * 100)
    print("POSITION PREDICTION ERROR LOG (sampled every 0.5s)")
    print("-" * 100)
    print(f"  {'Time':>7s}  {'Act_X':>8s} {'Act_Y':>8s} {'Act_Z':>6s}  "
          f"{'Prd_X':>8s} {'Prd_Y':>8s} {'Prd_Z':>6s}  "
          f"{'Err_XY':>7s} {'Err_Z':>6s}  Phase")

    error_log = []  # (t, err_xy, err_z, phase)
    last_print_t = -1
    for sample_t, ax, ay, az in tracker.samples:
        if sample_t - last_print_t < 0.5:
            continue
        last_print_t = sample_t

        px, py, pz, phase = predictor.predict(sample_t)
        err_xy = math.sqrt((ax - px)**2 + (ay - py)**2)
        err_z = abs(az - pz)
        error_log.append((sample_t, err_xy, err_z, phase))

        # Color-code errors
        xy_flag = " !" if err_xy > 5.0 else ("!!" if err_xy > 10.0 else "  ")
        z_flag = " !" if err_z > 1.0 else ("!!" if err_z > 2.0 else "  ")

        print(f"  {sample_t:7.2f}s  {ax:8.2f} {ay:8.2f} {az:6.2f}  "
              f"{px:8.2f} {py:8.2f} {pz:6.2f}  "
              f"{err_xy:6.2f}{xy_flag} {err_z:5.2f}{z_flag}  {phase}")

    # ── Error summary by phase ──
    print("\n" + "-" * 70)
    print("ERROR SUMMARY BY PHASE")
    print("-" * 70)

    phase_errors = {}  # phase -> [(t, err_xy, err_z)]
    for t_s, exy, ez, ph in error_log:
        # Group by phase prefix (before the colon)
        prefix = ph.split(":")[0] if ":" in ph else ph
        if prefix not in phase_errors:
            phase_errors[prefix] = []
        phase_errors[prefix].append((t_s, exy, ez))

    for prefix, entries in phase_errors.items():
        xy_errs = [e[1] for e in entries]
        z_errs = [e[2] for e in entries]
        avg_xy = sum(xy_errs) / len(xy_errs) if xy_errs else 0
        max_xy = max(xy_errs) if xy_errs else 0
        avg_z = sum(z_errs) / len(z_errs) if z_errs else 0
        max_z = max(z_errs) if z_errs else 0
        print(f"  {prefix:25s}  "
              f"XY avg={avg_xy:6.2f} max={max_xy:6.2f}  "
              f"Z  avg={avg_z:5.2f} max={max_z:5.2f}  "
              f"({len(entries)} samples)")

    # ── Cumulative time drift ──
    print("\n" + "-" * 70)
    print("TIME DRIFT ANALYSIS")
    print("-" * 70)

    pred_total = predictor.events[-1][0] if predictor.events else 0
    print(f"  Predicted total:  {pred_total:.1f}s")
    print(f"  Actual total:     {actual_duration:.1f}s")
    print(f"  Drift:            {actual_duration - pred_total:+.1f}s "
          f"({(actual_duration - pred_total) / max(pred_total, 0.1) * 100:+.1f}%)")

    # Find when position error first exceeds 5mm XY
    first_big_error = None
    for t_s, exy, ez, ph in error_log:
        if exy > 5.0 and first_big_error is None:
            first_big_error = (t_s, exy, ph)
            break

    if first_big_error:
        print(f"\n  First XY error > 5mm at t={first_big_error[0]:.1f}s "
              f"(err={first_big_error[1]:.1f}mm, phase={first_big_error[2]})")
        print(f"  This is where the estimate starts diverging significantly.")
    else:
        print(f"\n  XY error never exceeded 5mm — estimate tracks well!")

    return actual_duration, error_log


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    print("=" * 70)
    print("  HYBRID EXECUTION DIAGNOSTIC TEST")
    print("  Plate: 24-well  |  Print: A1  |  Ink: A2")
    print("  Buffer: A3  |  Wash: B2  |  Waste: B3")
    print("=" * 70)

    plate, well_model, path_points, settings, ctrl = setup()

    # Test 4 first — inspect trajectory without executing
    traj_result = test_inwell_trajectory(plate, path_points, settings)

    # Test 3 — Z sequencing
    test_z_sequencing(ctrl, settings, plate)

    # Reset controller position for full test
    if ctrl.is_zp_connected:
        ctrl.move_z_absolute(settings.travel_z_height, from_zero_ref=True)
        time.sleep(0.5)
    if ctrl.is_xy_connected:
        ctrl.move_xy_absolute(0, 0, from_zero_ref=True, fast=False)
        time.sleep(1.0)

    # Test 1 — time estimation
    estimated = test_estimate_time(plate, well_model, path_points, settings, ctrl)

    # Test 2 — full execution
    actual, samples = test_full_execution(
        plate, well_model, path_points, settings, ctrl, estimated)

    # Reset controller for test 5
    if ctrl.is_zp_connected:
        ctrl.move_z_absolute(settings.travel_z_height, from_zero_ref=True)
        time.sleep(0.5)
    if ctrl.is_xy_connected:
        ctrl.move_xy_absolute(0, 0, from_zero_ref=True, fast=False)
        time.sleep(1.0)

    # Test 5 — position prediction vs actual (the key diagnostic)
    actual5, error_log = test_position_prediction(
        plate, well_model, path_points, settings, ctrl)

    # ── Final summary ──
    print("\n" + "=" * 70)
    print("  SUMMARY")
    print("=" * 70)
    print(f"  Estimated time:    {estimated:.1f}s")
    print(f"  Actual (test 2):   {actual:.1f}s")
    print(f"  Actual (test 5):   {actual5:.1f}s")
    if estimated > 0:
        print(f"  Error (test 2):    {abs(actual-estimated)/estimated*100:.1f}%")
        print(f"  Error (test 5):    {abs(actual5-estimated)/estimated*100:.1f}%")
    print(f"  Position samples:  {len(samples)}")
    print(f"  Error log entries: {len(error_log)}")

    # Error statistics from test 5
    if error_log:
        xy_errs = [e[1] for e in error_log]
        z_errs = [e[2] for e in error_log]
        print(f"\n  XY error:  avg={sum(xy_errs)/len(xy_errs):.2f}mm  "
              f"max={max(xy_errs):.2f}mm")
        print(f"  Z  error:  avg={sum(z_errs)/len(z_errs):.2f}mm  "
              f"max={max(z_errs):.2f}mm")

    # Check for Z anomalies
    z_at_low = [(t, z) for t, x, y, z in samples if z < 0]
    if z_at_low:
        print(f"  ✗ Z went BELOW 0: {len(z_at_low)} samples (min Z={min(z for _, z in z_at_low):.2f})")
    else:
        print(f"  ✓ Z never went below 0")

    print("=" * 70)


if __name__ == "__main__":
    main()
