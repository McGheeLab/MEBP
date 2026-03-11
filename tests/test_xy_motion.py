#!/usr/bin/env python3
"""
Systematic test of XY stage motion methods in simulation mode.

Tests from lowest level (raw simulator) up through StageController,
trying different approaches to commanding moves and verifying positions.

Run: python3 tests/test_xy_motion.py
"""

import sys
import os
import time
import math
import threading

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Suppress Qt imports if any
os.environ["QT_QPA_PLATFORM"] = "offscreen"

import logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("test_xy")


def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def report(name, passed, detail=""):
    status = "PASS" if passed else "FAIL"
    print(f"  [{status}] {name}" + (f" — {detail}" if detail else ""))
    return passed


# ═══════════════════════════════════════════════════════════════════
# TEST 1: Raw XYStageSimulator — direct API
# ═══════════════════════════════════════════════════════════════════

def test_1_raw_simulator():
    """Test the simulator directly, bypassing all wrapper layers."""
    print("\n" + "="*70)
    print("TEST 1: Raw XYStageSimulator (direct API)")
    print("="*70)

    from SupportClasses.XYStageSimulator import XYStageSimulator

    sim = XYStageSimulator()
    sim.start()
    time.sleep(0.1)  # let physics loop start

    results = []

    # 1a: Check initial position
    pos = sim.get_current_position()
    results.append(report(
        "Initial position at origin",
        abs(pos[0]) < 1.0 and abs(pos[1]) < 1.0,
        f"pos=({pos[0]:.1f}, {pos[1]:.1f})"
    ))

    # 1b: send_command("G ...") — should block until arrival
    target_x, target_y = 10000.0, 5000.0  # 10mm, 5mm in µm
    t0 = time.monotonic()
    response = sim.send_command(f"G {target_x:.0f},{target_y:.0f}")
    elapsed = time.monotonic() - t0
    pos = sim.get_current_position()
    dist = distance(pos[0], pos[1], target_x, target_y)
    results.append(report(
        "send_command('G x,y') blocks until arrival",
        dist < 1.0,  # within 1 µm
        f"target=({target_x:.0f},{target_y:.0f}), "
        f"actual=({pos[0]:.1f},{pos[1]:.1f}), "
        f"dist={dist:.2f}µm, elapsed={elapsed:.3f}s, resp='{response}'"
    ))

    # 1c: Check mode is idle after move
    with sim._lock:
        mode = sim.mode
    results.append(report(
        "Mode is 'idle' after blocking move",
        mode == "idle",
        f"mode='{mode}'"
    ))

    # 1d: Second move to different position
    target_x2, target_y2 = 30000.0, 20000.0
    t0 = time.monotonic()
    sim.send_command(f"G {target_x2:.0f},{target_y2:.0f}")
    elapsed2 = time.monotonic() - t0
    pos2 = sim.get_current_position()
    dist2 = distance(pos2[0], pos2[1], target_x2, target_y2)
    results.append(report(
        "Second move also blocks correctly",
        dist2 < 1.0,
        f"target=({target_x2:.0f},{target_y2:.0f}), "
        f"actual=({pos2[0]:.1f},{pos2[1]:.1f}), "
        f"dist={dist2:.2f}µm, elapsed={elapsed2:.3f}s"
    ))

    # 1e: Position query via send_command("P")
    p_response = sim.send_command("P")
    results.append(report(
        "Position query 'P' returns correct format",
        p_response is not None and "," in p_response,
        f"P response: '{p_response}'"
    ))

    sim.stop()
    return results


# ═══════════════════════════════════════════════════════════════════
# TEST 2: XYStageManager wrapper (simulate=True)
# ═══════════════════════════════════════════════════════════════════

def test_2_xy_stage_manager():
    """Test XYStageManager which wraps the simulator."""
    print("\n" + "="*70)
    print("TEST 2: XYStageManager (simulate=True)")
    print("="*70)

    from SupportClasses.XYStage import XYStageManager

    stage = XYStageManager(simulate=True)
    time.sleep(0.1)

    results = []

    # 2a: get_current_position returns reasonable initial value
    pos = stage.get_current_position()
    results.append(report(
        "Initial position readable",
        pos[0] is not None and pos[1] is not None,
        f"pos={pos}"
    ))

    # 2b: send_command("G ...") through manager
    target_x, target_y = 15000.0, 8000.0
    t0 = time.monotonic()
    resp = stage.send_command(f"G {round(target_x)},{round(target_y)}")
    elapsed = time.monotonic() - t0
    pos = stage.get_current_position()
    dist = distance(pos[0], pos[1], target_x, target_y)
    results.append(report(
        "send_command('G x,y') via manager",
        dist < 5.0,
        f"dist={dist:.2f}µm, elapsed={elapsed:.3f}s, pos=({pos[0]:.1f},{pos[1]:.1f})"
    ))

    # 2c: move_stage_to_position
    target_x2, target_y2 = 25000.0, 12000.0
    t0 = time.monotonic()
    stage.move_stage_to_position(target_x2, target_y2)
    elapsed2 = time.monotonic() - t0
    pos2 = stage.get_current_position()
    dist2 = distance(pos2[0], pos2[1], target_x2, target_y2)
    results.append(report(
        "move_stage_to_position blocks correctly",
        dist2 < 5.0,
        f"dist={dist2:.2f}µm, elapsed={elapsed2:.3f}s, pos=({pos2[0]:.1f},{pos2[1]:.1f})"
    ))

    # 2d: _send_protocol_command with fallback (no protocol loaded)
    target_x3, target_y3 = 5000.0, 3000.0
    has_protocol = stage._protocol is not None
    t0 = time.monotonic()
    stage._send_protocol_command(
        "move_absolute",
        fallback_cmd=f"G {round(target_x3)},{round(target_y3)}",
        x=round(target_x3), y=round(target_y3),
    )
    elapsed3 = time.monotonic() - t0
    pos3 = stage.get_current_position()
    dist3 = distance(pos3[0], pos3[1], target_x3, target_y3)
    results.append(report(
        f"_send_protocol_command (protocol={'loaded' if has_protocol else 'None'})",
        dist3 < 5.0,
        f"dist={dist3:.2f}µm, elapsed={elapsed3:.3f}s"
    ))

    stage.stop()
    return results


# ═══════════════════════════════════════════════════════════════════
# TEST 3: StageController (full stack)
# ═══════════════════════════════════════════════════════════════════

def test_3_stage_controller():
    """Test through full StageController stack."""
    print("\n" + "="*70)
    print("TEST 3: StageController (full stack, simulate)")
    print("="*70)

    from SupportClasses.StageController import StageController

    ctrl = StageController(simulate_xy=True, simulate_zp=True)
    ctrl.connect_stages(xy=True, zp=True)
    time.sleep(0.5)  # let poller + physics settle

    # Set zero position (simulate calibration)
    ctrl._calibrate_zero()
    time.sleep(0.2)

    results = []

    # 3a: Position reading — cached vs direct
    pos_cached = ctrl.get_xy_position(cached=True)
    pos_direct = ctrl.get_xy_position(cached=False)
    results.append(report(
        "Cached position readable",
        pos_cached[0] is not None,
        f"cached={pos_cached}"
    ))
    results.append(report(
        "Direct position readable",
        pos_direct[0] is not None,
        f"direct={pos_direct}"
    ))

    # 3b: mm conversion
    pos_mm = ctrl.get_xy_position_mm(cached=False)
    results.append(report(
        "Position in mm near zero after calibration",
        pos_mm[0] is not None and abs(pos_mm[0]) < 0.1 and abs(pos_mm[1]) < 0.1,
        f"mm={pos_mm}"
    ))

    # 3c: move_xy_absolute (from_zero_ref=True, values in mm)
    target_mm = (20.0, 10.0)
    t0 = time.monotonic()
    ctrl.move_xy_absolute(target_mm[0], target_mm[1], from_zero_ref=True, fast=False)
    elapsed = time.monotonic() - t0

    # Check position immediately after (should be at target since sim blocks)
    pos_after = ctrl.get_xy_position_mm(cached=False)
    if pos_after[0] is not None:
        dist_mm = distance(pos_after[0], pos_after[1], target_mm[0], target_mm[1])
    else:
        dist_mm = 999
    results.append(report(
        "move_xy_absolute(20, 10) mm — position correct after",
        dist_mm < 0.5,
        f"target={target_mm}, actual={pos_after}, dist={dist_mm:.3f}mm, elapsed={elapsed:.3f}s"
    ))

    # 3d: wait_for_xy_arrival (should pass immediately since sim already moved)
    t0 = time.monotonic()
    arrived = ctrl.wait_for_xy_arrival(target_mm[0], target_mm[1],
                                        tolerance_mm=0.5, timeout_s=5.0)
    wait_elapsed = time.monotonic() - t0
    results.append(report(
        "wait_for_xy_arrival returns True immediately",
        arrived and wait_elapsed < 1.0,
        f"arrived={arrived}, wait_time={wait_elapsed:.3f}s"
    ))

    # 3e: Move to second position, then check
    target2_mm = (38.6, 19.3)  # typical well position
    ctrl.move_xy_absolute(target2_mm[0], target2_mm[1], from_zero_ref=True)
    pos2 = ctrl.get_xy_position_mm(cached=False)
    if pos2[0] is not None:
        dist2 = distance(pos2[0], pos2[1], target2_mm[0], target2_mm[1])
    else:
        dist2 = 999
    arrived2 = ctrl.wait_for_xy_arrival(target2_mm[0], target2_mm[1],
                                         tolerance_mm=0.5, timeout_s=5.0)
    results.append(report(
        "Move to well position (38.6, 19.3) mm",
        dist2 < 0.5 and arrived2,
        f"dist={dist2:.3f}mm, arrived={arrived2}, pos={pos2}"
    ))

    # 3f: Cached position eventually updates
    time.sleep(0.5)  # wait for poller cycle
    pos_cached_after = ctrl.get_xy_position_mm(cached=True)
    if pos_cached_after[0] is not None:
        dist_cached = distance(pos_cached_after[0], pos_cached_after[1],
                               target2_mm[0], target2_mm[1])
    else:
        dist_cached = 999
    results.append(report(
        "Cached position updates within 0.5s",
        dist_cached < 1.0,
        f"cached_mm={pos_cached_after}, dist={dist_cached:.3f}mm"
    ))

    ctrl.disconnect_stages()
    return results


# ═══════════════════════════════════════════════════════════════════
# TEST 4: Blocking move + wait pattern (DirectCommandExecutor style)
# ═══════════════════════════════════════════════════════════════════

def test_4_blocking_move_pattern():
    """Test the exact move+wait pattern used by DirectCommandExecutor."""
    print("\n" + "="*70)
    print("TEST 4: Blocking move + wait pattern (DirectCommandExecutor)")
    print("="*70)

    from SupportClasses.StageController import StageController

    ctrl = StageController(simulate_xy=True, simulate_zp=True)
    ctrl.connect_stages(xy=True, zp=True)
    time.sleep(0.5)
    ctrl._calibrate_zero()
    time.sleep(0.2)

    results = []

    # 4a: Pattern A — move_xy_absolute then wait_for_xy_arrival
    target = (38.6, 19.3)
    t0 = time.monotonic()
    ctrl.move_xy_absolute(target[0], target[1], from_zero_ref=True, fast=False)
    arrived = ctrl.wait_for_xy_arrival(target[0], target[1],
                                        tolerance_mm=0.5, timeout_s=5.0)
    elapsed = time.monotonic() - t0
    pos = ctrl.get_xy_position_mm(cached=False)
    results.append(report(
        "Pattern A: move_xy_absolute + wait_for_xy_arrival",
        arrived,
        f"arrived={arrived}, pos={pos}, elapsed={elapsed:.3f}s"
    ))

    # 4b: Pattern B — non-blocking move (bypass simulator blocking)
    #     Send raw command without blocking, then poll
    target2 = (10.0, 25.0)
    target2_um = (target2[0] * 1000, target2[1] * 1000)
    zero_x = ctrl.zero_position.get("x", 0)
    zero_y = ctrl.zero_position.get("y", 0)
    abs_x = target2_um[0] + zero_x
    abs_y = target2_um[1] + zero_y

    # Use the simulator's _send_command_raw (non-blocking) directly
    sim = ctrl.xy_stage.spo
    t0 = time.monotonic()
    sim._send_command_raw(f"G {round(abs_x)},{round(abs_y)}")
    # Don't call _wait_for_idle — poll from outside
    arrived2 = False
    for _ in range(100):  # poll for up to 5s
        pos_check = ctrl.get_xy_position_mm(cached=False)
        if pos_check[0] is not None:
            d = distance(pos_check[0], pos_check[1], target2[0], target2[1])
            if d < 0.5:
                arrived2 = True
                break
        time.sleep(0.05)
    elapsed2 = time.monotonic() - t0
    pos2 = ctrl.get_xy_position_mm(cached=False)
    results.append(report(
        "Pattern B: non-blocking raw cmd + poll position",
        arrived2,
        f"arrived={arrived2}, pos={pos2}, elapsed={elapsed2:.3f}s"
    ))

    # 4c: Z move + XY move sequence (the actual travel_to_well pattern)
    target3 = (30.0, 15.0)
    safe_z = 5.0  # mm

    # Z up
    ctrl.move_z_absolute(safe_z, from_zero_ref=True)
    z_arrived = ctrl.wait_for_z_arrival(safe_z, tolerance_mm=0.1, timeout_s=5.0)

    # XY travel
    ctrl.move_xy_absolute(target3[0], target3[1], from_zero_ref=True, fast=False)
    xy_arrived = ctrl.wait_for_xy_arrival(target3[0], target3[1],
                                           tolerance_mm=0.5, timeout_s=10.0)

    # Z down
    ctrl.move_z_absolute(1.0, from_zero_ref=True)
    z_down_arrived = ctrl.wait_for_z_arrival(1.0, tolerance_mm=0.1, timeout_s=5.0)

    pos3 = ctrl.get_xy_position_mm(cached=False)
    results.append(report(
        "Pattern C: Z-up → XY-move → Z-down sequence",
        z_arrived and xy_arrived and z_down_arrived,
        f"z_up={z_arrived}, xy={xy_arrived}, z_down={z_down_arrived}, "
        f"xy_pos={pos3}"
    ))

    ctrl.disconnect_stages()
    return results


# ═══════════════════════════════════════════════════════════════════
# TEST 5: Threading — moves from background thread
# ═══════════════════════════════════════════════════════════════════

def test_5_threaded_moves():
    """Test moves from a background thread (like HybridPlanExecutor)."""
    print("\n" + "="*70)
    print("TEST 5: Moves from background thread")
    print("="*70)

    from SupportClasses.StageController import StageController

    ctrl = StageController(simulate_xy=True, simulate_zp=True)
    ctrl.connect_stages(xy=True, zp=True)
    time.sleep(0.5)
    ctrl._calibrate_zero()
    time.sleep(0.2)

    results = []
    thread_results = {}

    def worker_move_and_check(ctrl, target_mm, result_key):
        """Simulates what DirectCommandExecutor.move_xy does."""
        try:
            ctrl.move_xy_absolute(target_mm[0], target_mm[1],
                                  from_zero_ref=True, fast=False)
            pos = ctrl.get_xy_position_mm(cached=False)
            arrived = ctrl.wait_for_xy_arrival(
                target_mm[0], target_mm[1],
                tolerance_mm=0.5, timeout_s=10.0)
            thread_results[result_key] = {
                "arrived": arrived,
                "pos": pos,
                "target": target_mm,
            }
        except Exception as e:
            thread_results[result_key] = {"error": str(e)}

    # 5a: Single move from background thread
    target = (25.0, 15.0)
    t = threading.Thread(target=worker_move_and_check,
                         args=(ctrl, target, "single"))
    t0 = time.monotonic()
    t.start()
    t.join(timeout=15.0)
    elapsed = time.monotonic() - t0

    r = thread_results.get("single", {})
    if "error" in r:
        results.append(report("Single move from thread", False, f"error: {r['error']}"))
    else:
        results.append(report(
            "Single move from background thread",
            r.get("arrived", False),
            f"arrived={r.get('arrived')}, pos={r.get('pos')}, elapsed={elapsed:.3f}s"
        ))

    # 5b: Sequential moves from background thread
    def worker_sequential(ctrl):
        """Multiple sequential moves like HybridPlanExecutor would do."""
        moves = [
            (38.6, 19.3),   # waste well
            (38.6, 0.0),    # wash well
            (20.0, 10.0),   # ink well
            (10.0, 5.0),    # print well
        ]
        results_list = []
        for i, target in enumerate(moves):
            t0 = time.monotonic()
            ctrl.move_xy_absolute(target[0], target[1],
                                  from_zero_ref=True, fast=False)
            arrived = ctrl.wait_for_xy_arrival(
                target[0], target[1],
                tolerance_mm=0.5, timeout_s=10.0)
            elapsed = time.monotonic() - t0
            pos = ctrl.get_xy_position_mm(cached=False)
            results_list.append({
                "target": target, "arrived": arrived,
                "pos": pos, "elapsed": elapsed,
            })
        thread_results["sequential"] = results_list

    t2 = threading.Thread(target=worker_sequential, args=(ctrl,))
    t0 = time.monotonic()
    t2.start()
    t2.join(timeout=60.0)
    total_elapsed = time.monotonic() - t0

    seq = thread_results.get("sequential", [])
    all_arrived = all(r.get("arrived", False) for r in seq)
    details = "; ".join(
        f"#{i}: {'OK' if r['arrived'] else 'FAIL'} "
        f"({r['elapsed']:.2f}s) pos={r['pos']}"
        for i, r in enumerate(seq)
    )
    results.append(report(
        f"Sequential {len(seq)} moves from thread",
        all_arrived and len(seq) == 4,
        f"total={total_elapsed:.2f}s — {details}"
    ))

    ctrl.disconnect_stages()
    return results


# ═══════════════════════════════════════════════════════════════════
# TEST 6: Z + XY interleaved from thread (full travel_to_well pattern)
# ═══════════════════════════════════════════════════════════════════

def test_6_full_travel_pattern():
    """Test the complete travel_to_well pattern from a background thread."""
    print("\n" + "="*70)
    print("TEST 6: Full travel_to_well pattern from thread")
    print("="*70)

    from SupportClasses.StageController import StageController

    ctrl = StageController(simulate_xy=True, simulate_zp=True)
    ctrl.connect_stages(xy=True, zp=True)
    time.sleep(0.5)
    ctrl._calibrate_zero()
    time.sleep(0.2)

    results = []
    thread_results = {}

    def travel_to_well(ctrl, wx, wy, safe_z, print_z, result_key):
        """Replicate DirectCommandExecutor.travel_to_well exactly."""
        try:
            steps = {}

            # Phase 1: Z up to safe height
            ctrl.move_z_absolute(safe_z, from_zero_ref=True)
            z_up = ctrl.wait_for_z_arrival(safe_z, tolerance_mm=0.1, timeout_s=10.0)
            steps["z_up"] = z_up

            # Phase 2: XY move
            ctrl.move_xy_absolute(wx, wy, from_zero_ref=True, fast=False)
            xy_pos_after_cmd = ctrl.get_xy_position_mm(cached=False)
            xy_arrived = ctrl.wait_for_xy_arrival(
                wx, wy, tolerance_mm=0.5, timeout_s=20.0)
            steps["xy_arrived"] = xy_arrived
            steps["xy_pos_after_cmd"] = xy_pos_after_cmd

            # Phase 3: Z down to print height
            ctrl.move_z_absolute(print_z, from_zero_ref=True)
            z_down = ctrl.wait_for_z_arrival(print_z, tolerance_mm=0.1, timeout_s=10.0)
            steps["z_down"] = z_down

            # Final positions
            steps["final_xy"] = ctrl.get_xy_position_mm(cached=False)
            steps["final_z"] = ctrl.get_zp_position(cached=False)

            thread_results[result_key] = steps
        except Exception as e:
            thread_results[result_key] = {"error": str(e)}

    # 6a: Travel to well A1
    t = threading.Thread(
        target=travel_to_well,
        args=(ctrl, 38.6, 19.3, 5.0, 1.0, "well_A1"))
    t0 = time.monotonic()
    t.start()
    t.join(timeout=30.0)
    elapsed = time.monotonic() - t0

    r = thread_results.get("well_A1", {})
    if "error" in r:
        results.append(report("Travel to well A1", False, f"error: {r['error']}"))
    else:
        all_ok = r.get("z_up") and r.get("xy_arrived") and r.get("z_down")
        results.append(report(
            "Travel to well A1 (38.6, 19.3)",
            all_ok,
            f"z_up={r.get('z_up')}, xy={r.get('xy_arrived')}, "
            f"z_down={r.get('z_down')}, "
            f"xy_after_cmd={r.get('xy_pos_after_cmd')}, "
            f"final_xy={r.get('final_xy')}, "
            f"elapsed={elapsed:.2f}s"
        ))

    # 6b: Travel to well B1 (second well)
    t2 = threading.Thread(
        target=travel_to_well,
        args=(ctrl, 38.6, 0.0, 5.0, 1.0, "well_B1"))
    t0 = time.monotonic()
    t2.start()
    t2.join(timeout=30.0)
    elapsed2 = time.monotonic() - t0

    r2 = thread_results.get("well_B1", {})
    if "error" in r2:
        results.append(report("Travel to well B1", False, f"error: {r2['error']}"))
    else:
        all_ok2 = r2.get("z_up") and r2.get("xy_arrived") and r2.get("z_down")
        results.append(report(
            "Travel to well B1 (38.6, 0.0)",
            all_ok2,
            f"z_up={r2.get('z_up')}, xy={r2.get('xy_arrived')}, "
            f"z_down={r2.get('z_down')}, "
            f"xy_after_cmd={r2.get('xy_pos_after_cmd')}, "
            f"final_xy={r2.get('final_xy')}, "
            f"elapsed={elapsed2:.2f}s"
        ))

    ctrl.disconnect_stages()
    return results


# ═══════════════════════════════════════════════════════════════════
# TEST 7: Alternative approach — non-blocking command + poll
# ═══════════════════════════════════════════════════════════════════

def test_7_nonblocking_approach():
    """Test non-blocking command approach as alternative."""
    print("\n" + "="*70)
    print("TEST 7: Non-blocking move + external position polling")
    print("="*70)

    from SupportClasses.StageController import StageController

    ctrl = StageController(simulate_xy=True, simulate_zp=True)
    ctrl.connect_stages(xy=True, zp=True)
    time.sleep(0.5)
    ctrl._calibrate_zero()
    time.sleep(0.2)

    results = []

    # 7a: Bypass blocking — set target directly on simulator
    sim = ctrl.xy_stage.spo
    target_mm = (25.0, 15.0)
    target_um = (target_mm[0] * 1000 + ctrl.zero_position["x"],
                 target_mm[1] * 1000 + ctrl.zero_position["y"])

    with sim._lock:
        sim.target_x = target_um[0]
        sim.target_y = target_um[1]
        sim.mode = "absolute"

    # Poll until arrival
    t0 = time.monotonic()
    arrived = False
    for _ in range(200):  # 10s max
        pos = ctrl.get_xy_position_mm(cached=False)
        if pos[0] is not None:
            d = distance(pos[0], pos[1], target_mm[0], target_mm[1])
            if d < 0.5:
                arrived = True
                break
        time.sleep(0.05)
    elapsed = time.monotonic() - t0
    pos_final = ctrl.get_xy_position_mm(cached=False)
    results.append(report(
        "Direct simulator target set + poll",
        arrived,
        f"pos={pos_final}, elapsed={elapsed:.3f}s"
    ))

    # 7b: Use _send_command_raw (non-blocking) + poll
    target_mm2 = (10.0, 30.0)
    target_um2 = (target_mm2[0] * 1000 + ctrl.zero_position["x"],
                  target_mm2[1] * 1000 + ctrl.zero_position["y"])

    t0 = time.monotonic()
    sim._send_command_raw(f"G {round(target_um2[0])},{round(target_um2[1])}")

    arrived2 = False
    for _ in range(200):
        pos = ctrl.get_xy_position_mm(cached=False)
        if pos[0] is not None:
            d = distance(pos[0], pos[1], target_mm2[0], target_mm2[1])
            if d < 0.5:
                arrived2 = True
                break
        time.sleep(0.05)
    elapsed2 = time.monotonic() - t0
    pos_final2 = ctrl.get_xy_position_mm(cached=False)
    results.append(report(
        "Non-blocking _send_command_raw + poll",
        arrived2,
        f"pos={pos_final2}, elapsed={elapsed2:.3f}s"
    ))

    ctrl.disconnect_stages()
    return results


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    print("\n" + "#"*70)
    print("# XY Stage Motion — Systematic Test Suite")
    print("#"*70)

    all_results = []

    tests = [
        ("1: Raw Simulator", test_1_raw_simulator),
        ("2: XYStageManager", test_2_xy_stage_manager),
        ("3: StageController", test_3_stage_controller),
        ("4: Blocking Move Patterns", test_4_blocking_move_pattern),
        ("5: Threaded Moves", test_5_threaded_moves),
        ("6: Full Travel Pattern", test_6_full_travel_pattern),
        ("7: Non-blocking Approach", test_7_nonblocking_approach),
    ]

    for name, test_fn in tests:
        try:
            results = test_fn()
            all_results.extend(results)
        except Exception as e:
            print(f"\n  [ERROR] Test {name} crashed: {e}")
            import traceback
            traceback.print_exc()
            all_results.append(False)

    # Summary
    passed = sum(1 for r in all_results if r)
    failed = sum(1 for r in all_results if not r)
    total = len(all_results)

    print("\n" + "="*70)
    print(f"SUMMARY: {passed}/{total} passed, {failed} failed")
    print("="*70)

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
