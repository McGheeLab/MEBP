"""
selftest.py — end-to-end verification against the simulated board.

Run from the repo root::

    python tools/incubator/selftest.py            # logic only
    python tools/incubator/selftest.py --gui      # also build the window offscreen

Exercises the real production code path (the same controller/link/parser the GUI
uses) against :class:`~.fake_marlin.FakeMarlinLink`, including the paths that are
awkward or dangerous to reach on hardware: bang-bang firmware, PID-autotune
failure modes, thermal-runaway faults, a detached thermistor, and the
out-of-band emergency path.

Exit code 0 = all checks passed.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path


def _bootstrap() -> Path:
    here = Path(__file__).resolve().parent
    for cand in (here, here.parent, here.parent.parent, Path.cwd()):
        if (cand / "SupportClasses").is_dir():
            if str(cand) not in sys.path:
                sys.path.insert(0, str(cand))
            return cand
    return here.parent.parent


_ROOT = _bootstrap()

from tools.incubator.calibration import CalibrationStore  # noqa: E402
from tools.incubator.controller import IncubatorController  # noqa: E402
from tools.incubator.fake_marlin import FakeCapabilities  # noqa: E402
from tools.incubator.marlin_gcode import (  # noqa: E402
    classify_line, parse_autotune_failure, parse_autotune_result,
    parse_pid_dump, parse_temp_line,
)
from tools.incubator.sensors import StaticSensorSource  # noqa: E402
from tools.incubator.stability import StabilityTracker  # noqa: E402

PASS = 0
FAIL = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global PASS, FAIL
    if ok:
        PASS += 1
        print(f"  [PASS] {name}")
    else:
        FAIL += 1
        print(f"  [FAIL] {name}" + (f"  -- {detail}" if detail else ""))


def section(title: str) -> None:
    print(f"\n=== {title} ===")


def wait_until(pred, timeout_s: float = 20.0, period: float = 0.1) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(period)
    return False


def _make(**kw) -> IncubatorController:
    """Controller wired to a throwaway simulated board (no files touched)."""
    tmp_cal = Path(_ROOT) / "tools" / "incubator" / "_data" / "_selftest_cal.json"
    c = IncubatorController(calibration_path=tmp_cal)
    sim_kwargs = {"state_file": None}
    sim_kwargs.update(kw.pop("sim_kwargs", {}))
    ok = c.connect(simulate=True, sim_time_scale=kw.pop("scale", 900.0),
                   sim_kwargs=sim_kwargs)
    if not ok:
        raise RuntimeError("simulated connect failed")
    return c


# ═══════════════════════════════════════════════════════════════════

def test_parser() -> None:
    section("1. Generic temperature parser")

    f = parse_temp_line("ok T:24.31 /0.00 B:36.90 /37.00 @:0 B@:114")
    check("M105 reply parses", f is not None and f.temp("B") == 36.90)
    check("target parsed from split token", f.target("B") == 37.00)

    # Duty scale. Marlin's soft-PWM period is 127 ticks and every writer stores
    # `control_value >> 1`, so 127 — not 255 — is full power. Getting this wrong
    # halves every duty figure, and duty at hold is the number that says whether
    # the heater is big enough, so an understated value invites exactly the wrong
    # conclusion ("plenty of headroom" when it is flat out).
    check("full-scale duty (B@:127) reads as 100 %",
          abs(parse_temp_line("ok B:36.9 /37.0 B@:127").power_pct("B@") - 100.0)
          < 0.01,
          str(parse_temp_line("ok B:36.9 /37.0 B@:127").power_pct("B@")))
    check("duty converted to percent on the 127 scale",
          abs(f.power_pct("B@") - 89.76) < 0.1, str(f.power_pct("B@")))
    check("zero duty reads as 0 %", f.power_pct("@") == 0.0)
    check("an out-of-range duty is clamped, not reported above 100 %",
          parse_temp_line("ok B:36.9 /37.0 B@:255").power_pct("B@") == 100.0)

    f2 = parse_temp_line("T:24.31 /0.00 B:36.90 /37.00 @:0 B@:114")
    check("bare autoreport parses identically",
          f2 is not None and f2.fields.keys() == f.fields.keys())

    f3 = parse_temp_line("ok B:36.90 /37.00 B@:114")
    check("hotend-absent frame parses (no T:)",
          f3 is not None and f3.temp("T") is None and f3.temp("B") == 36.90)

    f4 = parse_temp_line("ok T:24.31/0.00 B:36.9/37.0 C:30.1 /0.0 R:36.85")
    check("unexpected extra sensors captured generically",
          f4 is not None and set(f4.temperature_keys()) == {"T", "B", "C", "R"})

    f5 = parse_temp_line(" T:24.5 E:0 W:?")
    check("M109 wait tokens E:/W: do not become phantom channels",
          f5 is not None and f5.temperature_keys() == ["T"])

    check("plain 'ok' is not a temperature line", parse_temp_line("ok") is None)

    dump = ["echo:  M301 P24.50 I1.07 D139.83",
            "echo:  M304 P97.10 I1.41 D1675.16", "ok"]
    check("bed PID read from M503", parse_pid_dump(dump, "M304").kp == 97.10)
    check("PID absent returns None, NOT zeros",
          parse_pid_dump(["echo:  M301 P1 I2 D3"], "M304") is None)

    # Marlin renamed the autotune result macros between 2.0.x and 2.1.x. Both
    # must parse, or results vanish the moment the board is reflashed.
    check("autotune result, Marlin 2.0.x naming",
          parse_autotune_result("echo: #define DEFAULT_bedKp 97.10")
          == ("bed", "p", 97.10))
    check("autotune result, Marlin 2.1.x naming",
          parse_autotune_result("#define DEFAULT_BED_KP 41.78")
          == ("bed", "p", 41.78))
    check("autotune result, 2.1.x hotend naming",
          parse_autotune_result("#define DEFAULT_KP 21.73")
          == ("hotend", "p", 21.73))
    check("autotune result lines classify as results",
          classify_line("#define DEFAULT_BED_KD 158.93").name
          == "AUTOTUNE_RESULT")
    # Likewise the failure text differs between versions.
    check("autotune failure, 2.0.x wording",
          parse_autotune_failure("PID Autotune failed! Bad extruder number")
          == "Bad extruder number")
    check("autotune failure, 2.1.x wording",
          parse_autotune_failure("PID Autotune failed! Bad heater id")
          == "Bad heater id")


def test_probe_and_zones() -> None:
    section("2. Connect, probe, and independent two-zone control")
    c = _make(scale=900.0)
    try:
        r = c.report
        check("firmware identified", bool(r.firmware_name), str(r.firmware_name))
        check("both sensor fields discovered",
              set(r.sensor_fields) >= {"B", "T"}, str(r.sensor_fields))
        check("bed PID detected", r.zone("bed").pid_available)
        check("hotend PID detected", r.zone("hotend").pid_available)
        check("integer setpoint quantisation detected",
              r.integer_setpoints, f"res={r.setpoint_resolution_c}")
        check("capabilities parsed", r.capabilities.get("EEPROM") is True)
        check("no probe errors", not r.errors, str(r.errors))

        check("autoreport produced channels",
              wait_until(lambda: len(c.hub.all_channels()) >= 2, 10.0))

        c.set_target("bed", 37.0)
        c.set_target("hotend", 37.0)

        def _near(key: str, want: float, tol: float) -> bool:
            ch = c.hub.marlin_channel(key)
            return ch is not None and abs(ch.value_c - want) < tol

        ok_bed = wait_until(lambda: _near("B", 37.0, 0.6), 60.0)
        check("Zone A (bed) converged on 37 C", ok_bed,
              f"bed={c.hub.marlin_channel('B').value_c:.2f}")

        ok_hot = wait_until(lambda: _near("T", 37.0, 0.8), 60.0)
        check("Zone B (hotend) converged on 37 C independently", ok_hot,
              f"hotend={c.hub.marlin_channel('T').value_c:.2f}")

        rep = c.zone_runtime("bed").tracker.report()
        check("stability tracker reports a rate", rep.rate_c_per_min is not None)

        # Steady-state duty only becomes meaningful once the zone is actually
        # holding and a few in-band samples have accumulated, so wait for it
        # rather than sampling the instant convergence is first detected.
        got_duty = wait_until(
            lambda: c.zone_runtime("bed").tracker.report().steady_duty_pct
            is not None, 60.0)
        check("stability tracker reports steady-state duty once holding", got_duty,
              f"duty={c.zone_runtime('bed').tracker.report().steady_duty_pct}")

        c.heater_off("bed")
        c.heater_off("hotend")
        check("heater off clears the target",
              wait_until(lambda: c.zone_runtime("bed").requested_c == 0.0, 5.0))
    finally:
        c.disconnect()


def test_setpoint_translation() -> None:
    section("3. Setpoint quantisation, calibration and ceiling")
    c = _make(scale=900.0)
    try:
        plan = c.preview_setpoint("bed", 37.0)
        check("37.0 commands S37", plan["commanded_c"] == 37)

        plan = c.preview_setpoint("bed", 37.4)
        check("37.4 is quantised to a whole degree",
              plan["commanded_c"] in (37, 38) and
              abs(plan["quantisation_error_c"]) > 0.05,
              str(plan))

        chk = c.set_target("bed", 300.0)
        check("setpoint clamped to the safety ceiling",
              chk.allowed_c == c.MAX_SETPOINT_C and chk.clamped)
        check("high setpoint asks for confirmation", chk.needs_confirm)
        c.heater_off("bed")

        # Calibration: pretend the reference says 1.5 C higher than the board.
        ch = c.hub.marlin_channel("B")
        ok = c.calibrate_single_point("bed", ch.raw_c + 1.5)
        check("single-point calibration applied", ok)
        cal = c.calibration.get(ch.uid)
        check("offset is ~+1.5 C", abs(cal.offset_c - 1.5) < 0.25,
              f"offset={cal.offset_c:.3f}")

        plan = c.preview_setpoint("bed", 37.0)
        check("calibration inverts the setpoint (asks the board for less)",
              plan["commanded_c"] < 37, str(plan["commanded_c"]))
        check("corrected reading differs from raw",
              wait_until(lambda: (c.hub.marlin_channel("B").calibrated and
                                  c.hub.marlin_channel("B").value_c
                                  != c.hub.marlin_channel("B").raw_c), 8.0))

        c.clear_calibration("bed")
        check("calibration cleared",
              not c.calibration.get(ch.uid).active)
    finally:
        c.disconnect()


def test_dither() -> None:
    section("4. Fine (fractional) setpoint via host dither")
    c = _make(scale=900.0)
    try:
        c.set_fine_target("bed", 37.4, period_s=10.0)
        rt = c.zone_runtime("bed")
        check("dither engaged for a fractional target", rt.dither_enabled)
        seen: set[int] = set()
        end = time.monotonic() + 25.0
        while time.monotonic() < end and len(seen) < 2:
            seen.add(int(rt.commanded_c))
            time.sleep(0.2)
        check("board target alternates between 37 and 38",
              seen >= {37, 38} or len(seen) >= 2, str(sorted(seen)))

        c.set_fine_target("bed", 37.0, period_s=10.0)
        check("whole-degree target does not dither",
              not c.zone_runtime("bed").dither_enabled)
    finally:
        c.disconnect()


def test_autotune_success() -> None:
    section("5. PID autotune — success path")
    c = _make(scale=900.0)
    done: list = []
    c.on_autotune_done(lambda z, pid, err, hint: done.append((z, pid, err, hint)))
    try:
        started = c.start_autotune("bed", 37.0, cycles=2, apply_result=True)
        check("autotune accepted", started)
        check("autotune completed", wait_until(lambda: bool(done), 90.0))
        if done:
            z, pid, err, _hint = done[0]
            check("result is for the requested zone", z == "bed", str(z))
            check("plausible Kp/Ki/Kd returned",
                  pid is not None and pid.kp > 0 and pid.kd > 0, str(pid))
            check("no error reported", not err, str(err))
    finally:
        c.disconnect()


def test_bangbang_and_autotune_failure() -> None:
    section("6. Bang-bang firmware (PIDTEMPBED disabled)")
    caps = FakeCapabilities(bed_pid=False)
    c = _make(scale=900.0, sim_kwargs={"caps": caps})
    done: list = []
    c.on_autotune_done(lambda z, pid, err, hint: done.append((z, pid, err, hint)))
    try:
        r = c.report
        check("bed reported as bang-bang, not PID",
              not r.zone("bed").pid_available)
        check("bed PID reads 'unavailable' rather than zeros",
              r.zone("bed").pid is None)
        check("hotend PID still works", r.zone("hotend").pid_available)
        check("a firmware warning names PIDTEMPBED",
              any("PIDTEMPBED" in w for w in r.warnings()), str(r.warnings()))

        started = c.start_autotune("bed", 37.0, cycles=2)
        check("autotune refused up-front on a bang-bang zone", not started)

        # Force it past the guard to prove the firmware rejection is handled.
        c.zone_runtime("bed").pid_available = True
        c.start_autotune("bed", 37.0, cycles=2)
        check("firmware 'Bad extruder number' surfaced",
              wait_until(lambda: bool(done), 30.0))
        if done:
            _z, pid, err, hint = done[0]
            check("failure reported with no PID", pid is None and bool(err), str(err))
            check("hint names the firmware option", "PIDTEMPBED" in hint, hint)

        check("bed still holds temperature in bang-bang mode",
              _bangbang_holds(c), "bang-bang did not reach setpoint")
    finally:
        c.disconnect()


def _bangbang_holds(c: IncubatorController) -> bool:
    c.set_target("bed", 30.0)
    return wait_until(
        lambda: abs(c.hub.marlin_channel("B").value_c - 30.0) < 1.5, 60.0
    )


def test_eeprom_roundtrip() -> None:
    section("7. PID set + EEPROM round-trip")
    state = Path(_ROOT) / "tools" / "incubator" / "_data" / "_selftest_eeprom.json"
    if state.exists():
        state.unlink()
    c = _make(scale=900.0, sim_kwargs={"state_file": state})
    try:
        c.set_pid("bed", 111.0, 2.5, 1234.0)
        check("PID applied to RAM",
              wait_until(lambda: (c.zone_runtime("bed").pid is not None and
                                  abs(c.zone_runtime("bed").pid.kp - 111.0) < 0.01),
                         10.0))
        c.save_eeprom()
        check("EEPROM file written", wait_until(lambda: state.exists(), 10.0))

        c.set_pid("bed", 5.0, 0.1, 10.0)
        wait_until(lambda: abs(c.zone_runtime("bed").pid.kp - 5.0) < 0.01, 10.0)
        c.load_eeprom()
        check("reload restores the saved values",
              wait_until(lambda: abs(c.zone_runtime("bed").pid.kp - 111.0) < 0.01,
                         10.0),
              f"kp={c.zone_runtime('bed').pid.kp}")
    finally:
        c.disconnect()
        if state.exists():
            state.unlink()


def test_faults() -> None:
    section("8. Fault latch, detached sensor, and emergency path")
    c = _make(scale=900.0)
    faults: list = []
    c.on_fault(faults.append)
    try:
        c.set_target("bed", 37.0)
        time.sleep(0.5)
        c._port_obj.inject_fault("thermal_runaway", "bed")
        check("thermal runaway latched", wait_until(lambda: bool(faults), 10.0))
        if faults:
            lf = faults[0]
            check("fault attributed to Zone A", lf.zone_id == "bed", str(lf.zone_id))
            check("fault kind classified",
                  lf.fault.kind == "thermal_runaway", lf.fault.kind)
            check("hint explains the likely false trip on this rig",
                  "WATCH" in lf.fault.hint or "false" in lf.fault.hint.lower())
        check("latch blocks further commands",
              bool(c.fault_latch.blocking_reason()))
        c.acknowledge_fault()
        check("fault can be acknowledged", not c.fault_latch.active)
    finally:
        c.disconnect(heaters_off=False)

    # A detached thermistor must be reported as MINTEMP, not silently ignored.
    c2 = _make(scale=900.0)
    faults2: list = []
    c2.on_fault(faults2.append)
    try:
        c2._port_obj.detach_sensor("bed", True)
        c2._port_obj.inject_fault("mintemp", "bed")
        check("MINTEMP from a detached thermistor latched",
              wait_until(lambda: bool(faults2), 10.0))
        if faults2:
            check("hint names a disconnected thermistor",
                  "DISCONNECTED" in faults2[0].fault.hint.upper(),
                  faults2[0].fault.hint)
    finally:
        c2.disconnect(heaters_off=False)

    # Emergency path must reach the board out-of-band.
    c3 = _make(scale=900.0)
    try:
        c3.emergency_stop()
        check("M112 halts the simulated board",
              wait_until(lambda: c3._port_obj._killed, 5.0))
    finally:
        c3.disconnect(heaters_off=False)


def test_priority_during_transaction() -> None:
    section("9. Out-of-band priority write during a long transaction")
    c = _make(scale=900.0)
    try:
        # Start a board-side wait, which holds the fake board's command queue.
        c.set_target("bed", 45.0, board_side_wait=True)
        time.sleep(0.8)
        c.cancel_wait()  # M108 via send_priority
        check("M108 was accepted while a blocking wait was in flight",
              wait_until(lambda: c._port_obj._cancel_wait.is_set(), 5.0))
    finally:
        c.disconnect()


def test_phase2_seam() -> None:
    section("10. Phase-2 seam: an extra sensor source appears with no other change")
    c = _make(scale=900.0)
    try:
        before = len(c.hub.all_channels())
        box = StaticSensorSource("sensorbox")
        c.hub.add_source(box)
        box.push([("A1", "Chamber air", 36.4), ("A2", "Lid", 35.1)])
        chans = c.hub.all_channels()
        check("extra channels visible in the hub", len(chans) == before + 2,
              f"{before} -> {len(chans)}")
        uids = {ch.uid for ch in chans}
        check("channels namespaced by source",
              {"sensorbox:A1", "sensorbox:A2"} <= uids, str(sorted(uids)))
        check("existing Marlin channels unaffected",
              any(u.startswith("marlin:") for u in uids))
    finally:
        c.disconnect()


def test_telemetry() -> None:
    section("11. JSONL telemetry")
    import json
    c = _make(scale=900.0)
    try:
        path = c.start_logging("selftest")
        check("log file created", path is not None and Path(path).exists(), str(path))
        c.set_target("bed", 37.0)
        check("samples written", wait_until(lambda: c.telemetry.line_count > 3, 15.0),
              f"lines={c.telemetry.line_count}")
        c.stop_logging()
        if path:
            with open(path, "r", encoding="utf-8") as f:
                rows = [json.loads(ln) for ln in f if ln.strip()]
            check("every line is valid JSON", len(rows) > 3, f"{len(rows)} rows")
            check("manifest recorded", rows[0].get("ev") == "session_start")
            check("samples carry channel data",
                  any(r.get("ev") == "sample" and r.get("ch") for r in rows))
            Path(path).unlink(missing_ok=True)
    finally:
        c.disconnect()


def test_stability_math() -> None:
    section("12. Stability maths on synthetic data")
    tr = StabilityTracker("t", settle_dwell_s=5.0)
    t0 = 1000.0
    tr.set_target(37.0, now=t0)
    # Rise 22 -> 37 over 300 s, then hold with small ripple.
    for i in range(31):
        t = t0 + i * 10
        tr.add(22.0 + (37.0 - 22.0) * (i / 30.0), 100.0, now=t)
    for i in range(30):
        t = t0 + 310 + i * 10
        tr.add(37.0 + (0.05 if i % 2 else -0.05), 40.0, now=t)
    rep = tr.report()
    check("rate is positive during the rise or settled after",
          rep.settled or (rep.rate_c_per_min or 0) > 0)
    check("settle time computed", rep.settle_time_s is not None,
          str(rep.settle_time_s))
    check("ripple ~ +/-0.05 C",
          rep.ripple_half_c is not None and abs(rep.ripple_half_c - 0.05) < 0.02,
          str(rep.ripple_half_c))
    check("steady duty ~40%",
          rep.steady_duty_pct is not None and abs(rep.steady_duty_pct - 40) < 1,
          str(rep.steady_duty_pct))
    check("headroom note produced", bool(rep.headroom_note), rep.headroom_note)

    saturated = StabilityTracker("s", settle_dwell_s=5.0)
    saturated.set_target(37.0, now=t0)
    for i in range(20):
        saturated.add(37.0, 99.0, now=t0 + i * 10)
    check("saturated heater is called out",
          "saturated" in saturated.report().headroom_note.lower(),
          saturated.report().headroom_note)


def test_calibration_store() -> None:
    section("13. Calibration store round-trip")
    p = Path(_ROOT) / "tools" / "incubator" / "_data" / "_selftest_cal2.json"
    if p.exists():
        p.unlink()
    st = CalibrationStore(p)
    cal = st.get("marlin:B")
    cal.set_single_point(36.4, 37.0)
    check("offset computed", abs(cal.offset_c - 0.6) < 1e-6)
    check("apply corrects upward", abs(cal.apply(36.4) - 37.0) < 1e-6)
    check("invert is the exact inverse", abs(cal.invert(37.0) - 36.4) < 1e-6)
    st.save()

    st2 = CalibrationStore(p)
    check("survives reload", abs(st2.get("marlin:B").offset_c - 0.6) < 1e-6)

    two = st2.get("marlin:T")
    two.set_two_point(20.0, 21.0, 40.0, 40.5)
    check("two-point slope computed", abs(two.slope - 0.975) < 1e-6, str(two.slope))
    tiny = st2.get("marlin:X")
    tiny.set_two_point(37.0, 37.5, 37.2, 37.7)
    check("degenerate two-point span falls back to an offset",
          tiny.mode == "offset", tiny.mode)
    p.unlink(missing_ok=True)


def test_ramp() -> None:
    section("14. Watchdog-safe staircase ramp")
    from tools.incubator.ramp import (
        MAX_SAFE_STEP_C, SetpointRamp, watchdog_arm_threshold_c,
    )

    thresh = watchdog_arm_threshold_c()
    check("watchdog arm threshold matches Marlin (INCREASE+HYSTERESIS+1)",
          abs(thresh - 6.0) < 1e-9, str(thresh))
    check("default step stays below the arming threshold",
          MAX_SAFE_STEP_C < thresh, f"max step {MAX_SAFE_STEP_C} vs {thresh}")

    # Pure-logic check with a synthetic zone: every commanded target must stay
    # within the safe delta of the temperature at the moment it was issued.
    # A synthetic heater that warms on its OWN clock, like real hardware --
    # it keeps rising toward whatever target was last commanded, rather than
    # only moving when a command arrives.
    temp = {"c": 20.0}
    issued: list[tuple[float, float]] = []

    def read():
        return temp["c"]

    def cmd(c):
        issued.append((temp["c"], c))

    def warm():
        while temp["c"] < 40.0:
            time.sleep(0.01)
            temp["c"] += 0.35

    import threading as _th
    _th.Thread(target=warm, daemon=True).start()

    r = SetpointRamp("bed", read_temp=read, command_target=cmd,
                     step_c=3.0, arrive_band_c=0.5, tick_s=0.02)
    check("ramp starts", r.start(37.0))
    check("ramp finishes", wait_until(lambda: not r.active, 30.0))
    check("ramp issued multiple rising steps", len(issued) >= 4,
          f"{len(issued)} steps")
    worst = max((c - t) for t, c in issued) if issued else 0.0
    check("EVERY step stayed under the watchdog threshold",
          worst < thresh, f"largest jump above current temp was {worst:.2f} C")
    check("ramp reached the final target",
          any(abs(c - 37.0) < 0.01 for _t, c in issued), str(issued[-3:]))

    # Stall detection: a heater that cannot get there must not hang forever.
    temp2 = {"c": 20.0}
    r2 = SetpointRamp("bed", read_temp=lambda: temp2["c"],
                      command_target=lambda c: None,
                      step_c=3.0, tick_s=0.02, stall_timeout_s=1.0)
    r2.start(37.0)
    check("stalled ramp gives up rather than hanging",
          wait_until(lambda: r2.snapshot().stalled, 20.0))
    r2.stop()

    # Integration: a ramp against the simulated board, and a direct setpoint
    # must supersede it rather than fighting it.
    c = _make(scale=900.0)
    try:
        check("controller starts a ramp", c.start_ramp("bed", 37.0))
        check("ramp reported active", c.ramp_active("bed"))
        check("ramp drives the zone upward",
              wait_until(lambda: (c.hub.marlin_channel("B").value_c > 26.0), 60.0),
              f"bed={c.hub.marlin_channel('B').value_c:.2f}")
        c.set_target("bed", 30.0)
        check("a direct setpoint cancels the ramp", not c.ramp_active("bed"))
        c.start_ramp("bed", 37.0)
        c.heater_off("bed")
        check("heater off cancels the ramp", not c.ramp_active("bed"))
    finally:
        c.disconnect(heaters_off=False)


def test_decoupled_imports() -> None:
    section("15. Standalone imports stay decoupled from the app")
    import subprocess
    code = (
        "import sys;"
        "sys.path.insert(0, r'" + str(_ROOT) + "');"
        "import tools.incubator.controller, tools.incubator.device_config;"
        "from tools.incubator._serial_helpers import "
        "using_shared_serialutils, list_serial_ports;"
        "print('SHARED', using_shared_serialutils());"
        "print('PORTS_OK', isinstance(list_serial_ports(), list));"
        "print('SUPPORTCLASSES_IMPORTED', 'SupportClasses' in sys.modules);"
        "print('ZPSTAGE_IMPORTED', "
        "any(m.startswith('SupportClasses.ZPStage') for m in sys.modules));"
    )
    out = subprocess.run([sys.executable, "-c", code], capture_output=True,
                         text=True, timeout=120)
    txt = out.stdout
    check("controller imports without the SupportClasses package",
          "SUPPORTCLASSES_IMPORTED False" in txt, txt.strip() or out.stderr[-300:])
    check("ZPStageManager is never imported",
          "ZPSTAGE_IMPORTED False" in txt, txt.strip())
    check("the app's SerialUtils logic is still reused",
          "SHARED True" in txt, txt.strip())
    check("port enumeration works", "PORTS_OK True" in txt, txt.strip())


def test_stale_data_warning() -> None:
    section("16. A silently hung board is surfaced, not just frozen")
    c = _make(scale=900.0)
    msgs: list[str] = []
    c.on_status(msgs.append)
    try:
        check("data flowing initially",
              wait_until(lambda: len(c.hub.all_channels()) >= 2, 10.0))
        # Shorten the staleness threshold so the test does not wait 10 s+, then
        # stop the board talking WITHOUT closing the port (a hang, not an unplug).
        c.hub.STALE_AFTER_S = 1.0
        c._use_polling = False
        c._port_obj._autoreport_s = 0.0
        c._port_obj._stop.set()          # freeze the fake board's threads
        check("stale-data warning raised",
              wait_until(lambda: any("gone quiet" in m for m in msgs), 20.0),
              str(msgs[-1:]))
        check("channels marked stale",
              all(ch.stale for ch in c.hub.all_channels()))
    finally:
        try:
            c.disconnect(heaters_off=False)
        except Exception:
            pass


def test_gui_offscreen() -> None:
    section("18. GUI builds offscreen")
    import os
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    try:
        from PySide6.QtWidgets import QApplication
    except ImportError:
        print("  [SKIP] PySide6 not installed")
        return

    app = QApplication.instance() or QApplication([])
    from PySide6.QtGui import QPalette
    from PySide6.QtWidgets import QStyleFactory
    from tools.incubator.theme import install_theme
    install_theme(app)

    # Assert the things that actually cause the "white panels" bug, not the
    # style's objectName (which is empty under the offscreen platform).
    check("Fusion style is available and applied",
          QStyleFactory.create("Fusion") is not None
          and app.style() is not None)
    win_col = app.palette().color(QPalette.Window)
    check("dark palette applied (unstyled widgets are not white)",
          win_col.lightness() < 90, f"window lightness={win_col.lightness()}")
    qss = app.styleSheet()
    check("supplemental QSS covers the widgets the app theme misses",
          "QPlainTextEdit" in qss and "QHeaderView" in qss and "QTabBar" in qss,
          f"stylesheet length={len(qss)}")

    from tools.incubator.gui import IncubatorWindow

    c = _make(scale=900.0)
    try:
        win = IncubatorWindow(c)
        win.show()
        app.processEvents()
        check("window constructed", win is not None)
        check("both zone cards present", len(win._cards) == 2)
        check("sensor table populated",
              wait_until(lambda: (app.processEvents() or True) and
                         win._table.rowCount() >= 2, 12.0),
              f"rows={win._table.rowCount()}")
        c.set_target("bed", 37.0)
        for _ in range(30):
            app.processEvents()
            time.sleep(0.1)
        check("trend plot received samples", bool(win._plot._series))
        win._plot.repaint()
        app.processEvents()
        check("plot painted without error", True)
        check("firmware tab filled", "Firmware" in win._fw_text.toPlainText()
              or len(win._fw_text.toPlainText()) > 40)

        # The rescan affordance must be reachable, and the per-zone one must
        # appear exactly when a zone is blocked by its sensor — that is the
        # moment the operator needs it.
        check("global rescan button present", win._rescan_all_btn is not None)
        bed_card = win._cards["bed"]
        check("healthy zone hides the per-zone re-check button",
              not bed_card._rescan_btn.isVisible())
        c._port_obj.detach_sensor("bed", True)
        c.rescan_sensors()
        ok_vis = wait_until(
            lambda: (app.processEvents() or True) and bed_card._rescan_btn.isVisible(),
            10.0,
        )
        check("a sensor fault surfaces the per-zone re-check button", ok_vis)
        check("a sensor fault disables the setpoint controls",
              not bed_card._set_btn.isEnabled())
        c._port_obj.detach_sensor("bed", False)
        c.rescan_sensors()
        ok_gone = wait_until(
            lambda: (app.processEvents() or True)
            and bed_card._set_btn.isEnabled()
            and not bed_card._rescan_btn.isVisible(),
            10.0,
        )
        check("re-check re-enables the card and takes the button away", ok_gone)
        # The banner must stop claiming a sensor problem. It legitimately still
        # shows the remaining firmware limitations (integer setpoints, and the
        # simulated bang-bang bed), so "hidden" would be the wrong assertion —
        # what matters is that the resolved complaint is gone.
        banner_text = win._banner._label.text()
        check("the banner stops reporting a sensor problem once resolved",
              "sensor problem" not in banner_text.lower(), banner_text)

        # Disconnect BEFORE closing: closeEvent deliberately raises a modal
        # "leave the heaters running?" confirmation while connected, which would
        # block forever with no operator present. That prompt is a real safety
        # feature on hardware, so the test works around it rather than removing it.
        c.disconnect(heaters_off=False)
        app.processEvents()
        win.close()
        app.processEvents()
        check("window closed cleanly while disconnected", True)
    finally:
        c.disconnect(heaters_off=False)


def test_sensor_rescan() -> None:
    section("17. Sensor hot-plug re-check")
    # The scenario this exists for: the tool decides at connect whether a zone's
    # sensor is usable, and that verdict GATES heating. Plug a thermistor in
    # afterwards and the board reports it on the very next M105 — but a cached
    # verdict would keep refusing to heat, which looks like a broken tool.
    c = _make(scale=900.0)
    statuses: list[str] = []
    probes: list = []
    c.on_status(statuses.append)
    c.on_probe(probes.append)
    try:
        check("bed sensor healthy at connect", c.zone_runtime("bed").sensor_ok)

        # ── a sensor coming loose is noticed by a re-check ──
        c._port_obj.detach_sensor("bed", True)
        probes.clear()
        c.rescan_sensors()
        check("rescan republishes the report",
              wait_until(lambda: bool(probes), 10.0))
        check("re-check sees the detached sensor",
              not c.zone_runtime("bed").sensor_ok,
              f"sensor_ok={c.zone_runtime('bed').sensor_ok}")

        # The load-bearing assertion: the heating gate FOLLOWS the re-check.
        statuses.clear()
        c.set_target("bed", 37.0)
        time.sleep(0.3)
        check("heating refused while the sensor reads open-circuit",
              c.zone_runtime("bed").requested_c == 0.0
              and any("refusing to heat" in s for s in statuses),
              f"requested={c.zone_runtime('bed').requested_c}")

        # ── plug it back in: a re-check must unblock, with no reconnect ──
        c._port_obj.detach_sensor("bed", False)
        probes.clear()
        statuses.clear()
        c.rescan_sensors()
        check("re-check after plugging in restores the sensor",
              wait_until(lambda: c.zone_runtime("bed").sensor_ok, 10.0),
              f"sensor_ok={c.zone_runtime('bed').sensor_ok}")
        check("recovery is reported to the operator",
              any("unblocked" in s for s in statuses),
              "; ".join(statuses[-2:]))
        check("no reconnect was needed", c.connected)

        statuses.clear()
        c.set_target("bed", 30.0)
        time.sleep(0.3)
        check("heating is permitted again after the re-check",
              c.zone_runtime("bed").requested_c == 30.0,
              f"requested={c.zone_runtime('bed').requested_c}")
        c.heater_off("bed")

        # ── an unconfigured sensor is a DIFFERENT problem, reported as such ──
        # TEMP_SENSOR_x 0 means the field never appears in M105 at all, so no
        # amount of plugging in will help. Conflating that with a loose wire would
        # send the operator hunting for a fault that is in the firmware build.
        c._port_obj.configure_sensor("hotend", False)
        probes.clear()
        statuses.clear()
        c.rescan_sensors()
        check("re-check notices a zone the firmware has no sensor for",
              wait_until(
                  lambda: not c.report.zone("hotend").sensor_present, 10.0),
              f"present={c.report.zone('hotend').sensor_present}")
        check("it is reported as a firmware-build problem, not a loose wire",
              any("FIRMWARE" in s.upper() for s in statuses),
              "; ".join(statuses[-2:]))
        c._port_obj.configure_sensor("hotend", True)
    finally:
        c.disconnect(heaters_off=False)

    # ── a heater at full power must read 100 %, not 50 % ──
    # Regression for a real bug: the host divided M105's duty by 255, but Marlin's
    # soft-PWM full scale is 127, so a heater flat out was displayed as "50 %".
    # That is the worst possible place to lose a factor of two — steady-state duty
    # is precisely the number that tells you whether the heater can hold the
    # setpoint, so halving it suggests headroom that does not exist.
    c3 = _make(scale=1200.0)
    try:
        # Command far above what this heater can reach, so PID saturates.
        c3.set_target("bed", 49.0)
        got = wait_until(
            lambda: (ch := c3.hub.marlin_channel("B")) is not None
            and (ch.power_pct or 0) > 99.0,
            25.0,
        )
        ch = c3.hub.marlin_channel("B")
        check("a saturated heater reports ~100 % duty, not ~50 %",
              got, f"duty={None if ch is None else ch.power_pct}")

        raw = c3._port_obj._temp_report()
        check("the simulator reports duty on Marlin's 127 scale",
              "B@:127" in raw, raw)
    finally:
        c3.disconnect(heaters_off=True)

    # Classification logic on its own, with a deterministic link. Done separately
    # from the live controller because send_and_wait belongs to the single command
    # worker thread — calling it from a second thread races for the 'ok'.
    from types import SimpleNamespace
    from tools.incubator.probe import (
        FirmwareReport, ZoneCapability, rescan_sensors as _rescan,
    )

    class _StubLink:
        def __init__(self, line: str):
            self.line = line

        def send_and_wait(self, cmd, timeout_s=6.0, hard_ceiling_s=None):
            return SimpleNamespace(lines=[self.line], ok=True, failed=False)

    def _report_with(bed_ok: bool, hot_ok: bool) -> FirmwareReport:
        rep = FirmwareReport()
        for zid, good in (("bed", bed_ok), ("hotend", hot_ok)):
            rep.zones[zid] = ZoneCapability(
                zone_id=zid, sensor_present=True,
                sensor_fault="" if good else "reads -14.0 °C",
            )
        return rep

    rep = _report_with(bed_ok=True, hot_ok=True)
    res = _rescan(_StubLink("ok B:22.00 /0.00 B@:0"), rep)
    check("an absent M105 field is classed as not_configured, not a wiring fault",
          res.not_configured == ["hotend"]
          and "hotend" not in res.lost and "hotend" not in res.still_bad,
          f"nc={res.not_configured} lost={res.lost} bad={res.still_bad}")
    check("its summary names the firmware build as the thing to change",
          "FIRMWARE" in res.summary().upper(), res.summary())

    rep = _report_with(bed_ok=False, hot_ok=True)
    res = _rescan(_StubLink("ok T:23.00 /0.00 B:22.00 /0.00 @:0 B@:0"), rep)
    check("a healthy reading on a previously-bad zone counts as recovered",
          res.recovered == ["bed"] and res.changed, str(res.recovered))

    rep = _report_with(bed_ok=False, hot_ok=True)
    res = _rescan(_StubLink("ok T:23.00 /0.00 B:-14.00 /0.00 @:0 B@:0"), rep)
    check("a still-open circuit is still_bad, not a fresh 'lost'",
          res.still_bad == ["bed"] and not res.lost and not res.changed,
          f"bad={res.still_bad} lost={res.lost}")
    check("an unchanged healthy board reports nothing alarming",
          "all healthy" in _rescan(
              _StubLink("ok T:23.00 /0.00 B:22.00 /0.00 @:0 B@:0"),
              _report_with(bed_ok=True, hot_ok=True),
          ).summary())

    # ── a halted board cannot be re-read, and we must say so ──
    c2 = _make(scale=900.0)
    st2: list[str] = []
    c2.on_status(st2.append)
    try:
        c2._port_obj.inject_fault("mintemp", "bed")
        check("fault latched", wait_until(lambda: c2.fault_latch.active, 10.0))
        st2.clear()
        c2.rescan_sensors()
        time.sleep(0.3)
        check("re-check on a halted board asks for a power-cycle instead of "
              "pretending to work",
              any("POWER-CYCLE" in s.upper() for s in st2),
              "; ".join(st2))
    finally:
        c2.disconnect(heaters_off=False)


# ═══════════════════════════════════════════════════════════════════

def _clean_artefacts() -> None:
    """Remove any files a previous (possibly aborted) self-test left behind."""
    data_dir = Path(_ROOT) / "tools" / "incubator" / "_data"
    for stray in data_dir.glob("_selftest_*"):
        try:
            stray.unlink()
        except Exception:
            pass


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    want_gui = "--gui" in argv

    print("Incubator tool self-test (simulated board, no hardware required)")
    print(f"repo root: {_ROOT}")

    # Clear stale artefacts BEFORE running, not just after. A run that dies
    # part-way (or is interrupted) otherwise leaves a saved calibration behind,
    # which silently offsets later sections and produces failures that look like
    # product bugs. Tests must not depend on the previous run having tidied up.
    _clean_artefacts()

    test_parser()
    test_probe_and_zones()
    test_setpoint_translation()
    test_dither()
    test_autotune_success()
    test_bangbang_and_autotune_failure()
    test_eeprom_roundtrip()
    test_faults()
    test_priority_during_transaction()
    test_phase2_seam()
    test_telemetry()
    test_stability_math()
    test_calibration_store()
    test_ramp()
    test_decoupled_imports()
    test_stale_data_warning()
    test_sensor_rescan()
    if want_gui:
        test_gui_offscreen()
    else:
        print("\n(skipping GUI test; pass --gui to include it)")

    # Leave no artefacts behind — the self-test should be repeatable and should
    # not leave a stale calibration lying around for the real tool to pick up.
    data_dir = Path(_ROOT) / "tools" / "incubator" / "_data"
    for stray in data_dir.glob("_selftest_*"):
        try:
            stray.unlink()
        except Exception:
            pass

    print(f"\n{'=' * 60}")
    print(f"PASS {PASS}   FAIL {FAIL}")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
