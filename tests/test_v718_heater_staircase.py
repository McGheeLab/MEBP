"""
test_v718_heater_staircase.py -- the bench tool's power staircase.

This is the tool the operator runs when the board vanishes off USB the
moment the heater starts (app.log 2026-08-17: WriteFile ERROR_BAD_COMMAND
0.45 s after M140 S37, four attempts out of four). It only ever runs at the
bench, against hardware that is already misbehaving -- so every part of it
that CAN be exercised without a board is exercised here, driving the real
staircase_test() against a fake Marlin that browns out at a chosen duty.

The recorded lesson this guards: a diagnostic that crashes at the moment it
is needed is worthless (the first draft of this tool died on a decorative
emoji under cp1252).
"""

from __future__ import annotations

import io
import unittest
from contextlib import redirect_stdout

import tools_incubator_heater_diagnostic as diag


class _Clock:
    """A monotonic clock the test advances, so a 12 s rung costs no time."""

    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        self.t += float(dt)


class FakeLink:
    """A Marlin bed that obeys pure-P PID and dies above a current draw.

    duty = clamp(P * error / 2, 0, 127)  -- the relation the tool inverts.
    Once duty exceeds `dies_above_duty` the port raises, which is what a
    board leaving USB looks like to pyserial.
    """

    def __init__(self, *, bed_c=22.0, dies_above_duty=None, clock=None,
                 fault_above_duty=None, silent_above_duty=None,
                 duty_stuck_zero=False):
        self.bed_c = bed_c
        self.duty_stuck_zero = duty_stuck_zero
        self.p = 0.0
        self.target = 0.0
        self.dies_above_duty = dies_above_duty
        self.fault_above_duty = fault_above_duty
        self.silent_above_duty = silent_above_duty
        self.sent: list[str] = []
        self.dead = False
        self.clock = clock

    # -- the duty the firmware would be driving right now --
    def _duty(self):
        if self.duty_stuck_zero:
            return 0                    # bed disabled / latched fault
        if self.target <= self.bed_c:
            return 0
        return int(max(0, min(127, self.p * (self.target - self.bed_c) / 2.0)))

    def txn(self, cmd, timeout=6.0):
        self.sent.append(cmd)
        if self.dead:
            raise OSError("WriteFile failed (the device does not recognize "
                          "the command)")
        if cmd.startswith("M304"):
            for tok in cmd.split():
                if tok.startswith("P"):
                    self.p = float(tok[1:])
            return ["ok"]
        if cmd.startswith("M140"):
            self.target = float(cmd.split("S")[1]) if "S" in cmd else 0.0
            return ["ok"]
        if cmd.startswith("M105"):
            duty = self._duty()
            # heating: the sensor is bonded to the film, so it follows duty
            self.bed_c += duty / 127.0 * 0.4
            if self.dies_above_duty is not None and duty > self.dies_above_duty:
                self.dead = True
                raise OSError("WriteFile failed (the device does not "
                              "recognize the command)")
            if (self.silent_above_duty is not None
                    and duty > self.silent_above_duty):
                return []
            if (self.fault_above_duty is not None
                    and duty > self.fault_above_duty):
                return ["Error:Thermal Runaway", "ok"]
            return [f"ok T:-15.00 /0.00 B:{self.bed_c:.2f} /{self.target:.2f} "
                    f"@:0 B@:{duty}"]
        return ["ok"]


def _run(link, **kw):
    """Run the real staircase against `link`, returning its printed report."""
    clock = _Clock()
    link.clock = clock
    real = diag.time
    diag.time = clock                    # monotonic + sleep both come from it
    buf = io.StringIO()
    try:
        with redirect_stdout(buf):
            diag.staircase_test(
                link,
                ceiling_c=kw.pop("ceiling_c", 40),
                rungs=kw.pop("rungs", [10, 25, 50, 75, 100]),
                rung_seconds=kw.pop("rung_seconds", 4.0),
                abort_above=kw.pop("abort_above", 45.0),
                mode=kw.pop("mode", "pid"),
                pwm_period=kw.pop("pwm_period", 4.0),
                orig_pid=kw.pop("orig_pid", (41.78, 7.32, 158.93)),
            )
    finally:
        diag.time = real
    return buf.getvalue()


# -- pure helpers ---------------------------------------------------

class TestPlanStaircase(unittest.TestCase):

    def test_parses_sorts_and_dedupes(self):
        self.assertEqual([10, 25, 50], diag.plan_staircase("50,10,25,10"))

    def test_ascending_order_is_the_point(self):
        """The run stops at the first killing rung, so 'highest survived'
        is only true if the lower rungs were tried first."""
        self.assertEqual([5, 90], diag.plan_staircase("90,5"))

    def test_rejects_out_of_range_and_junk(self):
        for bad in ("0", "101", "-5", "abc", "", ","):
            with self.assertRaises(ValueError, msg=bad):
                diag.plan_staircase(bad)


class TestPGainForDuty(unittest.TestCase):

    def test_inverts_the_duty_relation(self):
        # duty = P*err/2  =>  P = 2*duty/err
        p = diag.p_gain_for_duty(50, 18.0)
        duty = p * 18.0 / 2.0
        self.assertAlmostEqual(0.50 * diag.DUTY_FULL, duty, places=6)

    def test_monotonic_in_requested_duty(self):
        vals = [diag.p_gain_for_duty(pct, 18.0) for pct in (10, 25, 50, 100)]
        self.assertEqual(vals, sorted(vals))

    def test_tiny_error_cannot_divide_by_zero(self):
        self.assertLessEqual(diag.p_gain_for_duty(100, 0.0), 500.0)
        self.assertGreater(diag.p_gain_for_duty(100, 0.0), 0.0)


class TestParseBedPid(unittest.TestCase):

    def test_reads_the_m503_line(self):
        self.assertEqual(
            (41.78, 7.32, 158.93),
            diag.parse_bed_pid(["echo:  M304 P41.78 I7.32 D158.93"]))

    def test_absent_when_the_build_has_no_pidtempbed(self):
        self.assertIsNone(diag.parse_bed_pid(["echo:  M92 X80", "ok"]))


class TestPollClassification(unittest.TestCase):

    def test_ok(self):
        kind, s, _ = diag.poll_once(FakeLink())
        self.assertEqual("ok", kind)
        self.assertIsNotNone(s)

    def test_dropped_is_the_port_dying(self):
        link = FakeLink()
        link.dead = True
        kind, _s, detail = diag.poll_once(link)
        self.assertEqual("dropped", kind)
        self.assertIn("WriteFile", detail)

    def test_fault_line_outranks_a_parseable_reply(self):
        link = FakeLink(fault_above_duty=-1)     # always faults
        link.p, link.target = 10.0, 40.0
        kind, _s, detail = diag.poll_once(link)
        self.assertEqual("fault", kind)
        self.assertIn("Thermal Runaway", detail)

    def test_silent_board_is_not_reported_as_a_usb_drop(self):
        """Port open + no answer is kill(), a DIFFERENT remedy from a
        brownout, so the two must never be conflated."""
        link = FakeLink(silent_above_duty=-1)
        link.p, link.target = 10.0, 40.0
        kind, _s, _ = diag.poll_once(link)
        self.assertEqual("silent", kind)


# -- the staircase, end to end --------------------------------------

class TestStaircaseFindsTheLevel(unittest.TestCase):

    def test_reports_the_highest_surviving_rung(self):
        # dies above ~50% of 127
        link = FakeLink(dies_above_duty=70)
        out = _run(link, rungs=[10, 25, 50, 100])
        self.assertIn("STAIRCASE VERDICT", out)
        self.assertIn("Highest level SURVIVED", out)
        self.assertIn("FAILED at the 100% rung", out)
        self.assertIn("dropped", out)

    def test_a_board_that_survives_everything_says_so(self):
        link = FakeLink(dies_above_duty=None)
        out = _run(link, rungs=[10, 50, 100])
        self.assertIn("survived every rung", out)
        self.assertNotIn("FAILED at", out)

    def test_it_stops_at_the_first_killing_rung(self):
        """Continuing past a dead port would just print noise, and every
        later 'result' would be a lie."""
        link = FakeLink(dies_above_duty=20)
        out = _run(link, rungs=[10, 25, 50, 75, 100])
        self.assertIn("FAILED at the 25% rung", out)
        for higher in ("FAILED at the 50%", "FAILED at the 75%",
                       "FAILED at the 100%"):
            self.assertNotIn(higher, out)

    def test_the_delivered_duty_tracks_the_requested_rung(self):
        """The whole test is worthless if asking for 25% delivers 100%.

        Asserts the DELIVERED duty (the printed column, i.e. what the board
        reported), not the gain that was commanded -- the gain is trimmed
        against a shrinking error, so reconstructing a duty from it says
        nothing about what the heater actually did.
        """
        link = FakeLink(dies_above_duty=None)
        out = _run(link, rungs=[25], rung_seconds=3.0)
        duties = [int(ln.split()[4]) for ln in out.splitlines()
                  if ln.strip().startswith("25 ") or
                  (ln.strip().startswith("25") and len(ln.split()) == 6
                   and ln.split()[0] == "25")]
        self.assertTrue(duties, f"no duty rows parsed from:\n{out}")
        peak = max(duties)
        want = 0.25 * diag.DUTY_FULL
        self.assertLess(abs(peak - want), 0.20 * diag.DUTY_FULL,
                        f"asked for 25% ({want:.0f}/127), delivered {peak}/127")

    def test_a_rung_that_never_delivered_is_not_called_a_pass(self):
        """A bed at the ceiling drives 0% duty. Reporting that as 'survived
        100%' is a false all-clear -- the operator would conclude the supply
        is fine and go looking for a software fault that isn't there."""
        link = FakeLink(bed_c=39.5, dies_above_duty=None)   # ceiling 40
        out = _run(link, rungs=[100], ceiling_c=40)
        self.assertNotIn("Highest level SURVIVED", out)
        self.assertIn("no error for the firmware to drive", out)

    def test_duty_stuck_at_zero_is_inconclusive_not_a_pass(self):
        """The round-4 'upstream fault' case: the board answers normally but
        never drives the output (bed disabled, latched fault). The link is
        then never loaded, so calling it a pass would hide the real fault."""
        link = FakeLink(duty_stuck_zero=True)
        out = _run(link, rungs=[50, 100])
        self.assertIn("not delivered", out.lower())
        self.assertIn("NOTHING WAS PROVEN", out)
        self.assertNotIn("Highest level SURVIVED", out)

    def test_a_firmware_halt_is_not_blamed_on_the_supply(self):
        link = FakeLink(fault_above_duty=20)
        out = _run(link, rungs=[10, 50])
        self.assertIn("FIRMWARE stopping", out)
        self.assertIn("POWER CYCLE", out)
        self.assertNotIn("left USB", out)

    def test_temperature_abort_is_not_reported_as_a_failure(self):
        """Hitting the thermal limit means the heater WORKS -- reporting it
        as a link failure would send the operator after the wrong fault."""
        link = FakeLink(bed_c=30.0, dies_above_duty=None)
        out = _run(link, rungs=[100], ceiling_c=45, abort_above=31.0,
                   rung_seconds=30.0)
        self.assertIn("[ABORT]", out)
        self.assertNotIn("FAILED at", out)
        self.assertIn("temperature limit", out.lower())
        # The rung it was measuring counts: the link carried that duty right
        # up to the abort, so discarding it would report "NOTHING WAS PROVEN"
        # over a log showing 99%.
        self.assertIn("Highest level SURVIVED", out)
        self.assertNotIn("NOTHING WAS PROVEN", out)


class TestStaircaseRestoresTheBoard(unittest.TestCase):

    def test_pid_is_put_back_after_a_clean_run(self):
        link = FakeLink(dies_above_duty=None)
        out = _run(link, rungs=[10])
        self.assertIn("Bed PID restored", out)
        self.assertEqual("M304 P41.78 I7.32 D158.93", link.sent[-1])

    def test_pid_restore_is_attempted_even_after_a_failure(self):
        link = FakeLink(dies_above_duty=5)
        out = _run(link, rungs=[50])
        # The port is dead, so the restore CANNOT succeed -- it must say so
        # and name the EEPROM values rather than claim success.
        self.assertIn("COULD NOT RESTORE", out)
        self.assertIn("Power-cycle", out)

    def test_pwm_mode_never_touches_the_pid(self):
        """It exists to test full-current bursts; changing the control law
        would make it measure something else."""
        link = FakeLink(dies_above_duty=None)
        _run(link, rungs=[50], mode="pwm", rung_seconds=6.0)
        self.assertEqual([], [c for c in link.sent if c.startswith("M304")])

    def test_pwm_mode_actually_gates_the_heater_on_and_off(self):
        link = FakeLink(dies_above_duty=None)
        _run(link, rungs=[50], mode="pwm", rung_seconds=12.0, pwm_period=4.0)
        m140 = [c for c in link.sent if c.startswith("M140")]
        self.assertIn("M140 S0", m140)
        self.assertIn("M140 S40", m140)


class TestOpenCircuitGuard(unittest.TestCase):
    """Bench 2026-08-17: HE0 reported duty 127/127 for 20 s with the heater
    UNPLUGGED -- no heat, no supply sag. The duty field is what Marlin
    COMMANDS, not proof current flowed, so a survival claim built on it
    alone would tell the operator their supply carries full power when
    nothing was connected."""

    def test_sustained_full_duty_with_no_rise_is_an_open_circuit(self):
        from SupportClasses.incubator import power_staircase as ps
        self.assertTrue(ps.rung_open_circuit(100.0, 20.0, 0.17))

    def test_a_brief_low_duty_rung_is_not_accused(self):
        """A real heater at 3% for two seconds legitimately shows nothing."""
        from SupportClasses.incubator import power_staircase as ps
        self.assertFalse(ps.rung_open_circuit(3.0, 2.0, 0.0))
        self.assertFalse(ps.rung_open_circuit(100.0, 2.0, 0.0))

    def test_a_rung_that_actually_heated_is_not_accused(self):
        from SupportClasses.incubator import power_staircase as ps
        self.assertFalse(ps.rung_open_circuit(100.0, 20.0, 3.34))

    def test_the_verdict_says_the_link_was_never_loaded(self):
        from SupportClasses.incubator import power_staircase as ps
        out = ps.StaircaseOutcome(survived=[(100, 127)],
                                  no_heat=[(100, 127, 0.17)])
        text = chr(10).join(ps.verdict_lines(out))
        self.assertIn("NOTHING WAS DRAWING", text)
        self.assertIn("never loaded", text)
        text.encode("ascii")


class TestOpenCircuitIsPopulatedByTheRun(unittest.TestCase):
    """Guard against the dead-field trap: the pure check is worthless if no
    production path ever fills `no_heat`. A fake board that reports full duty
    while its sensor never moves is exactly the unplugged-heater case."""

    def test_the_tool_flags_a_rung_that_commanded_power_and_never_heated(self):
        class OpenCircuit(FakeLink):
            """Full duty reported, no heat delivered -- unplugged heater."""
            def _duty(self):
                return 127
            def txn(self, cmd, timeout=6.0):
                before = self.bed_c
                out = super().txn(cmd, timeout)
                self.bed_c = before          # nothing ever gets warm
                return out
        out = _run(OpenCircuit(dies_above_duty=None), rungs=[100],
                   rung_seconds=20.0)
        self.assertIn("nothing is drawing", out.lower())
        self.assertIn("NOTHING WAS DRAWING", out)

    def test_a_rung_that_really_heated_is_not_flagged(self):
        link = FakeLink(dies_above_duty=None)
        out = _run(link, rungs=[100], rung_seconds=20.0)
        self.assertNotIn("NOTHING WAS DRAWING", out)


class TestOutputIsConsoleSafe(unittest.TestCase):

    def test_no_non_ascii_anywhere_in_the_tool(self):
        """A console here kills the diagnostic on a decorative character at
        exactly the moment it is needed."""
        with open(diag.__file__, "rb") as fh:
            data = fh.read()
        offenders = [i for i, b in enumerate(data) if b > 127]
        self.assertEqual([], offenders,
                         f"non-ASCII byte(s) at {offenders[:5]}")

    def test_the_shared_verdict_text_is_ascii_too(self):
        """The verdict moved into SupportClasses so the GUI could share it.
        The tool still PRINTS it, so the console rule follows it there --
        the source-bytes check above cannot see another module."""
        from SupportClasses.incubator import power_staircase as ps
        cases = [
            ps.StaircaseOutcome(survived=[(50, 60)],
                                died_at=(75, 90, "dropped", "OSError: x")),
            ps.StaircaseOutcome(undelivered=[(100, 0)]),
            ps.StaircaseOutcome(survived=[(100, 127)]),
            ps.StaircaseOutcome(aborted_hot=True, survived=[(100, 126)]),
            ps.StaircaseOutcome(cancelled=True),
            ps.StaircaseOutcome(mode="pwm",
                                died_at=(50, 60, "fault", "Thermal Runaway")),
            ps.StaircaseOutcome(died_at=(10, 0, "refused", "Error: no bed")),
        ]
        for out in cases:
            text = "\n".join(ps.verdict_lines(out))
            text.encode("ascii")          # raises on anything decorative

    def test_the_report_encodes_to_cp1252(self):
        link = FakeLink(dies_above_duty=70)
        out = _run(link, rungs=[10, 100])
        out.encode("cp1252")             # raises if anything is undecorative


if __name__ == "__main__":
    unittest.main()
