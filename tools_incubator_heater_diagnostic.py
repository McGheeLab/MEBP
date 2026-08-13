"""
tools_incubator_heater_diagnostic.py -- why is the bed heater not heating?

Run this with **MEBP CLOSED** (the app owns the board's COM port while it is
running). It talks to the Marlin board directly and answers ONE question that
the GUI cannot: *is the board actually driving the heater output, or not?*

That question splits the whole problem in half, because Marlin reports its own
bed PWM duty in every ``M105`` reply (the ``B@:`` field, 0-127):

    B@ > 0, temperature flat   -> the firmware IS driving the output.
                                  The fault is DOWNSTREAM: no VIN on the
                                  board, blown bed fuse/MOSFET, open heater
                                  element, bad crimp, wrong screw terminal.

    B@ == 0 while below target -> the firmware is NOT driving.
                                  The fault is UPSTREAM: target not applied,
                                  bed disabled in the firmware build, a
                                  latched thermal fault, MINTEMP/MAXTEMP.

    temperature rises          -> the heater works; the earlier failure was in
                                  the control path (watchdog halt/wrong zone).

It also reports:
  * every serial port with its VID:PID, and WHICH one answers as Marlin --
    the ports on this rig have renumbered before (FIRMWARE_NOTES.md still
    says COM6; the app found the board on COM4 on 2026-08-12);
  * ``M115`` capabilities and ``M503`` (bed PID + whether M304 exists at all);
  * ``M122`` -- the TMC2209 drivers are powered from **VIN**, not USB, so a
    board with no main power answers M105 happily while its drivers report
    nothing. That is an indirect but non-invasive VIN check.

SAFETY
  * Never moves an axis. Only ever touches the BED output (``M140``).
  * The commanded target is capped, the run is short, and ``M140 S0`` is sent
    in a ``finally`` -- including on Ctrl-C.
  * Aborts immediately if the sensor reads above ``--abort-above``.
  * Best case, and the setup this is tuned for: the thermistor bonded
    DIRECTLY to the heater film. The sensor then measures the heater itself,
    so (a) a working heater shows a rise within a few seconds, and (b) the
    abort threshold really protects the film. (2026-08-13 bench: this is how
    ME3B V1 is wired for the test.)
  * !! If the sensor is instead on the water block while the film hangs loose
    in air, the sensor CANNOT see the film's temperature -- a loose film at
    full duty can reach a damaging temperature while the readout still says
    room temperature. In that arrangement use ``--seconds 0`` (duty check
    only; the answer arrives in the first two seconds) and stay with it.

OUTPUT IS DELIBERATELY PURE ASCII. A Windows console here is cp1252, and a
UnicodeEncodeError from a decorative character would kill the diagnostic at
exactly the moment it is needed (the same trap already recorded for
tools_install_tucsen_sdk.ps1).

Usage:
    python tools_incubator_heater_diagnostic.py                 # scan + check
    python tools_incubator_heater_diagnostic.py --port COM4
    python tools_incubator_heater_diagnostic.py --seconds 0     # duty only
    python tools_incubator_heater_diagnostic.py --report-only   # no heating
"""

from __future__ import annotations

import argparse
import re
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:                                    # pragma: no cover
    sys.exit("pyserial is required:  pip install pyserial")


BAUD = 115200          # native USB CDC ignores this; harmless on FTDI too
ERROR_MARKERS = ("error:", "!!", "kill", "thermal", "mintemp", "maxtemp",
                 "heating failed", "printer halted")


# -- serial link ----------------------------------------------------

class Link:
    """One synchronous command -> reply-lines transaction, like the app's."""

    def __init__(self, port: str, baud: int = BAUD):
        self.port = port
        self.ser = serial.Serial(port, baud, timeout=0.25)
        time.sleep(2.0)          # a native-USB board reboots on port open
        self.ser.reset_input_buffer()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def txn(self, cmd: str, timeout: float = 6.0) -> list[str]:
        """Write one line, read until 'ok' (or timeout). Returns every line."""
        self.ser.reset_input_buffer()
        self.ser.write((cmd.strip() + "\n").encode("ascii", "replace"))
        self.ser.flush()
        lines: list[str] = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", "replace").strip()
            if not text:
                continue
            lines.append(text)
            low = text.lower()
            if low.startswith("ok") or low.startswith("error"):
                break
            if low.startswith("busy") or low.startswith("echo:busy"):
                deadline = time.monotonic() + timeout   # board alive, wait on
        return lines


def problem_lines(lines: list[str]) -> list[str]:
    return [ln for ln in lines
            if any(m in ln.lower() for m in ERROR_MARKERS)]


# -- M105 parsing ---------------------------------------------------

_RE_BED = re.compile(r"\bB:\s*(-?[\d.]+)\s*/\s*(-?[\d.]+)")
_RE_HOT = re.compile(r"\bT:\s*(-?[\d.]+)\s*/\s*(-?[\d.]+)")
_RE_BED_DUTY = re.compile(r"\bB@:\s*(\d+)")


def parse_m105(lines: list[str]) -> dict | None:
    """Marlin puts the temperatures ON the ok line: `ok T:.. /.. B:.. B@:..`"""
    for text in lines:
        m = _RE_BED.search(text)
        if not m:
            continue
        out = {
            "bed_c": float(m.group(1)),
            "bed_target_c": float(m.group(2)),
            "bed_duty": None,
            "hot_c": None,
            "hot_target_c": None,
            "raw": text,
        }
        d = _RE_BED_DUTY.search(text)
        if d:
            out["bed_duty"] = int(d.group(1))
        h = _RE_HOT.search(text)
        if h:
            out["hot_c"] = float(h.group(1))
            out["hot_target_c"] = float(h.group(2))
        return out
    return None


# -- port discovery -------------------------------------------------

def scan_ports() -> list:
    ports = list(list_ports.comports())
    print("Serial ports on this machine:")
    if not ports:
        print("   (none)")
    for p in ports:
        print(f"   {p.device:<8} {p.description}")
        print(f"            hwid: {p.hwid}")
    print()
    return ports


def find_marlin(preferred: str | None = None):
    """Probe ports with M115 and return (port, lines) for the first Marlin."""
    ports = [p.device for p in list_ports.comports()]
    if preferred:
        ports = [preferred] + [p for p in ports if p != preferred]
    for dev in ports:
        try:
            link = Link(dev)
        except Exception as e:
            print(f"   {dev}: cannot open -- {e}")
            continue
        try:
            lines = link.txn("M115", timeout=3.0)
            blob = " ".join(lines).lower()
            if "marlin" in blob or "firmware_name" in blob:
                print(f"   {dev}: answered as Marlin  [OK]")
                return dev, lines
            print(f"   {dev}: opened, but no Marlin identity "
                  f"({'no reply' if not lines else lines[0][:60]})")
        finally:
            link.close()
    return None


# -- the checks -----------------------------------------------------

def report_state(link: Link) -> dict:
    print("-- Firmware identity (M115) " + "-" * 39)
    for ln in link.txn("M115", timeout=4.0):
        print("   " + ln)

    print()
    print("-- Settings (M503): the bed PID + thermal config " + "-" * 18)
    m503 = link.txn("M503", timeout=8.0)
    interesting = [ln for ln in m503
                   if re.match(r"^\s*(echo:\s*)?(M30[1-5]|M14[0-9])", ln)
                   or "bed" in ln.lower()]
    for ln in (interesting or m503[:20]):
        print("   " + ln)
    if not any("M304" in ln for ln in m503):
        print("   !! NO M304 line -- PIDTEMPBED may be disabled in this build")
        print("      (the bed would run bang-bang, but it would still HEAT).")

    print()
    print("-- Stepper drivers (M122): an indirect VIN check " + "-" * 18)
    print("   The TMC2209s are powered from VIN, not USB. If the main supply")
    print("   is off, the board still answers M105 but the drivers report")
    print("   nothing -- and the heater MOSFET has no rail to switch.")
    m122 = link.txn("M122", timeout=8.0)
    if not m122:
        print("   (no reply -- M122 may not be compiled in)")
    for ln in m122[:24]:
        print("   " + ln)

    print()
    print("-- Baseline temperatures (M105 x3) " + "-" * 32)
    sample = None
    for _ in range(3):
        sample = parse_m105(link.txn("M105", timeout=4.0))
        if sample:
            duty = sample["bed_duty"]
            print(f"   bed {sample['bed_c']:7.2f} C   target "
                  f"{sample['bed_target_c']:5.1f}   duty "
                  f"{duty if duty is not None else '?'}"
                  f"   |  hotend {sample['hot_c']} C")
        else:
            print("   (could not parse an M105 reply)")
        time.sleep(0.6)
    return sample or {}


def resistance_table(supply_v: float) -> list[str]:
    """What the heater should measure, for THIS rig's supply voltage.

    P = V^2/R and I = V/R, so at 12 V a given wattage draws twice the current
    it would at 24 V. That matters twice over: it is how you identify the
    film with a multimeter, and it is how you find out whether the supply and
    the board's bed output can actually deliver it.
    """
    rows = [f"           at {supply_v:g} V:  "
            f"{'watts':>7}  {'ohms':>7}  {'amps':>6}"]
    for watts in (5, 10, 20, 25, 50, 100):
        ohms = (supply_v * supply_v) / watts
        amps = supply_v / ohms
        rows.append(f"                       {watts:7d}  {ohms:7.1f}  "
                    f"{amps:6.2f}")
    rows.append(f"           (measured R  ->  P = {supply_v:g}^2/R,  "
                f"I = {supply_v:g}/R)")
    return rows


def drive_test(link: Link, target_c: int, seconds: int,
               abort_above: float, supply_v: float = 12.0) -> None:
    print()
    print("=" * 68)
    print(f"  COMMANDING THE BED to {target_c} C  (M140 S{target_c})")
    print(f"  Watching for {seconds}s. Ctrl-C stops and turns the heater off.")
    print("=" * 68)

    lines = link.txn(f"M140 S{target_c}", timeout=8.0)
    bad = problem_lines(lines)
    print(f"   reply: {lines if lines else '(none)'}")
    if bad:
        print("   *** The board REFUSED the command or reported a fault:")
        for ln in bad:
            print("      " + ln)
        return

    start = time.monotonic()
    first = None
    max_duty = 0
    duty_seen = False
    no_reply = 0
    print()
    print("    t(s)   sensor C   target   duty(0-127)   change")
    print("    ----   --------   ------   -----------   ------")
    while True:
        elapsed = time.monotonic() - start
        try:
            lines = link.txn("M105", timeout=4.0)
            # Marlin emits its fault text UNSOLICITED, between polls. Catching
            # it here is the whole point of a long soak: `Error:Thermal
            # Runaway` followed by silence is the kill() signature, and after
            # kill() the board answers nothing until it is power-cycled.
            faults = problem_lines(lines)
            if faults:
                print()
                print("   *** THE BOARD REPORTED A FAULT after "
                      f"{elapsed:.1f}s of heating:")
                for ln in faults:
                    print("       " + ln)
                print()
                print("       Marlin has almost certainly called kill(): it")
                print("       stops answering G-code entirely and needs a")
                print("       POWER CYCLE. That is what 'the heater does not")
                print("       heat' looks like from the app -- the target is")
                print("       lost and the board is gone.")
                print()
                print("       This is a firmware-window problem, not a")
                print("       hardware one. See FIRMWARE_NOTES.md: the stock")
                print("       thermal-protection windows are sized for a 3D")
                print("       printer, not this load.")
                return
            if not lines:
                no_reply += 1
                if no_reply >= 3:
                    print()
                    print(f"   *** THE BOARD STOPPED ANSWERING after "
                          f"{elapsed:.1f}s of heating "
                          f"({no_reply} silent polls).")
                    print("       The port is still open, so this is not a USB")
                    print("       drop -- it is the kill() signature. Marlin")
                    print("       has halted and needs a POWER CYCLE.")
                    print("       Check the lines printed just above for the")
                    print("       reason (thermal runaway / heating failed).")
                    return
            else:
                no_reply = 0
            s = parse_m105(lines)
        except Exception as e:
            # The board vanishing from USB the instant the heater switches on
            # is itself a decisive result -- app.log shows exactly this
            # (WriteFile ERROR_BAD_COMMAND) around every heat attempt on
            # 2026-08-12. Report it rather than dying with a traceback.
            print()
            print(f"   *** THE BOARD DROPPED OFF USB after {elapsed:.1f}s "
                  f"of heating.")
            print(f"       ({type(e).__name__}: {e})")
            print()
            print("       This is a POWER fault, not a software one. Switching")
            print("       the heater pulled the board's supply down far enough")
            print("       to reset it -- which points at a short or a")
            print("       near-short in the heater or its wiring, or a supply")
            print("       that cannot deliver the heater's current.")
            print()
            print("       Next: with everything powered OFF, measure the")
            print("       resistance across the heater leads. A dead short")
            print("       (near 0 ohm) or a value far below the film's rating")
            print("       confirms it. Do NOT retry until that is resolved.")
            return
        if s is None:
            print("    (unparseable M105)")
        else:
            if first is None:
                first = s["bed_c"]
            duty = s["bed_duty"]
            if duty is not None:
                duty_seen = True
                max_duty = max(max_duty, duty)
            print(f"   {elapsed:5.1f}   {s['bed_c']:8.2f}   "
                  f"{s['bed_target_c']:6.1f}   "
                  f"{(str(duty) if duty is not None else '?'):>11}   "
                  f"{s['bed_c'] - first:+6.2f} C")
            if s["bed_c"] >= abort_above:
                print(f"\n   [ABORT] sensor reached {s['bed_c']:.1f} C "
                      f"(limit {abort_above:.1f}). Turning the heater off.")
                break
        if elapsed >= seconds:
            break
        time.sleep(1.0)

    last = parse_m105(link.txn("M105", timeout=4.0))
    rise = None
    if first is not None and last is not None:
        rise = last["bed_c"] - first
    verdict(duty_seen, max_duty, rise, seconds, supply_v)


def verdict(duty_seen: bool, max_duty: int, rise: float | None,
            seconds: int, supply_v: float = 12.0) -> None:
    print()
    print("=" * 68)
    print("  VERDICT")
    print("=" * 68)
    if not duty_seen:
        print("  ?   This firmware did not report a B@: duty field, so the")
        print("      'is it driving?' question could not be answered")
        print("      directly. Fall back to the temperature trend above.")
    elif max_duty == 0:
        print("  *** The board is NOT driving the bed output (duty stayed 0)")
        print("      while the bed was below target.")
        print()
        print("      The fault is UPSTREAM of the wiring -- look at:")
        print("        - did the target actually stick? (target column above)")
        print("        - a latched thermal fault: power-cycle the board, retry")
        print("        - TEMP_SENSOR_BED / HEATER_BED_PIN in the firmware build")
        print("        - MAXTEMP already exceeded, or a MINTEMP condition")
    else:
        pct = 100.0 * max_duty / 127.0
        print(f"  [OK] The board IS driving the bed output -- peak duty "
              f"{max_duty}/127 ({pct:.0f}% power).")
        print()
        if rise is None:
            print("      Temperature trend unavailable.")
        elif seconds <= 0:
            print("      No soak was run, so heat output was not measured.")
            print("      Re-run with --seconds 30 (attended) to see the rise.")
        elif rise >= 0.5:
            rate = rise * 60.0 / max(1, seconds)
            print(f"      And the sensor ROSE {rise:+.2f} C in {seconds}s "
                  f"({rate:+.1f} C/min) --")
            print("      the heater IS working. The fault is in the control")
            print("      path, not the hardware: most likely Marlin's heat-up")
            print("      watchdog halting the board mid-climb (see")
            print("      FIRMWARE_NOTES.md), or the app driving the wrong zone.")
        else:
            print(f"      But the sensor only moved {rise:+.2f} C in "
                  f"{seconds}s.")
            print()
            print("      *** Firmware is switching the MOSFET and no heat is")
            print("          arriving. With the thermistor bonded to the film,")
            print("          a live heater would have moved within seconds, so")
            print("          this is conclusive. The fault is DOWNSTREAM:")
            print("        1. VIN -- is the main PSU on and connected? The")
            print("           board runs its logic from USB alone, so")
            print("           everything 'works' with no power supply. Check")
            print("           the M122 output above: dead/zeroed drivers means")
            print("           no VIN. (Or jog Z in the app -- if the motor")
            print("           moves, VIN is present.)")
            print("        2. Heater resistance -- measure across the film")
            print("           with everything powered off. Open circuit (OL)")
            print("           means a broken element or crimp, and you are")
            print("           done. Otherwise:")
            for row in resistance_table(supply_v):
                print("        " + row)
            print("        3. Supply headroom -- the current from the table")
            print("           above has to come from the SAME supply feeding")
            print("           the board and steppers. A 12 V rail draws twice")
            print("           the amps 24 V would for the same wattage, so an")
            print("           undersized PSU browns out when the heater")
            print("           switches on. Compare it against the PSU's")
            print("           rating, and against the board's bed-output")
            print("           limit for your revision (BTT's spec sheet).")
            print("        4. The screw terminal -- HB is the bed pair.")
            print("           Confirm both wires are clamped on copper, not")
            print("           on insulation, and that neither has backed out.")
            print("        5. The bed MOSFET -- with duty high, measure for")
            print(f"           {supply_v:g} V across the HB terminal itself. "
                  f"Present at")
            print("           the terminal but no heat = wiring/element.")
            print("           Absent at the terminal = the board's output.")
    print()


# -- main -----------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Diagnose a Marlin bed heater that will not heat.")
    ap.add_argument("--port", help="serial port (default: probe for Marlin)")
    ap.add_argument("--target", type=int, default=None,
                    help="bed target C (default: current + 10, capped 40)")
    ap.add_argument("--seconds", type=int, default=30,
                    help="soak duration; 0 = duty check only (default 30 -- "
                         "with the sensor on the film a live heater shows in "
                         "well under that)")
    ap.add_argument("--abort-above", type=float, default=45.0,
                    help="abort if the sensor exceeds this C (default 45)")
    ap.add_argument("--supply-volts", type=float, default=12.0,
                    help="the board's VIN, used to work out what the heater "
                         "should measure and draw (ME3B V1 is 12 V)")
    ap.add_argument("--report-only", action="store_true",
                    help="report state only -- never command a heater")
    ap.add_argument("--yes", action="store_true",
                    help="skip the confirmation prompt")
    args = ap.parse_args()

    print()
    print("MEBP incubator heater diagnostic")
    print("=" * 68)
    print("Run this with the MEBP app CLOSED -- it holds the board's port.")
    print()

    scan_ports()

    print("Probing for the Marlin board:")
    found = find_marlin(args.port)
    if not found:
        print()
        print("*** No port answered as Marlin.")
        print("    - Is MEBP still running? Close it and retry.")
        print("    - Is the board's USB cable connected?")
        print("    - Try --port COMn explicitly against the port list above.")
        return 2
    port, _ident = found
    print(f"\nUsing {port}\n")

    link = Link(port)
    try:
        link.txn("M155 S0", timeout=3.0)     # no autoreport: clean replies
        state = report_state(link)

        if args.report_only:
            print("\n--report-only: no heater was commanded.")
            return 0

        bed_now = state.get("bed_c")
        if bed_now is None:
            print("\n*** Could not read the bed temperature -- stopping "
                  "before commanding any heat.")
            return 2

        target = args.target
        if target is None:
            target = int(min(40, round(bed_now) + 10))
        if target > 45:
            print(f"\n*** Refusing a {target} C target -- this rig holds "
                  f"37 C; anything above 45 is a mistake here.")
            return 2

        if not args.yes:
            print()
            print(f"About to command the BED heater to {target} C for "
                  f"{args.seconds}s.")
            print(f"The run aborts by itself above {args.abort_above:.0f} C, "
                  f"and the heater is")
            print("switched off on every exit path including Ctrl-C.")
            print()
            print("!! This is only safe while the thermistor is on the thing")
            print("   being heated. If the sensor is on the block and the film")
            print("   hangs loose in air, the readout cannot see the film --")
            print("   answer n and re-run with --seconds 0 (duty check only).")
            try:
                if input("Type y to continue: ").strip().lower() not in (
                        "y", "yes"):
                    print("Cancelled -- no heater was commanded.")
                    return 0
            except (EOFError, KeyboardInterrupt):
                print("\nCancelled.")
                return 0

        drive_test(link, target, max(0, args.seconds), args.abort_above,
                   args.supply_volts)
        return 0

    except KeyboardInterrupt:
        print("\n\nInterrupted.")
        return 130
    finally:
        # The heater goes off on EVERY exit path, and we verify the ack.
        try:
            off = link.txn("M140 S0", timeout=5.0)
            ok = any(ln.lower().startswith("ok") for ln in off)
            print(f"Heater off (M140 S0): "
                  f"{'acknowledged [OK]' if ok else off}")
        except Exception as e:
            print(f"!! COULD NOT CONFIRM HEATER OFF ({e}) -- "
                  f"power-cycle the board.")
        link.close()


if __name__ == "__main__":
    sys.exit(main())
