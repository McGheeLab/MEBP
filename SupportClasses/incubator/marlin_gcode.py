"""
marlin_gcode.py — pure Marlin line parsing + command building.

NO serial I/O, NO threads, NO Qt. Everything here is a free function or a
frozen dataclass so it can be unit-tested in isolation.

Design note — the temperature parser is deliberately GENERIC.
-------------------------------------------------------------
Marlin reports temperatures as whitespace-separated ``KEY:value`` tokens,
optionally with a ``/target`` suffix::

    ok T:24.31 /0.00 B:36.90 /37.00 @:0 B@:114
    T:24.31 /0.00 B:36.90 /37.00 @:0 B@:114        <- M155 autoreport push

Rather than hard-code which letters exist (which varies with what the
firmware was compiled with — chamber ``C:``, probe ``P:``, redundant ``R:``,
second hotend ``T1:``, board ``M:`` …), we tokenize and accept ANY key. The
caller decides which keys it cares about. This means:

  * a board with no hotend configured (no ``T:`` at all) parses fine,
  * extra sensors added later need no parser change,
  * we never silently mis-map a field we didn't anticipate.

Two token shapes matter:
  * ``KEY:value`` optionally followed by a separate ``/value`` token, or
    ``KEY:value/target`` fused into one token.
  * Power/duty tokens (``@:``, ``B@:``, ``C@:``) which are 0-255 PWM and
    never carry a target.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Iterable

# ═══════════════════════════════════════════════════════════════════
# Temperature frames
# ═══════════════════════════════════════════════════════════════════

#: Keys that carry a heater PWM duty, not a temperature.
POWER_KEYS = ("@", "B@", "C@", "L@")

#: Full scale for the ``@``/``B@`` duty fields — **127, not 255.**
#:
#: This catches people out (it caught us: the tool reported a heater flat out as
#: "50%"). Marlin's soft-PWM period is 127 ticks — ``if (pwm_count_tmp >= 127)``
#: in ``Temperature::isr()`` — and every writer halves the 0-255 control value to
#: match:
#:
#:     temp_bed.soft_pwm_amount = (int)get_pid_output_bed() >> 1;   // PID
#:     temp_bed.soft_pwm_amount = MAX_BED_POWER >> 1;               // bang-bang
#:
#: ``M105`` reports ``soft_pwm_amount`` verbatim via ``getHeaterPower()``. The
#: bang-bang line is the decisive proof: that branch means "heater fully on", and
#: with the stock ``MAX_BED_POWER 255`` it writes **127**. So ``B@:127`` is 100 %
#: duty, and dividing by 255 understates every duty figure by exactly half.
#:
#: Verified against Marlin 2.1.2.8 ``src/module/temperature.cpp`` (lines 1644,
#: 1652, 1654, 3575); the same ``>> 1`` / 127-tick scheme is long-standing, so it
#: holds for the bugfix-2.0.x build on the bench too.
HEATER_PWM_FULL_SCALE = 127.0

#: Keys that appear in temperature-shaped output but are NOT sensors.
#: Marlin's M109/M190 wait output looks like ``T:24.5 E:0 W:?`` — ``E`` is the
#: extruder index and ``W`` a residency countdown. Capturing them would
#: invent phantom sensor channels.
IGNORED_KEYS = ("E", "W")

#: Conventional Marlin field letters, for labelling only. The parser does
#: NOT depend on this table being complete or correct — unknown keys are
#: still captured, they just get a generic label.
FIELD_LABELS = {
    "T": "Hotend",
    "T0": "Hotend 0",
    "T1": "Hotend 1",
    "T2": "Hotend 2",
    "B": "Bed",
    "C": "Chamber",
    "P": "Probe",
    "R": "Redundant",
    "M": "Board",
    "L": "Cooler",
    "A": "Ambient",
}

_TOKEN_RE = re.compile(
    r"""^
    (?P<key>[A-Za-z][A-Za-z0-9]*@?|@)   # T, T0, B, B@, @, C, R ...
    :
    (?P<val>-?\d+(?:\.\d+)?)            # 36.90
    (?:\s*/\s*(?P<tgt>-?\d+(?:\.\d+)?))?  # optional fused /37.00
    $""",
    re.VERBOSE,
)

_BARE_TARGET_RE = re.compile(r"^/\s*(-?\d+(?:\.\d+)?)$")


@dataclass(frozen=True)
class SensorField:
    """One ``KEY:value[/target]`` field from a temperature line."""

    key: str
    value: float
    target: float | None = None
    #: True when this field is a heater PWM duty rather than a temperature.
    #: Full scale is :data:`HEATER_PWM_FULL_SCALE` (127), not 255.
    is_power: bool = False

    @property
    def label(self) -> str:
        return FIELD_LABELS.get(self.key, self.key)

    @property
    def power_pct(self) -> float | None:
        """
        Duty as a percentage, for power fields only.

        Scaled against :data:`HEATER_PWM_FULL_SCALE` — see the note there for why
        that is 127. This is the single place duty is converted, so it feeds the
        duty bar, the trend plot and the steady-state-duty heater-adequacy metric
        alike.
        """
        if not self.is_power:
            return None
        return max(0.0, min(100.0, self.value / HEATER_PWM_FULL_SCALE * 100.0))


@dataclass(frozen=True)
class TempFrame:
    """A parsed temperature report — any number of fields, all optional."""

    fields: dict[str, SensorField] = field(default_factory=dict)
    raw_line: str = ""

    def __bool__(self) -> bool:
        return bool(self.fields)

    def temp(self, key: str) -> float | None:
        f = self.fields.get(key)
        return None if f is None or f.is_power else f.value

    def target(self, key: str) -> float | None:
        f = self.fields.get(key)
        return None if f is None else f.target

    def power_pct(self, key: str) -> float | None:
        f = self.fields.get(key)
        return None if f is None else f.power_pct

    def temperature_keys(self) -> list[str]:
        """Field keys that represent temperatures (excludes PWM fields)."""
        return [k for k, f in self.fields.items() if not f.is_power]


def parse_temp_line(line: str) -> TempFrame | None:
    """
    Parse a Marlin temperature report into a :class:`TempFrame`.

    Accepts both the ``ok``-prefixed M105 reply and the bare M155 autoreport
    push. Returns ``None`` if the line contains no recognizable temperature
    field at all (so it can be used as a cheap "is this a temp line?" test).

    Tolerates: missing hotend fields, unknown/extra sensor letters, targets
    fused (``B:36.9/37.0``) or split (``B:36.9 /37.0``) across tokens.
    """
    if not line:
        return None

    text = line.strip()
    if not text:
        return None

    # Strip a leading 'ok' so M105 replies and autoreport pushes are identical
    # from here on.
    low = text.lower()
    if low == "ok":
        return None
    if low.startswith("ok "):
        text = text[3:].strip()

    fields: dict[str, SensorField] = {}
    last_key: str | None = None

    for token in text.split():
        m = _TOKEN_RE.match(token)
        if m:
            key = m.group("key")
            if key.upper() in IGNORED_KEYS:
                last_key = None
                continue
            val = float(m.group("val"))
            tgt_s = m.group("tgt")
            is_power = key in POWER_KEYS or key.endswith("@")
            fields[key] = SensorField(
                key=key,
                value=val,
                target=float(tgt_s) if tgt_s is not None else None,
                is_power=is_power,
            )
            last_key = key
            continue

        # A bare "/37.00" token belongs to the preceding key.
        bt = _BARE_TARGET_RE.match(token)
        if bt and last_key is not None:
            prev = fields[last_key]
            if not prev.is_power and prev.target is None:
                fields[last_key] = SensorField(
                    key=prev.key,
                    value=prev.value,
                    target=float(bt.group(1)),
                    is_power=prev.is_power,
                )
            continue

        # Anything else (stray words) is ignored — a temp line can be
        # embedded in a longer status string.

    if not fields:
        return None
    # Require at least one actual temperature; a line of only PWM tokens is
    # not a temperature report.
    if all(f.is_power for f in fields.values()):
        return None

    return TempFrame(fields=fields, raw_line=line.rstrip())


# ═══════════════════════════════════════════════════════════════════
# Line classification
# ═══════════════════════════════════════════════════════════════════

class LineKind(Enum):
    OK = auto()
    ERROR = auto()
    BUSY = auto()
    RESET = auto()
    TEMP = auto()
    AUTOTUNE_PROGRESS = auto()
    AUTOTUNE_RESULT = auto()
    AUTOTUNE_DONE = auto()
    AUTOTUNE_FAILED = auto()
    FIRMWARE = auto()
    CAPABILITY = auto()
    PID_DUMP = auto()
    ECHO = auto()
    OTHER = auto()


#: Substrings that mean the board rebooted (position/state lost).
RESET_MARKERS = ("start", "firmware_name", "marlin ready")

#: Substrings that mean a heater fault / the board has been KILLED. After any
#: of these Marlin stops acknowledging G-code and needs a physical reset.
FAULT_MARKERS = (
    "thermal runaway",
    "heating failed",
    "mintemp",
    "maxtemp",
    "thermal protection",
    "kill() called",
    "printer halted",
    "temp sensor",
)


def classify_line(line: str) -> LineKind:
    """
    Classify one inbound line. Order matters: fault/error detection wins over
    everything, then the ok-handshake, then content sniffing.
    """
    if line is None:
        return LineKind.OTHER
    text = line.strip()
    if not text:
        return LineKind.OTHER
    low = text.lower()

    # Faults are reported as "Error:..." but also sometimes bare.
    if low.startswith("error") or any(m in low for m in FAULT_MARKERS):
        return LineKind.ERROR

    if low == "ok" or low.startswith("ok "):
        # An M105 reply is "ok T:.. B:..", which is BOTH an ok and a temp
        # line. The reader forwards it to the temp parser separately; for
        # handshake purposes it is an OK.
        return LineKind.OK

    if "busy" in low:
        return LineKind.BUSY

    if low.startswith("cap:"):
        return LineKind.CAPABILITY

    if "firmware_name" in low:
        return LineKind.FIRMWARE

    # A bare "start" banner means the board reset mid-session.
    if low == "start" or low.startswith("marlin ready"):
        return LineKind.RESET

    if "pid autotune failed" in low:
        return LineKind.AUTOTUNE_FAILED
    if "pid autotune finished" in low:
        return LineKind.AUTOTUNE_DONE
    # Covers both "DEFAULT_bedKp" (2.0.x) and "DEFAULT_BED_KP" (2.1.x).
    if "#define default_" in low and ("kp" in low or "ki" in low or "kd" in low):
        return LineKind.AUTOTUNE_RESULT
    if low.startswith("bias:") or "ku:" in low or "classic pid" in low:
        return LineKind.AUTOTUNE_PROGRESS
    if _PID_DUMP_RE.search(text):
        return LineKind.PID_DUMP

    if parse_temp_line(text) is not None:
        return LineKind.TEMP

    if low.startswith("echo:"):
        return LineKind.ECHO

    return LineKind.OTHER


def is_terminal(kind: LineKind) -> bool:
    """True for line kinds that END an in-flight command transaction."""
    return kind in (LineKind.OK, LineKind.ERROR, LineKind.RESET)


# ═══════════════════════════════════════════════════════════════════
# Faults
# ═══════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class Fault:
    """A latched heater fault parsed from an Error: line."""

    kind: str            # "thermal_runaway" | "mintemp" | "maxtemp" | "killed" | "error"
    heater_id: str | None  # "bed" / "E0" / None
    message: str
    #: Operator-facing explanation of the most likely physical cause.
    hint: str = ""


_HEATER_ID_RE = re.compile(r"heater[_ ]?id\s*:?\s*([A-Za-z0-9_-]+)", re.IGNORECASE)


def parse_fault(line: str) -> Fault | None:
    """Extract a :class:`Fault` from an error line, or ``None``."""
    if not line:
        return None
    text = line.strip()
    low = text.lower()

    if not (low.startswith("error") or any(m in low for m in FAULT_MARKERS)):
        return None

    hid_m = _HEATER_ID_RE.search(text)
    heater_id = hid_m.group(1) if hid_m else None

    if "thermal runaway" in low:
        kind = "thermal_runaway"
        hint = (
            "Marlin decided the heater is not responding as commanded. On a "
            "large water-filled block a LOW-POWER heater legitimately cannot "
            "rise 2 C in 60 s, so this is most likely a FALSE trip from the "
            "stock WATCH_BED_TEMP_PERIOD / WATCH_TEMP_PERIOD windows. See "
            "FIRMWARE_NOTES.md."
        )
    elif "heating failed" in low:
        kind = "thermal_runaway"
        hint = (
            "The heater did not reach the commanded rise within the firmware's "
            "watch window. Same likely cause as thermal runaway on this rig — "
            "widen WATCH_*_TEMP_PERIOD / _INCREASE."
        )
    elif "mintemp" in low or "temp sensor" in low:
        kind = "mintemp"
        hint = (
            "Reads below MINTEMP. The usual cause is a DISCONNECTED or broken "
            "thermistor (an open circuit reads as very cold), or a MINTEMP set "
            "above room temperature."
        )
    elif "maxtemp" in low:
        kind = "maxtemp"
        hint = (
            "Reads above MAXTEMP. Check for a shorted thermistor, the wrong "
            "TEMP_SENSOR_* table, or genuine overheating."
        )
    elif "kill() called" in low or "printer halted" in low:
        kind = "killed"
        hint = "The board has halted. It will ignore all G-code until power-cycled."
    else:
        kind = "error"
        hint = ""

    return Fault(kind=kind, heater_id=heater_id, message=text, hint=hint)


# ═══════════════════════════════════════════════════════════════════
# M503 settings dump — PID readback
# ═══════════════════════════════════════════════════════════════════

# Matches the command word of "echo:  M304 P97.10 I1.41 D1675.16" (bed) or the
# M301 hotend variant. The P/I/D values are pulled out separately so the regex
# stays trivially readable and order-independent.
_PID_DUMP_RE = re.compile(r"\bM(?P<cmd>30[14])\b", re.IGNORECASE)
_PID_P_RE = re.compile(r"\bP(-?\d+(?:\.\d+)?)")
_PID_I_RE = re.compile(r"\bI(-?\d+(?:\.\d+)?)")
_PID_D_RE = re.compile(r"\bD(-?\d+(?:\.\d+)?)")


@dataclass(frozen=True)
class PidValues:
    kp: float
    ki: float
    kd: float

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.kp, self.ki, self.kd)


def parse_pid_dump(lines: Iterable[str], command: str) -> PidValues | None:
    """
    Scan an M503 dump for the PID line of a given command (``"M304"`` for the
    bed, ``"M301"`` for hotend 0).

    Returns ``None`` when the line is ABSENT — which is the meaningful signal
    that PID is not compiled in for that heater (``PIDTEMPBED`` disabled ⇒ the
    bed runs bang-bang). Callers must surface that as "unavailable" and must
    NOT substitute zeros.
    """
    want = command.upper().lstrip("M")
    for line in lines or ():
        if not line:
            continue
        m = _PID_DUMP_RE.search(line)
        if not m or m.group("cmd") != want:
            continue
        # Only look at the text AFTER the command word, so a stray P/I/D
        # earlier in the line (e.g. an "echo:" prefix) cannot be picked up.
        tail = line[m.end():]
        p = _PID_P_RE.search(tail)
        i = _PID_I_RE.search(tail)
        d = _PID_D_RE.search(tail)
        if not (p and i and d):
            continue
        return PidValues(kp=float(p.group(1)), ki=float(i.group(1)),
                         kd=float(d.group(1)))
    return None


# ═══════════════════════════════════════════════════════════════════
# PID autotune
# ═══════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class AutotuneProgress:
    """One parsed progress event from a running M303."""

    cycle: int | None = None
    bias: float | None = None
    d: float | None = None
    t_min: float | None = None
    t_max: float | None = None
    ku: float | None = None
    tu: float | None = None
    kp: float | None = None
    ki: float | None = None
    kd: float | None = None
    note: str = ""


_NUM = r"(-?\d+(?:\.\d+)?)"
_BIAS_RE = re.compile(rf"bias:\s*{_NUM}", re.IGNORECASE)
_D_RE = re.compile(rf"\bd:\s*{_NUM}", re.IGNORECASE)
_MIN_RE = re.compile(rf"\bmin:\s*{_NUM}", re.IGNORECASE)
_MAX_RE = re.compile(rf"\bmax:\s*{_NUM}", re.IGNORECASE)
_KU_RE = re.compile(rf"\bKu:\s*{_NUM}", re.IGNORECASE)
_TU_RE = re.compile(rf"\bTu:\s*{_NUM}", re.IGNORECASE)
_KP_RE = re.compile(rf"\bKp:\s*{_NUM}")
_KI_RE = re.compile(rf"\bKi:\s*{_NUM}")
_KD_RE = re.compile(rf"\bKd:\s*{_NUM}")


def _f(rx: re.Pattern[str], text: str) -> float | None:
    m = rx.search(text)
    return float(m.group(1)) if m else None


def parse_autotune_progress(line: str) -> AutotuneProgress | None:
    """
    Parse a mid-run M303 line. Marlin emits, roughly once per cycle::

        bias: 118 d: 118 min: 35.62 max: 38.41
        Ku: 12.34 Tu: 45.67
        Classic PID
        Kp: 24.50 Ki: 1.07 Kd: 139.83

    Returns ``None`` for lines that carry no progress information.
    """
    if not line:
        return None
    text = line.strip()
    low = text.lower()

    if low.startswith("echo:"):
        text = text[5:].strip()
        low = text.lower()

    has_bias = "bias:" in low
    has_ku = "ku:" in low
    has_k = bool(_KP_RE.search(text) or _KI_RE.search(text) or _KD_RE.search(text))
    is_classic = "classic pid" in low

    if not (has_bias or has_ku or has_k or is_classic):
        return None

    return AutotuneProgress(
        bias=_f(_BIAS_RE, text),
        d=_f(_D_RE, text),
        t_min=_f(_MIN_RE, text),
        t_max=_f(_MAX_RE, text),
        ku=_f(_KU_RE, text),
        tu=_f(_TU_RE, text),
        kp=_f(_KP_RE, text),
        ki=_f(_KI_RE, text),
        kd=_f(_KD_RE, text),
        note="Classic PID" if is_classic else "",
    )


# Marlin renamed these macros between versions, and the autotune output prints
# whatever the current names are. Both spellings must be accepted or results are
# silently dropped after a firmware upgrade:
#   Marlin 2.0.x : "#define DEFAULT_bedKp 97.10"   /  "#define DEFAULT_Kp 24.50"
#   Marlin 2.1.x : "#define DEFAULT_BED_KP 97.10"  /  "#define DEFAULT_KP 24.50"
#                  (and DEFAULT_CHAMBER_KP for a chamber heater)
_RESULT_RE = re.compile(
    r"#define\s+DEFAULT_"
    r"(?:(?P<heater>BED|CHAMBER|bed|chamber)_?)?"
    r"K(?P<which>[pid])"
    r"\s+(?P<val>-?\d+(?:\.\d+)?)",
    re.IGNORECASE,
)


def parse_autotune_result(line: str) -> tuple[str, str, float] | None:
    """
    Parse a final autotune constant line, in either Marlin naming style::

        echo: #define DEFAULT_bedKp 97.10      -> ("bed", "p", 97.10)   [2.0.x]
        echo: #define DEFAULT_BED_KP 97.10     -> ("bed", "p", 97.10)   [2.1.x]
        echo: #define DEFAULT_Kp 24.50         -> ("hotend", "p", 24.50)
        echo: #define DEFAULT_KP 24.50         -> ("hotend", "p", 24.50)

    Returns ``(heater, which, value)`` or ``None``. ``heater`` is one of
    ``"bed"``, ``"chamber"`` or ``"hotend"``.
    """
    if not line:
        return None
    m = _RESULT_RE.search(line)
    if not m:
        return None
    grp = (m.group("heater") or "").lower()
    heater = grp if grp in ("bed", "chamber") else "hotend"
    return (heater, m.group("which").lower(), float(m.group("val")))


def parse_autotune_failure(line: str) -> str | None:
    """
    Extract the reason from a ``PID Autotune failed! <reason>`` line.

    Known reasons include ``Bad extruder number`` (the heater has no PID
    compiled in — e.g. ``PIDTEMPBED`` disabled), ``Temperature too high``,
    and ``timeout`` (very possible on a large thermal mass, since Marlin
    caps a cycle at MAX_CYCLE_TIME_PID_AUTOTUNE).
    """
    if not line:
        return None
    low = line.lower()
    idx = low.find("pid autotune failed")
    if idx < 0:
        return None
    tail = line[idx + len("pid autotune failed"):].lstrip("!:. ").strip()
    return tail or "unknown reason"


# ═══════════════════════════════════════════════════════════════════
# M115 identity + capabilities
# ═══════════════════════════════════════════════════════════════════

def parse_firmware_name(lines: Iterable[str]) -> str | None:
    """Pull the FIRMWARE_NAME value out of an M115 reply."""
    for line in lines or ():
        if not line:
            continue
        low = line.lower()
        if "firmware_name" not in low:
            continue
        idx = low.find("firmware_name")
        tail = line[idx + len("firmware_name"):].lstrip(": ")
        # Trim at the next KEY:VALUE pair (e.g. " SOURCE_CODE_URL:...").
        cut = re.split(r"\s+[A-Z_]{3,}:", tail, maxsplit=1)
        return cut[0].strip() or None
    return None


def parse_capabilities(lines: Iterable[str]) -> dict[str, bool]:
    """
    Parse ``Cap:NAME:0|1`` lines from an extended M115 reply.

    Returns an empty dict when the firmware was built without
    EXTENDED_CAPABILITIES_REPORT — absence of a capability line is NOT the
    same as the capability being off, so callers should treat a missing key
    as "unknown" and probe behaviourally instead.
    """
    caps: dict[str, bool] = {}
    for line in lines or ():
        if not line:
            continue
        text = line.strip()
        if not text.lower().startswith("cap:"):
            continue
        body = text[4:]
        if ":" not in body:
            continue
        name, _, val = body.rpartition(":")
        name = name.strip().upper()
        if name:
            caps[name] = val.strip() == "1"
    return caps


# ═══════════════════════════════════════════════════════════════════
# Command builders
# ═══════════════════════════════════════════════════════════════════

def build_autoreport(interval_s: int) -> str:
    """M155 — enable periodic temperature autoreport (0 disables)."""
    return f"M155 S{max(0, min(255, int(interval_s)))}"


def build_autotune(*, extruder: int, target_c: float, cycles: int,
                   apply_result: bool) -> str:
    """
    M303 — PID autotune. ``extruder=-1`` selects the bed, ``0`` hotend 0.
    ``apply_result`` sets U1, which loads the result into live RAM (still
    needs M500 to persist).
    """
    return (
        f"M303 E{int(extruder)} S{float(target_c):.0f} "
        f"C{max(1, int(cycles))} U{1 if apply_result else 0}"
    )
