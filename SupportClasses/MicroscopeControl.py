"""
MicroscopeControl.py — motorized microscope body control (Nikon Ti Eclipse).

v7.5.x: the rig gains a Nikon Ti Eclipse whose **filter-cube cassette**,
**nosepiece (objective turret)** and **focus drive (Z)** are motorized. This
module is the GUI-free backend for driving those three devices manually. It is
deliberately standalone — no print/workflow/calibration code calls into it yet
(integration into the workflows and calibrations is a later, separate step).

v7.17 adds three more devices the body exposes and nothing was driving — the
**epi (excitation) shutter**, the **transmitted-light (dia) lamp** and the
**light-path drive** (eyepiece ↔ camera port) — plus the one piece of *behaviour*
in this module: an optional **shutter↔cassette interlock** that closes the
excitation shutter for the duration of a filter-cube rotation and then restores
it. Rotating the cassette with the shutter open sweeps the excitation beam across
every cube that passes, flashing the sample with out-of-band excitation. See
:meth:`MicroscopeController._with_shutter_closed`. All three are **accessories**:
a body without one is normal, so every getter degrades to "not fitted" rather
than raising.

Two layers:

* :class:`MicroscopeBackend` — the driver contract (connect, read/set turret
  position, read/set focus). Three implementations ship:

  - :class:`SimulatedMicroscopeBackend` — always available; models turret
    positions and a focus axis so the whole UI is exercisable with no hardware.
  - :class:`NikonTiSdkBackend` — the Nikon Ti SDK's COM automation object
    (``Nikon.LvMic.NikonTi``) via ``comtypes``. **The device/property mapping is
    written defensively and is UNVERIFIED against real hardware** — see
    :meth:`NikonTiSdkBackend.diagnostics`, which dumps what the COM object
    actually exposes so the mapping can be finished at the bench in one pass.
  - :class:`MicroManagerBackend` — drives the body through Micro-Manager's
    battle-tested ``NikonTI`` device adapter via ``pymmcore``. Recommended when
    a Micro-Manager install is available: its focus device is µm-native, so
    there is no device-unit scaling to get wrong.

* :class:`MicroscopeController` — the thread-safe facade the GUI uses. **Every
  backend call runs on ONE dedicated worker thread**, for two reasons:
  (1) COM objects have apartment affinity — the object must be used on the
  thread that created it; and (2) a turret rotation is a multi-hundred-ms
  blocking call, and this codebase has repeatedly had to move exactly that kind
  of call off the Qt event loop (see the jog/pump/stage "GUI freeze" fixes).
  Callers submit operations and read a cached :class:`MicroscopeState`
  snapshot, so rendering never blocks on hardware.

Turret positions are **1-based** throughout, matching the numbers engraved on
the physical turret. Focus is **µm** throughout, with each backend responsible
for converting its own device units.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass, field, replace
from typing import Callable, Optional

logger = logging.getLogger(__name__)


class MicroscopeError(RuntimeError):
    """A backend-level failure: not connected, SDK missing, move refused."""


# ── State snapshot ─────────────────────────────────────────────────

@dataclass(frozen=True)
class MountedOptic:
    """What is physically fitted in one turret slot.

    A motorised Nikon body knows its own optics: the cassette reports the cube
    name ("DAPI"), and the nosepiece reports each objective's product code,
    magnification, NA and working distance. Reading that beats making the
    operator type it — and it cannot drift out of date.
    """

    position: int
    present: bool = False
    #: Short label for a combo box, e.g. "DAPI" or "4x".
    label: str = ""
    #: Vendor identifier, e.g. "MRH20040".
    code: str = ""
    #: Longer one-line description for a setup table.
    detail: str = ""

    # ── numerics, kept as NUMBERS ────────────────────────────────────
    # These used to be read from the SDK, formatted into ``detail`` and then
    # thrown away. Anything that has to SIZE something from the optics — the
    # depth of field that sets a focus step, the working distance that bounds
    # how far a focus sweep may travel before the front lens reaches the plate —
    # needs them as floats. Re-parsing them back out of ``detail`` would fail
    # silently into a wrong step size or a wrong collision bound, so the numbers
    # are carried directly. ``None`` means "the body did not report it", which
    # callers must treat as unknown rather than substituting a default.
    magnification: float | None = None
    numerical_aperture: float | None = None
    working_distance_mm: float | None = None


@dataclass(frozen=True)
class MicroscopeState:
    """Immutable snapshot of what we last knew about the microscope body.

    The GUI renders from this, never from a live hardware read, so a slow or
    wedged turret can't stall a paint.
    """

    backend: str = "none"
    connected: bool = False
    busy: bool = False
    #: Filter cube cassette.
    filter_position: Optional[int] = None
    filter_count: int = 0
    #: Nosepiece / objective turret.
    objective_position: Optional[int] = None
    objective_count: int = 0
    #: Focus drive, µm in the backend's own absolute frame.
    focus_um: Optional[float] = None
    focus_min_um: Optional[float] = None
    focus_max_um: Optional[float] = None
    # ── v7.17: illumination + light path ────────────────────────────
    # Each ``*_present`` says the body actually has that accessory; the value
    # beside it is ``None`` for "fitted but not readable right now", which is a
    # different thing and must not render as a state.
    #
    # These are POLLED, unlike the mounted optics, because the Ti-E has physical
    # buttons and a lamp knob ON THE BODY — the operator can change any of them
    # without the software, and polling is the only way the panel tells the truth.
    epi_shutter_present: bool = False
    #: True = open (excitation reaching the sample). Already corrected for the
    #: operator's ``epi_shutter_invert`` override.
    epi_shutter_open: Optional[bool] = None
    dia_lamp_present: bool = False
    dia_lamp_on: Optional[bool] = None
    #: Is the lamp under software (Remote) control? ``False`` = the body's own
    #: front-panel knob owns it and the SDK refuses every write. ``None`` = the
    #: driver cannot tell, so don't gate anything on it.
    dia_lamp_remote: Optional[bool] = None
    #: Lamp level in the DEVICE'S OWN units, with the range the device declares.
    #: Deliberately not rescaled to a percentage — see set_dia_lamp_intensity.
    dia_lamp_intensity: Optional[float] = None
    dia_lamp_min: Optional[float] = None
    dia_lamp_max: Optional[float] = None
    #: Eyepiece / camera-port selector, 1-based like the turrets.
    light_path_position: Optional[int] = None
    light_path_count: int = 0
    native_light_path_names: tuple = ()
    #: Names the hardware itself reports, when it reports any (Micro-Manager
    #: state labels, SDK objective info). Empty when unavailable — the operator
    #: assignments in MicroscopeConfigStore are the fallback and the override.
    native_filter_names: tuple = ()
    native_objective_names: tuple = ()
    #: What the body reports is physically fitted in each slot. Populated on
    #: connect and by refresh_mounted() — optics change only when someone
    #: physically swaps them, so this is not on the ~1 s poll.
    mounted_filters: tuple = ()
    mounted_objectives: tuple = ()
    #: Last error text, cleared by the next successful operation.
    error: Optional[str] = None
    last_op: Optional[str] = None

    @property
    def has_filter(self) -> bool:
        return self.filter_count > 0

    @property
    def has_objective(self) -> bool:
        return self.objective_count > 0

    @property
    def has_focus(self) -> bool:
        return self.focus_um is not None

    @property
    def has_light_path(self) -> bool:
        return self.light_path_count > 0


# ── Backend contract ───────────────────────────────────────────────

class MicroscopeBackend:
    """Driver contract. Every method may raise :class:`MicroscopeError`.

    Implementations are used from a single thread (the controller's worker), so
    they need no internal locking.
    """

    name = "base"
    display_name = "Base"

    # -- lifecycle --
    def connect(self) -> None:
        raise NotImplementedError

    def disconnect(self) -> None:
        return None

    @property
    def is_connected(self) -> bool:
        return False

    # -- filter cube cassette --
    def filter_count(self) -> int:
        return 0

    def get_filter(self) -> Optional[int]:
        return None

    def set_filter(self, position: int) -> None:
        raise MicroscopeError("this microscope has no motorized filter turret")

    def filter_names(self) -> tuple:
        return ()

    # -- nosepiece --
    def objective_count(self) -> int:
        return 0

    def get_objective(self) -> Optional[int]:
        return None

    def set_objective(self, position: int) -> None:
        raise MicroscopeError("this microscope has no motorized nosepiece")

    def objective_names(self) -> tuple:
        return ()

    # -- what is physically fitted (optional; () = the driver can't tell) --
    def mounted_filters(self) -> tuple:
        """``MountedOptic`` per cassette slot, or ``()`` if unsupported."""
        return ()

    def mounted_objectives(self) -> tuple:
        """``MountedOptic`` per nosepiece position, or ``()`` if unsupported."""
        return ()

    # -- focus --
    def get_focus_um(self) -> Optional[float]:
        return None

    def set_focus_um(self, value_um: float) -> None:
        raise MicroscopeError("this microscope has no motorized focus")

    def move_focus_um(self, delta_um: float) -> None:
        """Relative focus move. Default: read-modify-write in absolute µm."""
        current = self.get_focus_um()
        if current is None:
            raise MicroscopeError("focus position is unknown")
        self.set_focus_um(current + float(delta_um))

    def focus_limits_um(self) -> Optional[tuple]:
        """Hardware travel limits ``(min, max)`` in µm, or ``None`` if unknown."""
        return None

    # -- epi (excitation) shutter --
    #
    # v7.17. All three device groups below are ACCESSORIES: a body without one is
    # entirely normal, so the default is "not fitted" and the ``has_*`` probe is
    # what callers gate on. Raising from a setter that was never advertised keeps
    # a mis-wired UI loud instead of silently doing nothing.

    def has_epi_shutter(self) -> bool:
        return False

    def epi_shutter_open(self) -> Optional[bool]:
        """``True`` = open. ``None`` = fitted but unreadable, or not fitted."""
        return None

    def set_epi_shutter(self, open_: bool) -> None:
        raise MicroscopeError("this microscope has no motorized epi shutter")

    # -- transmitted-light (diascopic) lamp --

    def has_dia_lamp(self) -> bool:
        return False

    def dia_lamp_on(self) -> Optional[bool]:
        return None

    def set_dia_lamp_on(self, on: bool) -> None:
        raise MicroscopeError("this microscope has no controllable dia lamp")

    def dia_lamp_intensity(self) -> Optional[float]:
        """Lamp level in the device's own units, or ``None``."""
        return None

    def set_dia_lamp_intensity(self, value: float) -> None:
        raise MicroscopeError("this microscope's dia lamp is not dimmable")

    def dia_lamp_intensity_range(self) -> Optional[tuple]:
        """``(min, max)`` the device declares for its level, else ``None``."""
        return None

    def dia_lamp_remote(self) -> Optional[bool]:
        """Is the lamp under SOFTWARE control? ``None`` = the driver can't tell.

        ⚠ Hardware-measured on a Ti-E: the dia lamp has two modes and the SDK
        **refuses every write in the wrong one** — ``IsControlled = 0
        ('MainMode')`` means the body's own front-panel knob owns the lamp, and
        level/switch writes come back ``0xE01004BB`` / ``0xE01004BE``. ``1
        ('RemoteMode')`` accepts them.
        """
        return None

    def set_dia_lamp_remote(self, on: bool) -> None:
        """Take/release software control of the lamp.

        Deliberately a separate, explicit operation rather than something the
        setters do for you: switching to RemoteMode takes the lamp away from the
        knob on the microscope, which changes what the person standing at the
        body can do. That is theirs to choose, not a side effect of moving a
        slider.
        """
        raise MicroscopeError(
            "this microscope's dia lamp has no software/manual mode switch")

    # -- light path (eyepiece / camera port selector) --

    def light_path_count(self) -> int:
        return 0

    def get_light_path(self) -> Optional[int]:
        return None

    def set_light_path(self, position: int) -> None:
        raise MicroscopeError("this microscope has no motorized light path")

    def light_path_names(self) -> tuple:
        return ()

    # -- diagnostics --
    def diagnostics(self) -> str:
        return f"{self.display_name}: no diagnostics available."


# ── Simulated backend ──────────────────────────────────────────────

class SimulatedMicroscopeBackend(MicroscopeBackend):
    """No-hardware stand-in: turrets and a focus axis held in memory.

    Moves are modelled as instantaneous. This is what runs when no microscope is
    attached, so the whole UI (assignment, switching, focus jog) is exercisable
    and testable without the scope.
    """

    name = "simulated"
    display_name = "Simulated microscope"

    #: Plausible stand-in names for a Ti light-path selector.
    _LIGHT_PATHS = ("Eyepiece", "Left port", "Right port", "Bottom port")

    def __init__(self, filter_slots: int = 6, objective_slots: int = 6,
                 focus_um: float = 5000.0,
                 focus_range_um: tuple = (0.0, 10000.0), *,
                 epi_shutter: bool = True, dia_lamp: bool = True,
                 light_path_slots: int = 4, dia_lamp_remote: bool = True):
        self._filter_slots = max(1, int(filter_slots))
        self._objective_slots = max(1, int(objective_slots))
        self._filter_pos = 1
        self._objective_pos = 1
        self._focus_um = float(focus_um)
        self._focus_range = (float(focus_range_um[0]), float(focus_range_um[1]))
        self._connected = False
        # v7.17 accessories. Each can be switched off at construction so the
        # "body without this device" path is testable, which is the case a
        # simulator that always has everything would hide.
        self._has_epi = bool(epi_shutter)
        self._has_lamp = bool(dia_lamp)
        self._light_path_slots = max(0, int(light_path_slots))
        # Closed / off at rest: that is the state an excitation shutter and a
        # lamp should be found in, and it means a freshly-connected simulator is
        # not modelling light on the sample.
        self._epi_open = False
        self._lamp_on = False
        self._lamp_level = 50.0
        self._lamp_range = (0.0, 100.0)
        self._light_path = 1
        # Modelled because the real body has it and REFUSES every lamp write in
        # MainMode: a simulator that always accepts them would leave that whole
        # branch untested (and it is the one the operator hits first).
        self._lamp_remote = bool(dia_lamp_remote)

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def _require(self) -> None:
        if not self._connected:
            raise MicroscopeError("microscope is not connected")

    # filter
    def filter_count(self) -> int:
        return self._filter_slots

    def get_filter(self) -> Optional[int]:
        return self._filter_pos if self._connected else None

    def set_filter(self, position: int) -> None:
        self._require()
        pos = int(position)
        if not 1 <= pos <= self._filter_slots:
            raise MicroscopeError(
                f"filter slot {pos} out of range 1-{self._filter_slots}")
        self._filter_pos = pos

    # nosepiece
    def objective_count(self) -> int:
        return self._objective_slots

    def get_objective(self) -> Optional[int]:
        return self._objective_pos if self._connected else None

    def set_objective(self, position: int) -> None:
        self._require()
        pos = int(position)
        if not 1 <= pos <= self._objective_slots:
            raise MicroscopeError(
                f"objective position {pos} out of range 1-{self._objective_slots}")
        self._objective_pos = pos

    # focus
    def get_focus_um(self) -> Optional[float]:
        return self._focus_um if self._connected else None

    def set_focus_um(self, value_um: float) -> None:
        self._require()
        lo, hi = self._focus_range
        self._focus_um = max(lo, min(hi, float(value_um)))

    def focus_limits_um(self) -> Optional[tuple]:
        return self._focus_range

    # v7.17 accessories

    def has_epi_shutter(self) -> bool:
        return self._has_epi

    def epi_shutter_open(self) -> Optional[bool]:
        if not (self._connected and self._has_epi):
            return None
        return self._epi_open

    def set_epi_shutter(self, open_: bool) -> None:
        self._require()
        if not self._has_epi:
            raise MicroscopeError("this microscope has no motorized epi shutter")
        self._epi_open = bool(open_)

    def has_dia_lamp(self) -> bool:
        return self._has_lamp

    def dia_lamp_on(self) -> Optional[bool]:
        if not (self._connected and self._has_lamp):
            return None
        return self._lamp_on

    def set_dia_lamp_on(self, on: bool) -> None:
        self._require()
        if not self._has_lamp:
            raise MicroscopeError("this microscope has no controllable dia lamp")
        self._require_lamp_remote()
        self._lamp_on = bool(on)

    def dia_lamp_remote(self) -> Optional[bool]:
        if not (self._connected and self._has_lamp):
            return None
        return self._lamp_remote

    def set_dia_lamp_remote(self, on: bool) -> None:
        self._require()
        if not self._has_lamp:
            raise MicroscopeError("this microscope has no controllable dia lamp")
        self._lamp_remote = bool(on)

    def _require_lamp_remote(self) -> None:
        if not self._lamp_remote:
            raise MicroscopeError(
                "the dia lamp is in MainMode — the microscope's own front-panel "
                "control owns it and the SDK refuses software changes. Switch it "
                "to software (Remote) control first.")

    def dia_lamp_intensity(self) -> Optional[float]:
        if not (self._connected and self._has_lamp):
            return None
        return self._lamp_level

    def set_dia_lamp_intensity(self, value: float) -> None:
        self._require()
        if not self._has_lamp:
            raise MicroscopeError("this microscope's dia lamp is not dimmable")
        self._require_lamp_remote()
        lo, hi = self._lamp_range
        self._lamp_level = max(lo, min(hi, float(value)))

    def dia_lamp_intensity_range(self) -> Optional[tuple]:
        return self._lamp_range if self._has_lamp else None

    def light_path_count(self) -> int:
        return self._light_path_slots

    def get_light_path(self) -> Optional[int]:
        if not (self._connected and self._light_path_slots):
            return None
        return self._light_path

    def set_light_path(self, position: int) -> None:
        self._require()
        if not self._light_path_slots:
            raise MicroscopeError("this microscope has no motorized light path")
        pos = int(position)
        if not 1 <= pos <= self._light_path_slots:
            raise MicroscopeError(
                f"light path {pos} out of range 1-{self._light_path_slots}")
        self._light_path = pos

    def light_path_names(self) -> tuple:
        return tuple(
            self._LIGHT_PATHS[p - 1] if p <= len(self._LIGHT_PATHS)
            else f"port {p}"
            for p in range(1, self._light_path_slots + 1))

    def mounted_filters(self) -> tuple:
        """Plausible stand-in optics so the setup page is exercisable."""
        names = ("DAPI", "FITC", "TxRed", "Cy5")
        return tuple(
            MountedOptic(position=p, present=p <= len(names),
                         label=names[p - 1] if p <= len(names) else "",
                         code=f"SIM{p}" if p <= len(names) else "",
                         detail=("simulated filter block"
                                 if p <= len(names) else "empty"))
            for p in range(1, self._filter_slots + 1))

    def mounted_objectives(self) -> tuple:
        # (magnification, NA, working distance mm) for plausible Nikon dry
        # objectives. The NA and WD are what size a focus step and bound a
        # sweep, so the simulator must report them or the whole planning layer
        # is untestable without a body. The 20x's ~1 mm WD is deliberate: it is
        # the case that makes a naive ±1 mm sweep a collision.
        specs = (("4", 0.13, 16.4), ("10", 0.30, 16.0), ("20", 0.45, 1.0))
        out = []
        for p in range(1, self._objective_slots + 1):
            if p > len(specs):
                out.append(MountedOptic(position=p, present=False,
                                        detail="empty"))
                continue
            mag, na, wd = specs[p - 1]
            out.append(MountedOptic(
                position=p, present=True, label=f"{mag}x",
                code=f"SIMOBJ{p}",
                detail=f"simulated objective · NA {na:g} · WD {wd:g} mm",
                magnification=float(mag),
                numerical_aperture=na, working_distance_mm=wd))
        return tuple(out)

    def filter_names(self) -> tuple:
        return tuple(o.label for o in self.mounted_filters())

    def objective_names(self) -> tuple:
        return tuple(o.label for o in self.mounted_objectives())

    def diagnostics(self) -> str:
        return (f"{self.display_name}\n"
                f"  filter slots     : {self._filter_slots} (at {self._filter_pos})\n"
                f"  objective slots  : {self._objective_slots} "
                f"(at {self._objective_pos})\n"
                f"  focus            : {self._focus_um:.2f} µm "
                f"in {self._focus_range[0]:.0f}..{self._focus_range[1]:.0f}\n"
                f"  epi shutter      : "
                + (("open" if self._epi_open else "closed") if self._has_epi
                   else "not fitted") + "\n"
                f"  dia lamp         : "
                + (f"{'on' if self._lamp_on else 'off'} at {self._lamp_level:g} "
                   f"in {self._lamp_range[0]:g}..{self._lamp_range[1]:g}"
                   if self._has_lamp else "not fitted") + "\n"
                f"  light path       : "
                + (f"{self._light_path} of {self._light_path_slots}"
                   if self._light_path_slots else "not fitted"))


# ── Nikon Ti SDK (COM) backend ─────────────────────────────────────

#: ProgIDs tried in order when none is configured.
#:
#: ``Nikon.TiScope.NikonTi`` is CONFIRMED: it was read straight out of the COM
#: type information in ``NikonTi.dll`` v4.4.1.714 (the Ti SDK redistributable
#: ``TiSDKRedist64-4.4.1.714``, which self-registers that DLL). That build
#: publishes 61 ``Nikon.TiScope.*`` classes — including ``Nosepiece``,
#: ``FilterBlockCassette1`` and ``ZDrive``, matching the device aliases below —
#: and contains **no** ``LvMic`` string at all. So ``Nikon.LvMic.*`` (quoted in
#: various Micro-Manager-era notes) is NOT what a Ti-E class SDK registers; it
#: is kept only as a fallback for other/newer SDK generations.
TI_PROG_IDS = (
    "Nikon.TiScope.NikonTi",     # confirmed, Ti/Ti-E SDK 4.4.x
    "Nikon.LvMic.NikonTi",       # fallback: other SDK generations
    "Nikon.LvMic.NikonTi2",      # fallback: Ti2 bodies
)

#: Attribute names probed for each logical device, most-likely first.
#:
#: CONFIRMED against ``NikonTi.dll`` v4.4.1.714's published class list, which
#: includes ``Nikon.TiScope.Nosepiece``, ``Nikon.TiScope.FilterBlockCassette1``
#: and ``Nikon.TiScope.ZDrive`` — i.e. the first alias in each group below is
#: the SDK's own name. The remaining entries stay as cheap insurance for other
#: SDK generations.
_TI_DEVICE_ALIASES = {
    "filter": ("FilterBlockCassette{n}", "FilterBlockCassette",
               "FilterBlock", "FilterCassette"),
    "objective": ("Nosepiece", "NosePiece", "Objective"),
    "focus": ("ZDrive", "ZDrive1", "Focus", "FocusDrive"),
    # v7.17 accessories. ``EpiShutter``, ``DiaLamp`` and ``LightPathDrive`` are
    # all in the confirmed 61-class list published by NikonTi.dll v4.4.1.714 and
    # were seen on this body's own diagnostics dump, so the first alias in each
    # group is the SDK's own name. What is NOT yet verified is the *semantics* of
    # their values — see _shutter_codes and set_dia_lamp_intensity.
    "epi_shutter": ("EpiShutter", "EpiShutter1", "FluoShutter", "Shutter"),
    "dia_lamp": ("DiaLamp", "DiaLamp1", "TransmittedLamp", "Lamp"),
    "light_path": ("LightPathDrive", "LightPathDrive1", "LightPath"),
}

#: Devices whose absence means the connection is not usable. The rest are
#: accessories: a body without an epi shutter is perfectly normal, so failing to
#: find one is logged quietly and never warns on every connect.
_TI_CORE_DEVICES = frozenset({"filter", "objective", "focus"})


try:  # the exception comtypes raises for a failed COM call
    from _ctypes import COMError  # type: ignore
except Exception:  # pragma: no cover - non-Windows
    class COMError(Exception):  # type: ignore
        """Stand-in so the except clauses below are always valid."""


def _com_message(exc) -> str:
    """Readable text out of a ``COMError``.

    comtypes packs the useful part into ``details`` as
    ``(description, source, helpfile, helpcontext, scode)`` — the Nikon SDK puts
    its own diagnosis there (e.g. *"No available instruments."*), which is far
    more actionable than the HRESULT or a Python traceback.
    """
    details = tuple(getattr(exc, "details", None) or ())
    desc = str(details[0]).strip() if len(details) > 0 and details[0] else ""
    source = str(details[1]).strip() if len(details) > 1 and details[1] else ""
    hres = getattr(exc, "hresult", None)
    hr_txt = f"HRESULT 0x{hres & 0xFFFFFFFF:08X}" if isinstance(hres, int) else ""
    if desc:
        return f"{desc} [{source or hr_txt}]" if (source or hr_txt) else desc
    parts = [p for p in (source, hr_txt) if p]
    return ("the microscope SDK rejected the request ("
            + ", ".join(parts) + ")") if parts else "the microscope SDK rejected the request"


def _first_attr(obj, names):
    """Return the first present, non-None attribute from ``names``."""
    for name in names:
        try:
            value = getattr(obj, name, None)
        except Exception:  # COM property getters can raise
            continue
        if value is not None:
            return name, value
    return None, None


class NikonTiSdkBackend(MicroscopeBackend):
    """Nikon Ti body over the Nikon Ti SDK COM automation object.

    ⚠ **Unverified against hardware.** The Ti SDK exposes each device as a COM
    object with a ``Position`` property whose ``Value`` is the 1-based turret
    index; the Z drive's ``Position.Value`` is in device units (10 nm on a Ti,
    hence the default ``z_units_per_um=100``). Every access below probes several
    spellings and degrades to a clear error rather than guessing silently, and
    :meth:`diagnostics` dumps the live object model so the mapping can be
    confirmed — and corrected, if needed — in a single bench session.

    ``z_units_per_um`` is a per-machine setting (MicroscopeConfigStore) rather
    than a constant precisely because getting it wrong scales every focus move.
    """

    name = "nikon_ti"
    display_name = "Nikon Ti (SDK)"

    def __init__(self, prog_id: Optional[str] = None,
                 z_units_per_um: float = 100.0, cassette: int = 1):
        self._prog_id = prog_id or None
        self._z_units_per_um = float(z_units_per_um) or 1.0
        self._cassette = max(1, int(cassette))
        self._scope = None
        self._devices: dict[str, object] = {}
        self._resolved_prog_id: Optional[str] = None
        #: Resolved lazily from the SDK's declared focus unit; see
        #: focus_units_per_um(). Cleared on connect/disconnect.
        self._focus_factor: Optional[float] = None
        #: ``(closed_raw, open_raw)`` for the epi shutter, resolved lazily from
        #: the SDK's own declared range + DisplayString. See _shutter_codes().
        self._shutter_codes_cache: Optional[tuple] = None
        #: Which property carries the dia lamp's on/off, once found.
        self._lamp_switch_attr: Optional[str] = None

    # -- lifecycle --

    def connect(self) -> None:
        try:
            import comtypes  # noqa: F401
            import comtypes.client as cc
        except Exception as exc:  # pragma: no cover - platform dependent
            raise MicroscopeError(
                "the Nikon Ti SDK backend needs 'comtypes' (Windows only): "
                f"{exc}") from exc
        # COM is apartment-threaded; the controller guarantees every call
        # (including this one) runs on the same dedicated worker thread.
        try:
            comtypes.CoInitialize()
        except Exception:
            pass

        candidates = [self._prog_id] if self._prog_id else list(TI_PROG_IDS)
        errors = []
        scope = None
        for prog_id in candidates:
            try:
                scope = cc.CreateObject(prog_id)
                self._resolved_prog_id = prog_id
                break
            except Exception as exc:  # pragma: no cover - hardware dependent
                errors.append(f"{prog_id}: {exc}")
        if scope is None:
            raise MicroscopeError(
                "could not create the Nikon Ti SDK COM object. Is the Nikon Ti "
                "SDK / NIS driver installed and the body powered on?\n  "
                + "\n  ".join(errors))
        self._scope = scope
        self._focus_factor = None      # re-resolve against this connection
        self._shutter_codes_cache = None
        self._lamp_switch_attr = None
        self._devices = self._discover_devices(scope)
        self._prime_devices()
        # Judge the connection on the CORE devices only: an accessory-only match
        # (say an epi shutter but no turrets at all) is a broken link, not a
        # usable body, and must still be reported as one.
        if not (set(self._devices) & _TI_CORE_DEVICES):
            raise MicroscopeError(
                f"connected to {self._resolved_prog_id} but found none of the "
                "expected devices (nosepiece / filter cassette / Z drive). Use "
                "the Diagnostics report to see what this body exposes.")

    def _prime_devices(self) -> None:
        """Touch each device once so its first real read is not stale.

        ⚠ **Hardware-measured on the Ti-E (2026-08-12).** Reading a device's
        ``Position`` as the *first* COM access after connect returns a default —
        the light-path drive sitting at 3 answered **1**, and kept answering 1 for
        as long as nothing else on the device was touched (6 reads over 0.9 s).
        One access to any other property (``IsMounted``) makes the very next
        ``Position`` read return the true 3. So it is the access that primes it,
        not elapsed time.

        Every position getter already happens to call ``_require_mounted`` first
        and is therefore primed **by accident of ordering** — as is
        ``dia_lamp_intensity`` in ``_read_all``, because ``has_dia_lamp`` runs
        ahead of it. Relying on that is a latent trap: reorder those two lines and
        the first state published after connect silently carries a wrong value.
        Priming once, explicitly, removes the accident. Six cheap reads, no sleep.
        """
        for logical, device in self._devices.items():
            try:
                self._is_mounted(device)
            except Exception as exc:
                logger.debug(f"Nikon Ti: priming {logical} failed: {exc}")

    def _discover_devices(self, scope) -> dict:
        found = {}
        for logical, aliases in _TI_DEVICE_ALIASES.items():
            names = [a.format(n=self._cassette) for a in aliases]
            attr, device = _first_attr(scope, names)
            if device is not None:
                found[logical] = device
                logger.info(
                    f"Nikon Ti: {logical} device resolved to '{attr}'")
            elif logical in _TI_CORE_DEVICES:
                logger.warning(
                    f"Nikon Ti: no {logical} device found (tried {names})")
            else:
                logger.info(
                    f"Nikon Ti: no {logical} accessory on this body "
                    f"(tried {names})")
        return found

    def disconnect(self) -> None:
        self._devices = {}
        self._scope = None
        self._focus_factor = None
        self._shutter_codes_cache = None
        self._lamp_switch_attr = None
        try:
            import comtypes
            comtypes.CoUninitialize()
        except Exception:
            pass

    @property
    def is_connected(self) -> bool:
        return self._scope is not None

    def _device(self, logical: str):
        if self._scope is None:
            raise MicroscopeError("microscope is not connected")
        device = self._devices.get(logical)
        if device is None:
            raise MicroscopeError(f"this body has no {logical} device")
        return device

    # -- position plumbing (probed, not assumed) --

    #: Plausible turret index / limit. Anything outside this is the SDK handing
    #: back uninitialised memory, which is what it does when no instrument is
    #: live — see _coerce_int.
    _SANE_POSITION_MAX = 99

    @staticmethod
    def _coerce_int(value, what: str, max_abs=None) -> int:
        """Convert an SDK-returned value to int, or explain why it can't be.

        With no live instrument behind it the SDK returns junk (observed:
        raw pointer bytes), and a bare ``int()`` then dies with an opaque
        *"invalid literal for int()"*. Diagnose it instead.

        ``max_abs`` bounds the plausible magnitude — it is the turret-index
        ceiling for a cassette/nosepiece, but must stay ``None`` for the Z
        drive, whose position is in device units and legitimately reaches the
        hundreds of thousands.
        """
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise MicroscopeError(
                f"{what} returned {type(value).__name__}, not a number — the "
                "SDK is loaded but has no live instrument behind it (check the "
                "microscope is powered on and its USB driver actually loaded)")
        out = int(value)
        if max_abs is not None and not -1 <= out <= max_abs:
            raise MicroscopeError(
                f"{what} returned {out}, which is not a plausible position — "
                "the SDK reports no live instrument")
        return out

    # ---------------------------------------------------------------
    # The Ti SDK does NOT expose plain numbers. Every device property
    # (Position, IsMounted, …) returns an ``IMipParameter`` wrapper whose
    # number lives on ``RawValue``, with ``RangeLowerLimit`` /
    # ``RangeHigherLimit`` giving the real bounds, ``Unit`` the physical unit
    # and ``DisplayString`` a human string (an unmounted device reports
    # "Device not available"). Confirmed by introspection against a live
    # Nikon.TiScope.NikonTi on SDK 4.4.1.714.
    # ---------------------------------------------------------------

    @staticmethod
    def _param(device, name):
        """The ``IMipParameter`` (or plain value) behind a device property."""
        try:
            return getattr(device, name, None)
        except COMError as exc:
            raise MicroscopeError(_com_message(exc)) from exc

    @staticmethod
    def _param_attr(param, attr, default=None):
        """Read one field off a parameter wrapper, tolerating COM failures."""
        if param is None:
            return default
        try:
            value = getattr(param, attr, default)
        except COMError:
            return default
        except Exception:
            return default
        return default if value is None else value

    @classmethod
    def _param_number(cls, param, what: str, max_abs=None) -> int:
        """The number behind a parameter: ``RawValue``, or the value itself."""
        if param is None:
            raise MicroscopeError(f"{what} is not exposed by this device")
        if isinstance(param, (int, float)) and not isinstance(param, bool):
            return cls._coerce_int(param, what, max_abs)
        try:
            raw = getattr(param, "RawValue", None)
        except COMError as exc:
            raise MicroscopeError(_com_message(exc)) from exc
        if raw is None:
            raise MicroscopeError(
                f"{what} exposes no RawValue — unexpected SDK object model")
        return cls._coerce_int(raw, what, max_abs)

    @classmethod
    def _is_mounted(cls, device):
        """``True``/``False`` from the SDK's ``IsMounted``, else ``None``."""
        try:
            param = cls._param(device, "IsMounted")
        except MicroscopeError:
            return None
        if param is None:
            return None
        if isinstance(param, bool):
            return param
        if isinstance(param, (int, float)):
            return bool(param)
        raw = cls._param_attr(param, "RawValue")
        return None if raw is None else bool(raw)

    @classmethod
    def _mount_detail(cls, device) -> str:
        """The SDK's own words for the mount state (e.g. 'Device not available')."""
        try:
            param = cls._param(device, "IsMounted")
        except MicroscopeError:
            return ""
        text = cls._param_attr(param, "DisplayString", "")
        return str(text or "").strip()

    @classmethod
    def _require_mounted(cls, device, what: str) -> None:
        if cls._is_mounted(device) is False:
            detail = cls._mount_detail(device)
            raise MicroscopeError(
                f"the {what} is not available"
                + (f" — the microscope reports: {detail}" if detail else "")
                + ". The SDK is loaded but has no live link to the body "
                  "(check it is powered on and its USB driver loaded).")

    @classmethod
    def _read_position(cls, device, max_abs=None) -> int:
        cls._require_mounted(device, "device")
        return cls._param_number(
            cls._param(device, "Position"), "Position", max_abs)

    @classmethod
    def _write_position(cls, device, value) -> None:
        param = cls._param(device, "Position")
        if param is None:
            raise MicroscopeError("device exposes no Position property")
        try:
            if hasattr(param, "RawValue"):
                param.RawValue = value
            else:
                device.Position = value
        except COMError as exc:
            # The SDK's own text ("No available instruments.") beats a traceback.
            raise MicroscopeError(_com_message(exc)) from exc

    @classmethod
    def _position_range(cls, device):
        """``(lower, upper)`` the SDK declares for this device, else ``None``."""
        try:
            param = cls._param(device, "Position")
        except MicroscopeError:
            return None
        lo = cls._param_attr(param, "RangeLowerLimit")
        hi = cls._param_attr(param, "RangeHigherLimit")

        def _num(v):
            return isinstance(v, (int, float)) and not isinstance(v, bool)

        if _num(lo) and _num(hi) and hi >= lo:
            return int(lo), int(hi)
        return None

    def _set_turret(self, logical: str, position: int, what: str) -> None:
        """Move a turret, REFUSING an out-of-range index.

        ⚠ Hardware-verified 2026-07-30: asked for filter slot 999, the Ti SDK
        **silently clamped it to 6 and reported success**. That is the worst
        possible failure mode for a discrete selector — a mistyped or stale
        slot number would quietly image through the wrong cube while the app
        reported the move as fine. Validate against the SDK's own declared
        range before writing.

        (The focus drive is deliberately treated differently — see
        ``set_focus_um``: it is a continuous axis where running to the end of
        travel is normal, so it clamps rather than refuses.)
        """
        device = self._device(logical)
        pos = int(position)
        rng = self._position_range(device)
        if rng is not None and not rng[0] <= pos <= rng[1]:
            raise MicroscopeError(
                f"{what} {pos} is outside this turret's range "
                f"{rng[0]}-{rng[1]} — refused, because the SDK would otherwise "
                f"clamp it and report success")
        self._write_position(device, pos)

    @classmethod
    def _read_range(cls, device, default_max: int) -> int:
        """Number of turret positions, from the SDK's own declared range."""
        try:
            param = cls._param(device, "Position")
        except MicroscopeError:
            return default_max
        hi = cls._param_attr(param, "RangeHigherLimit")
        if isinstance(hi, (int, float)) and not isinstance(hi, bool):
            try:
                out = cls._coerce_int(hi, "RangeHigherLimit",
                                      cls._SANE_POSITION_MAX)
                if out >= 1:
                    return out
            except MicroscopeError:
                pass
        return default_max

    # -- filter --

    def filter_count(self) -> int:
        try:
            return self._read_range(self._device("filter"), 6)
        except MicroscopeError:
            return 0

    def get_filter(self) -> Optional[int]:
        try:
            return self._read_position(
                self._device("filter"), self._SANE_POSITION_MAX)
        except MicroscopeError:
            return None

    def set_filter(self, position: int) -> None:
        self._set_turret("filter", position, "filter slot")

    # -- nosepiece --

    def objective_count(self) -> int:
        try:
            return self._read_range(self._device("objective"), 6)
        except MicroscopeError:
            return 0

    def get_objective(self) -> Optional[int]:
        try:
            return self._read_position(
                self._device("objective"), self._SANE_POSITION_MAX)
        except MicroscopeError:
            return None

    def set_objective(self, position: int) -> None:
        self._set_turret("objective", position, "objective position")

    # -- focus --

    #: Display-unit string → micrometres per one of that unit.
    _UNIT_TO_UM = {
        "um": 1.0, "µm": 1.0, "micron": 1.0, "microns": 1.0,
        "micrometer": 1.0, "micrometre": 1.0,
        "nm": 0.001, "nanometer": 0.001, "nanometre": 0.001,
        "mm": 1000.0, "millimeter": 1000.0, "millimetre": 1000.0,
    }

    def focus_units_per_um(self) -> float:
        """Device (raw) units per micrometre for the focus drive.

        ⚠ **``Unit`` describes the DISPLAY value, not ``RawValue``.** An earlier
        cut of this method read ``Unit == 'um'`` and concluded the drive was
        µm-native (factor 1.0) — which made every focus move **40× too small**
        on real hardware. The SDK's actual model is::

            display_value [in Unit] = RawValue × DisplayScale

        so micrometres per raw unit = ``DisplayScale × unit_to_um(Unit)`` and
        the factor this method returns is its reciprocal.

        Measured on the Ti-E (SDK 4.4.1): ``DisplayScale = 0.025``,
        ``Unit = 'um'`` ⇒ 0.025 µm (25 nm) per raw unit ⇒ **40 units/µm**.
        Cross-checked three independent ways:

        * ``RawValue 30844`` → ``DisplayString '771.100 um'``
          (30844 × 0.025 = 771.100, exact);
        * ``ZDrive.Resolution = 25`` with the ZDrive's device ``Unit = 'nm'``;
        * the declared range ``400000`` → ``'10000.000 um'`` = **10 mm**, which
          is the Ti's real focus travel (400 mm, the old reading, is absurd).

        Falls back to the configured ``z_units_per_um`` only when the SDK
        exposes no usable ``DisplayScale``.
        """
        if self._focus_factor is not None:
            return self._focus_factor
        self._focus_factor = self._resolve_focus_factor()
        return self._focus_factor

    def _resolve_focus_factor(self) -> float:
        try:
            param = self._param(self._device("focus"), "Position")
        except MicroscopeError:
            return self._z_units_per_um
        scale = self._param_attr(param, "DisplayScale")
        unit = str(self._param_attr(param, "Unit", "") or "").strip().lower()
        if not isinstance(scale, (int, float)) or isinstance(scale, bool):
            logger.warning(
                "Nikon Ti: focus exposes no DisplayScale; falling back to the "
                f"configured {self._z_units_per_um} units/µm")
            return self._z_units_per_um
        unit_um = self._UNIT_TO_UM.get(unit)
        if unit_um is None:
            logger.warning(
                f"Nikon Ti: focus reports an unrecognised display unit "
                f"{unit!r}; falling back to the configured "
                f"{self._z_units_per_um} units/µm")
            return self._z_units_per_um
        um_per_raw = float(scale) * unit_um
        if um_per_raw <= 0:
            logger.warning(
                f"Nikon Ti: focus DisplayScale {scale!r} is not usable; "
                f"falling back to the configured {self._z_units_per_um}")
            return self._z_units_per_um
        factor = 1.0 / um_per_raw
        logger.info(
            f"Nikon Ti: focus DisplayScale {scale} {unit} → {um_per_raw:g} µm "
            f"per raw unit → {factor:g} units/µm "
            f"(configured fallback {self._z_units_per_um} not used)")
        return factor

    def get_focus_um(self) -> Optional[float]:
        try:
            raw = self._read_position(self._device("focus"))
        except MicroscopeError:
            return None
        return float(raw) / self.focus_units_per_um()

    def set_focus_um(self, value_um: float) -> None:
        """Drive the focus, CLAMPED to the declared travel.

        Unlike a turret index (see ``_set_turret``), the focus is a continuous
        axis where reaching the end of travel during a jog is normal operation,
        not a mistake — so an over-range target clamps rather than refusing.
        """
        device = self._device("focus")
        raw = int(round(float(value_um) * self.focus_units_per_um()))
        rng = self._position_range(device)
        if rng is not None:
            clamped = max(rng[0], min(rng[1], raw))
            if clamped != raw:
                logger.info(
                    f"Nikon Ti: focus target {raw} clamped to the drive's "
                    f"declared travel {rng[0]}-{rng[1]}")
            raw = clamped
        self._write_position(device, raw)

    # -- what is physically fitted --

    #: ``Objective.Magnification`` is an INDEX into the SDK's own table, not a
    #: magnification. Verified against
    #: ``C:\\Program Files\\Nikon\\Shared\\Data\\Ti\\ObjectiveMagnifications.txt``
    #: and cross-checked on hardware (codes 5/7/9 → 4x/10x/20x, matching the
    #: NA and working distance the same objectives reported).
    _MAGNIFICATIONS = ("---", "1", "1.5", "2", "2.5", "4", "5", "10", "16",
                       "20", "40", "50", "60", "100", "150", "200")
    _TI_DATA_DIR = r"C:\Program Files\Nikon\Shared\Data\Ti"

    @classmethod
    def _magnification_table(cls) -> tuple:
        """The SDK's table from disk when available, else the verified copy."""
        try:
            import os
            path = os.path.join(cls._TI_DATA_DIR, "ObjectiveMagnifications.txt")
            with open(path, encoding="utf-8", errors="replace") as handle:
                rows = [line.strip() for line in handle]
            if rows:
                return tuple(rows)
        except Exception:
            pass
        return cls._MAGNIFICATIONS

    @staticmethod
    def _safe(fn, default=None):
        """Read one optic field; an empty slot raises 'No database code…'."""
        try:
            value = fn()
        except Exception:
            return default
        return default if value is None else value

    def _collection(self, logical: str, attr: str):
        device = self._device(logical)
        coll = self._safe(lambda: getattr(device, attr, None))
        if coll is None:
            raise MicroscopeError(
                f"this body does not expose {attr} — mounted optics cannot be "
                "read from it")
        return coll

    def mounted_filters(self) -> tuple:
        try:
            coll = self._collection("filter", "FilterBlocks")
            count = int(self._safe(lambda: coll.Count, 0) or 0)
        except MicroscopeError:
            return ()
        out = []
        for pos in range(1, count + 1):
            item = self._safe(lambda p=pos: coll.Item(p))
            if item is None:
                continue
            code = self._safe(lambda: int(item.Code), 0) or 0
            name = str(self._safe(lambda: item.Name, "") or "").strip()
            # An empty slot reports Code 0 and a placeholder name ("-----").
            present = bool(code) and bool(name) and set(name) != {"-"}
            out.append(MountedOptic(
                position=pos, present=present,
                label=name if present else "",
                code=str(code) if present else "",
                detail=(f"filter block code {code}" if present else "empty")))
        return tuple(out)

    def mounted_objectives(self) -> tuple:
        try:
            coll = self._collection("objective", "Objectives")
            count = int(self._safe(lambda: coll.Count, 0) or 0)
        except MicroscopeError:
            return ()
        table = self._magnification_table()
        out = []
        for pos in range(1, count + 1):
            item = self._safe(lambda p=pos: coll.Item(p))
            if item is None:
                continue
            # Code 0 = nothing fitted; every other field then raises
            # "No database code is associated with this optical element."
            code = self._safe(lambda: int(item.Code), 0) or 0
            if not code:
                out.append(MountedOptic(position=pos, present=False,
                                        detail="empty"))
                continue
            product = str(self._safe(lambda: item.Name, "") or "").strip()
            mag_idx = self._safe(lambda: int(item.Magnification))
            mag = (table[mag_idx]
                   if isinstance(mag_idx, int) and 0 < mag_idx < len(table)
                   else "")
            na = self._safe(lambda: float(item.NumericalAperture))
            wd = self._safe(lambda: float(item.WorkingDistance))
            label = f"{mag}x" if mag else (product or f"position {pos}")
            bits = [product] if product else []
            if na:
                bits.append(f"NA {na:g}")
            if wd:
                bits.append(f"WD {wd:g} mm")
            # `mag` is the SDK's magnification TABLE entry (a string like "20"),
            # because item.Magnification is an INDEX into that table, not a
            # magnification. Carry the numeric form alongside the label.
            try:
                mag_num = float(mag) if mag else None
            except (TypeError, ValueError):
                mag_num = None
            out.append(MountedOptic(
                position=pos, present=True, label=label, code=product,
                detail=" · ".join(bits),
                magnification=mag_num,
                numerical_aperture=(float(na) if na else None),
                working_distance_mm=(float(wd) if wd else None)))
        return tuple(out)

    def filter_names(self) -> tuple:
        return tuple(o.label for o in self.mounted_filters())

    def objective_names(self) -> tuple:
        return tuple(o.label for o in self.mounted_objectives())

    def focus_limits_um(self) -> Optional[tuple]:
        """Travel limits the SDK declares for the Z drive, converted to µm.

        Observed on a Ti-E / SDK 4.4.1: ``0 … 400000`` with ``Unit='um'``, i.e.
        400 mm of travel. Real limits beat a guess — the panel clamps to them.
        """
        try:
            param = self._param(self._device("focus"), "Position")
        except MicroscopeError:
            return None
        lo = self._param_attr(param, "RangeLowerLimit")
        hi = self._param_attr(param, "RangeHigherLimit")
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   for v in (lo, hi)):
            return None
        factor = self.focus_units_per_um() or 1.0
        lo_um, hi_um = float(lo) / factor, float(hi) / factor
        return (min(lo_um, hi_um), max(lo_um, hi_um))

    # ── v7.17 accessories: epi shutter / dia lamp / light path ──────
    #
    # These use their own small read/write helpers rather than the turret ones
    # because the value-carrying property is NOT always ``Position`` on this
    # family of devices (a lamp's level and a shutter's state are both commonly
    # ``Value``). The turret/focus paths above are hardware-verified and are left
    # byte-identical.

    #: Property names probed, in order, for a device's value-carrying parameter.
    _VALUE_PROPS = ("Position", "Value")

    #: Property names probed for the dia lamp's separate on/off switch.
    #: ``IsOn`` is the one this SDK actually has (hardware-confirmed).
    _LAMP_SWITCH_PROPS = ("IsOn", "SwitchValue", "Switch", "LampSwitch", "OnOff")

    #: The SDK's "I don't know" sentinel on a boolean status parameter. Measured
    #: on a Ti-E: an unfitted shutter answers ``IsOpened`` with
    #: ``RawValue=-1, DisplayString='Status Unknown'``. It must read as UNKNOWN,
    #: never as a state — "closed" would tell the operator the sample is dark.
    _STATUS_UNKNOWN = -1

    def _accessory_present(self, logical: str) -> bool:
        """Is this accessory PHYSICALLY FITTED (not merely known to the SDK)?

        ⚠ **Hardware-measured, and it is not the same question as "did the
        attribute resolve".** The Ti SDK publishes a COM object for every device
        it knows about whether or not the body has one: on this Ti-E,
        ``EpiShutter`` and ``DiaShutter`` both resolve and then report
        ``IsMounted = 0 ('Device not available')`` with
        ``IsOpened = -1 ('Status Unknown')``, while ``DiaLamp`` and
        ``LightPathDrive`` report ``IsMounted = 1 ('Device mounted')``.

        Resolving the attribute alone therefore claims hardware that is not
        there — and for the shutter that is not cosmetic: the interlock would
        believe it had a shutter to close and silently protect nothing.

        ``IsMounted`` absent (``None``) counts as present, so a device that
        simply does not report its mount state is still usable.
        """
        device = self._devices.get(logical)
        if device is None:
            return False
        return self._is_mounted(device) is not False

    @classmethod
    def _status_bool(cls, param):
        """A named boolean status parameter as ``True``/``False``/``None``.

        ``None`` for the SDK's ``-1 / 'Status Unknown'`` sentinel and for
        anything non-numeric.
        """
        raw = cls._param_attr(param, "RawValue")
        if isinstance(raw, bool):
            return raw
        if not isinstance(raw, (int, float)):
            return None
        value = int(raw)
        return None if value < 0 else bool(value)

    def _value_param(self, logical: str):
        """The parameter that carries this device's value, or ``None``."""
        device = self._devices.get(logical)
        if device is None:
            return None
        for prop in self._VALUE_PROPS:
            param = self._param_attr(device, prop)
            if param is not None:
                return param
        return None

    def _write_value(self, logical: str, raw) -> None:
        """Write a raw value to whichever property this device exposes."""
        device = self._devices.get(logical)
        if device is None:
            raise MicroscopeError(f"this body has no {logical} device")
        for prop in self._VALUE_PROPS:
            param = self._param_attr(device, prop)
            if param is None:
                continue
            try:
                if hasattr(param, "RawValue"):
                    param.RawValue = raw
                else:
                    setattr(device, prop, raw)
            except COMError as exc:
                raise MicroscopeError(_com_message(exc)) from exc
            return
        raise MicroscopeError(
            f"the {logical} device exposes no writable value property "
            f"(tried {', '.join(self._VALUE_PROPS)})")

    def _read_value_raw(self, logical: str):
        """The raw number behind this device's value, or ``None``."""
        param = self._value_param(logical)
        if param is None:
            return None
        if isinstance(param, (int, float)) and not isinstance(param, bool):
            return float(param)
        raw = self._param_attr(param, "RawValue")
        if not isinstance(raw, (int, float)) or isinstance(raw, bool):
            return None
        return float(raw)

    def _value_range(self, logical: str):
        """``(lower, upper)`` the SDK declares for this device, else ``None``."""
        param = self._value_param(logical)
        lo = self._param_attr(param, "RangeLowerLimit")
        hi = self._param_attr(param, "RangeHigherLimit")

        def _num(v):
            return isinstance(v, (int, float)) and not isinstance(v, bool)

        if _num(lo) and _num(hi) and hi > lo:
            return float(lo), float(hi)
        return None

    # -- epi shutter --

    #: Ti convention when the SDK declares no range at all: 1 = closed, 2 = open.
    _SHUTTER_FALLBACK_CODES = (1, 2)

    def has_epi_shutter(self) -> bool:
        return self._accessory_present("epi_shutter")

    def epi_shutter_interlock_enabled(self):
        """The **body's own** interlock flag, if it has one. Read-only.

        ``IEpiShutter`` publishes ``IsInterlockEnabled`` — so the SDK has its own
        notion of an interlock, separate from ours. Surfaced (in Diagnostics) but
        never written: what it interlocks against is undocumented here, and this
        rig's shutter is unfitted so it reads ``-1 'Status Unknown'``. Worth
        settling with Nikon before anything relies on it.
        """
        if "epi_shutter" not in self._devices:
            return None
        return self._status_bool(
            self._param_attr(self._devices["epi_shutter"], "IsInterlockEnabled"))

    def _shutter_codes(self) -> tuple:
        """``(closed_raw, open_raw)`` for this body's epi shutter.

        ⚠ **The mapping is DERIVED, not assumed — and is not yet confirmed on
        hardware.** Three stages, most trustworthy first:

        1. **The SDK's declared range.** A shutter declares two states; the Ti
           convention is that the lower code is closed and the higher is open.
        2. **Refined by the live ``DisplayString``.** A shutter reports its state
           in words ("Open" / "Closed"), so the raw value currently in force
           tells us which code that word belongs to, and the other endpoint
           follows by elimination. When the SDK populates the string this is a
           *measurement* rather than a convention, and it overrides stage 1.
        3. **Fallback** to ``1 = closed, 2 = open`` when no range is declared,
           logged as the guess it is.

        The operator's ``epi_shutter_invert`` setting is applied one layer up, in
        :class:`MicroscopeController`, so a body that disagrees with all three is
        a checkbox rather than a code change.
        """
        if self._shutter_codes_cache is not None:
            return self._shutter_codes_cache
        rng = self._value_range("epi_shutter")
        if rng is None:
            codes = self._SHUTTER_FALLBACK_CODES
            logger.warning(
                "Nikon Ti: the epi shutter declares no range; assuming "
                f"{codes[0]}=closed / {codes[1]}=open. Verify on the body and "
                "use 'Shutter reads inverted' if it is the other way round.")
        else:
            codes = (int(round(rng[0])), int(round(rng[1])))
            text = str(self._param_attr(
                self._value_param("epi_shutter"), "DisplayString", "") or "")
            raw = self._read_value_raw("epi_shutter")
            said = self._state_from_text(text)
            if said is not None and raw is not None:
                here = int(round(raw))
                other = codes[0] if here == codes[1] else codes[1]
                # `said` describes the code currently in force; the remaining
                # endpoint is necessarily the opposite state.
                codes = (other, here) if said else (here, other)
                logger.info(
                    f"Nikon Ti: epi shutter reports {text!r} at raw {here} → "
                    f"{codes[0]}=closed / {codes[1]}=open")
            else:
                logger.info(
                    f"Nikon Ti: epi shutter range {codes} → "
                    f"{codes[0]}=closed / {codes[1]}=open (from the declared "
                    "range; the body reported no state text to confirm it)")
        self._shutter_codes_cache = codes
        return codes

    @staticmethod
    def _state_from_text(text: str):
        """``True``/``False`` if this state text names open/closed, else ``None``."""
        low = str(text or "").strip().lower()
        if not low:
            return None
        # Check 'closed' first: "closed" contains no "open", but being explicit
        # keeps a future "not open" style string from reading as open.
        if "clos" in low or "shut" in low:
            return False
        if "open" in low:
            return True
        return None

    def epi_shutter_open(self) -> Optional[bool]:
        """``True`` = open, from ``IsOpened`` where the SDK provides it.

        ⚠ **``Value`` is NOT the shutter state on this SDK.** Measured on a
        Ti-E: ``IEpiShutter`` carries a *named boolean* ``IsOpened`` alongside a
        generic ``Value`` that sits at 1 in a declared 1–2 range regardless. So
        the named property is read first and the derived
        :meth:`_shutter_codes` mapping is only a fallback for an SDK generation
        that has no ``IsOpened`` — which is what turns the riskiest guess in this
        module into no guess at all wherever ``IsOpened`` exists.
        """
        if not self.has_epi_shutter():
            return None
        named = self._param_attr(self._devices["epi_shutter"], "IsOpened")
        if named is not None:
            return self._status_bool(named)
        raw = self._read_value_raw("epi_shutter")
        if raw is None:
            return None
        closed, opened = self._shutter_codes()
        # Compare against both codes rather than truthiness: neither endpoint is
        # guaranteed to be 0/1, and a value matching neither is unknown.
        here = int(round(raw))
        if here == opened:
            return True
        if here == closed:
            return False
        return None

    def set_epi_shutter(self, open_: bool) -> None:
        """Open/close via the SDK's own ``Open()`` / ``Close()`` where present.

        Hardware-confirmed to exist on ``IEpiShutter``. Preferred over writing a
        derived code to ``Value`` for the same reason as the read above: an
        explicitly named action cannot be got backwards, whereas an encoding can.
        """
        if not self.has_epi_shutter():
            raise MicroscopeError(
                "this body has no epi shutter fitted (the SDK exposes the "
                "device but reports it 'not available') — check the Diagnostics "
                "report")
        action = self._param_attr(
            self._devices["epi_shutter"], "Open" if open_ else "Close")
        if callable(action):
            try:
                action()
            except COMError as exc:
                raise MicroscopeError(_com_message(exc)) from exc
            return
        closed, opened = self._shutter_codes()
        self._write_value("epi_shutter", opened if open_ else closed)

    # -- dia lamp --

    def has_dia_lamp(self) -> bool:
        return self._accessory_present("dia_lamp")

    #: ``IsControlled``: 0 = MainMode (front panel owns the lamp), 1 = RemoteMode
    #: (software may write). Hardware-measured; see ``dia_lamp_remote``.
    def dia_lamp_remote(self) -> Optional[bool]:
        if not self.has_dia_lamp():
            return None
        return self._status_bool(
            self._param_attr(self._devices["dia_lamp"], "IsControlled"))

    def set_dia_lamp_remote(self, on: bool) -> None:
        if not self.has_dia_lamp():
            raise MicroscopeError("this body has no dia lamp fitted")
        param = self._param_attr(self._devices["dia_lamp"], "IsControlled")
        if param is None or not hasattr(param, "RawValue"):
            raise MicroscopeError(
                "this body's dia lamp exposes no software/manual mode switch")
        try:
            param.RawValue = 1 if on else 0
        except COMError as exc:
            raise MicroscopeError(_com_message(exc)) from exc

    def _require_lamp_writable(self) -> None:
        """Refuse a lamp write in MainMode, with the reason and the remedy.

        Checked BEFORE writing rather than translating the HRESULT afterwards:
        the SDK's own answer is a bare ``0xE01004BB``, which tells the operator
        nothing about the knob on the front of their microscope.
        """
        if self.dia_lamp_remote() is False:
            raise MicroscopeError(
                "the dia lamp is in MainMode — the microscope's own front-panel "
                "control owns it and the SDK refuses software changes. Switch it "
                "to software (Remote) control first.")

    def _lamp_switch(self):
        """``(attr_name, param)`` for the lamp's on/off, or ``(None, None)``."""
        device = self._devices.get("dia_lamp")
        if device is None:
            return None, None
        names = ([self._lamp_switch_attr] if self._lamp_switch_attr
                 else list(self._LAMP_SWITCH_PROPS))
        attr, param = _first_attr(device, names)
        if attr is not None:
            self._lamp_switch_attr = attr
        return attr, param

    def dia_lamp_on(self) -> Optional[bool]:
        if not self.has_dia_lamp():
            return None
        attr, param = self._lamp_switch()
        if attr is None:
            return None
        if isinstance(param, bool):
            return param
        if isinstance(param, (int, float)):
            return bool(param)
        return self._status_bool(param)

    def set_dia_lamp_on(self, on: bool) -> None:
        """Switch the lamp via the SDK's ``On()`` / ``Off()`` where present.

        Hardware-confirmed on ``IDiaLamp``; writing ``IsOn.RawValue`` is the
        fallback for a generation without them.
        """
        if not self.has_dia_lamp():
            raise MicroscopeError("this body has no dia lamp fitted")
        self._require_lamp_writable()
        device = self._devices["dia_lamp"]
        action = self._param_attr(device, "On" if on else "Off")
        if callable(action):
            try:
                action()
            except COMError as exc:
                raise MicroscopeError(_com_message(exc)) from exc
            return
        attr, param = self._lamp_switch()
        if attr is None:
            raise MicroscopeError(
                "this body's dia lamp exposes no on/off switch "
                f"(tried On/Off and {', '.join(self._LAMP_SWITCH_PROPS)}) — set "
                "its level instead, or check the Diagnostics report")
        try:
            if hasattr(param, "RawValue"):
                param.RawValue = 1 if on else 0
            else:
                setattr(device, attr, 1 if on else 0)
        except COMError as exc:
            raise MicroscopeError(_com_message(exc)) from exc

    def dia_lamp_intensity(self) -> Optional[float]:
        if not self.has_dia_lamp():
            return None
        return self._read_value_raw("dia_lamp")

    def dia_lamp_intensity_range(self) -> Optional[tuple]:
        if not self.has_dia_lamp():
            return None
        return self._value_range("dia_lamp")

    def set_dia_lamp_intensity(self, value: float) -> None:
        """Set the lamp level, CLAMPED to the range the device declares.

        Clamped, not refused, on the same principle as ``set_focus_um``: this is
        a continuous level where asking for "full" and landing at the declared
        maximum is ordinary operation, not a mistake. (A *discrete* selector —
        a turret, the light path — refuses instead; see ``_set_turret``.)

        The value is in the DEVICE'S OWN units, whatever the SDK declares. It is
        deliberately not rescaled to a percentage: the declared range may be
        volts or an arbitrary index, and a "percentage" of an unknown quantity is
        a fabricated number that reads as a measured one.

        ⚠ **MEASURED LIMITATION on this Ti-E (2026-08-12) — the level does not
        stick.** In RemoteMode the write IS accepted (no error) and ``Value``
        reads back the number written, but it **reverts to the front-panel knob's
        setting within about a second**, and ``MeasuredVoltage`` never moves at
        all — so the lamp's brightness does not appear to change. Identical via
        this method and via a direct parameter write, and ``Increase()`` does
        nothing either. The lamp's **on/off does work** in RemoteMode.

        Left writing rather than refusing, because the write is correct and
        harmless and may well hold on another body or configuration; the panel
        polls about once a second, so a level that does not stick is visibly
        rejected on screen rather than silently accepted. What governs it is a
        question for Nikon, not something to guess at here.
        """
        if not self.has_dia_lamp():
            raise MicroscopeError("this body has no dia lamp fitted")
        self._require_lamp_writable()
        raw = float(value)
        rng = self._value_range("dia_lamp")
        if rng is not None:
            clamped = max(rng[0], min(rng[1], raw))
            if clamped != raw:
                logger.info(
                    f"Nikon Ti: dia lamp level {raw:g} clamped to the declared "
                    f"range {rng[0]:g}-{rng[1]:g}")
            raw = clamped
        self._write_value("dia_lamp", int(round(raw)))

    # -- light path --

    def light_path_count(self) -> int:
        if not self._accessory_present("light_path"):
            return 0
        try:
            return self._read_range(self._device("light_path"), 0)
        except MicroscopeError:
            return 0

    def get_light_path(self) -> Optional[int]:
        if not self._accessory_present("light_path"):
            return None
        try:
            return self._read_position(
                self._device("light_path"), self._SANE_POSITION_MAX)
        except MicroscopeError:
            return None

    def set_light_path(self, position: int) -> None:
        """Select a light path, REFUSING an out-of-range index.

        Routed through the same ``_set_turret`` as the cassette and nosepiece
        precisely because it is the same kind of device: hardware-verified in
        v7.5.x, this SDK **silently clamps an out-of-range discrete index and
        reports success**, so a stale index would quietly send the light
        somewhere else while the app called the move fine.
        """
        if not self._accessory_present("light_path"):
            raise MicroscopeError("this body has no light path drive fitted")
        self._set_turret("light_path", position, "light path")

    def light_path_names(self) -> tuple:
        """``()`` — the SDK exposes no light-path name collection.

        Unlike ``FilterBlocks``/``Objectives`` there is no per-position table to
        read, and the only text available (``DisplayString``) describes the
        position currently in force. Naming the others would mean moving the
        drive to read them, so the UI shows position numbers and the current
        name is surfaced in Diagnostics instead of inventing labels.
        """
        return ()

    def light_path_name_now(self) -> str:
        """The SDK's own words for the light path currently selected, if any."""
        return str(self._param_attr(
            self._value_param("light_path"), "DisplayString", "") or "").strip()

    # -- diagnostics --

    def diagnostics(self) -> str:
        """Dump the live COM object model.

        This exists so the device/property mapping above can be confirmed on the
        real body without another code round-trip: run it once at the bench and
        the report names every device and every property actually exposed.
        """
        lines = [f"{self.display_name}",
                 f"  ProgID: {self._resolved_prog_id or '(not connected)'}"]
        if self._scope is None:
            lines.append("  (not connected)")
            return "\n".join(lines)
        try:
            attrs = sorted(a for a in dir(self._scope)
                           if not a.startswith("_"))
            lines.append(f"  scope attributes: {', '.join(attrs)}")
        except Exception as exc:
            lines.append(f"  scope attributes unavailable: {exc}")
        for logical, device in self._devices.items():
            lines.append(f"  [{logical}] {type(device).__name__}")
            try:
                dev_attrs = sorted(a for a in dir(device)
                                   if not a.startswith("_"))
                lines.append(f"    attributes: {', '.join(dev_attrs)}")
            except Exception as exc:
                lines.append(f"    attributes unavailable: {exc}")
            # IsMounted is the SDK's own health signal and its DisplayString
            # says WHY ("Device not available"), which distinguishes "turret
            # absent" from "no live link to the body at all".
            mounted = self._is_mounted(device)
            detail = self._mount_detail(device)
            lines.append(
                f"    IsMounted -> {'unknown' if mounted is None else mounted}"
                + (f"  ({detail})" if detail else ""))
            try:
                lines.append(f"    Name -> "
                             f"{self._param_attr(device, 'Name', '')!r}")
            except Exception:
                pass
            # Each device property is an IMipParameter — show what is inside it
            # rather than the pointer.
            for prop in ("Position", "Value"):
                param = None
                try:
                    param = self._param(device, prop)
                except MicroscopeError as exc:
                    lines.append(f"    {prop} unavailable: {exc}")
                if param is None:
                    continue
                bits = []
                for field in ("RawValue", "DisplayString", "RangeLowerLimit",
                              "RangeHigherLimit", "RangeIncrement", "Unit",
                              "IsReadOnly"):
                    value = self._param_attr(param, field)
                    if value is not None:
                        bits.append(f"{field}={value!r}")
                lines.append(f"    {prop} -> "
                             + (", ".join(bits) if bits else "<no fields>"))
            if logical == "focus":
                lines.append(f"    focus scale -> "
                             f"{self.focus_units_per_um()} device units per µm "
                             f"(configured fallback {self._z_units_per_um})")
            # The DERIVED semantics of the v7.17 accessories — the part that is
            # not yet hardware-verified, so it is exactly what a bench session
            # needs to see. Read these before enabling the shutter interlock.
            elif logical == "epi_shutter":
                # FITTED is the first question: this SDK publishes the COM
                # object either way, and an unfitted shutter is one the
                # interlock must not believe in.
                lines.append(f"    FITTED -> {self.has_epi_shutter()}"
                             + ("" if self.has_epi_shutter()
                                else "   <-- no shutter on this body; the "
                                     "cassette interlock degrades to a plain "
                                     "move"))
                lines.append(f"    IsOpened -> "
                             f"{self._describe_bool(self.epi_shutter_open())}"
                             f"   (named property, preferred over Value)")
                lines.append(
                    f"    Open()/Close() -> "
                    f"{callable(self._param_attr(device, 'Open'))}"
                    f"   (used in preference to a derived code)")
                lines.append(
                    f"    body's own IsInterlockEnabled -> "
                    f"{self._describe_bool(self.epi_shutter_interlock_enabled())}"
                    f"   (read-only; the SDK has its own interlock notion)")
                if self._param_attr(device, "IsOpened") is None:
                    closed, opened = self._shutter_codes()
                    lines.append(f"    fallback codes -> {closed}=closed, "
                                 f"{opened}=open  (no IsOpened on this SDK)")
            elif logical == "dia_lamp":
                attr, _param = self._lamp_switch()
                lines.append(f"    FITTED -> {self.has_dia_lamp()}")
                lines.append(f"    on/off -> "
                             f"{self._describe_bool(self.dia_lamp_on())}"
                             f"   via {'On()/Off()' if callable(self._param_attr(device, 'On')) else attr or 'NOTHING'}")
                lines.append(f"    level -> {self.dia_lamp_intensity()} in "
                             f"{self.dia_lamp_intensity_range()}"
                             f"   step {self._param_attr(self._param_attr(device, 'Resolution'), 'RawValue')}"
                             f" {self._param_attr(device, 'Unit') or '?'}")
                remote = self.dia_lamp_remote()
                lines.append(
                    f"    IsControlled -> "
                    + ("unknown" if remote is None
                       else ("RemoteMode (software may write)" if remote
                             else "MainMode  <-- the SDK REFUSES every software "
                                  "write until this is Remote")))
                lines.append(f"    MeasuredVoltage -> "
                             f"{self._param_attr(self._param_attr(device, 'MeasuredVoltage'), 'RawValue')}"
                             f"   (read-only; measured on this rig NOT to track "
                             f"the level, so do not read it as feedback)")
            elif logical == "light_path":
                lines.append(f"    FITTED -> "
                             f"{self._accessory_present('light_path')}")
                lines.append(f"    positions -> {self.light_path_count()}, "
                             f"now {self.get_light_path()}"
                             f" ({self.light_path_name_now() or 'unnamed'})")
        for logical in sorted(set(_TI_DEVICE_ALIASES) - set(self._devices)):
            lines.append(f"  [{logical}] not present on this body")
        lines.append(f"  z_units_per_um: {self._z_units_per_um}")
        return "\n".join(lines)

    @staticmethod
    def _describe_bool(value) -> str:
        return "unknown" if value is None else str(value)


# ── Micro-Manager backend ──────────────────────────────────────────

class MicroManagerBackend(MicroscopeBackend):
    """Drive the body through Micro-Manager's ``NikonTI`` device adapter.

    Recommended path when a Micro-Manager install is present: the adapter is
    long-proven against Ti hardware, state devices report their own labels, and
    the focus device is **µm-native** — so there is no device-unit scale factor
    to get wrong (the one number the SDK backend has to be told).

    Requires ``pymmcore`` plus a Micro-Manager configuration (.cfg) that loads
    the body's devices — **and Nikon's own driver + SDK underneath**. The
    adapter is a wrapper: per Micro-Manager's documentation, *"This adapter uses
    the driver and API supplied by Nikon"* (`NikonTI <https://micro-manager.org/
    NikonTI>`_), loading ``C:\\Program Files\\Nikon\\Shared\\Bin\\NikonTi.dll``.
    Installing Micro-Manager alone does **not** make an un-driven body reachable.

    Default device names are the adapter's published ones (``TINosePiece``,
    ``TIFilterBlock1``, ``TIZDrive``); they hang off the ``TIScope`` hub, which
    the configuration must load first. All three are overridable, since a
    configuration may relabel them.
    """

    name = "micromanager"
    display_name = "Micro-Manager (NikonTI)"

    #: Property names probed for the dia lamp's level. MM adapters differ on
    #: what they call it, and asking is cheaper than assuming.
    _LAMP_LEVEL_PROPS = ("Intensity", "Voltage", "Level", "Brightness")

    def __init__(self, config_path: str = "", mm_dir: Optional[str] = None,
                 filter_device: str = "TIFilterBlock1",
                 objective_device: str = "TINosePiece",
                 focus_device: str = "TIZDrive",
                 epi_shutter_device: str = "TIEpiShutter",
                 dia_lamp_device: str = "TIDiaLamp",
                 light_path_device: str = "TILightPath"):
        self._config_path = config_path
        self._mm_dir = mm_dir
        self._filter_device = filter_device
        self._objective_device = objective_device
        self._focus_device = focus_device
        self._epi_shutter_device = epi_shutter_device
        self._dia_lamp_device = dia_lamp_device
        self._light_path_device = light_path_device
        self._core = None
        self._loaded: set = set()

    def connect(self) -> None:
        if not self._config_path:
            raise MicroscopeError(
                "no Micro-Manager configuration file is set (Microscope "
                "settings → Micro-Manager config)")
        try:
            import pymmcore
        except Exception as exc:
            raise MicroscopeError(
                "the Micro-Manager backend needs 'pymmcore' "
                "(pip install pymmcore): " f"{exc}") from exc
        core = pymmcore.CMMCore()
        if self._mm_dir:
            try:
                core.setDeviceAdapterSearchPaths([self._mm_dir])
            except Exception as exc:
                raise MicroscopeError(
                    f"invalid Micro-Manager directory {self._mm_dir!r}: {exc}"
                ) from exc
        try:
            core.loadSystemConfiguration(self._config_path)
        except Exception as exc:
            raise MicroscopeError(
                f"failed to load {self._config_path}: {exc}") from exc
        self._core = core
        try:
            self._loaded = set(core.getLoadedDevices())
        except Exception:
            self._loaded = set()

    def disconnect(self) -> None:
        core, self._core = self._core, None
        self._loaded = set()
        if core is not None:
            try:
                core.reset()
            except Exception as exc:
                logger.debug(f"Micro-Manager reset failed: {exc}")

    @property
    def is_connected(self) -> bool:
        return self._core is not None

    def _require(self, device: str):
        if self._core is None:
            raise MicroscopeError("microscope is not connected")
        if self._loaded and device not in self._loaded:
            raise MicroscopeError(
                f"device {device!r} is not in the loaded configuration "
                f"({', '.join(sorted(self._loaded)) or 'none'})")
        return self._core

    # State devices are 0-based in Micro-Manager; the UI is 1-based.
    def _state_count(self, device: str) -> int:
        try:
            return int(self._require(device).getNumberOfStates(device))
        except Exception:
            return 0

    def _get_state(self, device: str) -> Optional[int]:
        try:
            return int(self._require(device).getState(device)) + 1
        except Exception:
            return None

    def _set_state(self, device: str, position: int) -> None:
        core = self._require(device)
        try:
            core.setState(device, int(position) - 1)
            core.waitForDevice(device)
        except Exception as exc:
            raise MicroscopeError(f"{device}: {exc}") from exc

    def _labels(self, device: str) -> tuple:
        try:
            return tuple(self._require(device).getStateLabels(device))
        except Exception:
            return ()

    def filter_count(self) -> int:
        return self._state_count(self._filter_device)

    def get_filter(self) -> Optional[int]:
        return self._get_state(self._filter_device)

    def set_filter(self, position: int) -> None:
        self._set_state(self._filter_device, position)

    def filter_names(self) -> tuple:
        return self._labels(self._filter_device)

    def objective_count(self) -> int:
        return self._state_count(self._objective_device)

    def get_objective(self) -> Optional[int]:
        return self._get_state(self._objective_device)

    def set_objective(self, position: int) -> None:
        self._set_state(self._objective_device, position)

    def objective_names(self) -> tuple:
        return self._labels(self._objective_device)

    def get_focus_um(self) -> Optional[float]:
        try:
            return float(self._require(self._focus_device)
                         .getPosition(self._focus_device))
        except Exception:
            return None

    def set_focus_um(self, value_um: float) -> None:
        core = self._require(self._focus_device)
        try:
            core.setPosition(self._focus_device, float(value_um))
            core.waitForDevice(self._focus_device)
        except Exception as exc:
            raise MicroscopeError(f"{self._focus_device}: {exc}") from exc

    # ── v7.17 accessories ───────────────────────────────────────────
    #
    # A configuration is free not to load these — they are accessories — so
    # presence is "is this device in the loaded set", and every read degrades to
    # None rather than raising. Mirrors _require()'s treatment of an
    # un-enumerable device list: if we could not read the list at all, don't
    # second-guess it, try the call and let it fail with the adapter's own words.

    def _has(self, device: str) -> bool:
        if self._core is None or not device:
            return False
        return (device in self._loaded) if self._loaded else True

    def has_epi_shutter(self) -> bool:
        return self._has(self._epi_shutter_device)

    def epi_shutter_open(self) -> Optional[bool]:
        if not self._has(self._epi_shutter_device):
            return None
        try:
            return bool(self._core.getShutterOpen(self._epi_shutter_device))
        except Exception:
            return None

    def set_epi_shutter(self, open_: bool) -> None:
        core = self._require(self._epi_shutter_device)
        try:
            core.setShutterOpen(self._epi_shutter_device, bool(open_))
            core.waitForDevice(self._epi_shutter_device)
        except Exception as exc:
            raise MicroscopeError(
                f"{self._epi_shutter_device}: {exc}") from exc

    def has_dia_lamp(self) -> bool:
        return self._has(self._dia_lamp_device)

    def dia_lamp_on(self) -> Optional[bool]:
        if not self._has(self._dia_lamp_device):
            return None
        try:
            return bool(self._core.getShutterOpen(self._dia_lamp_device))
        except Exception:
            return None

    def set_dia_lamp_on(self, on: bool) -> None:
        core = self._require(self._dia_lamp_device)
        try:
            core.setShutterOpen(self._dia_lamp_device, bool(on))
        except Exception as exc:
            raise MicroscopeError(f"{self._dia_lamp_device}: {exc}") from exc

    def _lamp_level_prop(self) -> Optional[str]:
        if self._core is None:
            return None
        for prop in self._LAMP_LEVEL_PROPS:
            try:
                if self._core.hasProperty(self._dia_lamp_device, prop):
                    return prop
            except Exception:
                continue
        return None

    def dia_lamp_intensity(self) -> Optional[float]:
        prop = self._lamp_level_prop()
        if prop is None:
            return None
        try:
            return float(self._core.getProperty(self._dia_lamp_device, prop))
        except Exception:
            return None

    def dia_lamp_intensity_range(self) -> Optional[tuple]:
        prop = self._lamp_level_prop()
        if prop is None:
            return None
        try:
            lo = float(self._core.getPropertyLowerLimit(
                self._dia_lamp_device, prop))
            hi = float(self._core.getPropertyUpperLimit(
                self._dia_lamp_device, prop))
        except Exception:
            return None
        return (lo, hi) if hi > lo else None

    def set_dia_lamp_intensity(self, value: float) -> None:
        core = self._require(self._dia_lamp_device)
        prop = self._lamp_level_prop()
        if prop is None:
            raise MicroscopeError(
                f"{self._dia_lamp_device} exposes no level property "
                f"(tried {', '.join(self._LAMP_LEVEL_PROPS)})")
        raw = float(value)
        rng = self.dia_lamp_intensity_range()
        if rng is not None:
            raw = max(rng[0], min(rng[1], raw))
        try:
            core.setProperty(self._dia_lamp_device, prop, raw)
        except Exception as exc:
            raise MicroscopeError(f"{self._dia_lamp_device}: {exc}") from exc

    def light_path_count(self) -> int:
        return (self._state_count(self._light_path_device)
                if self._has(self._light_path_device) else 0)

    def get_light_path(self) -> Optional[int]:
        if not self._has(self._light_path_device):
            return None
        return self._get_state(self._light_path_device)

    def set_light_path(self, position: int) -> None:
        self._set_state(self._light_path_device, position)

    def light_path_names(self) -> tuple:
        return (self._labels(self._light_path_device)
                if self._has(self._light_path_device) else ())

    def diagnostics(self) -> str:
        lines = [self.display_name, f"  config: {self._config_path}"]
        if self._core is None:
            lines.append("  (not connected)")
            return "\n".join(lines)
        lines.append(f"  loaded devices: {', '.join(sorted(self._loaded))}")
        for label, device in (("filter", self._filter_device),
                              ("objective", self._objective_device),
                              ("focus", self._focus_device),
                              ("light path", self._light_path_device)):
            lines.append(f"  [{label}] {device}")
            if label == "focus":
                lines.append(f"    position: {self.get_focus_um()} µm")
            else:
                lines.append(f"    states: {self._state_count(device)} "
                             f"labels: {', '.join(self._labels(device))}")
        lines.append(f"  [epi shutter] {self._epi_shutter_device}")
        lines.append(f"    present: {self.has_epi_shutter()}  "
                     f"open: {self.epi_shutter_open()}")
        lines.append(f"  [dia lamp] {self._dia_lamp_device}")
        lines.append(f"    present: {self.has_dia_lamp()}  "
                     f"on: {self.dia_lamp_on()}  "
                     f"level property: {self._lamp_level_prop()}  "
                     f"level: {self.dia_lamp_intensity()} "
                     f"in {self.dia_lamp_intensity_range()}")
        return "\n".join(lines)


# ── Backend factory ────────────────────────────────────────────────

def build_backend(name: str, **kwargs) -> MicroscopeBackend:
    """Construct a backend by identifier. Raises for an unknown name."""
    if name == "simulated":
        return SimulatedMicroscopeBackend(**kwargs)
    if name == "nikon_ti":
        return NikonTiSdkBackend(**kwargs)
    if name == "micromanager":
        return MicroManagerBackend(**kwargs)
    raise MicroscopeError(f"unknown microscope backend: {name!r}")


# ── Controller ─────────────────────────────────────────────────────

@dataclass
class _Op:
    name: str
    fn: Callable[[], None]
    done: threading.Event = field(default_factory=threading.Event)
    error: Optional[str] = None


class MicroscopeController:
    """Thread-safe facade over a :class:`MicroscopeBackend`.

    All hardware access is serialized onto ONE dedicated worker thread (COM
    apartment affinity + never blocking the Qt event loop). Callers submit
    operations and read the cached :class:`MicroscopeState`.

    Set ``threaded=False`` to run operations inline on the calling thread — used
    by the tests so assertions are deterministic without sleeps.
    """

    #: Ops older than this are dropped from the queue rather than executed late
    #: — a queued turret move whose button was clicked 30 s ago is not what the
    #: operator wants to happen now.
    STALE_OP_S = 20.0

    def __init__(self, store=None, *, threaded: bool = True):
        if store is None:
            from SupportClasses.MicroscopeConfigStore import get_store
            store = get_store()
        self._store = store
        self._threaded = bool(threaded)
        self._backend: Optional[MicroscopeBackend] = None
        self._state = MicroscopeState()
        self._state_lock = threading.RLock()
        self._listeners: list[Callable[[MicroscopeState], None]] = []
        self._queue: deque = deque()
        self._queue_cv = threading.Condition()
        self._pending = 0
        self._shutdown = False
        self._thread: Optional[threading.Thread] = None
        # Exclusivity lease — see try_acquire().
        self._lease_owner: Optional[str] = None
        self._lease_thread: Optional[int] = None
        self._lease_lock = threading.RLock()

    # ── Exclusivity lease ─────────────────────────────────────────
    #
    # This controller is a process-wide singleton shared by the jog panel's
    # Microscope card (which polls ~1 Hz), the Hardware Setup microscope page
    # and any long-running calibration. That sharing is not benign: STALE_OP_S
    # drops any op that has waited in the queue longer than 20 s, SILENTLY, with
    # error='dropped (stale)'. A dropped set_objective during a focus survey
    # means every subsequent sample is taken through the WRONG objective — and
    # the resulting measurement looks perfectly well-formed.
    #
    # The lease is bound to the acquiring THREAD, not just to a name, because
    # that is what the hazard actually looks like: one worker thread owns the
    # body while every competing surface lives on the GUI thread. Ops submitted
    # from any other thread while a lease is held fail fast with an explanatory
    # error instead of queueing up behind the survey.

    #: Ops that must work even while another owner holds the lease — a wedged
    #: calibration must never be able to prevent releasing the hardware.
    _LEASE_EXEMPT_OPS = frozenset({"disconnect"})

    def try_acquire(self, owner: str, timeout: float = 0.0) -> bool:
        """Take exclusive ownership of the body. Re-entrant for the same thread.

        Returns False if someone else holds it. ALWAYS pair with release() in a
        finally — a leaked lease locks every other microscope surface out.
        """
        deadline = time.monotonic() + max(0.0, float(timeout))
        me = threading.get_ident()
        while True:
            with self._lease_lock:
                if self._lease_owner is None:
                    self._lease_owner = str(owner)
                    self._lease_thread = me
                    return True
                if self._lease_owner == str(owner) and self._lease_thread == me:
                    return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)

    def release(self, owner: str) -> None:
        """Release a lease taken by ``owner``. Safe to call when not held."""
        with self._lease_lock:
            if self._lease_owner == str(owner):
                self._lease_owner = None
                self._lease_thread = None

    def lease_owner(self) -> Optional[str]:
        with self._lease_lock:
            return self._lease_owner

    def _lease_blocks(self, name: str) -> Optional[str]:
        """The refusal text if this caller may not submit ``name``, else None."""
        if name in self._LEASE_EXEMPT_OPS:
            return None
        with self._lease_lock:
            owner, thr = self._lease_owner, self._lease_thread
        if owner is None or thr == threading.get_ident():
            return None
        return f"microscope is reserved by {owner}"

    # ── State / listeners ─────────────────────────────────────────

    def state(self) -> MicroscopeState:
        with self._state_lock:
            return self._state

    def add_listener(self, cb: Callable[[MicroscopeState], None]) -> None:
        """Register a state-change callback. **Called on the worker thread** —
        a Qt consumer must marshal to the GUI thread (emit a signal)."""
        self._listeners.append(cb)

    def remove_listener(self, cb) -> None:
        try:
            self._listeners.remove(cb)
        except ValueError:
            pass

    def _emit(self, **changes) -> None:
        with self._state_lock:
            self._state = replace(self._state, **changes)
            snapshot = self._state
        for cb in list(self._listeners):
            try:
                cb(snapshot)
            except Exception as exc:
                logger.debug(f"microscope listener failed: {exc}")

    # ── Worker thread ─────────────────────────────────────────────

    def _ensure_thread(self) -> None:
        if not self._threaded or self._shutdown:
            return
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(
            target=self._worker_loop, name="MicroscopeWorker", daemon=True)
        self._thread.start()

    def _worker_loop(self) -> None:
        while True:
            with self._queue_cv:
                while not self._queue and not self._shutdown:
                    self._queue_cv.wait(0.5)
                if self._shutdown and not self._queue:
                    return
                op, queued_at = self._queue.popleft()
            if (op.name != "disconnect"
                    and time.monotonic() - queued_at > self.STALE_OP_S):
                op.error = "dropped (stale)"
                op.done.set()
                self._finish_op()
                continue
            self._run_op(op)
            self._finish_op()

    def _finish_op(self) -> None:
        with self._queue_cv:
            self._pending = max(0, self._pending - 1)
            if self._pending == 0:
                self._emit(busy=False)
            self._queue_cv.notify_all()

    def _run_op(self, op: _Op) -> None:
        try:
            op.fn()
            op.error = None
        except MicroscopeError as exc:
            op.error = str(exc)
            logger.warning(f"microscope {op.name} failed: {exc}")
            self._emit(error=str(exc), last_op=op.name)
        except Exception as exc:  # never let a driver bug kill the worker
            op.error = str(exc)
            logger.exception(f"microscope {op.name} raised")
            self._emit(error=str(exc), last_op=op.name)
        finally:
            op.done.set()

    def _submit(self, name: str, fn: Callable[[], None]) -> _Op:
        op = _Op(name=name, fn=fn)
        blocked = self._lease_blocks(name)
        if blocked is not None:
            # Fail fast rather than queue. Queueing here is what produces the
            # silent 'dropped (stale)' 20 s later, by which point the caller has
            # long since assumed the move happened.
            op.error = blocked
            op.done.set()
            return op
        if not self._threaded:
            self._emit(busy=True)
            self._run_op(op)
            self._emit(busy=False)
            return op
        self._ensure_thread()
        with self._queue_cv:
            self._queue.append((op, time.monotonic()))
            self._pending += 1
            # Publish "busy" while still holding the lock: the worker cannot pop
            # the op until we release, so its busy=False can never land first
            # and leave the UI stuck showing a move that already finished.
            self._emit(busy=True)
            self._queue_cv.notify_all()
        return op

    def wait_idle(self, timeout: float = 10.0) -> bool:
        """Block until the queue drains. Returns False on timeout."""
        if not self._threaded:
            return True
        deadline = time.monotonic() + timeout
        with self._queue_cv:
            while self._pending > 0:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._queue_cv.wait(remaining)
        return True

    # ── Operations ────────────────────────────────────────────────

    def connect(self, backend_name: Optional[str] = None) -> _Op:
        """Build the configured backend and connect. Any previous backend is
        disconnected first."""
        name = backend_name or self._store.get_backend()
        kwargs = (self._store.backend_kwargs()
                  if backend_name in (None, self._store.get_backend()) else {})

        def _do():
            self._close_backend()
            backend = build_backend(name, **kwargs)
            backend.connect()
            self._backend = backend
            self._emit(backend=name, connected=True, error=None,
                       last_op="connect")
            self._read_all(include_mounted=True)

        return self._submit("connect", _do)

    def disconnect(self) -> _Op:
        def _do():
            self._close_backend()
            self._emit(backend="none", connected=False, busy=False,
                       filter_position=None, objective_position=None,
                       focus_um=None, mounted_filters=(),
                       mounted_objectives=(), error=None,
                       last_op="disconnect",
                       # Accessories too, or a reconnect to a different body
                       # inherits the previous one's shutter/lamp readout.
                       epi_shutter_present=False, epi_shutter_open=None,
                       dia_lamp_present=False, dia_lamp_on=None,
                       dia_lamp_remote=None,
                       dia_lamp_intensity=None, dia_lamp_min=None,
                       dia_lamp_max=None, light_path_position=None,
                       light_path_count=0, native_light_path_names=())

        return self._submit("disconnect", _do)

    def _close_backend(self) -> None:
        backend, self._backend = self._backend, None
        if backend is None:
            return
        try:
            backend.disconnect()
        except Exception as exc:
            logger.debug(f"microscope disconnect failed: {exc}")

    def refresh(self) -> _Op:
        """Re-read turret positions + focus from hardware."""
        return self._submit("refresh", self._read_all)

    def refresh_mounted(self) -> _Op:
        """Re-read which optics are physically fitted.

        Separate from :meth:`refresh` because optics only change when someone
        swaps them by hand — putting a dozen extra COM calls on the ~1 s poll
        would be pure waste.
        """
        return self._submit(
            "refresh_mounted", lambda: self._read_all(include_mounted=True))

    def set_filter(self, position: int) -> _Op:
        """Rotate the cassette — with the epi shutter interlock, if enabled.

        The interlock lives HERE, in the one chokepoint every cassette move
        reaches (the jog card's combo, the Hardware Setup slot table's ``Go``
        button, and anything added later), rather than in a separate
        ``set_filter_interlocked``. A second entry point is how the ``Go`` button
        would have ended up unprotected.

        Close → move → restore is one submitted op on purpose: this controller is
        a process-wide singleton whose jog card polls it about once a second, so
        three separate ops would let a ``refresh`` land between the close and the
        move.
        """
        def _do():
            backend = self._require()
            self._with_shutter_closed(
                backend, lambda: backend.set_filter(int(position)))
            self._emit(error=None, last_op="set_filter")
            self._read_all()

        return self._submit("set_filter", _do)

    # ── The epi-shutter interlock ─────────────────────────────────

    def _with_shutter_closed(self, backend, fn) -> None:
        """Run ``fn`` with the epi shutter closed, then put it BACK as it was.

        Rotating the cassette with the excitation shutter open sweeps the
        excitation beam across every cube that passes through the light path:
        the sample is flashed with out-of-band excitation and any camera
        integrating at that moment gets a bright artefact frame.

        Three properties this has to have, each of which is a way to get it
        wrong:

        * **Restore the PREVIOUS state, never "open".** A shutter the operator
          had deliberately closed must stay closed — reopening it would
          illuminate a sample they had gone dark on.
        * **Restore in a ``finally``.** A cassette move that fails mid-rotation
          must still reopen the shutter, or the next acquisition is black with
          nothing on screen explaining why.
        * **Do nothing at all when the prior state is unknown.** If the shutter
          cannot be read we cannot restore it, and closing it on the way in would
          leave the body in a state we invented rather than one we found.
        """
        if not self._interlock_enabled() or not backend.has_epi_shutter():
            fn()
            return
        invert = self._shutter_invert()
        was_open = self._apply_invert(backend.epi_shutter_open(), invert)
        if was_open is None:
            logger.warning(
                "microscope: epi-shutter interlock skipped — the shutter's state "
                "could not be read, and a state we cannot read is one we cannot "
                "restore")
            fn()
            return
        if not was_open:
            fn()          # already closed: nothing to protect, nothing to restore
            return

        self._set_shutter(backend, False, invert)
        moved = False
        try:
            fn()
            moved = True
        finally:
            try:
                self._set_shutter(backend, True, invert)
            except Exception as exc:
                if moved:
                    # The cube changed but the light is still off. That is worth
                    # an error of its own: every following image would be black.
                    raise MicroscopeError(
                        "the filter cube changed, but reopening the epi shutter "
                        f"failed — the light path is still closed: {exc}") from exc
                logger.error(
                    "microscope: the epi shutter did not reopen after a failed "
                    f"filter move: {exc}")

    def _interlock_enabled(self) -> bool:
        return bool(self._store.filter_shutter_interlock())

    def _shutter_invert(self) -> bool:
        return bool(self._store.epi_shutter_invert())

    @staticmethod
    def _apply_invert(value, invert):
        """Apply the operator's inversion override. ``None`` stays ``None``.

        Pure and total on purpose: it is called from inside ``_read_all``, whose
        per-device reads are exception-swallowed, so anything that could raise
        here would turn a policy flag into a silently missing readout.
        """
        return None if value is None else (bool(value) != bool(invert))

    def _set_shutter(self, backend, open_: bool, invert: bool) -> None:
        """Drive the shutter in OPERATOR terms, applying the inversion override."""
        backend.set_epi_shutter(bool(open_) != bool(invert))

    def set_epi_shutter(self, open_: bool) -> _Op:
        """Open/close the epi (excitation) shutter. ``open_`` is in operator terms."""
        want = bool(open_)

        def _do():
            backend = self._require()
            self._set_shutter(backend, want, self._shutter_invert())
            self._emit(error=None, last_op="set_epi_shutter")
            self._read_all()

        return self._submit("set_epi_shutter", _do)

    def set_dia_lamp_on(self, on: bool) -> _Op:
        want = bool(on)

        def _do():
            self._require().set_dia_lamp_on(want)
            self._emit(error=None, last_op="set_dia_lamp_on")
            self._read_all()

        return self._submit("set_dia_lamp_on", _do)

    def set_dia_lamp_remote(self, on: bool) -> _Op:
        """Take (or release) software control of the dia lamp.

        Explicit, because RemoteMode takes the lamp away from the knob on the
        microscope — a change to what the person standing at the body can do, so
        it is not something a slider should cause as a side effect.
        """
        want = bool(on)

        def _do():
            self._require().set_dia_lamp_remote(want)
            self._emit(error=None, last_op="set_dia_lamp_remote")
            self._read_all()

        return self._submit("set_dia_lamp_remote", _do)

    def set_dia_lamp_intensity(self, value: float) -> _Op:
        """Set the lamp level, in the device's own declared units."""
        level = float(value)

        def _do():
            self._require().set_dia_lamp_intensity(level)
            self._emit(error=None, last_op="set_dia_lamp_intensity")
            self._read_all()

        return self._submit("set_dia_lamp_intensity", _do)

    def set_light_path(self, position: int) -> _Op:
        """Select the eyepiece / camera port.

        ⚠ Selecting a port the camera is not on makes every acquired frame
        black — which downstream reads as an exposure or focus fault, not as a
        light-path setting. There is deliberately no new guard for that: a scan
        or calibration holding this controller's exclusivity lease already makes
        this op fail fast with *"microscope is reserved by …"*.
        """
        def _do():
            self._require().set_light_path(int(position))
            self._emit(error=None, last_op="set_light_path")
            self._read_all()

        return self._submit("set_light_path", _do)

    def set_objective(self, position: int) -> _Op:
        def _do():
            self._require().set_objective(int(position))
            self._emit(error=None, last_op="set_objective")
            self._read_all()

        return self._submit("set_objective", _do)

    def set_focus_um(self, value_um: float) -> _Op:
        target = self._clamp_focus(float(value_um))

        def _do():
            self._require().set_focus_um(target)
            self._emit(error=None, last_op="set_focus")
            self._read_all()

        return self._submit("set_focus", _do)

    def move_focus_um(self, delta_um: float) -> _Op:
        """Relative focus move, clamped to the operator soft limits."""
        delta = float(delta_um)

        def _do():
            backend = self._require()
            current = backend.get_focus_um()
            if current is None:
                raise MicroscopeError("focus position is unknown")
            backend.set_focus_um(self._clamp_focus(current + delta))
            self._emit(error=None, last_op="move_focus")
            self._read_all()

        return self._submit("move_focus", _do)

    def diagnostics(self) -> str:
        backend = self._backend
        if backend is None:
            return "Microscope is not connected."
        try:
            return backend.diagnostics()
        except Exception as exc:
            return f"Diagnostics failed: {exc}"

    def shutdown(self) -> None:
        """Disconnect and stop the worker thread (app shutdown)."""
        if self._threaded and self._thread is not None:
            self.disconnect()
            self.wait_idle(timeout=5.0)
        else:
            self._close_backend()
        self._shutdown = True
        with self._queue_cv:
            self._queue_cv.notify_all()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=3.0)
        self._thread = None

    # ── Internals ─────────────────────────────────────────────────

    def _require(self) -> MicroscopeBackend:
        backend = self._backend
        if backend is None or not backend.is_connected:
            raise MicroscopeError("microscope is not connected")
        return backend

    def _clamp_focus(self, value_um: float) -> float:
        lo, hi = self._store.focus_soft_limits_um()
        if lo is not None:
            value_um = max(lo, value_um)
        if hi is not None:
            value_um = min(hi, value_um)
        return value_um

    def _read_all(self, include_mounted: bool = False) -> None:
        """Poll every device and publish one combined state update."""
        backend = self._backend
        if backend is None or not backend.is_connected:
            self._emit(connected=False, filter_position=None,
                       objective_position=None, focus_um=None,
                       epi_shutter_open=None, dia_lamp_on=None,
                       dia_lamp_intensity=None, light_path_position=None)
            return

        def _safe(fn, default=None):
            try:
                return fn()
            except Exception as exc:
                logger.debug(f"microscope read failed: {exc}")
                return default

        if include_mounted:
            self._emit(
                mounted_filters=tuple(_safe(backend.mounted_filters, ()) or ()),
                mounted_objectives=tuple(
                    _safe(backend.mounted_objectives, ()) or ()))

        limits = _safe(backend.focus_limits_um)
        # v7.17 accessories. Polled like the turrets, not cached like the mounted
        # optics: the Ti-E has physical buttons and a lamp knob on the body, so
        # the operator can change any of these without the software and a cached
        # value would quietly disagree with the microscope in front of them.
        lamp_range = _safe(backend.dia_lamp_intensity_range)
        invert = _safe(self._shutter_invert, False)
        self._emit(
            connected=True,
            filter_position=_safe(backend.get_filter),
            filter_count=_safe(backend.filter_count, 0) or 0,
            objective_position=_safe(backend.get_objective),
            objective_count=_safe(backend.objective_count, 0) or 0,
            focus_um=_safe(backend.get_focus_um),
            focus_min_um=(limits[0] if limits else None),
            focus_max_um=(limits[1] if limits else None),
            native_filter_names=tuple(_safe(backend.filter_names, ()) or ()),
            native_objective_names=tuple(
                _safe(backend.objective_names, ()) or ()),
            epi_shutter_present=bool(_safe(backend.has_epi_shutter, False)),
            epi_shutter_open=self._apply_invert(
                _safe(backend.epi_shutter_open), invert),
            dia_lamp_present=bool(_safe(backend.has_dia_lamp, False)),
            dia_lamp_on=_safe(backend.dia_lamp_on),
            dia_lamp_remote=_safe(backend.dia_lamp_remote),
            dia_lamp_intensity=_safe(backend.dia_lamp_intensity),
            dia_lamp_min=(lamp_range[0] if lamp_range else None),
            dia_lamp_max=(lamp_range[1] if lamp_range else None),
            light_path_position=_safe(backend.get_light_path),
            light_path_count=_safe(backend.light_path_count, 0) or 0,
            native_light_path_names=tuple(
                _safe(backend.light_path_names, ()) or ()),
        )


# ── Module-level singleton ──────────────────────────────────────────

_controller: Optional[MicroscopeController] = None


def get_microscope() -> MicroscopeController:
    """Return the shared :class:`MicroscopeController`.

    A singleton because the jog context panel is instantiated on several pages
    (Jog, Calibration, every jog-capable workflow) and they must all drive — and
    observe — the same physical body over one connection.
    """
    global _controller
    if _controller is None:
        _controller = MicroscopeController()
    return _controller


def shutdown_microscope() -> None:
    """Tear down the shared controller (app close)."""
    global _controller
    if _controller is not None:
        try:
            _controller.shutdown()
        except Exception as exc:
            logger.debug(f"microscope shutdown failed: {exc}")
        _controller = None
