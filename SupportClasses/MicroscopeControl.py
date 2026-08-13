"""
MicroscopeControl.py — motorized microscope body control (Nikon Ti Eclipse).

v7.5.x: the rig gains a Nikon Ti Eclipse whose **filter-cube cassette**,
**nosepiece (objective turret)** and **focus drive (Z)** are motorized. This
module is the GUI-free backend for driving those three devices manually. It is
deliberately standalone — no print/workflow/calibration code calls into it yet
(integration into the workflows and calibrations is a later, separate step).

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
    #: Result of the last probe_optic_write_support() call — see there for the
    #: shape. {} until probed, or when the driver has no notion of this.
    optic_write_support: dict = field(default_factory=dict)

    # ── Illumination accessories (v7.17) ──────────────────────────────
    # EpiShutter / DiaLamp / LightPathDrive are ACCESSORIES: a body may not
    # have them fitted at all, and on this rig's Ti-E the epi shutter is not
    # (IsMounted=0), while the dia lamp and light path are.
    #
    # ⚠ ABSENT, UNKNOWN and KNOWN are three different readings and the GUI
    # renders them differently. ``*_present`` False means the body told us
    # there is no such device; a ``None`` value means it IS fitted and did not
    # answer. That is why the value fields default to None and not to False —
    # "closed"/"off" is a state we would be inventing, and for a shutter that
    # is the difference between "the sample is protected" and "we do not know".
    #
    # ⚠ NOTHING POPULATES THESE YET. They exist so the panel can honestly
    # report "not fitted"; the backend that reads the devices is not present on
    # any branch (see the v7.17.1 update plan). Wiring a driver must set these
    # from real reads — never default them to a comfortable value.
    epi_shutter_present: bool = False
    epi_shutter_open: Optional[bool] = None
    dia_lamp_present: bool = False
    dia_lamp_on: Optional[bool] = None
    dia_lamp_intensity: Optional[float] = None
    #: The SDK's OWN declared range — never rescaled to a percentage. The unit
    #: may be volts or an arbitrary index, and a percentage of an unknown
    #: quantity is a fabricated number that reads as measured.
    dia_lamp_min: Optional[float] = None
    dia_lamp_max: Optional[float] = None
    #: False = the body's front-panel knob owns the lamp (Ti "MainMode") and
    #: the SDK refuses every write. None = not known.
    dia_lamp_remote: Optional[bool] = None
    light_path_position: Optional[int] = None
    light_path_count: int = 0
    native_light_path_names: tuple = ()

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

    # -- can the body's OWN optics database be told what is mounted? --
    def probe_optic_write_support(self) -> dict:
        """**READ-ONLY** inspection: does this body's optics interface declare
        its name/identity fields as settable, so a rename in the app could make
        the body's own display follow?

        Commands NOTHING and writes NOTHING — it inspects the driver's own type
        information. ``{}`` means "nothing to report" (driver has no such
        notion, or no optics are enumerable). Otherwise
        ``{"filter": {<field>: True | False | None}, "objective": {...}}``,
        where the verdict is per FIELD (writability is a property of the
        interface, not of one slot) and ``None`` means undeterminable.

        ⚠ Deliberately does **not** settle the question by writing a field's
        own current value back to itself. That looks safe and is not: writing a
        device property the value it already holds is hardware-verified in this
        codebase to be destructive on at least one instrument SDK — see
        ``tucam_backend._capa_set`` (a redundant ``auto_exposure`` write reset
        the exposure to the sensor minimum and blacked out the preview). On a
        nosepiece the equivalent field (``Code``) is what resolves NA and
        working distance, which bound a focus sweep — so a corrupted one is a
        collision hazard, not a cosmetic bug.

        ⚠⚠ **A ``True`` here is NOT an answer.** Hardware-verified on the real
        Ti-E: it declares ``Name`` and ``Code`` settable and then refuses every
        write at runtime (*"Database entry cannot be modified."*). Treat this as
        "the typelib does not rule it out"; only :meth:`set_optic_name` settles
        it. See that method for the full finding.
        """
        return {}

    def set_optic_name(self, logical: str, position: int, name: str) -> str:
        """Write one optic's name into the body's own database and VERIFY it.

        ``logical`` is ``"filter"`` or ``"objective"``; ``position`` is 1-based.
        Returns the name the body reports **after** the write, so a driver that
        accepts a write and silently keeps the old value is caught (the Ti SDK
        has form here — it silently clamped an out-of-range turret index and
        reported success; see ``NikonTiSdkBackend._set_turret``).

        Raises :class:`MicroscopeError` when unsupported or refused.
        """
        raise MicroscopeError(
            "this microscope cannot be told what optics are fitted")

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

    def __init__(self, filter_slots: int = 6, objective_slots: int = 6,
                 focus_um: float = 5000.0,
                 focus_range_um: tuple = (0.0, 10000.0)):
        self._filter_slots = max(1, int(filter_slots))
        self._objective_slots = max(1, int(objective_slots))
        self._filter_pos = 1
        self._objective_pos = 1
        self._focus_um = float(focus_um)
        self._focus_range = (float(focus_range_um[0]), float(focus_range_um[1]))
        self._connected = False

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
                f"in {self._focus_range[0]:.0f}..{self._focus_range[1]:.0f}")


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
}


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
        self._devices = self._discover_devices(scope)
        if not self._devices:
            raise MicroscopeError(
                f"connected to {self._resolved_prog_id} but found none of the "
                "expected devices (nosepiece / filter cassette / Z drive). Use "
                "the Diagnostics report to see what this body exposes.")

    def _discover_devices(self, scope) -> dict:
        found = {}
        for logical, aliases in _TI_DEVICE_ALIASES.items():
            names = [a.format(n=self._cassette) for a in aliases]
            attr, device = _first_attr(scope, names)
            if device is not None:
                found[logical] = device
                logger.info(
                    f"Nikon Ti: {logical} device resolved to '{attr}'")
            else:
                logger.warning(
                    f"Nikon Ti: no {logical} device found (tried {names})")
        return found

    def disconnect(self) -> None:
        self._devices = {}
        self._scope = None
        self._focus_factor = None
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

    # -- can the body's own optics database be written to? --
    #
    # ``mounted_objectives()`` above already tells most of the story: an empty
    # position has ``Code == 0`` and every OTHER field (Name, Magnification,
    # NA, WD) then raises "No database code is associated with this optical
    # element." That is the signature of a value being RESOLVED by looking
    # ``Code`` up in a catalogue — and the SDK redistributable ships exactly
    # such catalogues (``ObjectiveNames.txt`` — 199 rows, ``FilterBlockNames.txt``
    # — 34 rows). The most plausible reading is that ``Code`` is SENSED from a
    # *coded* Nikon optic's physical ring/chip, not typed by an operator — in
    # which case the body's own display already tracks a coded optic
    # automatically, with nothing for software to push, and an UNCODED optic
    # has no ``Code`` to attach a name to either way.
    #
    # ⚠ The probe is READ-ONLY BY DESIGN and must stay that way. Settling this
    # by writing each field's own current value back to itself reads as
    # perfectly safe and is NOT: ``tucam_backend._capa_set`` records a
    # hardware-verified case where writing a device property the value it
    # ALREADY held reset the exposure to the sensor minimum. Here the analogous
    # field is ``Code``, which resolves the NA and working distance that bound a
    # focus sweep — so a corrupted one is a collision hazard, not a cosmetic
    # bug. Writability is therefore inferred from the COM wrapper's own type
    # information, and an actual write happens ONLY through the explicit,
    # single-target, read-back-verified ``set_optic_name`` below.

    #: The two identity fields a physical display could plausibly show. ``Code``
    #: is REPORTED ON but never written by ``set_optic_name`` (see above).
    _OPTIC_IDENTITY_FIELDS = ("Name", "Code")

    #: logical device -> the collection attribute holding its optics.
    _OPTIC_COLLECTIONS = {"filter": "FilterBlocks", "objective": "Objectives"}

    @staticmethod
    def _declared_writability(item, field_name: str):
        """``True``/``False`` from the COM wrapper's own type info, else ``None``.

        comtypes builds early-bound wrappers from the registered typelib and
        exposes each COM property as a Python ``property`` — get-only when the
        typelib declares no ``propput``. So the presence of a setter is
        readable without touching the instrument. (Early binding is confirmed
        for this body: ``diagnostics()`` gets real attribute names out of
        ``dir(device)``, which a late-bound dynamic dispatch would not provide.)

        ``None`` = undeterminable, which must be reported as such rather than
        guessed either way.
        """
        descriptor = getattr(type(item), field_name, None)
        if isinstance(descriptor, property):
            return descriptor.fset is not None
        # comtypes' underlying accessors, if the property itself is absent.
        if hasattr(type(item), f"_set_{field_name}"):
            return True
        if hasattr(type(item), f"_get_{field_name}"):
            return False
        return None

    def probe_optic_write_support(self) -> dict:
        out: dict[str, dict] = {}
        for logical, attr in self._OPTIC_COLLECTIONS.items():
            item = self._safe(lambda l=logical, a=attr: self._first_optic(l, a))
            if item is None:
                continue
            # Writability belongs to the INTERFACE, so one representative item
            # answers for every slot — reporting it per slot would be 12 lines
            # of identical noise implying per-slot variation that cannot exist.
            out[logical] = {name: self._declared_writability(item, name)
                            for name in self._OPTIC_IDENTITY_FIELDS}
        return out

    def _first_optic(self, logical: str, attr: str):
        """The first enumerable optic item on a collection, or ``None``."""
        coll = self._collection(logical, attr)
        count = int(self._safe(lambda: coll.Count, 0) or 0)
        for pos in range(1, count + 1):
            item = self._safe(lambda p=pos: coll.Item(p))
            if item is not None:
                return item
        return None

    def set_optic_name(self, logical: str, position: int, name: str) -> str:
        """Write ONE optic's ``Name`` into the body's database, then verify it.

        ⚠⚠ **HARDWARE-VERIFIED 2026-08-12 (real Ti-E, SDK 4.4.1.714): THIS
        BODY REFUSES. The Ti's optics database is READ-ONLY through this SDK.**
        Asked to set filter slot 4's ``Name``, the SDK answered, in its own
        words::

            Database entry cannot be modified. [Nikon.TiScope.FilterBlock.1]

        **And the typelib claimed otherwise** — ``_declared_writability``
        reported ``Name`` AND ``Code`` as settable for BOTH the cassette and the
        nosepiece (``propput`` present) on the very same body that then refused
        at runtime. So a declared setter is a **FALSE POSITIVE** here, and only
        an actual attempt settles it. That is exactly why the read-only probe
        must never be presented as an answer on its own.

        **Every other write path was tested too, and all are refused.** The
        typelib's ``IFilterBlock`` advertises writable ``ExcitationFilterCode``
        / ``DichroicMirrorCode`` / ``BarrierFilterCode`` / ``Composition``
        ("Gets or sets the codes for the optical elements in the filter block"),
        which looked like a way to declare a cube by its optical make-up
        (``Cy5`` = catalogue code 25 = excitation 20 / dichroic 11 / barrier 19
        per ``FilterCodes.txt``). Attempted on the EMPTY slot 4, all three
        refused with the **same** *"Database entry cannot be modified."*, and
        ``CanModify`` reads **0 on all six slots**, filled and empty alike.
        ``Nikon.TiScope.Database`` does expose ``FilterBlocks.Add``/``Remove``
        over a 309-entry catalogue, but that is the CATALOGUE — the live slot's
        ``Code`` is documented read-only and reports 0, so a new catalogue entry
        could not be bound to a slot anyway. The chain breaks at the sensing
        step, not the catalogue step.

        ⚠ **Micro-Manager / pymmcore cannot change this.** MM's ``NikonTI``
        adapter *wraps* this same ``NikonTi.dll`` ("This adapter uses the driver
        and API supplied by Nikon") and would take the identical refusal from
        the identical code path — the message itself lives in Nikon's
        ``MipDeviceMsg.dll``, beneath any wrapper. What MM offers is
        ``defineStateLabel``: **host-side** labels for turret positions, stored
        in MM's own configuration. That is the same kind of thing
        :class:`MicroscopeConfigStore` already provides, and it does not reach
        the body's display either.

        What the display actually follows: a ``Code`` the body SENSES from the
        fitted optic, resolved through Nikon's own catalogues in
        ``C:\\Program Files\\Nikon\\Shared\\Data\\Ti`` (``FilterBlockNames.txt``,
        ``ObjectiveNames.txt``). Verified against this rig: codes 4 / 15 / 23 →
        ``DAPI`` / ``FITC`` / ``TxRed``, matching those files' 0-based rows
        exactly (``Cy5`` is code 25). A slot reporting ``Code == 0`` is one the
        body sees nothing coded in — so there is no entry to name, and naming it
        is an app-side concern (``MicroscopeConfigStore``).

        Kept anyway, because it is what PRODUCED that answer and it is the only
        way another body / SDK generation can be settled in one call. Writes
        ``Name`` only — never ``Code`` (see the note above: ``Code`` resolves the
        NA/working distance a focus sweep is bounded by).

        Verification is not optional: this SDK has already been caught
        accepting an out-of-range turret index, clamping it and reporting
        success (``_set_turret``). So the value is read back and returned, and
        a write that did not take is raised as an error rather than reported as
        a success.
        """
        attr = self._OPTIC_COLLECTIONS.get(str(logical))
        if attr is None:
            raise MicroscopeError(
                f"unknown optic group {logical!r} (expected "
                f"{' or '.join(map(repr, self._OPTIC_COLLECTIONS))})")
        coll = self._collection(logical, attr)
        count = int(self._safe(lambda: coll.Count, 0) or 0)
        pos = int(position)
        if not 1 <= pos <= max(count, 0):
            raise MicroscopeError(
                f"{logical} slot {pos} is outside this body's 1-{count}")
        item = self._safe(lambda: coll.Item(pos))
        if item is None:
            raise MicroscopeError(
                f"the body did not return {logical} slot {pos}")

        # Nikon's OWN advertised gate, and the authority here: _IElementBase
        # exposes ``CanModify`` — "Determines if properties such as 'Name' can
        # be modified for this optical element (read-only)". Measured 0 on every
        # slot of this Ti-E, filled and empty alike, which is exactly why the
        # write is refused. Reading it first turns a COM error into a plain
        # explanation, and asks the SDK instead of guessing from the typelib
        # (which advertises setters it does not honour — see the docstring).
        can_modify = self._safe(lambda: item.CanModify)
        if can_modify is not None and not bool(can_modify):
            raise MicroscopeError(
                f"the body reports this {logical} entry as not modifiable "
                f"(CanModify=0), so its name is fixed. It is resolved from the "
                f"optic's own hardware code through Nikon's catalogue, not set "
                f"by software — name it in the app instead")

        declared = self._declared_writability(item, "Name")
        if declared is False:
            raise MicroscopeError(
                f"this body declares {logical} Name as read-only — it is "
                "resolved from the optic's own hardware code, not set by "
                "software, so there is nothing to write")
        before = self._safe(lambda: item.Name)
        try:
            item.Name = str(name)
        except COMError as exc:
            raise MicroscopeError(_com_message(exc)) from exc
        except Exception as exc:
            raise MicroscopeError(
                f"the SDK refused to set {logical} slot {pos} Name: {exc}"
            ) from exc

        after = self._safe(lambda: item.Name)
        after_txt = "" if after is None else str(after).strip()
        if after_txt != str(name).strip():
            raise MicroscopeError(
                f"the SDK accepted the write but {logical} slot {pos} still "
                f"reports {after_txt!r} (was {before!r}, asked for "
                f"{str(name)!r}) — the name did not take")
        logger.info(f"Nikon Ti: {logical} slot {pos} Name {before!r} -> "
                    f"{after_txt!r}")
        return after_txt

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
        lines.append(f"  z_units_per_um: {self._z_units_per_um}")
        return "\n".join(lines)


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

    def __init__(self, config_path: str = "", mm_dir: Optional[str] = None,
                 filter_device: str = "TIFilterBlock1",
                 objective_device: str = "TINosePiece",
                 focus_device: str = "TIZDrive"):
        self._config_path = config_path
        self._mm_dir = mm_dir
        self._filter_device = filter_device
        self._objective_device = objective_device
        self._focus_device = focus_device
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

    def diagnostics(self) -> str:
        lines = [self.display_name, f"  config: {self._config_path}"]
        if self._core is None:
            lines.append("  (not connected)")
            return "\n".join(lines)
        lines.append(f"  loaded devices: {', '.join(sorted(self._loaded))}")
        for label, device in (("filter", self._filter_device),
                              ("objective", self._objective_device),
                              ("focus", self._focus_device)):
            lines.append(f"  [{label}] {device}")
            if label == "focus":
                lines.append(f"    position: {self.get_focus_um()} µm")
            else:
                lines.append(f"    states: {self._state_count(device)} "
                             f"labels: {', '.join(self._labels(device))}")
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
    #: Set by ops that produce a value (e.g. the name a body reports back after
    #: a write). Only meaningful once ``done`` is set and ``error`` is None.
    result: Optional[object] = None


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
        return self._submit_op(_Op(name=name, fn=fn))

    def _submit_op(self, op: _Op) -> _Op:
        """Queue an already-built op — for ops whose ``fn`` needs to close over
        the op itself (to report a result back to the caller). Inline
        (``threaded=False``) execution happens here, so the op MUST be fully
        constructed before this is called."""
        name = op.name
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
                       mounted_objectives=(), optic_write_support={},
                       error=None, last_op="disconnect")

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

    def probe_optic_write_support(self) -> _Op:
        """READ-ONLY: see MicroscopeBackend.probe_optic_write_support.

        Read the result from ``state().optic_write_support`` once this op's
        ``.done`` (or ``wait_idle()``) confirms it has landed.
        """
        def _do():
            result = self._require().probe_optic_write_support()
            self._emit(optic_write_support=dict(result), error=None,
                       last_op="probe_optic_write_support")

        return self._submit("probe_optic_write_support", _do)

    def set_optic_name(self, logical: str, position: int, name: str) -> _Op:
        """Write one optic's name into the BODY's own database.

        The only operation in this module that mutates the body's optics
        configuration. On success ``op.result`` holds the name the body reports
        back afterwards; a write that did not take is an error, not a success.
        """
        op = _Op(name="set_optic_name", fn=lambda: None)

        def _do():
            op.result = self._require().set_optic_name(
                str(logical), int(position), str(name))
            self._emit(error=None, last_op="set_optic_name")
            self._read_all(include_mounted=True)

        op.fn = _do
        return self._submit_op(op)

    def set_filter(self, position: int) -> _Op:
        def _do():
            self._require().set_filter(int(position))
            self._emit(error=None, last_op="set_filter")
            self._read_all()

        return self._submit("set_filter", _do)

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
                       objective_position=None, focus_um=None)
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
