"""
camera_identity.py — Stable per-device camera identity on Windows (v7.5.x).

OpenCV opens UVC cameras by integer index and exposes no friendly name or
device path. The Windows DirectShow device enumeration does: each video input
device carries a ``FriendlyName`` (the model, e.g. ``"Teslong Camera"``) and a
``DevicePath`` that encodes VID/PID **and** the USB topology (hub/port). Two
physically-identical cameras share a name + VID/PID but get distinct
DevicePaths, so the path is a stable *per-port* identity.

We key µm/px calibration by this identity ("this camera on this port") rather
than by slot index, so a calibration follows the physical camera across slot
reassignments and restarts. Moving a camera to a different USB port yields a
new identity → recalibrate, by design.

Index alignment: OpenCV's ``CAP_DSHOW`` backend enumerates cameras in the same
order as this DirectShow enumeration, so list position == OpenCV index when
cameras are opened with ``CAP_DSHOW`` (see ``camera_widget``).

Everything degrades gracefully — empty list / fallback identities — on
non-Windows, without pygrabber/comtypes, or on any COM error.
"""

from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)

# DirectShow CLSID_VideoInputDeviceCategory.
_VIDEO_INPUT_CATEGORY = "{860BB310-5D01-11d0-BD3B-00A0C911CE86}"


def enumerate_directshow_cameras() -> list[dict]:
    """Return ``[{"index", "name", "device_path"}]`` in DirectShow order.

    ``index`` is the enumeration position (== OpenCV ``CAP_DSHOW`` index).
    Returns ``[]`` on any failure (non-Windows, missing deps, COM error).
    """
    try:
        from pygrabber.dshow_graph import SystemDeviceEnum, IPropertyBag
        from comtypes import GUID
    except Exception:
        return []
    try:
        sde = SystemDeviceEnum()
        enum = sde.system_device_enum.CreateClassEnumerator(
            GUID(_VIDEO_INPUT_CATEGORY), dwFlags=0)
        if enum is None:
            return []
        out: list[dict] = []
        idx = 0
        while True:
            try:
                moniker, count = enum.Next(1)
            except ValueError:
                break
            if not count:
                break
            name = f"Camera {idx}"
            path = ""
            try:
                pb = moniker.BindToStorage(
                    0, 0, IPropertyBag._iid_).QueryInterface(IPropertyBag)
                try:
                    name = str(pb.Read("FriendlyName", pErrorLog=None))
                except Exception:
                    pass
                try:
                    path = str(pb.Read("DevicePath", pErrorLog=None))
                except Exception:
                    path = ""
            except Exception:
                pass
            out.append({"index": idx, "name": name, "device_path": path})
            idx += 1
        return out
    except Exception as exc:
        logger.debug("DirectShow camera enumeration failed: %s", exc)
        return []


def short_port_tag(device_path: str) -> str:
    """Extract a compact USB-instance tag from a DevicePath, or ``""``.

    ``\\\\?\\usb#vid_f007&pid_a999&mi_00#6&29d1719c&2&0000#{guid}\\global``
    → ``"6&29d1719c&2"`` (drops the trailing ``&0000`` interface qualifier).
    """
    if not device_path:
        return ""
    parts = device_path.split("#")
    # ['\\\\?\\usb', 'vid_..&pid_..&mi_00', '6&29d1719c&2&0000', '{guid}\\global']
    if len(parts) >= 3:
        seg = parts[2]
        bits = seg.split("&")
        if len(bits) >= 3:
            return "&".join(bits[:3])
        return seg
    return ""


def label_for(name: str, device_path: str) -> str:
    """Human-readable combo label: ``"Teslong Camera (port 6&29d1719c&2)"``."""
    tag = short_port_tag(device_path)
    return f"{name} (port {tag})" if tag else name


def identity_for_source(
    source, ds_cameras: list[dict]
) -> Optional[tuple[str, str]]:
    """Resolve a CameraManager source tuple → ``(identity_key, friendly_name)``.

    ``identity_key`` is stable across restarts (used to key the calibration
    store); ``friendly_name`` is for display. Returns ``None`` for an empty /
    unrecognized source.

    - ``("opencv", idx)`` → ``("dshow:<device_path>", name)`` when a path is
      available, else ``("opencv:<idx>", "Camera <idx>")`` (index fallback).
    - ``("toupcam", id)`` → ``("toupcam:<id>", "ToupCam")``.
    - ``("simulated", mode)`` → ``("simulated:<mode>", "Simulated Camera")``.
    """
    if not source or not isinstance(source, (tuple, list)) or len(source) != 2:
        return None
    kind, value = source[0], source[1]
    if kind == "opencv":
        entry = next(
            (c for c in ds_cameras if c.get("index") == value), None)
        if entry is None or not entry.get("device_path"):
            return (f"opencv:{value}", f"Camera {value}")
        return (
            f"dshow:{entry['device_path']}",
            entry.get("name") or f"Camera {value}",
        )
    if kind == "toupcam":
        return (f"toupcam:{value}", "ToupCam")
    if kind == "andor":
        return (f"andor:{value}", "Andor Zyla")
    if kind == "simulated":
        return (f"simulated:{value}", "Simulated Camera")
    return None


def source_for_identity(identity: str, ds_cameras: list[dict]):
    """Inverse of ``identity_for_source`` — resolve a stored identity key back
    to a CameraManager source tuple given the current device enumeration.

    Returns the source tuple (e.g. ``("opencv", 1)``) or ``None`` if no
    currently-present device matches (camera unplugged / different port).
    """
    if not identity or not isinstance(identity, str):
        return None
    if identity.startswith("dshow:"):
        path = identity[len("dshow:"):]
        for c in ds_cameras:
            if c.get("device_path") == path:
                return ("opencv", c.get("index"))
        return None
    if identity.startswith("opencv:"):
        try:
            return ("opencv", int(identity[len("opencv:"):]))
        except ValueError:
            return None
    if identity.startswith("toupcam:"):
        return ("toupcam", identity[len("toupcam:"):])
    if identity.startswith("andor:"):
        return ("andor", identity[len("andor:"):])
    if identity.startswith("simulated:"):
        return ("simulated", identity[len("simulated:"):])
    return None
