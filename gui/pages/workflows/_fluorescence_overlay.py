"""_fluorescence_overlay.py — shared helper for showing captured fluorescence
mosaics as a registered background in any JogWorkspaceView-family widget.

The **Fluorescence Mosaic** workflow captures per-channel single-well mosaics and
persists them (``FluorescenceMosaicStore``, keyed by plate + well). This helper
lets every other workflow (Spheroid Pick & Place, Cell Targeting, Cell Labeling,
the Jog plate view, …) blend those channels into a false-colour image and push it
onto the view's fluorescence overlay layer — registered by absolute stage µm so
it lands on the right wells.

``view`` must be a ``JogWorkspaceView`` (or subclass, e.g. ``WorkspaceTargetView``)
— it provides ``pixmap_from_bgr`` / ``set_fluor_overlay`` / ``set_fluor_visible``.
"""

from __future__ import annotations

import logging

from SupportClasses import FluorescenceMosaicStore as _fms

logger = logging.getLogger(__name__)


def plate_key_of(hw_config) -> str | None:
    """Resolve the active plate key from a hardware config (or None)."""
    if hw_config is None:
        return None
    key = getattr(hw_config, "active_plate_key", None)
    if key:
        return str(key)
    key = getattr(hw_config, "plate_name", None) or getattr(hw_config, "plate_format", None)
    return str(key) if key else None


def load_plate_fluor_overlay(view, plate_key, channels=None,
                             visible: bool = True) -> bool:
    """Blend EVERY captured well of ``plate_key`` and push it onto ``view``'s
    fluorescence overlay. Returns True if an overlay was set, False if there is
    nothing stored (clears the overlay in that case)."""
    if view is None or plate_key is None or not hasattr(view, "set_fluor_overlay"):
        return False
    try:
        store = _fms.get_store()
        image, extent = store.composite_plate_overlay(plate_key, channels)
        if image is None or extent is None:
            view.set_fluor_overlay(None, None)
            return False
        pix = _pixmap_from_bgr(view, image)
        if pix is None:
            return False
        view.set_fluor_overlay(pix, extent)
        view.set_fluor_visible(bool(visible))
        return True
    except Exception as exc:
        logger.debug("load_plate_fluor_overlay failed: %s", exc)
        return False


def load_well_fluor_overlay(view, plate_key, well_name, channels=None,
                            visible: bool = True) -> bool:
    """Blend the captured channels of ONE well and push it onto ``view``'s
    fluorescence overlay. Returns True if an overlay was set."""
    if view is None or plate_key is None or not hasattr(view, "set_fluor_overlay"):
        return False
    try:
        store = _fms.get_store()
        image, extent = store.composite_overlay(plate_key, well_name, channels)
        if image is None or extent is None:
            return False
        pix = _pixmap_from_bgr(view, image)
        if pix is None:
            return False
        view.set_fluor_overlay(pix, extent)
        view.set_fluor_visible(bool(visible))
        return True
    except Exception as exc:
        logger.debug("load_well_fluor_overlay failed: %s", exc)
        return False


def has_any_fluor(plate_key) -> bool:
    """True if any well of ``plate_key`` has a stored fluorescence channel."""
    if plate_key is None:
        return False
    try:
        return bool(_fms.get_store().list_wells(plate_key))
    except Exception:
        return False


def _pixmap_from_bgr(view, image):
    """Use the view's own pixmap_from_bgr if present, else the module helper."""
    fn = getattr(view, "pixmap_from_bgr", None)
    if callable(fn):
        return fn(image)
    try:
        from gui.widgets.jog_workspace_view import pixmap_from_bgr
        return pixmap_from_bgr(image)
    except Exception:
        return None
