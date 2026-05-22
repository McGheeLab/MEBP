"""Inline-SVG icon factory (v7.4.2).

Self-contained icon set. No external dep — SVG strings are rendered
to ``QPixmap`` via ``QSvgRenderer`` and wrapped in a ``QIcon`` that
the rest of the UI can hang on buttons / actions / etc.

Style: Lucide-inspired stroke icons (2-px stroke, round caps, viewbox
0 0 24 24). Single color via ``currentColor`` substitution — the
default is the Catppuccin Mocha ``text`` color so icons read as
"solid color" against any of the app's dark surfaces.

Usage::

    from gui.widgets.icons import icon, icon_button

    btn = icon_button("Connect", "plug", object_name="successBtn")
    btn.clicked.connect(self._connect)

    # Or hand-roll if you need finer control:
    btn = QPushButton("Save")
    btn.setIcon(icon("save"))
    btn.setIconSize(QSize(16, 16))

Icons live in a dict keyed by short names so callers don't have to
remember the full SVG path. Adding a new one is a one-liner ``_ICONS["name"] = "<svg…>"``.
"""

from __future__ import annotations

from PySide6.QtCore import QByteArray, QSize, Qt
from PySide6.QtGui import QIcon, QPainter, QPixmap
from PySide6.QtSvg import QSvgRenderer
from PySide6.QtWidgets import QPushButton

from gui.scaling import s
from gui.styles import COLORS


# ── Icon SVG definitions ────────────────────────────────────────
#
# All icons use ``currentColor`` for the stroke / fill so a single
# template can be tinted to any color at render-time.
# Source: hand-drawn in the Lucide style (MIT-licensed siblings).

_SVG_HEADER = (
    '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" '
    'fill="none" stroke="currentColor" stroke-width="2" '
    'stroke-linecap="round" stroke-linejoin="round">'
)


def _icon(svg_body: str) -> str:
    """Wrap a body fragment in the standard SVG header/footer."""
    return f"{_SVG_HEADER}{svg_body}</svg>"


_ICONS: dict[str, str] = {
    # power / plug
    "plug": _icon(
        '<path d="M12 22v-5"/><path d="M9 7V2"/><path d="M15 7V2"/>'
        '<path d="M6 13V8h12v5a4 4 0 0 1-4 4h-4a4 4 0 0 1-4-4z"/>'
    ),
    # disconnect / close
    "x": _icon('<path d="M18 6 6 18"/><path d="m6 6 12 12"/>'),
    "x-circle": _icon(
        '<circle cx="12" cy="12" r="10"/>'
        '<path d="m15 9-6 6"/><path d="m9 9 6 6"/>'
    ),
    # save (floppy)
    "save": _icon(
        '<path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"/>'
        '<polyline points="17 21 17 13 7 13 7 21"/>'
        '<polyline points="7 3 7 8 15 8"/>'
    ),
    # folder (load / import)
    "folder": _icon(
        '<path d="M20 20a2 2 0 0 0 2-2V8a2 2 0 0 0-2-2h-7.93a2 2 0 0 1-1.66-.9l-.82-1.2A2 2 0 0 0 7.93 3H4a2 2 0 0 0-2 2v13a2 2 0 0 0 2 2Z"/>'
    ),
    "folder-open": _icon(
        '<path d="m6 14 1.5-2.9A2 2 0 0 1 9.24 10H20a2 2 0 0 1 1.94 2.5l-1.55 6a2 2 0 0 1-1.94 1.5H4a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h3.93a2 2 0 0 1 1.66.9l.82 1.2a2 2 0 0 0 1.66.9H18a2 2 0 0 1 2 2v2"/>'
    ),
    # refresh / rotate
    "refresh": _icon(
        '<path d="M3 12a9 9 0 0 1 9-9 9.75 9.75 0 0 1 6.74 2.74L21 8"/>'
        '<path d="M21 3v5h-5"/>'
        '<path d="M21 12a9 9 0 0 1-9 9 9.75 9.75 0 0 1-6.74-2.74L3 16"/>'
        '<path d="M8 16H3v5"/>'
    ),
    # plus (add)
    "plus": _icon('<path d="M5 12h14"/><path d="M12 5v14"/>'),
    # minus
    "minus": _icon('<path d="M5 12h14"/>'),
    # search / detect
    "search": _icon(
        '<circle cx="11" cy="11" r="8"/>'
        '<path d="m21 21-4.3-4.3"/>'
    ),
    # ruler (calibrate)
    "ruler": _icon(
        '<path d="M21.3 15.3a2.4 2.4 0 0 1 0 3.4l-2.6 2.6a2.4 2.4 0 0 1-3.4 0L2.7 8.7a2.41 2.41 0 0 1 0-3.4l2.6-2.6a2.41 2.41 0 0 1 3.4 0Z"/>'
        '<path d="m14.5 12.5 2-2"/>'
        '<path d="m11.5 9.5 2-2"/>'
        '<path d="m8.5 6.5 2-2"/>'
        '<path d="m17.5 15.5 2-2"/>'
    ),
    # gamepad
    "gamepad": _icon(
        '<line x1="6" x2="10" y1="12" y2="12"/>'
        '<line x1="8" x2="8" y1="10" y2="14"/>'
        '<line x1="15" x2="15.01" y1="13" y2="13"/>'
        '<line x1="18" x2="18.01" y1="11" y2="11"/>'
        '<rect width="20" height="12" x="2" y="6" rx="2"/>'
    ),
    # alert / warning
    "alert": _icon(
        '<path d="M10.29 3.86 1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"/>'
        '<line x1="12" x2="12" y1="9" y2="13"/>'
        '<line x1="12" x2="12.01" y1="17" y2="17"/>'
    ),
    # check
    "check": _icon('<path d="M20 6 9 17l-5-5"/>'),
    "check-circle": _icon(
        '<path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"/>'
        '<path d="m9 11 3 3L22 4"/>'
    ),
    # stop (emergency)
    "stop": _icon(
        '<circle cx="12" cy="12" r="10"/>'
        '<rect x="9" y="9" width="6" height="6" rx="1"/>'
    ),
    # arrow-up / arrow-down (used by jog)
    "arrow-up": _icon('<path d="M12 19V5"/><path d="m5 12 7-7 7 7"/>'),
    "arrow-down": _icon('<path d="M12 5v14"/><path d="m19 12-7 7-7-7"/>'),
    # settings cog
    "settings": _icon(
        '<path d="M12.22 2h-.44a2 2 0 0 0-2 2v.18a2 2 0 0 1-1 1.73l-.43.25a2 2 0 0 1-2 0l-.15-.08a2 2 0 0 0-2.73.73l-.22.38a2 2 0 0 0 .73 2.73l.15.1a2 2 0 0 1 1 1.72v.51a2 2 0 0 1-1 1.74l-.15.09a2 2 0 0 0-.73 2.73l.22.38a2 2 0 0 0 2.73.73l.15-.08a2 2 0 0 1 2 0l.43.25a2 2 0 0 1 1 1.73V20a2 2 0 0 0 2 2h.44a2 2 0 0 0 2-2v-.18a2 2 0 0 1 1-1.73l.43-.25a2 2 0 0 1 2 0l.15.08a2 2 0 0 0 2.73-.73l.22-.39a2 2 0 0 0-.73-2.73l-.15-.08a2 2 0 0 1-1-1.74v-.5a2 2 0 0 1 1-1.74l.15-.09a2 2 0 0 0 .73-2.73l-.22-.38a2 2 0 0 0-2.73-.73l-.15.08a2 2 0 0 1-2 0l-.43-.25a2 2 0 0 1-1-1.73V4a2 2 0 0 0-2-2z"/>'
        '<circle cx="12" cy="12" r="3"/>'
    ),
    # info
    "info": _icon(
        '<circle cx="12" cy="12" r="10"/>'
        '<path d="M12 16v-4"/><path d="M12 8h.01"/>'
    ),
    # trash (delete)
    "trash": _icon(
        '<path d="M3 6h18"/>'
        '<path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6"/>'
        '<path d="M8 6V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/>'
    ),
    # pencil (edit)
    "pencil": _icon(
        '<path d="M21.174 6.812a1 1 0 0 0-3.986-3.987L3.842 16.174a2 2 0 0 0-.5.83l-1.321 4.352a.5.5 0 0 0 .623.622l4.353-1.32a2 2 0 0 0 .83-.497z"/>'
    ),
    # zap / power (alternate)
    "zap": _icon(
        '<path d="M4 14a1 1 0 0 1-.78-1.63l9.9-10.2a.5.5 0 0 1 .86.46l-1.92 6.02A1 1 0 0 0 13 10h7a1 1 0 0 1 .78 1.63l-9.9 10.2a.5.5 0 0 1-.86-.46l1.92-6.02A1 1 0 0 0 11 14z"/>'
    ),
}


# ── Cache ──────────────────────────────────────────────────────
# Building a QIcon involves parsing the SVG + painting; cache the
# result keyed by (name, color, pixel_size).

_ICON_CACHE: dict[tuple[str, str, int], QIcon] = {}


def icon(name: str, color: str | None = None, px: int | None = None) -> QIcon:
    """Get a QIcon for the named SVG, tinted to ``color``, sized ``px``.

    Args:
        name: One of the keys in :data:`_ICONS`.
        color: Hex string (e.g. ``"#cdd6f4"``). Defaults to the
               Catppuccin Mocha ``text`` color.
        px: Pixel size of the rasterized icon (square). Defaults to
            scaled 18px (``s(18)``).

    Returns:
        A ``QIcon`` ready to ``setIcon`` on a QPushButton.
    """
    if name not in _ICONS:
        return QIcon()
    if color is None:
        color = COLORS.get("text", "#cdd6f4")
    if px is None:
        px = s(18)
    cache_key = (name, color, px)
    cached = _ICON_CACHE.get(cache_key)
    if cached is not None:
        return cached
    svg = _ICONS[name].replace("currentColor", color)
    renderer = QSvgRenderer(QByteArray(svg.encode("utf-8")))
    pix = QPixmap(px, px)
    pix.fill(Qt.transparent)
    painter = QPainter(pix)
    painter.setRenderHint(QPainter.Antialiasing, True)
    renderer.render(painter)
    painter.end()
    result = QIcon(pix)
    _ICON_CACHE[cache_key] = result
    return result


def icon_button(text: str, icon_name: str, *,
                object_name: str | None = None,
                tooltip: str | None = None,
                color: str | None = None,
                icon_px: int | None = None,
                parent=None) -> QPushButton:
    """Construct a consistently-sized icon+text QPushButton.

    Sets ``Qt.PointingHandCursor``, an icon size that matches the
    rasterized pixel size, and (optionally) an object-name so
    accent/success/danger button styling kicks in.

    Args:
        text: Visible label.
        icon_name: One of the keys in :data:`_ICONS`.
        object_name: Optional Qt object name for QSS theming
                     (``accentBtn`` / ``successBtn`` / ``dangerBtn``
                     / ``warningBtn`` are the supported variants).
        tooltip: Optional tooltip text.
        color: Override the icon color (defaults to text color, or
               the matching accent color when an objectName is given).
        icon_px: Override the icon pixel size.

    Returns:
        A configured ``QPushButton``.
    """
    btn = QPushButton(text, parent)
    if object_name:
        btn.setObjectName(object_name)
    if tooltip:
        btn.setToolTip(tooltip)
    btn.setCursor(Qt.PointingHandCursor)
    # When an accent variant is requested, tint the icon to match the
    # button's text color so the icon doesn't fight the label.
    if color is None and object_name:
        variant_color = {
            "successBtn": COLORS.get("green"),
            "dangerBtn":  COLORS.get("red"),
            "warningBtn": COLORS.get("yellow"),
            "accentBtn":  COLORS.get("mauve"),
        }.get(object_name)
        if variant_color:
            color = variant_color
    if icon_px is None:
        icon_px = s(18)
    btn.setIcon(icon(icon_name, color=color, px=icon_px))
    btn.setIconSize(QSize(icon_px, icon_px))
    return btn


def set_button_icon(btn: QPushButton, icon_name: str, *,
                    color: str | None = None,
                    icon_px: int | None = None) -> None:
    """Apply an icon to an existing QPushButton (sets size too)."""
    if icon_px is None:
        icon_px = s(18)
    if color is None and btn.objectName():
        variant_color = {
            "successBtn": COLORS.get("green"),
            "dangerBtn":  COLORS.get("red"),
            "warningBtn": COLORS.get("yellow"),
            "accentBtn":  COLORS.get("mauve"),
        }.get(btn.objectName())
        if variant_color:
            color = variant_color
    btn.setIcon(icon(icon_name, color=color, px=icon_px))
    btn.setIconSize(QSize(icon_px, icon_px))
