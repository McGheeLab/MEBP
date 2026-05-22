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
# v7.4.2 polish: flat filled SVGs (Phosphor Fill-inspired) in place
# of the earlier stroke icons. All shapes use ``currentColor`` for
# the fill so a single template can be tinted to any color at
# render-time. Default tint is white for a chunky stylized look
# against the dark theme.
# Source: hand-drawn shapes (MIT-style permissive); shapes informed
# by Phosphor Icons and Solar Bold (each available under MIT).

_SVG_HEADER = (
    '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 256 256" '
    'fill="currentColor">'
)


def _icon(svg_body: str) -> str:
    """Wrap a body fragment in the standard SVG header/footer."""
    return f"{_SVG_HEADER}{svg_body}</svg>"


# All paths are normalised to a 256×256 viewBox so visual weight stays
# consistent. Inspired by Phosphor "Fill" — chunky filled silhouettes
# with rounded corners.
_ICONS: dict[str, str] = {
    # ── plug (power / connect) ────────────────────────────────
    "plug": _icon(
        '<path d="M200,96H184V72a8,8,0,0,0-16,0V96H88V72a8,8,0,0,0-16,0V96H56a16,16,0,0,0-16,16v40a72,72,0,0,0,64,71.56V224a8,8,0,0,0,16,0V223.56A72,72,0,0,0,216,152V112A16,16,0,0,0,200,96Z"/>'
    ),
    # ── x (close / disconnect) ────────────────────────────────
    "x": _icon(
        '<path d="M205.66,194.34a8,8,0,0,1-11.32,11.32L128,139.31,61.66,205.66a8,8,0,0,1-11.32-11.32L116.69,128,50.34,61.66A8,8,0,0,1,61.66,50.34L128,116.69l66.34-66.35a8,8,0,0,1,11.32,11.32L139.31,128Z"/>'
    ),
    "x-circle": _icon(
        '<path d="M128,24A104,104,0,1,0,232,128,104.13,104.13,0,0,0,128,24Zm37.66,130.34a8,8,0,0,1-11.32,11.32L128,139.31l-26.34,26.35a8,8,0,0,1-11.32-11.32L116.69,128,90.34,101.66a8,8,0,0,1,11.32-11.32L128,116.69l26.34-26.35a8,8,0,0,1,11.32,11.32L139.31,128Z"/>'
    ),
    # ── save (floppy) ─────────────────────────────────────────
    "save": _icon(
        '<path d="M219.31,76.69,179.31,36.69A16,16,0,0,0,168,32H48A16,16,0,0,0,32,48V208a16,16,0,0,0,16,16H208a16,16,0,0,0,16-16V88A16,16,0,0,0,219.31,76.69ZM72,48h48V72a8,8,0,0,0,16,0V48h16a8.07,8.07,0,0,1,5.66,2.34L160,55.31V80H72ZM168,208H88V160h80Zm40,0H184V160a16,16,0,0,0-16-16H88a16,16,0,0,0-16,16v48H48V48H56V88a8,8,0,0,0,8,8H168a8,8,0,0,0,8-8V60.69l32,32Z"/>'
    ),
    # ── folder (load / import) ────────────────────────────────
    "folder": _icon(
        '<path d="M216,72H130.67L102.94,51.2a16.13,16.13,0,0,0-9.6-3.2H40A16,16,0,0,0,24,64V200.62A15.4,15.4,0,0,0,39.38,216H216.89A15.13,15.13,0,0,0,232,200.89V88A16,16,0,0,0,216,72Z"/>'
    ),
    "folder-open": _icon(
        '<path d="M245,110.64A16,16,0,0,0,232,104H216V88a16,16,0,0,0-16-16H130.67L102.94,51.2a16.13,16.13,0,0,0-9.6-3.2H40A16,16,0,0,0,24,64V208h0a8,8,0,0,0,8,8H211.1a8,8,0,0,0,7.59-5.47l28.49-85.47A16,16,0,0,0,245,110.64ZM93.34,64,116,81l13.33,10H40V64ZM216,200H43.1L62.43,142.65A8,8,0,0,1,70,137.14H224.83a8,8,0,0,1,7.59,10.53Z"/>'
    ),
    # ── refresh ───────────────────────────────────────────────
    "refresh": _icon(
        '<path d="M197.67,186.37a8,8,0,0,1,0,11.29C196.58,198.73,170.82,224,128,224,77.34,224,38.81,184.05,32.5,144.92L23.5,154a8,8,0,1,1-11.31-11.31l24-23.86h0a8,8,0,0,1,11.32,0l24,23.86A8,8,0,1,1,60.18,154l-12-12C53.83,177,87,208,128,208c36.6,0,58.78-21.25,59-21.46A8,8,0,0,1,197.67,186.37Z M243.81,113.31l-24-23.86h0a8,8,0,0,0-11.32,0l-24,23.86a8,8,0,1,0,11.31,11.32l12-12C202.17,79,169,48,128,48c-36.6,0-58.78,21.25-59,21.46a8,8,0,1,1-11.27-11.36C58.81,57.05,84.57,32,128,32c50.66,0,89.19,39.95,95.5,79.08L232.5,102a8,8,0,1,1,11.31,11.31Z"/>'
    ),
    # ── plus / minus ──────────────────────────────────────────
    "plus": _icon(
        '<path d="M224,128a8,8,0,0,1-8,8H136v80a8,8,0,0,1-16,0V136H40a8,8,0,0,1,0-16h80V40a8,8,0,0,1,16,0v80h80A8,8,0,0,1,224,128Z"/>'
    ),
    "minus": _icon(
        '<path d="M224,128a8,8,0,0,1-8,8H40a8,8,0,0,1,0-16H216A8,8,0,0,1,224,128Z"/>'
    ),
    # ── search / detect (magnifier) ───────────────────────────
    "search": _icon(
        '<path d="M232.49,215.51,185,168a92.12,92.12,0,1,0-17,17l47.53,47.54a12,12,0,0,0,17-17ZM44,112a68,68,0,1,1,68,68A68.07,68.07,0,0,1,44,112Z"/>'
    ),
    # ── ruler (calibrate) ─────────────────────────────────────
    "ruler": _icon(
        '<path d="M235.32,73.37,182.63,20.69a16,16,0,0,0-22.63,0L20.68,160a16,16,0,0,0,0,22.63l52.69,52.68a16,16,0,0,0,22.63,0L235.32,96A16,16,0,0,0,235.32,73.37ZM84.69,224,32,171.31l32-32,26.34,26.35a8,8,0,0,0,11.32-11.32L75.32,128,96,107.31l26.34,26.35a8,8,0,0,0,11.32-11.32L107.32,96,128,75.31l26.34,26.35a8,8,0,0,0,11.32-11.32L139.32,64l32-32L224,84.69Z"/>'
    ),
    # ── gamepad (controller) ──────────────────────────────────
    "gamepad": _icon(
        '<path d="M229.05,71.05a48.07,48.07,0,0,0-39.55-22.79L153.16,48H102.84L66.51,48.25A48.07,48.07,0,0,0,26.95,71.05L1.79,113.25A28,28,0,0,0,53.69,134l8.07-21.74A8,8,0,0,1,69.16,107H88V96a8,8,0,0,1,16,0v11h48V96a8,8,0,0,1,16,0v11h18.85a8,8,0,0,1,7.4,5.26L202.31,134A28,28,0,0,0,254.21,113.25ZM102,144a16,16,0,1,1,16-16A16,16,0,0,1,102,144Zm68,16a16,16,0,1,1,16-16A16,16,0,0,1,170,160Z"/>'
    ),
    # ── alert (warning triangle) ──────────────────────────────
    "alert": _icon(
        '<path d="M236.8,188.09,149.35,36.22a24.76,24.76,0,0,0-42.7,0L19.2,188.09a23.51,23.51,0,0,0,0,23.72,24.35,24.35,0,0,0,21.35,12.19h174.9a24.35,24.35,0,0,0,21.35-12.19A23.51,23.51,0,0,0,236.8,188.09ZM120,104a8,8,0,0,1,16,0v40a8,8,0,0,1-16,0Zm8,88a12,12,0,1,1,12-12A12,12,0,0,1,128,192Z"/>'
    ),
    # ── check ─────────────────────────────────────────────────
    "check": _icon(
        '<path d="M229.66,77.66l-128,128a8,8,0,0,1-11.32,0l-56-56a8,8,0,0,1,11.32-11.32L96,188.69,218.34,66.34a8,8,0,0,1,11.32,11.32Z"/>'
    ),
    "check-circle": _icon(
        '<path d="M128,24A104,104,0,1,0,232,128,104.11,104.11,0,0,0,128,24Zm49.53,85.41-58.66,56a8,8,0,0,1-11,0L82.13,142.92a8,8,0,0,1,11.05-11.58l24.06,23,53.16-50.74a8,8,0,0,1,11.05,11.57Z"/>'
    ),
    # ── stop ──────────────────────────────────────────────────
    "stop": _icon(
        '<path d="M128,24A104,104,0,1,0,232,128,104.11,104.11,0,0,0,128,24Zm32,128a8,8,0,0,1-8,8H104a8,8,0,0,1-8-8V104a8,8,0,0,1,8-8h48a8,8,0,0,1,8,8Z"/>'
    ),
    # ── arrow-up / arrow-down ─────────────────────────────────
    "arrow-up": _icon(
        '<path d="M205.66,117.66a8,8,0,0,1-11.32,0L136,59.31V216a8,8,0,0,1-16,0V59.31L61.66,117.66a8,8,0,0,1-11.32-11.32l72-72a8,8,0,0,1,11.32,0l72,72A8,8,0,0,1,205.66,117.66Z"/>'
    ),
    "arrow-down": _icon(
        '<path d="M205.66,149.66l-72,72a8,8,0,0,1-11.32,0l-72-72a8,8,0,0,1,11.32-11.32L120,196.69V40a8,8,0,0,1,16,0V196.69l58.34-58.35a8,8,0,0,1,11.32,11.32Z"/>'
    ),
    # ── settings (gear) ───────────────────────────────────────
    "settings": _icon(
        '<path d="M128,80a48,48,0,1,0,48,48A48.05,48.05,0,0,0,128,80Zm0,80a32,32,0,1,1,32-32A32,32,0,0,1,128,160Zm88-29.84q.06-2.16,0-4.32l14.92-18.64a8,8,0,0,0,1.48-7.06,107.21,107.21,0,0,0-10.88-26.25,8,8,0,0,0-6-3.93l-23.72-2.64q-1.49-1.56-3-3L186,40.54a8,8,0,0,0-3.94-6,107.71,107.71,0,0,0-26.25-10.87,8,8,0,0,0-7.06,1.49L130.16,40Q128,40,125.84,40L107.2,25.11a8,8,0,0,0-7.06-1.48A107.6,107.6,0,0,0,73.89,34.51a8,8,0,0,0-3.93,6L67.32,64.27q-1.56,1.49-3,3L40.54,70a8,8,0,0,0-6,3.94,107.71,107.71,0,0,0-10.87,26.25,8,8,0,0,0,1.49,7.06L40,125.84Q40,128,40,130.16L25.11,148.8a8,8,0,0,0-1.48,7.06,107.21,107.21,0,0,0,10.88,26.25,8,8,0,0,0,6,3.93l23.72,2.64q1.49,1.56,3,3L70,215.46a8,8,0,0,0,3.94,6,107.71,107.71,0,0,0,26.25,10.87,8,8,0,0,0,7.06-1.49L125.84,216q2.16.06,4.32,0l18.64,14.92a8,8,0,0,0,7.06,1.48,107.21,107.21,0,0,0,26.25-10.88,8,8,0,0,0,3.93-6l2.64-23.72q1.56-1.49,3-3L215.46,186a8,8,0,0,0,6-3.94,107.71,107.71,0,0,0,10.87-26.25,8,8,0,0,0-1.49-7.06Z"/>'
    ),
    # ── info ──────────────────────────────────────────────────
    "info": _icon(
        '<path d="M128,24A104,104,0,1,0,232,128,104.11,104.11,0,0,0,128,24Zm-4,48a12,12,0,1,1-12,12A12,12,0,0,1,124,72Zm12,112a16,16,0,0,1-16-16V128a8,8,0,0,1,0-16,16,16,0,0,1,16,16v40a8,8,0,0,1,0,16Z"/>'
    ),
    # ── trash ─────────────────────────────────────────────────
    "trash": _icon(
        '<path d="M216,48H176V40a24,24,0,0,0-24-24H104A24,24,0,0,0,80,40v8H40a8,8,0,0,0,0,16h8V208a16,16,0,0,0,16,16H192a16,16,0,0,0,16-16V64h8a8,8,0,0,0,0-16ZM112,168a8,8,0,0,1-16,0V104a8,8,0,0,1,16,0Zm48,0a8,8,0,0,1-16,0V104a8,8,0,0,1,16,0ZM160,48H96V40a8,8,0,0,1,8-8h48a8,8,0,0,1,8,8Z"/>'
    ),
    # ── pencil (edit) ─────────────────────────────────────────
    "pencil": _icon(
        '<path d="M227.31,73.37,182.63,28.68a16,16,0,0,0-22.62,0L36.69,152A15.86,15.86,0,0,0,32,163.31V208a16,16,0,0,0,16,16H92.69A15.86,15.86,0,0,0,104,219.31L227.31,96a16,16,0,0,0,0-22.63ZM92.69,208H48V163.31l88-88L180.69,120ZM192,108.68,147.31,64l24-24L216,84.68Z"/>'
    ),
    # ── zap (lightning) ───────────────────────────────────────
    "zap": _icon(
        '<path d="M215.79,118.17a8,8,0,0,0-5-5.66L153.18,90.9l14.66-73.33a8,8,0,0,0-13.69-7l-112,120a8,8,0,0,0,3,13l57.63,21.61L88.16,238.43a8,8,0,0,0,13.69,7l112-120A8,8,0,0,0,215.79,118.17Z"/>'
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
        color: Hex string (e.g. ``"#ffffff"``). Defaults to white —
               flat-white-icon style; variants (success / danger /
               accent) tint to their accent color via ``icon_button``.
        px: Pixel size of the rasterized icon (square). Defaults to
            scaled 18px (``s(18)``).

    Returns:
        A ``QIcon`` ready to ``setIcon`` on a QPushButton.
    """
    if name not in _ICONS:
        return QIcon()
    if color is None:
        color = "#ffffff"
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
