"""
theme.py — make the standalone window match the app's dark theme properly.

Three things are needed, and missing any one of them produces the "white panels
in places, dark text everywhere" look:

1. **The Fusion style.** ``main.py`` forces it for a reason: the native
   Windows/macOS styles ignore large parts of a stylesheet (combo-box popups
   especially), so QSS silently fails to apply.

2. **A dark QPalette.** ``gui/styles.py``'s ``QWidget`` rule sets a *foreground*
   colour and font but deliberately no background, so any widget the QSS does not
   name explicitly falls back to the platform palette — which is light. The app
   gets away with it because its own widgets are all styled by name; a new window
   built from stock widgets does not.

3. **Supplemental QSS** for the widgets ``build_theme()`` does not cover.
   Verified gaps: ``QPlainTextEdit`` (zero occurrences in the theme — this is the
   console, the firmware pane and the autotune log), plus thin coverage of
   ``QTabWidget``/``QTabBar``, ``QHeaderView`` and table viewports.

Everything here is additive and scoped to this tool; nothing in ``gui/styles.py``
is modified.
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPalette

from gui.styles import COLORS

# Resolve once, with fallbacks, so a future palette edit upstream cannot crash us.
_C = {
    "base": COLORS.get("base", "#1e1e2e"),
    "mantle": COLORS.get("mantle", "#181825"),
    "crust": COLORS.get("crust", "#11111b"),
    "surface0": COLORS.get("surface0", "#313244"),
    "surface1": COLORS.get("surface1", "#45475a"),
    "surface2": COLORS.get("surface2", "#585b70"),
    "overlay0": COLORS.get("overlay0", "#6c7086"),
    "subtext0": COLORS.get("subtext0", "#a6adc8"),
    "text": COLORS.get("text", "#cdd6f4"),
    "green": COLORS.get("green", "#a6e3a1"),
    "red": COLORS.get("red", "#f38ba8"),
    "yellow": COLORS.get("yellow", "#f9e2af"),
    "blue": COLORS.get("blue", "#89b4fa"),
    "mauve": COLORS.get("mauve", "#cba6f7"),
    "peach": COLORS.get("peach", "#fab387"),
}


# ═══════════════════════════════════════════════════════════════════
# Fonts
# ═══════════════════════════════════════════════════════════════════

#: UI font preference, best first. The app's QSS asks for
#: ``-apple-system, "SF Pro Text", "Inter", "Segoe UI", ...`` but Qt's stylesheet
#: engine resolves only the FIRST family in that list, so on Windows it requests
#: "-apple-system", fails, and falls back to whatever Qt picks — which is not
#: necessarily a font with good glyph coverage. Pinning a family that actually
#: exists is what keeps text looking right.
_UI_FONT_PREFS = (
    "Segoe UI Variable Text", "Segoe UI", "Inter", "SF Pro Text",
    "Helvetica Neue", "DejaVu Sans", "Arial",
)

_MONO_FONT_PREFS = (
    "Cascadia Mono", "Consolas", "SF Mono", "DejaVu Sans Mono",
    "Liberation Mono", "Courier New",
)


def _first_available(prefs: tuple[str, ...], fallback: str) -> str:
    try:
        from PySide6.QtGui import QFontDatabase
        families = set(QFontDatabase.families())
    except Exception:
        return fallback
    for name in prefs:
        if name in families:
            return name
    return fallback


def ui_font_family() -> str:
    """A proportional family that exists on this machine."""
    return _first_available(_UI_FONT_PREFS, "sans-serif")


def mono_font_family() -> str:
    """A monospace family that exists on this machine."""
    return _first_available(_MONO_FONT_PREFS, "monospace")


def mono_font(point_size: int | None = None):
    """A ready-to-use monospace QFont for consoles and dumps."""
    from PySide6.QtGui import QFont
    f = QFont(mono_font_family())
    f.setStyleHint(QFont.Monospace)
    if point_size:
        f.setPointSize(point_size)
    return f


def apply_dark_palette(app) -> None:
    """Give every unstyled widget a dark background instead of system white."""
    p = QPalette()

    p.setColor(QPalette.Window, QColor(_C["base"]))
    p.setColor(QPalette.WindowText, QColor(_C["text"]))
    p.setColor(QPalette.Base, QColor(_C["mantle"]))
    p.setColor(QPalette.AlternateBase, QColor(_C["surface0"]))
    p.setColor(QPalette.Text, QColor(_C["text"]))
    p.setColor(QPalette.BrightText, QColor(_C["red"]))
    p.setColor(QPalette.PlaceholderText, QColor(_C["overlay0"]))

    p.setColor(QPalette.Button, QColor(_C["surface0"]))
    p.setColor(QPalette.ButtonText, QColor(_C["text"]))

    p.setColor(QPalette.ToolTipBase, QColor(_C["surface0"]))
    p.setColor(QPalette.ToolTipText, QColor(_C["text"]))

    p.setColor(QPalette.Highlight, QColor(_C["mauve"]))
    p.setColor(QPalette.HighlightedText, QColor(_C["crust"]))

    p.setColor(QPalette.Link, QColor(_C["blue"]))
    p.setColor(QPalette.LinkVisited, QColor(_C["mauve"]))

    for group in (QPalette.Disabled,):
        p.setColor(group, QPalette.WindowText, QColor(_C["overlay0"]))
        p.setColor(group, QPalette.Text, QColor(_C["overlay0"]))
        p.setColor(group, QPalette.ButtonText, QColor(_C["overlay0"]))
        p.setColor(group, QPalette.Base, QColor(_C["mantle"]))
        p.setColor(group, QPalette.Button, QColor(_C["mantle"]))

    app.setPalette(p)


def supplemental_qss(k: float = 1.0) -> str:
    """
    QSS for widgets the app theme does not cover. ``k`` is the DPI scale.
    """
    def px(v: float) -> int:
        return max(1, int(round(v * k)))

    c = _C
    fam = ui_font_family()
    return f"""
/* ── font: override the app QSS's family list with ONE family that
      actually exists here. Qt resolves only the first family in a QSS
      list, and the app's list leads with "-apple-system". ─────────── */
QWidget {{
    font-family: "{fam}";
}}

/* ── the tool's own window chrome ───────────────────────────────── */
QMainWindow, QDialog {{
    background-color: {c['base']};
}}

/* ── text panes: NOT covered by build_theme at all ──────────────── */
QPlainTextEdit, QTextEdit {{
    background-color: {c['crust']};
    color: {c['text']};
    border: {px(1)}px solid {c['surface1']};
    border-radius: {px(4)}px;
    selection-background-color: {c['mauve']};
    selection-color: {c['crust']};
    padding: {px(4)}px;
}}

/* ── tabs ───────────────────────────────────────────────────────── */
QTabWidget::pane {{
    background-color: {c['base']};
    border: {px(1)}px solid {c['surface1']};
    border-radius: {px(4)}px;
    top: -{px(1)}px;
}}
QTabBar {{
    background: transparent;
    qproperty-drawBase: 0;
}}
QTabBar::tab {{
    background-color: {c['mantle']};
    color: {c['subtext0']};
    border: {px(1)}px solid {c['surface0']};
    border-bottom: none;
    border-top-left-radius: {px(4)}px;
    border-top-right-radius: {px(4)}px;
    padding: {px(6)}px {px(14)}px;
    margin-right: {px(2)}px;
}}
QTabBar::tab:selected {{
    background-color: {c['surface0']};
    color: {c['text']};
    border-color: {c['surface1']};
    border-bottom: {px(2)}px solid {c['mauve']};
}}
QTabBar::tab:hover:!selected {{
    background-color: {c['surface0']};
    color: {c['text']};
}}

/* ── tables ─────────────────────────────────────────────────────── */
QTableWidget, QTableView {{
    background-color: {c['crust']};
    alternate-background-color: {c['mantle']};
    color: {c['text']};
    gridline-color: {c['surface0']};
    border: {px(1)}px solid {c['surface1']};
    border-radius: {px(4)}px;
    selection-background-color: {c['surface2']};
    selection-color: {c['text']};
}}
QTableWidget::item, QTableView::item {{
    padding: {px(4)}px {px(6)}px;
}}
QHeaderView {{
    background-color: {c['mantle']};
}}
QHeaderView::section {{
    background-color: {c['surface0']};
    color: {c['subtext0']};
    border: none;
    border-right: {px(1)}px solid {c['mantle']};
    border-bottom: {px(1)}px solid {c['surface1']};
    padding: {px(5)}px {px(7)}px;
    font-weight: 600;
}}
QTableCornerButton::section {{
    background-color: {c['surface0']};
    border: none;
}}

/* ── group boxes: keep a visible card edge + readable title ─────── */
QGroupBox {{
    background-color: {c['mantle']};
    border: {px(1)}px solid {c['surface1']};
    border-radius: {px(6)}px;
    margin-top: {px(12)}px;
    padding-top: {px(10)}px;
    font-weight: 600;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: {px(10)}px;
    padding: 0 {px(6)}px;
    color: {c['mauve']};
}}

/* ── splitters ──────────────────────────────────────────────────── */
QSplitter::handle {{
    background-color: {c['surface0']};
}}
QSplitter::handle:horizontal {{ width: {px(4)}px; }}
QSplitter::handle:vertical   {{ height: {px(4)}px; }}
QSplitter::handle:hover {{ background-color: {c['mauve']}; }}

/* ── scrollbars ─────────────────────────────────────────────────── */
QScrollBar:vertical {{
    background: {c['mantle']}; width: {px(11)}px; margin: 0; border: none;
}}
QScrollBar::handle:vertical {{
    background: {c['surface2']}; min-height: {px(24)}px;
    border-radius: {px(5)}px; margin: {px(2)}px;
}}
QScrollBar::handle:vertical:hover {{ background: {c['overlay0']}; }}
QScrollBar:horizontal {{
    background: {c['mantle']}; height: {px(11)}px; margin: 0; border: none;
}}
QScrollBar::handle:horizontal {{
    background: {c['surface2']}; min-width: {px(24)}px;
    border-radius: {px(5)}px; margin: {px(2)}px;
}}
QScrollBar::handle:horizontal:hover {{ background: {c['overlay0']}; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; width: 0; border: none; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}

/* ── duty / progress bars ───────────────────────────────────────── */
QProgressBar {{
    background-color: {c['crust']};
    border: {px(1)}px solid {c['surface1']};
    border-radius: {px(4)}px;
    text-align: center;
    color: {c['text']};
    min-height: {px(16)}px;
}}
QProgressBar::chunk {{
    background-color: {c['blue']};
    border-radius: {px(3)}px;
}}

/* ── inputs: make the editable/disabled distinction obvious ────── */
QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox {{
    background-color: {c['surface0']};
    color: {c['text']};
    border: {px(1)}px solid {c['surface1']};
    border-radius: {px(4)}px;
    padding: {px(4)}px {px(6)}px;
    min-height: {px(20)}px;
}}
QComboBox:focus, QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {{
    border-color: {c['mauve']};
}}
QComboBox:disabled, QLineEdit:disabled, QSpinBox:disabled,
QDoubleSpinBox:disabled {{
    background-color: {c['mantle']};
    color: {c['overlay0']};
}}
QComboBox QAbstractItemView {{
    background-color: {c['surface0']};
    color: {c['text']};
    border: {px(1)}px solid {c['surface1']};
    selection-background-color: {c['mauve']};
    selection-color: {c['crust']};
    outline: none;
}}
QComboBox::drop-down {{ border: none; width: {px(18)}px; }}

/* ── buttons ────────────────────────────────────────────────────── */
QPushButton {{
    background-color: {c['surface0']};
    color: {c['text']};
    border: {px(1)}px solid {c['surface1']};
    border-radius: {px(4)}px;
    padding: {px(6)}px {px(12)}px;
    min-height: {px(20)}px;
}}
QPushButton:hover  {{ background-color: {c['surface1']}; }}
QPushButton:pressed{{ background-color: {c['surface2']}; }}
QPushButton:checked{{
    background-color: {c['mauve']}; color: {c['crust']}; font-weight: 600;
}}
QPushButton:disabled {{
    background-color: {c['mantle']};
    color: {c['overlay0']};
    border-color: {c['surface0']};
}}

/* Semantic buttons, set via setObjectName(). */
QPushButton#dangerButton {{
    background-color: {c['red']}; color: {c['crust']};
    border-color: {c['red']}; font-weight: 700;
}}
QPushButton#dangerButton:hover  {{ background-color: #ff9fb5; }}
QPushButton#primaryButton {{
    background-color: {c['blue']}; color: {c['crust']};
    border-color: {c['blue']}; font-weight: 600;
}}
QPushButton#primaryButton:hover {{ background-color: #a5c8ff; }}
QPushButton#warnButton {{
    background-color: {c['peach']}; color: {c['crust']};
    border-color: {c['peach']}; font-weight: 600;
}}

/* ── checkboxes ─────────────────────────────────────────────────── */
QCheckBox {{ color: {c['text']}; spacing: {px(6)}px; }}
QCheckBox::indicator {{
    width: {px(14)}px; height: {px(14)}px;
    border: {px(1)}px solid {c['surface2']};
    border-radius: {px(3)}px;
    background-color: {c['crust']};
}}
QCheckBox::indicator:checked {{
    background-color: {c['mauve']}; border-color: {c['mauve']};
}}
QCheckBox:disabled {{ color: {c['overlay0']}; }}

/* ── message boxes ──────────────────────────────────────────────── */
QMessageBox {{ background-color: {c['base']}; }}
QMessageBox QLabel {{ color: {c['text']}; }}
"""


def install_theme(app) -> float:
    """
    Apply style + palette + stylesheet in the correct order. Returns the DPI
    scale factor, which callers may want for their own sizing.

    Must be called AFTER the QApplication exists, because
    ``gui.scaling.scale_factor()`` memoises on its first call and would otherwise
    lock the UI scale to 1.0.
    """
    from PySide6.QtWidgets import QStyleFactory
    from gui.scaling import scale_factor
    from gui.styles import build_theme

    from PySide6.QtGui import QFont

    fusion = QStyleFactory.create("Fusion")
    if fusion is not None:
        app.setStyle(fusion)

    apply_dark_palette(app)

    # Set a concrete application font as well as the QSS rule, so widgets that
    # build their own QFont (the big temperature readout, the console) inherit a
    # family that exists rather than an unresolvable one.
    k = scale_factor()
    base = QFont(ui_font_family())
    base.setPointSizeF(max(8.0, 10.0 * k))
    app.setFont(base)

    app.setStyleSheet(build_theme(k) + supplemental_qss(k))
    return k
