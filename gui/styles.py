"""
Dark theme stylesheet — PyDracula-inspired, Catppuccin Mocha palette.

v7.3.6: All pixel and font-size dimensions are now DPI-aware via
build_theme(scale).  The gui/scaling.py module provides the scale
factor; app.py calls build_theme() at startup with the actual value.

Layout structure:
    ┌──────────────────────────────────────────────────────────────┐
    │  bgApp                                                       │
    │ ┌────┬──────────┬──────────────────────────────────────────┐ │
    │ │left│ extraLeft│  contentBox                               │ │
    │ │Menu│ Box      │ ┌──────────────────────────────────────┐ │ │
    │ │ Bg │ (context │ │ contentTopBg  (title + conn status)  │ │ │
    │ │    │  panel)  │ ├──────────────────────────────────────┤ │ │
    │ │icon│          │ │                                      │ │ │
    │ │icon│          │ │  contentBottom (stacked pages)       │ │ │
    │ │icon│          │ │                                      │ │ │
    │ │icon│          │ │                                      │ │ │
    │ │icon│          │ │                                      │ │ │
    │ │    │          │ │                                      │ │ │
    │ │ ⚙ │          │ ├──────────────────────────────────────┤ │ │
    │ │    │          │ │ console log (collapsible)            │ │ │
    │ └────┴──────────┴──────────────────────────────────────────┘ │
    │ ┌──────────────────────────────────────────────────────────┐ │
    │ │ bottomBar (status readouts)                              │ │
    │ └──────────────────────────────────────────────────────────┘ │
    └──────────────────────────────────────────────────────────────┘

Colour palette (Catppuccin Mocha):
    Base       #1e1e2e     Main background
    Mantle     #181825     Darker panels / left menu
    Crust      #11111b     Deepest background
    Surface0   #313244     Card / panel backgrounds
    Surface1   #45475a     Hover, borders
    Surface2   #585b70     Active borders
    Overlay0   #6c7086     Disabled / dim text
    Subtext0   #a6adc8     Secondary text
    Text       #cdd6f4     Primary text
    Green      #a6e3a1     Success / connected
    Red        #f38ba8     Error / danger / disconnect
    Yellow     #f9e2af     Warning / paused
    Blue       #89b4fa     Accent / links
    Mauve      #cba6f7     Selected accent (Dracula purple)
    Peach      #fab387     Highlights
    Pink       #f5c2e7     Secondary accent
"""

from __future__ import annotations

# ── Color Constants ──────────────────────────────────────────────
# Usable in Python code for dynamic styling
COLORS = {
    "base": "#1e1e2e",
    "mantle": "#181825",
    "crust": "#11111b",
    "surface0": "#313244",
    "surface1": "#45475a",
    "surface2": "#585b70",
    "overlay0": "#6c7086",
    "subtext0": "#a6adc8",
    "text": "#cdd6f4",
    "green": "#a6e3a1",
    "red": "#f38ba8",
    "yellow": "#f9e2af",
    "blue": "#89b4fa",
    "mauve": "#cba6f7",
    "peach": "#fab387",
    "pink": "#f5c2e7",
    # Derived
    "menu_bg": "#181825",
    "menu_hover": "#252536",
    "menu_active": "#313244",
    "panel_bg": "#232334",
    "content_bg": "#1e1e2e",
    "top_bar_bg": "#181825",
    "bottom_bar_bg": "#181825",
}


# ── Scaling helpers (local) ──────────────────────────────────────

def _p(val: int | float, k: float) -> int:
    """Scale a pixel value."""
    return round(val * k)


def _f(val: float, k: float) -> float:
    """Scale a font-size value (pt)."""
    return round(val * k, 1)


# ── Centralized Section Styles (v7.2.4) ─────────────────────────
# Use these instead of per-page _group_style() methods for consistency.

def build_section_title_style(k: float = 1.0) -> str:
    # v7.4.2 polish: roomier padding, softer borders, sub-pixel-clean title
    # baseline. Title sits on the top-left edge as a "pill" cut into the
    # frame, with a subtle accent dot to anchor the eye.
    return f"""
    QGroupBox {{
        font-size: {_f(10.5, k)}pt;
        font-weight: 500;
        color: {COLORS['text']};
        background-color: {COLORS['surface0']};
        border: 1px solid {COLORS['surface1']};
        border-radius: {_p(10, k)}px;
        margin-top: {_p(16, k)}px;
        padding: {_p(22, k)}px {_p(18, k)}px {_p(16, k)}px {_p(18, k)}px;
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        subcontrol-position: top left;
        left: {_p(14, k)}px;
        top: {_p(2, k)}px;
        padding: {_p(3, k)}px {_p(10, k)}px;
        background-color: {COLORS['base']};
        border: 1px solid {COLORS['surface1']};
        border-radius: {_p(6, k)}px;
        color: {COLORS['blue']};
        font-size: {_f(9.5, k)}pt;
        font-weight: 600;
        letter-spacing: 0.3px;
    }}
"""


def build_context_section_label_style(k: float = 1.0) -> str:
    return f"""
    font-size: {_f(10, k)}pt;
    font-weight: 600;
    color: {COLORS['blue']};
    padding: {_p(6, k)}px 0px {_p(2, k)}px 0px;
    border-bottom: 1px solid {COLORS['surface1']};
    margin-bottom: {_p(4, k)}px;
"""


def build_page_header_style(k: float = 1.0) -> str:
    return f"""
    font-size: {_f(14, k)}pt;
    font-weight: 700;
    color: {COLORS['text']};
    padding: {_p(4, k)}px 0px;
"""


def build_card_frame_style(k: float = 1.0) -> str:
    return f"""
    QFrame#cardFrame {{
        background-color: {COLORS['surface0']};
        border: 1px solid {COLORS['surface1']};
        border-radius: {_p(8, k)}px;
        padding: {_p(12, k)}px;
    }}
"""


# Module-level defaults (scale=1.0 for backward compat)
SECTION_TITLE_STYLE = build_section_title_style(1.0)
CONTEXT_SECTION_LABEL_STYLE = build_context_section_label_style(1.0)
PAGE_HEADER_STYLE = build_page_header_style(1.0)
CARD_FRAME_STYLE = build_card_frame_style(1.0)


# ── Menu Selection Stylesheet (appended dynamically) ─────────────
MENU_SELECTED_STYLESHEET = (
    "border-left: 3px solid #cba6f7;"
    "color: #cdd6f4;"
    "background-color: rgba(203, 166, 247, 0.08);"
)

# ── Menu Button State Styles (collapsed vs expanded) ─────────────
# Applied programmatically when menu toggles.
MENU_BTN_COLLAPSED_STYLE = (
    "text-align: center;"
    "padding-left: 0px;"
)

MENU_BTN_EXPANDED_STYLE = (
    "text-align: left;"
    "padding-left: 44px;"
)


# ── Main QSS Theme ───────────────────────────────────────────────

def build_theme(k: float = 1.0) -> str:
    """Generate the full QSS theme with all dimensions scaled by factor `k`.

    k=1.0 produces the original 96-DPI stylesheet.
    k=1.5 scales for ~144 DPI (e.g. 4K at 150% or Retina).
    """
    return f"""

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   GLOBAL
///////////////////////////////////////////////////////////////////////////////////////////////// */

QWidget {{
    color: #cdd6f4;
    font: {_f(10, k)}pt "Segoe UI", "Ubuntu", sans-serif;
}}

QToolTip {{
    color: #cdd6f4;
    background-color: rgba(30, 30, 46, 230);
    border: 1px solid #45475a;
    border-left: {_p(2, k)}px solid #cba6f7;
    padding: {_p(4, k)}px {_p(8, k)}px;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   APP BACKGROUND
///////////////////////////////////////////////////////////////////////////////////////////////// */

#bgApp {{
    background-color: #1e1e2e;
    border: 1px solid #313244;
    border-radius: {_p(8, k)}px;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   LEFT MENU
///////////////////////////////////////////////////////////////////////////////////////////////// */

#leftMenuBg {{
    background-color: #181825;
}}

#topLogo {{
    background-color: #181825;
    padding: {_p(8, k)}px;
}}

#titleLeftApp {{
    font: 63 {_f(12, k)}pt "Segoe UI Semibold";
    color: #cdd6f4;
}}

#titleLeftDescription {{
    font: {_f(8, k)}pt "Segoe UI";
    color: #cba6f7;
}}

/* Main menu buttons — default state is COLLAPSED (centered icons) */
#topMenu QPushButton {{
    background-position: center;
    background-repeat: no-repeat;
    border: none;
    border-left: {_p(3, k)}px solid transparent;
    background-color: transparent;
    text-align: center;
    padding-left: 0px;
    padding-top: {_p(5, k)}px;
    padding-bottom: {_p(5, k)}px;
    color: #a6adc8;
    margin: {_p(1, k)}px {_p(4, k)}px;
    border-radius: 0px {_p(4, k)}px {_p(4, k)}px 0px;
    font: {_f(14, k)}pt "Segoe UI Emoji", "Apple Color Emoji", "Noto Color Emoji", sans-serif;
}}

#topMenu QPushButton:hover {{
    background-color: #252536;
    border-left: {_p(3, k)}px solid #585b70;
    color: #cdd6f4;
}}

#topMenu QPushButton:pressed {{
    background-color: rgba(203, 166, 247, 0.15);
    border-left: {_p(3, k)}px solid #cba6f7;
    color: #cdd6f4;
}}

/* Bottom menu buttons — same collapsed-by-default pattern */
#bottomMenu QPushButton {{
    background-position: center;
    background-repeat: no-repeat;
    border: none;
    border-left: {_p(3, k)}px solid transparent;
    background-color: transparent;
    text-align: center;
    padding-left: 0px;
    padding-top: {_p(5, k)}px;
    padding-bottom: {_p(5, k)}px;
    color: #a6adc8;
    margin: {_p(1, k)}px {_p(4, k)}px;
    border-radius: 0px {_p(4, k)}px {_p(4, k)}px 0px;
    font: {_f(14, k)}pt "Segoe UI Emoji", "Apple Color Emoji", "Noto Color Emoji", sans-serif;
}}

#bottomMenu QPushButton:hover {{
    background-color: #252536;
    border-left: {_p(3, k)}px solid #585b70;
    color: #cdd6f4;
}}

#bottomMenu QPushButton:pressed {{
    background-color: rgba(203, 166, 247, 0.15);
    border-left: {_p(3, k)}px solid #cba6f7;
    color: #cdd6f4;
}}

#leftMenuFrame {{
    border-top: {_p(2, k)}px solid #313244;
}}

/* Toggle button */
#toggleButton {{
    background-position: left center;
    background-repeat: no-repeat;
    border: none;
    border-left: {_p(3, k)}px solid transparent;
    background-color: #181825;
    text-align: center;
    padding-left: 0px;
    color: #6c7086;
    margin: {_p(1, k)}px {_p(4, k)}px;
    border-radius: 0px {_p(4, k)}px {_p(4, k)}px 0px;
    font: {_f(12, k)}pt "Segoe UI";
}}

#toggleButton:hover {{
    background-color: #252536;
    color: #a6adc8;
}}

#toggleButton:pressed {{
    background-color: rgba(203, 166, 247, 0.15);
    color: #cba6f7;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   EXTRA LEFT BOX (context settings panel)
///////////////////////////////////////////////////////////////////////////////////////////////// */

#extraLeftBox {{
    background-color: #232334;
}}

#extraTopBg {{
    background-color: #cba6f7;
}}

#extraLabel {{
    color: #1e1e2e;
    font: bold {_f(10, k)}pt "Segoe UI";
}}

#extraCloseColumnBtn {{
    background-color: transparent;
    border: none;
    border-radius: {_p(4, k)}px;
}}

#extraCloseColumnBtn:hover {{
    background-color: rgba(30, 30, 46, 80);
}}

#extraContent {{
    border-top: {_p(2, k)}px solid #313244;
}}

#extraTopMenu QPushButton {{
    background-position: left center;
    background-repeat: no-repeat;
    border: none;
    border-left: {_p(22, k)}px solid transparent;
    background-color: transparent;
    text-align: left;
    padding-left: {_p(44, k)}px;
    color: #cdd6f4;
}}

#extraTopMenu QPushButton:hover {{
    background-color: #313244;
}}

/* Context panel scroll area */
#contextScrollArea {{
    background-color: transparent;
    border: none;
}}

#contextScrollArea QWidget {{
    background-color: transparent;
}}

/* Context panel section headers */
#contextSectionLabel {{
    color: #cba6f7;
    font: bold {_f(9, k)}pt "Segoe UI";
    padding: {_p(8, k)}px {_p(4, k)}px {_p(4, k)}px {_p(4, k)}px;
    border-bottom: 1px solid #313244;
}}

/* Context panel form labels */
#contextLabel {{
    color: #a6adc8;
    font: {_f(9, k)}pt "Segoe UI";
    padding: {_p(2, k)}px;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   CONTENT AREA
///////////////////////////////////////////////////////////////////////////////////////////////// */

#contentBox {{
    background-color: #1e1e2e;
}}

#contentTopBg {{
    background-color: #181825;
}}

#titleRightInfo {{
    padding-left: {_p(10, k)}px;
    color: #a6adc8;
    font: {_f(9, k)}pt "Segoe UI";
}}

#contentBottom {{
    border-top: {_p(2, k)}px solid #313244;
}}

/* Top right buttons */
#rightButtons QPushButton {{
    background-color: transparent;
    border: none;
    border-radius: {_p(4, k)}px;
    padding: {_p(4, k)}px;
}}

#rightButtons QPushButton:hover {{
    background-color: #313244;
}}

#rightButtons QPushButton:pressed {{
    background-color: #45475a;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   BOTTOM BAR
///////////////////////////////////////////////////////////////////////////////////////////////// */

#bottomBar {{
    background-color: #181825;
    border-top: 1px solid #313244;
}}

#bottomBar QLabel {{
    font: {_f(9, k)}pt "Consolas", "Ubuntu Mono", monospace;
    color: #6c7086;
    padding-left: {_p(8, k)}px;
    padding-right: {_p(8, k)}px;
    padding-bottom: {_p(2, k)}px;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   CONNECTION STATUS INDICATORS
///////////////////////////////////////////////////////////////////////////////////////////////// */

#connStatusFrame {{
    background-color: transparent;
    border: none;
}}

#connDotOff {{
    color: #f38ba8;
    font-size: {_p(10, k)}px;
}}

#connDotOn {{
    color: #a6e3a1;
    font-size: {_p(10, k)}px;
}}

#connDotWarn {{
    color: #f9e2af;
    font-size: {_p(10, k)}px;
}}

#connLabelOff {{
    font: {_f(9, k)}pt "Segoe UI";
    color: #6c7086;
}}

#connLabelOn {{
    font: {_f(9, k)}pt "Segoe UI";
    color: #a6e3a1;
}}

#connLabelWarn {{
    font: {_f(9, k)}pt "Segoe UI";
    color: #f9e2af;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   GROUP BOXES
///////////////////////////////////////////////////////////////////////////////////////////////// */

QGroupBox {{
    background-color: #232334;
    border: 1px solid #313244;
    border-radius: {_p(6, k)}px;
    margin-top: {_p(14, k)}px;
    padding: {_p(14, k)}px {_p(8, k)}px {_p(8, k)}px {_p(8, k)}px;
    font-weight: bold;
}}

QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: {_p(2, k)}px {_p(8, k)}px;
    color: #a6adc8;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   BUTTONS
///////////////////////////////////////////////////////////////////////////////////////////////// */

QPushButton {{
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: {_p(4, k)}px;
    padding: {_p(5, k)}px {_p(14, k)}px;
    min-height: {_p(22, k)}px;
    color: #cdd6f4;
}}

QPushButton:hover {{
    background-color: #45475a;
    border-color: #89b4fa;
}}

QPushButton:pressed {{
    background-color: #252536;
}}

QPushButton:disabled {{
    background-color: #252536;
    color: #6c7086;
    border-color: #313244;
}}

QPushButton#accentBtn {{
    background-color: #352b5a;
    border: 1px solid #cba6f7;
    color: #cba6f7;
}}

QPushButton#accentBtn:hover {{
    background-color: #453b6a;
}}

QPushButton#successBtn {{
    background-color: #2b5a3a;
    border-color: #a6e3a1;
    color: #a6e3a1;
}}

QPushButton#successBtn:hover {{
    background-color: #3a7a4f;
}}

QPushButton#dangerBtn {{
    background-color: #5a2b3a;
    border-color: #f38ba8;
    color: #f38ba8;
}}

QPushButton#dangerBtn:hover {{
    background-color: #7a3a4f;
}}

QPushButton#warningBtn {{
    background-color: #5a4a2b;
    border-color: #f9e2af;
    color: #f9e2af;
}}

QPushButton#warningBtn:hover {{
    background-color: #7a6a3a;
}}

QPushButton#jogBtn {{
    background-color: #313244;
    border: {_p(2, k)}px solid #45475a;
    border-radius: {_p(6, k)}px;
    min-width: {_p(46, k)}px;
    min-height: {_p(46, k)}px;
    font-size: {_f(16, k)}pt;
}}

QPushButton#jogBtn:hover {{
    background-color: #3a3a4f;
    border-color: #89b4fa;
}}

QPushButton#jogBtn:pressed {{
    background-color: #89b4fa;
    color: #1e1e2e;
}}

QPushButton#flatBtn {{
    background-color: transparent;
    border: none;
    color: #a6adc8;
    padding: {_p(4, k)}px {_p(8, k)}px;
}}

QPushButton#flatBtn:hover {{
    color: #cdd6f4;
    background-color: #313244;
    border-radius: {_p(4, k)}px;
}}

/* Legacy button names (backward compat with widgets) */
QPushButton#connectBtn {{
    background-color: #2b5a3a;
    border-color: #a6e3a1;
    color: #a6e3a1;
}}
QPushButton#connectBtn:hover {{
    background-color: #3a7a4f;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   INPUTS
///////////////////////////////////////////////////////////////////////////////////////////////// */

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: {_p(4, k)}px;
    padding: {_p(4, k)}px {_p(8, k)}px;
    color: #cdd6f4;
    min-height: {_p(22, k)}px;
}}

QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{
    border-color: #cba6f7;
}}

QComboBox::drop-down {{
    border: none;
    padding-right: {_p(6, k)}px;
}}

QComboBox QAbstractItemView {{
    background-color: #313244;
    border: 1px solid #45475a;
    color: #cdd6f4;
    selection-background-color: #45475a;
    selection-color: #cdd6f4;
    outline: 0;
}}

QComboBox QAbstractItemView::item {{
    background-color: #313244;
    color: #cdd6f4;
    padding: {_p(4, k)}px {_p(8, k)}px;
    min-height: {_p(22, k)}px;
}}

QComboBox QAbstractItemView::item:hover {{
    background-color: #45475a;
    color: #cdd6f4;
}}

QComboBox QAbstractItemView::item:selected {{
    background-color: #45475a;
    color: #cdd6f4;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   SLIDERS
///////////////////////////////////////////////////////////////////////////////////////////////// */

QSlider::groove:horizontal {{
    border: none;
    height: {_p(6, k)}px;
    background-color: #313244;
    border-radius: {_p(3, k)}px;
}}

QSlider::handle:horizontal {{
    background-color: #cba6f7;
    border: {_p(2, k)}px solid #cba6f7;
    width: {_p(14, k)}px;
    margin: -{_p(5, k)}px 0;
    border-radius: {_p(7, k)}px;
}}

QSlider::handle:horizontal:hover {{
    background-color: #f5c2e7;
    border-color: #f5c2e7;
}}

QSlider::sub-page:horizontal {{
    background-color: #cba6f7;
    border-radius: {_p(3, k)}px;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   SCROLLBAR
///////////////////////////////////////////////////////////////////////////////////////////////// */

QScrollBar:vertical {{
    background-color: #1e1e2e;
    width: {_p(10, k)}px;
    border: none;
}}

QScrollBar::handle:vertical {{
    background-color: #45475a;
    border-radius: {_p(5, k)}px;
    min-height: {_p(20, k)}px;
}}

QScrollBar::handle:vertical:hover {{
    background-color: #585b70;
}}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}

QScrollBar:horizontal {{
    background-color: #1e1e2e;
    height: {_p(10, k)}px;
    border: none;
}}

QScrollBar::handle:horizontal {{
    background-color: #45475a;
    border-radius: {_p(5, k)}px;
    min-width: {_p(20, k)}px;
}}

QScrollBar::handle:horizontal:hover {{
    background-color: #585b70;
}}

QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
    width: 0;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   PROGRESS BAR
///////////////////////////////////////////////////////////////////////////////////////////////// */

QProgressBar {{
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: {_p(4, k)}px;
    text-align: center;
    color: #cdd6f4;
    min-height: {_p(18, k)}px;
}}

QProgressBar::chunk {{
    background-color: #cba6f7;
    border-radius: {_p(3, k)}px;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   CHECKBOXES
///////////////////////////////////////////////////////////////////////////////////////////////// */

QCheckBox {{
    spacing: {_p(8, k)}px;
    color: #cdd6f4;
}}

QCheckBox::indicator {{
    width: {_p(16, k)}px;
    height: {_p(16, k)}px;
    border: {_p(2, k)}px solid #45475a;
    border-radius: {_p(3, k)}px;
    background-color: #313244;
}}

QCheckBox::indicator:hover {{
    border-color: #cba6f7;
}}

QCheckBox::indicator:checked {{
    background-color: #cba6f7;
    border-color: #cba6f7;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   SPLITTER
///////////////////////////////////////////////////////////////////////////////////////////////// */

QSplitter::handle {{
    background-color: #313244;
}}

QSplitter::handle:horizontal {{
    width: {_p(4, k)}px;
}}

QSplitter::handle:vertical {{
    height: {_p(2, k)}px;
}}

QSplitter::handle:hover {{
    background-color: #cba6f7;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   LABELS
///////////////////////////////////////////////////////////////////////////////////////////////// */

QLabel#headerLabel {{
    font: bold {_f(10, k)}pt "Segoe UI";
    color: #cdd6f4;
}}

QLabel#statusConnected {{
    color: #a6e3a1;
    font-weight: bold;
}}

QLabel#statusDisconnected {{
    color: #f38ba8;
}}

QLabel#sectionLabel {{
    color: #cba6f7;
    font: bold {_f(11, k)}pt "Segoe UI";
    padding-top: {_p(4, k)}px;
}}

QLabel#dimLabel {{
    color: #6c7086;
    font: {_f(9, k)}pt "Segoe UI";
}}

QLabel#valueLabel {{
    font: {_f(12, k)}pt "Consolas", "Ubuntu Mono", monospace;
    color: #cdd6f4;
}}

QLabel#pageTitle {{
    color: #cdd6f4;
    font: bold {_f(14, k)}pt "Segoe UI";
    padding: {_p(4, k)}px 0;
}}

QLabel#pageSubtitle {{
    color: #6c7086;
    font: {_f(9, k)}pt "Segoe UI";
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   UNIT LABELS (microns, mm, etc.)
///////////////////////////////////////////////////////////////////////////////////////////////// */

QLabel#unitLabel {{
    color: #6c7086;
    font: {_f(8, k)}pt "Segoe UI";
    padding: 0px {_p(2, k)}px;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   TEXT EDIT (console)
///////////////////////////////////////////////////////////////////////////////////////////////// */

QTextEdit {{
    background-color: #11111b;
    border: 1px solid #313244;
    border-radius: {_p(4, k)}px;
    color: #cdd6f4;
    font: {_f(9, k)}pt "Consolas", "Ubuntu Mono", monospace;
    padding: {_p(4, k)}px;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   LIST WIDGET
///////////////////////////////////////////////////////////////////////////////////////////////// */

QListWidget {{
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: {_p(4, k)}px;
    alternate-background-color: #2a2a3c;
    color: #cdd6f4;
}}

QListWidget::item:selected {{
    background-color: #45475a;
    color: #cdd6f4;
}}

QListWidget::item:hover {{
    background-color: #3a3a4c;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   TABLE WIDGET
///////////////////////////////////////////////////////////////////////////////////////////////// */

QTableWidget {{
    background-color: #313244;
    border: 1px solid #45475a;
    gridline-color: #45475a;
    color: #cdd6f4;
}}

QTableWidget::item:selected {{
    background-color: #45475a;
}}

QHeaderView::section {{
    background-color: #252536;
    border: 1px solid #313244;
    padding: {_p(4, k)}px;
    color: #a6adc8;
    font-weight: bold;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   DIALOGS
///////////////////////////////////////////////////////////////////////////////////////////////// */

QDialog {{
    background-color: #1e1e2e;
}}

QMessageBox {{
    background-color: #1e1e2e;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   TAB WIDGET (used inside pages for sub-sections)
///////////////////////////////////////////////////////////////////////////////////////////////// */

QTabWidget::pane {{
    border: 1px solid #313244;
    border-top: none;
    background-color: #1e1e2e;
}}

QTabBar::tab {{
    background-color: #232334;
    border: 1px solid #313244;
    border-bottom: none;
    border-top-left-radius: {_p(4, k)}px;
    border-top-right-radius: {_p(4, k)}px;
    padding: {_p(6, k)}px {_p(16, k)}px;
    margin-right: {_p(2, k)}px;
    color: #6c7086;
}}

QTabBar::tab:selected {{
    background-color: #1e1e2e;
    border-bottom: {_p(2, k)}px solid #cba6f7;
    color: #cdd6f4;
}}

QTabBar::tab:hover:!selected {{
    background-color: #313244;
    color: #a6adc8;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   FRAME CARDS (reusable styled containers)
///////////////////////////////////////////////////////////////////////////////////////////////// */

#cardFrame {{
    background-color: #232334;
    border: 1px solid #313244;
    border-radius: {_p(8, k)}px;
    padding: {_p(12, k)}px;
}}

#cardFrame:hover {{
    border-color: #45475a;
    background-color: #262639;
}}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   RESPONSIVE — smaller inputs for context panel
///////////////////////////////////////////////////////////////////////////////////////////////// */

#extraLeftBox QDoubleSpinBox,
#extraLeftBox QSpinBox,
#extraLeftBox QComboBox {{
    min-height: {_p(20, k)}px;
    padding: {_p(2, k)}px {_p(6, k)}px;
    font-size: {_f(9, k)}pt;
}}

#extraLeftBox QPushButton {{
    min-height: {_p(20, k)}px;
    padding: {_p(3, k)}px {_p(8, k)}px;
    font-size: {_f(9, k)}pt;
}}

#extraLeftBox QCheckBox {{
    spacing: {_p(6, k)}px;
    font-size: {_f(9, k)}pt;
}}

#extraLeftBox QProgressBar {{
    min-height: {_p(14, k)}px;
    font-size: {_f(8, k)}pt;
}}

#extraLeftBox QListWidget {{
    font-size: {_f(9, k)}pt;
}}

"""


def apply_scaled_styles(k: float = 1.0):
    """Update module-level style constants with the given scale factor.

    Call this once at app startup (after QApplication is created and the
    scale factor is known) so that any code importing the constants
    gets DPI-aware values.
    """
    global SECTION_TITLE_STYLE, CONTEXT_SECTION_LABEL_STYLE
    global PAGE_HEADER_STYLE, CARD_FRAME_STYLE
    global MENU_BTN_EXPANDED_STYLE, DARK_THEME

    SECTION_TITLE_STYLE = build_section_title_style(k)
    CONTEXT_SECTION_LABEL_STYLE = build_context_section_label_style(k)
    PAGE_HEADER_STYLE = build_page_header_style(k)
    CARD_FRAME_STYLE = build_card_frame_style(k)
    MENU_BTN_EXPANDED_STYLE = (
        "text-align: left;"
        f"padding-left: {_p(44, k)}px;"
    )
    DARK_THEME = build_theme(k)


# Default at scale=1.0 for backward compatibility and import-time access
DARK_THEME = build_theme(1.0)
