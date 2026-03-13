"""
Dark theme stylesheet — PyDracula-inspired, Catppuccin Mocha palette.

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



# ── Centralized Section Styles (v7.2.4) ─────────────────────────
# Use these instead of per-page _group_style() methods for consistency.

SECTION_TITLE_STYLE = f"""
    QGroupBox {{
        font-size: 11pt;
        font-weight: 600;
        color: {COLORS['text']};
        border: 1px solid {COLORS['surface1']};
        border-radius: 8px;
        margin-top: 14px;
        padding: 16px 12px 10px 12px;
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        subcontrol-position: top left;
        left: 12px;
        top: 0px;
        padding: 2px 8px;
        background-color: {COLORS['base']};
        border-radius: 4px;
        color: {COLORS['blue']};
        font-size: 10pt;
        font-weight: 600;
    }}
"""

CONTEXT_SECTION_LABEL_STYLE = f"""
    font-size: 10pt;
    font-weight: 600;
    color: {COLORS['blue']};
    padding: 6px 0px 2px 0px;
    border-bottom: 1px solid {COLORS['surface1']};
    margin-bottom: 4px;
"""

PAGE_HEADER_STYLE = f"""
    font-size: 14pt;
    font-weight: 700;
    color: {COLORS['text']};
    padding: 4px 0px;
"""

CARD_FRAME_STYLE = f"""
    QFrame#cardFrame {{
        background-color: {COLORS['surface0']};
        border: 1px solid {COLORS['surface1']};
        border-radius: 8px;
        padding: 12px;
    }}
"""

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
DARK_THEME = """

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   GLOBAL
///////////////////////////////////////////////////////////////////////////////////////////////// */

QWidget {
    color: #cdd6f4;
    font: 10pt "Segoe UI", "Ubuntu", sans-serif;
}

QToolTip {
    color: #cdd6f4;
    background-color: rgba(30, 30, 46, 230);
    border: 1px solid #45475a;
    border-left: 2px solid #cba6f7;
    padding: 4px 8px;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   APP BACKGROUND
///////////////////////////////////////////////////////////////////////////////////////////////// */

#bgApp {
    background-color: #1e1e2e;
    border: 1px solid #313244;
    border-radius: 8px;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   LEFT MENU
///////////////////////////////////////////////////////////////////////////////////////////////// */

#leftMenuBg {
    background-color: #181825;
}

#topLogo {
    background-color: #181825;
    padding: 8px;
}

#titleLeftApp {
    font: 63 12pt "Segoe UI Semibold";
    color: #cdd6f4;
}

#titleLeftDescription {
    font: 8pt "Segoe UI";
    color: #cba6f7;
}

/* Main menu buttons — default state is COLLAPSED (centered icons) */
#topMenu QPushButton {
    background-position: center;
    background-repeat: no-repeat;
    border: none;
    border-left: 3px solid transparent;
    background-color: transparent;
    text-align: center;
    padding-left: 0px;
    padding-top: 5px;
    padding-bottom: 5px;
    color: #a6adc8;
    margin: 1px 4px;
    border-radius: 0px 4px 4px 0px;
    font: 14pt "Segoe UI Emoji", "Apple Color Emoji", "Noto Color Emoji", sans-serif;
}

#topMenu QPushButton:hover {
    background-color: #252536;
    border-left: 3px solid #585b70;
    color: #cdd6f4;
}

#topMenu QPushButton:pressed {
    background-color: rgba(203, 166, 247, 0.15);
    border-left: 3px solid #cba6f7;
    color: #cdd6f4;
}

/* Bottom menu buttons — same collapsed-by-default pattern */
#bottomMenu QPushButton {
    background-position: center;
    background-repeat: no-repeat;
    border: none;
    border-left: 3px solid transparent;
    background-color: transparent;
    text-align: center;
    padding-left: 0px;
    padding-top: 5px;
    padding-bottom: 5px;
    color: #a6adc8;
    margin: 1px 4px;
    border-radius: 0px 4px 4px 0px;
    font: 14pt "Segoe UI Emoji", "Apple Color Emoji", "Noto Color Emoji", sans-serif;
}

#bottomMenu QPushButton:hover {
    background-color: #252536;
    border-left: 3px solid #585b70;
    color: #cdd6f4;
}

#bottomMenu QPushButton:pressed {
    background-color: rgba(203, 166, 247, 0.15);
    border-left: 3px solid #cba6f7;
    color: #cdd6f4;
}

#leftMenuFrame {
    border-top: 2px solid #313244;
}

/* Toggle button */
#toggleButton {
    background-position: left center;
    background-repeat: no-repeat;
    border: none;
    border-left: 3px solid transparent;
    background-color: #181825;
    text-align: center;
    padding-left: 0px;
    color: #6c7086;
    margin: 1px 4px;
    border-radius: 0px 4px 4px 0px;
    font: 12pt "Segoe UI";
}

#toggleButton:hover {
    background-color: #252536;
    color: #a6adc8;
}

#toggleButton:pressed {
    background-color: rgba(203, 166, 247, 0.15);
    color: #cba6f7;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   EXTRA LEFT BOX (context settings panel)
///////////////////////////////////////////////////////////////////////////////////////////////// */

#extraLeftBox {
    background-color: #232334;
}

#extraTopBg {
    background-color: #cba6f7;
}

#extraLabel {
    color: #1e1e2e;
    font: bold 10pt "Segoe UI";
}

#extraCloseColumnBtn {
    background-color: transparent;
    border: none;
    border-radius: 4px;
}

#extraCloseColumnBtn:hover {
    background-color: rgba(30, 30, 46, 80);
}

#extraContent {
    border-top: 2px solid #313244;
}

#extraTopMenu QPushButton {
    background-position: left center;
    background-repeat: no-repeat;
    border: none;
    border-left: 22px solid transparent;
    background-color: transparent;
    text-align: left;
    padding-left: 44px;
    color: #cdd6f4;
}

#extraTopMenu QPushButton:hover {
    background-color: #313244;
}

/* Context panel scroll area */
#contextScrollArea {
    background-color: transparent;
    border: none;
}

#contextScrollArea QWidget {
    background-color: transparent;
}

/* Context panel section headers */
#contextSectionLabel {
    color: #cba6f7;
    font: bold 9pt "Segoe UI";
    padding: 8px 4px 4px 4px;
    border-bottom: 1px solid #313244;
}

/* Context panel form labels */
#contextLabel {
    color: #a6adc8;
    font: 9pt "Segoe UI";
    padding: 2px;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   CONTENT AREA
///////////////////////////////////////////////////////////////////////////////////////////////// */

#contentBox {
    background-color: #1e1e2e;
}

#contentTopBg {
    background-color: #181825;
}

#titleRightInfo {
    padding-left: 10px;
    color: #a6adc8;
    font: 9pt "Segoe UI";
}

#contentBottom {
    border-top: 2px solid #313244;
}

/* Top right buttons */
#rightButtons QPushButton {
    background-color: transparent;
    border: none;
    border-radius: 4px;
    padding: 4px;
}

#rightButtons QPushButton:hover {
    background-color: #313244;
}

#rightButtons QPushButton:pressed {
    background-color: #45475a;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   BOTTOM BAR
///////////////////////////////////////////////////////////////////////////////////////////////// */

#bottomBar {
    background-color: #181825;
    border-top: 1px solid #313244;
}

#bottomBar QLabel {
    font: 9pt "Consolas", "Ubuntu Mono", monospace;
    color: #6c7086;
    padding-left: 8px;
    padding-right: 8px;
    padding-bottom: 2px;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   CONNECTION STATUS INDICATORS
///////////////////////////////////////////////////////////////////////////////////////////////// */

#connStatusFrame {
    background-color: transparent;
    border: none;
}

#connDotOff {
    color: #f38ba8;
    font-size: 10px;
}

#connDotOn {
    color: #a6e3a1;
    font-size: 10px;
}

#connDotWarn {
    color: #f9e2af;
    font-size: 10px;
}

#connLabelOff {
    font: 9pt "Segoe UI";
    color: #6c7086;
}

#connLabelOn {
    font: 9pt "Segoe UI";
    color: #a6e3a1;
}

#connLabelWarn {
    font: 9pt "Segoe UI";
    color: #f9e2af;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   GROUP BOXES
///////////////////////////////////////////////////////////////////////////////////////////////// */

QGroupBox {
    background-color: #232334;
    border: 1px solid #313244;
    border-radius: 6px;
    margin-top: 14px;
    padding: 14px 8px 8px 8px;
    font-weight: bold;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 2px 8px;
    color: #a6adc8;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   BUTTONS
///////////////////////////////////////////////////////////////////////////////////////////////// */

QPushButton {
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: 4px;
    padding: 5px 14px;
    min-height: 22px;
    color: #cdd6f4;
}

QPushButton:hover {
    background-color: #45475a;
    border-color: #89b4fa;
}

QPushButton:pressed {
    background-color: #252536;
}

QPushButton:disabled {
    background-color: #252536;
    color: #6c7086;
    border-color: #313244;
}

QPushButton#accentBtn {
    background-color: #352b5a;
    border: 1px solid #cba6f7;
    color: #cba6f7;
}

QPushButton#accentBtn:hover {
    background-color: #453b6a;
}

QPushButton#successBtn {
    background-color: #2b5a3a;
    border-color: #a6e3a1;
    color: #a6e3a1;
}

QPushButton#successBtn:hover {
    background-color: #3a7a4f;
}

QPushButton#dangerBtn {
    background-color: #5a2b3a;
    border-color: #f38ba8;
    color: #f38ba8;
}

QPushButton#dangerBtn:hover {
    background-color: #7a3a4f;
}

QPushButton#warningBtn {
    background-color: #5a4a2b;
    border-color: #f9e2af;
    color: #f9e2af;
}

QPushButton#warningBtn:hover {
    background-color: #7a6a3a;
}

QPushButton#jogBtn {
    background-color: #313244;
    border: 2px solid #45475a;
    border-radius: 6px;
    min-width: 46px;
    min-height: 46px;
    font-size: 16pt;
}

QPushButton#jogBtn:hover {
    background-color: #3a3a4f;
    border-color: #89b4fa;
}

QPushButton#jogBtn:pressed {
    background-color: #89b4fa;
    color: #1e1e2e;
}

QPushButton#flatBtn {
    background-color: transparent;
    border: none;
    color: #a6adc8;
    padding: 4px 8px;
}

QPushButton#flatBtn:hover {
    color: #cdd6f4;
    background-color: #313244;
    border-radius: 4px;
}

/* Legacy button names (backward compat with widgets) */
QPushButton#connectBtn {
    background-color: #2b5a3a;
    border-color: #a6e3a1;
    color: #a6e3a1;
}
QPushButton#connectBtn:hover {
    background-color: #3a7a4f;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   INPUTS
///////////////////////////////////////////////////////////////////////////////////////////////// */

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: 4px;
    padding: 4px 8px;
    color: #cdd6f4;
    min-height: 22px;
}

QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {
    border-color: #cba6f7;
}

QComboBox::drop-down {
    border: none;
    padding-right: 6px;
}

QComboBox QAbstractItemView {
    background-color: #313244;
    border: 1px solid #45475a;
    color: #cdd6f4;
    selection-background-color: #45475a;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   SLIDERS
///////////////////////////////////////////////////////////////////////////////////////////////// */

QSlider::groove:horizontal {
    border: none;
    height: 6px;
    background-color: #313244;
    border-radius: 3px;
}

QSlider::handle:horizontal {
    background-color: #cba6f7;
    border: 2px solid #cba6f7;
    width: 14px;
    margin: -5px 0;
    border-radius: 7px;
}

QSlider::handle:horizontal:hover {
    background-color: #f5c2e7;
    border-color: #f5c2e7;
}

QSlider::sub-page:horizontal {
    background-color: #cba6f7;
    border-radius: 3px;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   SCROLLBAR
///////////////////////////////////////////////////////////////////////////////////////////////// */

QScrollBar:vertical {
    background-color: #1e1e2e;
    width: 10px;
    border: none;
}

QScrollBar::handle:vertical {
    background-color: #45475a;
    border-radius: 5px;
    min-height: 20px;
}

QScrollBar::handle:vertical:hover {
    background-color: #585b70;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0;
}

QScrollBar:horizontal {
    background-color: #1e1e2e;
    height: 10px;
    border: none;
}

QScrollBar::handle:horizontal {
    background-color: #45475a;
    border-radius: 5px;
    min-width: 20px;
}

QScrollBar::handle:horizontal:hover {
    background-color: #585b70;
}

QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
    width: 0;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   PROGRESS BAR
///////////////////////////////////////////////////////////////////////////////////////////////// */

QProgressBar {
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: 4px;
    text-align: center;
    color: #cdd6f4;
    min-height: 18px;
}

QProgressBar::chunk {
    background-color: #cba6f7;
    border-radius: 3px;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   CHECKBOXES
///////////////////////////////////////////////////////////////////////////////////////////////// */

QCheckBox {
    spacing: 8px;
    color: #cdd6f4;
}

QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border: 2px solid #45475a;
    border-radius: 3px;
    background-color: #313244;
}

QCheckBox::indicator:hover {
    border-color: #cba6f7;
}

QCheckBox::indicator:checked {
    background-color: #cba6f7;
    border-color: #cba6f7;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   SPLITTER
///////////////////////////////////////////////////////////////////////////////////////////////// */

QSplitter::handle {
    background-color: #313244;
}

QSplitter::handle:horizontal {
    width: 4px;
}

QSplitter::handle:vertical {
    height: 2px;
}

QSplitter::handle:hover {
    background-color: #cba6f7;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   LABELS
///////////////////////////////////////////////////////////////////////////////////////////////// */

QLabel#headerLabel {
    font: bold 10pt "Segoe UI";
    color: #cdd6f4;
}

QLabel#statusConnected {
    color: #a6e3a1;
    font-weight: bold;
}

QLabel#statusDisconnected {
    color: #f38ba8;
}

QLabel#sectionLabel {
    color: #cba6f7;
    font: bold 11pt "Segoe UI";
    padding-top: 4px;
}

QLabel#dimLabel {
    color: #6c7086;
    font: 9pt "Segoe UI";
}

QLabel#valueLabel {
    font: 12pt "Consolas", "Ubuntu Mono", monospace;
    color: #cdd6f4;
}

QLabel#pageTitle {
    color: #cdd6f4;
    font: bold 14pt "Segoe UI";
    padding: 4px 0;
}

QLabel#pageSubtitle {
    color: #6c7086;
    font: 9pt "Segoe UI";
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   UNIT LABELS (microns, mm, etc.)
///////////////////////////////////////////////////////////////////////////////////////////////// */

QLabel#unitLabel {
    color: #6c7086;
    font: 8pt "Segoe UI";
    padding: 0px 2px;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   TEXT EDIT (console)
///////////////////////////////////////////////////////////////////////////////////////////////// */

QTextEdit {
    background-color: #11111b;
    border: 1px solid #313244;
    border-radius: 4px;
    color: #cdd6f4;
    font: 9pt "Consolas", "Ubuntu Mono", monospace;
    padding: 4px;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   LIST WIDGET
///////////////////////////////////////////////////////////////////////////////////////////////// */

QListWidget {
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: 4px;
    alternate-background-color: #2a2a3c;
    color: #cdd6f4;
}

QListWidget::item:selected {
    background-color: #45475a;
    color: #cdd6f4;
}

QListWidget::item:hover {
    background-color: #3a3a4c;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   TABLE WIDGET
///////////////////////////////////////////////////////////////////////////////////////////////// */

QTableWidget {
    background-color: #313244;
    border: 1px solid #45475a;
    gridline-color: #45475a;
    color: #cdd6f4;
}

QTableWidget::item:selected {
    background-color: #45475a;
}

QHeaderView::section {
    background-color: #252536;
    border: 1px solid #313244;
    padding: 4px;
    color: #a6adc8;
    font-weight: bold;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   DIALOGS
///////////////////////////////////////////////////////////////////////////////////////////////// */

QDialog {
    background-color: #1e1e2e;
}

QMessageBox {
    background-color: #1e1e2e;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   TAB WIDGET (used inside pages for sub-sections)
///////////////////////////////////////////////////////////////////////////////////////////////// */

QTabWidget::pane {
    border: 1px solid #313244;
    border-top: none;
    background-color: #1e1e2e;
}

QTabBar::tab {
    background-color: #232334;
    border: 1px solid #313244;
    border-bottom: none;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    padding: 6px 16px;
    margin-right: 2px;
    color: #6c7086;
}

QTabBar::tab:selected {
    background-color: #1e1e2e;
    border-bottom: 2px solid #cba6f7;
    color: #cdd6f4;
}

QTabBar::tab:hover:!selected {
    background-color: #313244;
    color: #a6adc8;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   FRAME CARDS (reusable styled containers)
///////////////////////////////////////////////////////////////////////////////////////////////// */

#cardFrame {
    background-color: #232334;
    border: 1px solid #313244;
    border-radius: 8px;
    padding: 12px;
}

#cardFrame:hover {
    border-color: #45475a;
    background-color: #262639;
}

/* /////////////////////////////////////////////////////////////////////////////////////////////////
   RESPONSIVE — smaller inputs for context panel
///////////////////////////////////////////////////////////////////////////////////////////////// */

#extraLeftBox QDoubleSpinBox,
#extraLeftBox QSpinBox,
#extraLeftBox QComboBox {
    min-height: 20px;
    padding: 2px 6px;
    font-size: 9pt;
}

#extraLeftBox QPushButton {
    min-height: 20px;
    padding: 3px 8px;
    font-size: 9pt;
}

#extraLeftBox QCheckBox {
    spacing: 6px;
    font-size: 9pt;
}

#extraLeftBox QProgressBar {
    min-height: 14px;
    font-size: 8pt;
}

#extraLeftBox QListWidget {
    font-size: 9pt;
}

"""
