"""
Dark theme stylesheet — Catppuccin Mocha palette.

Colour reference (https://catppuccin.com/palette):
    Base       #1e1e2e     Main background
    Mantle     #181825     Darker panels / canvas
    Crust      #11111b     Deepest background
    Surface0   #313244     Card / group box background
    Surface1   #45475a     Hover, borders
    Surface2   #585b70     Active borders
    Overlay0   #6c7086     Disabled / dim text
    Subtext0   #a6adc8     Secondary text
    Text       #cdd6f4     Primary text
    Green      #a6e3a1     Success / connected
    Red        #f38ba8     Error / danger / disconnect
    Yellow     #f9e2af     Warning / paused
    Blue       #89b4fa     Accent / links
    Mauve      #cba6f7     Selected tab accent
    Peach      #fab387     Highlights
"""

DARK_THEME = """
/* ── Global ───────────────────────────────────────────────────── */

QWidget {
    background-color: #1e1e2e;
    color: #cdd6f4;
    font-family: "Segoe UI", "Ubuntu", sans-serif;
    font-size: 10pt;
}

/* ── Group boxes ──────────────────────────────────────────────── */

QGroupBox {
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: 6px;
    margin-top: 10px;
    padding: 14px 8px 8px 8px;
    font-weight: bold;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 2px 8px;
    color: #a6adc8;
}

/* ── Buttons ──────────────────────────────────────────────────── */

QPushButton {
    background-color: #45475a;
    border: 1px solid #585b70;
    border-radius: 4px;
    padding: 5px 14px;
    min-height: 22px;
    color: #cdd6f4;
}

QPushButton:hover {
    background-color: #585b70;
    border-color: #89b4fa;
}

QPushButton:pressed {
    background-color: #313244;
}

QPushButton:disabled {
    background-color: #313244;
    color: #6c7086;
    border-color: #45475a;
}

QPushButton#connectBtn {
    background-color: #2b5a3a;
    border-color: #a6e3a1;
    color: #a6e3a1;
}

QPushButton#connectBtn:hover {
    background-color: #3a7a4f;
}

QPushButton#disconnectBtn {
    background-color: #5a2b3a;
    border-color: #f38ba8;
    color: #f38ba8;
}

QPushButton#disconnectBtn:hover {
    background-color: #7a3a4f;
}

QPushButton#jogBtn {
    background-color: #45475a;
    border: 2px solid #585b70;
    border-radius: 6px;
    min-width: 46px;
    min-height: 46px;
    font-size: 16pt;
}

QPushButton#jogBtn:pressed {
    background-color: #89b4fa;
    color: #1e1e2e;
}

/* ── Tabs ─────────────────────────────────────────────────────── */

QTabWidget::pane {
    border: 1px solid #45475a;
    border-top: none;
    background-color: #1e1e2e;
}

QTabBar::tab {
    background-color: #313244;
    border: 1px solid #45475a;
    border-bottom: none;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    padding: 6px 16px;
    margin-right: 2px;
    color: #a6adc8;
}

QTabBar::tab:selected {
    background-color: #1e1e2e;
    border-bottom: 2px solid #cba6f7;
    color: #cdd6f4;
}

QTabBar::tab:hover:!selected {
    background-color: #45475a;
}

/* ── Inputs ───────────────────────────────────────────────────── */

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: 4px;
    padding: 4px 8px;
    color: #cdd6f4;
    min-height: 22px;
}

QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {
    border-color: #89b4fa;
}

QComboBox::drop-down {
    border: none;
    padding-right: 6px;
}

QComboBox QAbstractItemView {
    background-color: #313244;
    border: 1px solid #585b70;
    color: #cdd6f4;
    selection-background-color: #45475a;
}

/* ── Sliders ──────────────────────────────────────────────────── */

QSlider::groove:horizontal {
    background: #45475a;
    height: 6px;
    border-radius: 3px;
}

QSlider::handle:horizontal {
    background: #89b4fa;
    width: 16px;
    margin: -5px 0;
    border-radius: 8px;
}

QSlider::handle:horizontal:hover {
    background: #b4d0fb;
}

QSlider::groove:vertical {
    background: #45475a;
    width: 6px;
    border-radius: 3px;
}

QSlider::handle:vertical {
    background: #89b4fa;
    height: 16px;
    margin: 0 -5px;
    border-radius: 8px;
}

/* ── Scroll area ──────────────────────────────────────────────── */

QScrollArea {
    border: none;
}

QScrollBar:vertical {
    background: #181825;
    width: 10px;
    margin: 0;
    border-radius: 5px;
}

QScrollBar::handle:vertical {
    background: #45475a;
    min-height: 30px;
    border-radius: 5px;
}

QScrollBar::handle:vertical:hover {
    background: #585b70;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0;
}

QScrollBar:horizontal {
    background: #181825;
    height: 10px;
    border-radius: 5px;
}

QScrollBar::handle:horizontal {
    background: #45475a;
    min-width: 30px;
    border-radius: 5px;
}

/* ── Progress bar ─────────────────────────────────────────────── */

QProgressBar {
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: 4px;
    text-align: center;
    color: #cdd6f4;
    min-height: 18px;
}

QProgressBar::chunk {
    background-color: #a6e3a1;
    border-radius: 3px;
}

/* ── Check boxes ──────────────────────────────────────────────── */

QCheckBox {
    spacing: 8px;
}

QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border: 1px solid #585b70;
    border-radius: 3px;
    background-color: #313244;
}

QCheckBox::indicator:checked {
    background-color: #89b4fa;
    border-color: #89b4fa;
}

/* ── Splitter ─────────────────────────────────────────────────── */

QSplitter::handle {
    background-color: #45475a;
    height: 3px;
}

QSplitter::handle:hover {
    background-color: #89b4fa;
}

/* ── Status bar ───────────────────────────────────────────────── */

QStatusBar {
    background-color: #181825;
    border-top: 1px solid #313244;
    color: #a6adc8;
    font-size: 9pt;
}

/* ── Labels ───────────────────────────────────────────────────── */

QLabel#headerLabel {
    font-weight: bold;
    font-size: 10pt;
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
    color: #89b4fa;
    font-size: 11pt;
    font-weight: bold;
    padding-top: 4px;
}

QLabel#dimLabel {
    color: #6c7086;
    font-size: 9pt;
}

QLabel#valueLabel {
    font-family: "Consolas", "Ubuntu Mono", monospace;
    font-size: 12pt;
    color: #cdd6f4;
}

/* ── Text edit (console) ──────────────────────────────────────── */

QTextEdit {
    background-color: #181825;
    border: 1px solid #313244;
    border-radius: 4px;
    color: #cdd6f4;
    font-family: "Consolas", "Ubuntu Mono", monospace;
    font-size: 9pt;
    padding: 4px;
}

/* ── List widget ──────────────────────────────────────────────── */

QListWidget {
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: 4px;
    alternate-background-color: #3b3c52;
    color: #cdd6f4;
}

QListWidget::item:selected {
    background-color: #45475a;
    color: #cdd6f4;
}

/* ── Table widget ─────────────────────────────────────────────── */

QTableWidget {
    background-color: #313244;
    border: 1px solid #45475a;
    gridline-color: #45475a;
    color: #cdd6f4;
}

QHeaderView::section {
    background-color: #45475a;
    border: 1px solid #585b70;
    padding: 4px;
    color: #cdd6f4;
    font-weight: bold;
}

/* ── Tooltip ──────────────────────────────────────────────────── */

QToolTip {
    background-color: #313244;
    border: 1px solid #585b70;
    color: #cdd6f4;
    padding: 4px;
}

/* ── Dialog ───────────────────────────────────────────────────── */

QDialog {
    background-color: #1e1e2e;
}
"""
