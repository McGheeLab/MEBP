"""
UI Functions — PyDracula-inspired animation and UI utility helpers.

Provides:
    - toggleMenu: Animate left sidebar expand/collapse
    - toggleLeftBox: Animate extra-left context panel open/close
    - selectMenu / deselectMenu / resetStyle: Menu button highlighting
    - uiDefinitions: Standard window setup (custom title bar, grips)
    - updateMenuButtonStates: Switch between icon-only (collapsed) and icon+label (expanded)

Adapted from PyDracula by Wanderson M. Pimenta, restyled for MEBP bioprinter.
"""

from PySide6.QtCore import (
    QPropertyAnimation, QEasingCurve, QParallelAnimationGroup, QEvent, Qt, QTimer,
)
from PySide6.QtWidgets import QPushButton, QSizeGrip, QGraphicsDropShadowEffect
from PySide6.QtGui import QColor
from gui.styles import (
    COLORS, MENU_SELECTED_STYLESHEET,
    MENU_BTN_COLLAPSED_STYLE, MENU_BTN_EXPANDED_STYLE,
)


class AppSettings:
    """Global UI settings — tweak these to customize the shell."""
    ENABLE_CUSTOM_TITLE_BAR = False      # Use native title bar (safer cross-platform)
    MENU_WIDTH = 200                     # Expanded left menu width (px)
    LEFT_BOX_WIDTH = 260                 # Extra-left context panel width (px)
    TIME_ANIMATION = 300                 # Animation duration (ms)

    # Dynamic style fragments
    MENU_SELECTED_STYLESHEET = MENU_SELECTED_STYLESHEET
    BTN_LEFT_BOX_COLOR = f"background-color: {COLORS['surface0']};"
    BTN_RIGHT_BOX_COLOR = f"background-color: {COLORS['surface0']};"


class UIFunctions:
    """Static helpers that operate on the MainWindow instance (passed as `self`)."""

    # ── Menu Toggle ──────────────────────────────────────────────

    @staticmethod
    def toggleMenu(window, animate: bool = True):
        """Expand or collapse the left navigation sidebar."""
        menu = window.ui_leftMenuBg
        width = menu.width()
        target = AppSettings.MENU_WIDTH if width <= 60 else 60

        # Show/hide text labels when expanding/collapsing
        expanding = target > 60
        if hasattr(window, '_logo_text'):
            window._logo_text.setVisible(expanding)

        # Update all menu buttons: icon-only when collapsed, icon+label when expanded
        UIFunctions.updateMenuButtonStates(window, expanding)

        if animate:
            group = QParallelAnimationGroup()
            for prop in (b"minimumWidth", b"maximumWidth"):
                anim = QPropertyAnimation(menu, prop)
                anim.setDuration(AppSettings.TIME_ANIMATION)
                anim.setStartValue(width)
                anim.setEndValue(target)
                anim.setEasingCurve(QEasingCurve.InOutQuart)
                group.addAnimation(anim)
            # Store reference so GC doesn't kill it mid-animation
            window._menu_anim = group
            group.start()
        else:
            menu.setMinimumWidth(target)
            menu.setMaximumWidth(target)

    # ── Menu Button State Management ─────────────────────────────

    @staticmethod
    def updateMenuButtonStates(window, expanded: bool):
        """
        Switch all menu buttons between collapsed (icon-only, centered)
        and expanded (icon + label, left-aligned) states.

        Requires menu buttons to have `_icon_text` and `_label_text`
        attributes set during creation (see MainWindow._make_menu_button).
        """
        for btn in window._menu_buttons:
            icon_text = getattr(btn, '_icon_text', '')
            label_text = getattr(btn, '_label_text', '')

            if expanded:
                # Show icon + label, left-aligned
                btn.setText(f"  {icon_text}   {label_text}")
                # Apply expanded alignment while preserving existing selection styles
                UIFunctions._apply_menu_alignment(btn, expanded=True)
            else:
                # Show icon only, centered
                btn.setText(icon_text)
                # Apply collapsed alignment while preserving existing selection styles
                UIFunctions._apply_menu_alignment(btn, expanded=False)

    @staticmethod
    def _apply_menu_alignment(btn: QPushButton, expanded: bool):
        """
        Apply text alignment to a menu button without disrupting selection styles.

        We inject alignment properties into the button's inline stylesheet,
        replacing any previous alignment block.
        """
        current = btn.styleSheet()
        # Remove any previous alignment block we inserted
        current = current.replace(MENU_BTN_COLLAPSED_STYLE, "")
        current = current.replace(MENU_BTN_EXPANDED_STYLE, "")
        # Add the new alignment block
        if expanded:
            btn.setStyleSheet(current + MENU_BTN_EXPANDED_STYLE)
        else:
            btn.setStyleSheet(current + MENU_BTN_COLLAPSED_STYLE)

    # ── Extra Left Box (Context Panel) Toggle ────────────────────

    @staticmethod
    def toggleLeftBox(window, animate: bool = True):
        """Open or close the extra-left context panel."""
        box = window.ui_extraLeftBox
        width = box.width()
        target = AppSettings.LEFT_BOX_WIDTH if width == 0 else 0

        if animate:
            group = QParallelAnimationGroup()
            for prop in (b"minimumWidth", b"maximumWidth"):
                anim = QPropertyAnimation(box, prop)
                anim.setDuration(AppSettings.TIME_ANIMATION)
                anim.setStartValue(width)
                anim.setEndValue(target)
                anim.setEasingCurve(QEasingCurve.InOutQuart)
                group.addAnimation(anim)
            window._left_box_anim = group
            group.start()
        else:
            box.setMinimumWidth(target)
            box.setMaximumWidth(target if target > 0 else 0)

    @staticmethod
    def setLeftBoxWidth(window, width: int, animate: bool = True):
        """Set the extra-left box to a specific width."""
        box = window.ui_extraLeftBox
        current = box.width()
        if current == width:
            return
        if animate:
            group = QParallelAnimationGroup()
            for prop in (b"minimumWidth", b"maximumWidth"):
                anim = QPropertyAnimation(box, prop)
                anim.setDuration(AppSettings.TIME_ANIMATION)
                anim.setStartValue(current)
                anim.setEndValue(width)
                anim.setEasingCurve(QEasingCurve.InOutQuart)
                group.addAnimation(anim)
            window._left_box_anim = group
            group.start()
        else:
            box.setMinimumWidth(width)
            box.setMaximumWidth(width if width > 0 else 0)

    # ── Menu Selection Styling ───────────────────────────────────

    @staticmethod
    def selectMenu(style: str) -> str:
        """Append the selection border to the button stylesheet."""
        return style + AppSettings.MENU_SELECTED_STYLESHEET

    @staticmethod
    def deselectMenu(style: str) -> str:
        """Remove the selection border from the button stylesheet."""
        return style.replace(AppSettings.MENU_SELECTED_STYLESHEET, "")

    @staticmethod
    def selectStandardMenu(window, widget_name: str):
        """Highlight the specified button in the top menu."""
        for btn in window.ui_topMenu.findChildren(QPushButton):
            if btn.objectName() == widget_name:
                btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))

    @staticmethod
    def resetStyle(window, except_widget: str):
        """Remove selection highlight from all top menu buttons except `except_widget`."""
        for btn in window.ui_topMenu.findChildren(QPushButton):
            if btn.objectName() != except_widget:
                btn.setStyleSheet(UIFunctions.deselectMenu(btn.styleSheet()))

    # ── Drop Shadow ──────────────────────────────────────────────

    @staticmethod
    def addShadow(widget, blur: int = 20, offset: tuple = (0, 0),
                  color: str = "#11111b"):
        """Apply a subtle drop shadow to a widget."""
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(blur)
        shadow.setXOffset(offset[0])
        shadow.setYOffset(offset[1])
        shadow.setColor(QColor(color))
        widget.setGraphicsEffect(shadow)
