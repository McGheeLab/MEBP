"""
page_transition.py — Smooth page transitions for QStackedWidget (v7.4.0-a).

Provides fade_swap() to softly transition between pages in a stack. Skips the
animation for any destination page containing a raw-QPainter camera widget
(CameraWidget / CameraFeedView / TargetOverlayCameraView), since
QGraphicsOpacityEffect can produce black frames during the fade.
"""

from __future__ import annotations

from PySide6.QtCore import QPropertyAnimation, QEasingCurve
from PySide6.QtWidgets import QStackedWidget, QGraphicsOpacityEffect, QWidget


_CAMERA_WIDGET_CLASS_NAMES = {
    "CameraWidget",
    "CameraFeedView",
    "TargetOverlayCameraView",
}


def _has_camera_widget(page: QWidget) -> bool:
    """Walk children for any known camera widget by class name."""
    if page is None:
        return False
    if type(page).__name__ in _CAMERA_WIDGET_CLASS_NAMES:
        return True
    for child in page.findChildren(QWidget):
        if type(child).__name__ in _CAMERA_WIDGET_CLASS_NAMES:
            return True
    return False


def fade_swap(stack: QStackedWidget, new_index: int, ms: int = 120) -> None:
    """Switch the stack to new_index with a brief opacity fade.

    Falls back to an instant swap if either:
        * the destination page contains a camera widget, or
        * new_index is invalid or already current.
    """
    if stack is None or new_index < 0 or new_index >= stack.count():
        return
    if stack.currentIndex() == new_index:
        return

    target = stack.widget(new_index)
    if target is None or _has_camera_widget(target):
        stack.setCurrentIndex(new_index)
        return

    # Apply opacity effect and animate 0 → 1 as we swap in.
    effect = QGraphicsOpacityEffect(target)
    effect.setOpacity(0.0)
    target.setGraphicsEffect(effect)

    stack.setCurrentIndex(new_index)

    anim = QPropertyAnimation(effect, b"opacity", target)
    anim.setDuration(ms)
    anim.setStartValue(0.0)
    anim.setEndValue(1.0)
    anim.setEasingCurve(QEasingCurve.OutCubic)

    def _cleanup():
        target.setGraphicsEffect(None)

    anim.finished.connect(_cleanup)
    anim.start(QPropertyAnimation.DeleteWhenStopped)
