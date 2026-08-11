"""
camera_popout_dialog.py — detach a live camera view into its own window.

v7.15, operator: *"I need the ability to pop out any live view to make it
larger if needed, all mouse clicks will still land where they should to select
locations etc."*

The view itself MOVES into the window (their choice over opening a second
independent view), and the page shows a placeholder until it comes back. Clicks
keep working because the view's own geometry is rebuilt on every render — see
``gui/widgets/view_geometry.py`` — so a bigger window simply means a bigger
pixmap and a different scale, not a different mapping.

REPARENTING HAZARDS, each handled explicitly rather than hoped about:

* ``setParent(None)`` is NEVER used. ``calibration.py`` records that it kills
  the display of a running camera; ``addWidget`` reparents without it.
* ``CameraFeedView.closeEvent`` calls ``_disconnect_camera()``. Closing this
  window would otherwise tear down the feed the page is about to get back, so
  the view is returned to its slot BEFORE the dialog is destroyed.
* ``eventFilter`` refuses to emit ``clicked`` while ``_last_pixmap`` is None,
  and ``_on_frame`` early-returns while the view is not visible — so the view
  is re-rendered on both legs of the trip instead of waiting for a frame that
  may be seconds away on a long exposure.
* The view's cached camera-settings dialog is parented to ``self.window()``;
  it is dropped across the move so it cannot outlive the wrong window.
* The page is never hidden. Several pages stop their camera in ``hideEvent``,
  and the needle-bore wizard disarms the print floor there — a pop-out that
  worked by hiding its host would be a safety change, not a layout one.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDialog, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget)

from gui.styles import COLORS
from gui.scaling import s

logger = logging.getLogger(__name__)


class _Placeholder(QWidget):
    """Stands in for the detached view, and brings it back when clicked."""

    def __init__(self, on_return, parent=None):
        super().__init__(parent)
        self._on_return = on_return
        lay = QVBoxLayout(self)
        lay.setContentsMargins(s(8), s(8), s(8), s(8))
        lbl = QLabel("This view is open in its own window.")
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setWordWrap(True)
        lbl.setStyleSheet(f"color: {COLORS['subtext0']};")
        lay.addStretch()
        lay.addWidget(lbl)
        btn = QPushButton("Bring it back")
        btn.setCursor(Qt.PointingHandCursor)
        btn.clicked.connect(lambda: self._on_return())
        row = QHBoxLayout()
        row.addStretch()
        row.addWidget(btn)
        row.addStretch()
        lay.addLayout(row)
        lay.addStretch()
        self.setStyleSheet(
            f"background: {COLORS['mantle']};"
            f"border: 1px dashed {COLORS['surface1']}; border-radius: 6px;")


class CameraPopoutDialog(QDialog):
    """Hosts a detached :class:`CameraFeedView` until it is closed."""

    def __init__(self, title="Camera", parent=None):
        super().__init__(parent)
        self._view = None
        self.setWindowTitle(f"{title} — popped out")
        self.setSizeGripEnabled(True)
        # A real window (minimise/maximise), not a fixed utility dialog.
        self.setWindowFlags(Qt.Window)
        self.resize(s(900), s(700))

        self._lay = QVBoxLayout(self)
        self._lay.setContentsMargins(0, 0, 0, 0)

        self._origin = None          # (layout, index, stretch)
        self._placeholder = None

    # ── Detach / restore ──────────────────────────────────────────

    def take(self, view) -> bool:
        """Move ``view`` out of its layout, leaving a placeholder.

        ⚠ The origin is captured BEFORE the view is reparented. Adding it to
        this dialog first makes ``indexOf`` report a slot in the DIALOG's
        layout, and the placeholder then lands in the wrong window entirely.
        """
        parent = view.parentWidget()
        layout = parent.layout() if parent is not None else None
        if layout is None:
            return False
        idx = layout.indexOf(view)
        if idx < 0:
            return False
        stretch = 0
        try:
            stretch = layout.stretch(idx)
        except (AttributeError, TypeError):
            pass                       # not a box layout — restore without it
        self._origin = (layout, idx, stretch)

        # Reparent by adding to this layout — never setParent(None), which
        # kills the display of a running camera (calibration.py records it).
        self._lay.addWidget(view)
        self._view = view

        self._placeholder = _Placeholder(self.close, parent)
        try:
            layout.insertWidget(idx, self._placeholder, stretch)
        except (AttributeError, TypeError):
            layout.addWidget(self._placeholder)
        return True

    def _restore(self):
        """Put the view back before this dialog is destroyed."""
        view, self._view = self._view, None
        if view is None or self._origin is None:
            return
        layout, idx, stretch = self._origin
        self._origin = None
        try:
            if self._placeholder is not None:
                # hide() + deleteLater(), never setParent(None) — see the
                # module docstring; removeWidget alone leaves it on screen
                # until the event loop collects it.
                layout.removeWidget(self._placeholder)
                self._placeholder.hide()
                self._placeholder.deleteLater()
                self._placeholder = None
            try:
                layout.insertWidget(idx, view, stretch)
            except (AttributeError, TypeError):
                layout.addWidget(view)
            view.show()
            view.on_returned_from_popout()
        except Exception as exc:      # never strand the view in a dead dialog
            logger.warning(f"camera view could not be restored: {exc}")

    # ── Lifecycle ─────────────────────────────────────────────────

    def done(self, result):
        # BEFORE super(): once the dialog is finished its children are on the
        # way out, and CameraFeedView.closeEvent disconnects the camera.
        self._restore()
        super().done(result)

    def closeEvent(self, event):
        self._restore()
        super().closeEvent(event)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_F11:
            self.setWindowState(self.windowState() ^ Qt.WindowFullScreen)
            event.accept()
            return
        if event.key() == Qt.Key_Escape and self.isFullScreen():
            self.setWindowState(self.windowState() & ~Qt.WindowFullScreen)
            event.accept()
            return
        super().keyPressEvent(event)
