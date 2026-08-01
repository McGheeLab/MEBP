"""
microscope_settings_dialog.py — the ⚙ pop-out on the jog panel's Microscope card.

A thin wrapper around :class:`MicroscopeSetupPanel`, which is the **one**
microscope setup surface (also hosted by Hardware Setup → Microscope). Keeping
the dialog as a wrapper rather than a second implementation is deliberate: two
independently written surfaces editing the same settings is the failure recorded
in ``MEBP_v75x_UNIFIED_MOSAIC_CALIBRATION.md``.

Nothing is written to the store until OK — Cancel leaves the previous
configuration untouched, including the driver selection.
"""

from __future__ import annotations

from PySide6.QtWidgets import (
    QDialog, QDialogButtonBox, QScrollArea, QVBoxLayout,
)

from gui.pages.hardware.microscope_setup_panel import (
    BACKEND_OPTIONS, MicroscopeSetupPanel,
)
from gui.scaling import s

__all__ = ["MicroscopeSettingsDialog", "BACKEND_OPTIONS"]


class MicroscopeSettingsDialog(QDialog):
    """Modal wrapper: OK commits the panel, Cancel discards."""

    def __init__(self, store, parent=None, *, controller=None):
        super().__init__(parent)
        self._store = store
        self.setWindowTitle("Microscope Setup")
        self.setMinimumSize(s(620), s(560))

        root = QVBoxLayout(self)
        root.setContentsMargins(s(12), s(12), s(12), s(12))
        root.setSpacing(s(10))

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        # show_save=False: this dialog's OK button is the commit.
        self.panel = MicroscopeSetupPanel(
            store, controller=controller, show_save=False)
        scroll.setWidget(self.panel)
        root.addWidget(scroll, stretch=1)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        root.addWidget(btns)

    def accept(self) -> None:  # noqa: D102 (Qt override)
        if self.panel.commit():
            super().accept()
        # commit() already explained the refusal; keep the dialog open.
