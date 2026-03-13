"""
pp_operation_queue.py — Pick & Place Operation Queue page.

v7.3.3: Users define pick-and-place operations from marked targets,
configure operation parameters (spheroid/trypsin/fluorescent), and
arrange the execution order.

Layout:
    ┌────────────────────────────────────────────────────┐
    │ Mode: [● Spheroid] [○ Trypsin] [○ Fluorescent]    │
    │ ┌────────────────────────────────────────────────┐ │
    │ │ Mode-specific config panel                     │ │
    │ └────────────────────────────────────────────────┘ │
    │ ┌────────────────────────────────────────────────┐ │
    │ │ Operation Queue list                           │ │
    │ │ [Add] [Remove] [Move Up] [Move Down] [Clear]   │ │
    │ └────────────────────────────────────────────────┘ │
    │ [Send to Execution ▶]                              │
    └────────────────────────────────────────────────────┘
"""

from __future__ import annotations

import logging
from typing import Optional

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QListWidget, QListWidgetItem,
    QComboBox, QDoubleSpinBox, QSpinBox, QCheckBox,
    QRadioButton, QButtonGroup, QFrame, QSizePolicy,
    QStackedWidget, QScrollArea, QMessageBox,
)
from PySide6.QtCore import Qt, Signal

from gui.styles import COLORS
from SupportClasses.PickAndPlaceManager import (
    PickPlaceTarget, PickPlaceOperation, OperationType, OperationStatus,
    OperationQueue, SpheroidPickupConfig, TrypsinPickupConfig,
    FluorescentTaggingConfig, DyeConfig,
)

logger = logging.getLogger(__name__)


class PPOperationQueuePage(QWidget):
    """Pick & Place Operation Queue — define and arrange operations."""

    queue_ready = Signal(object)  # Emits OperationQueue for execution

    def __init__(self, parent=None):
        super().__init__(parent)

        self._targets: list[PickPlaceTarget] = []
        self._queue = OperationQueue()
        self._hardware_config = None

        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # Scrollable content
        _bg = COLORS['base']
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet(f"QScrollArea {{ background-color: {_bg}; border: none; }}")

        content = QWidget()
        content.setStyleSheet(f"background-color: {_bg};")
        content_layout = QVBoxLayout(content)
        content_layout.setSpacing(12)

        # ── Mode selector ────────────────────────────────────────
        mode_group = QGroupBox("Operation Mode")
        mode_group.setStyleSheet(self._group_style())
        mode_layout = QHBoxLayout(mode_group)

        self._rb_spheroid = QRadioButton("Spheroid Pickup")
        self._rb_trypsin = QRadioButton("Trypsin Cell Pickup")
        self._rb_fluorescent = QRadioButton("Fluorescent Tagging")
        self._rb_spheroid.setChecked(True)

        self._mode_group = QButtonGroup(self)
        self._mode_group.addButton(self._rb_spheroid, 0)
        self._mode_group.addButton(self._rb_trypsin, 1)
        self._mode_group.addButton(self._rb_fluorescent, 2)
        self._mode_group.idToggled.connect(self._on_mode_changed)

        mode_layout.addWidget(self._rb_spheroid)
        mode_layout.addWidget(self._rb_trypsin)
        mode_layout.addWidget(self._rb_fluorescent)
        mode_layout.addStretch()
        content_layout.addWidget(mode_group)

        # ── Config panels (stacked) ──────────────────────────────
        self._config_stack = QStackedWidget()
        self._spheroid_panel = self._build_spheroid_config()
        self._trypsin_panel = self._build_trypsin_config()
        self._fluorescent_panel = self._build_fluorescent_config()
        self._config_stack.addWidget(self._spheroid_panel)
        self._config_stack.addWidget(self._trypsin_panel)
        self._config_stack.addWidget(self._fluorescent_panel)
        content_layout.addWidget(self._config_stack)

        # ── Operation Queue ──────────────────────────────────────
        queue_group = QGroupBox("Operation Queue")
        queue_group.setStyleSheet(self._group_style())
        queue_layout = QVBoxLayout(queue_group)

        self._queue_list = QListWidget()
        self._queue_list.setStyleSheet(f"""
            QListWidget {{
                background-color: {COLORS['surface0']};
                border: 1px solid {COLORS['surface1']};
                border-radius: 4px;
                color: {COLORS['text']};
            }}
            QListWidget::item {{
                padding: 6px;
                border-bottom: 1px solid {COLORS['surface1']};
            }}
            QListWidget::item:selected {{
                background-color: {COLORS['surface1']};
            }}
        """)
        self._queue_list.setMinimumHeight(200)
        queue_layout.addWidget(self._queue_list)

        # Queue buttons
        btn_row = QHBoxLayout()

        self._btn_add = QPushButton("Add from Targets")
        self._btn_add.setToolTip("Create operations for all selected targets")
        self._btn_add.clicked.connect(self._add_operations_from_targets)
        btn_row.addWidget(self._btn_add)

        self._btn_remove = QPushButton("Remove")
        self._btn_remove.clicked.connect(self._remove_selected_op)
        btn_row.addWidget(self._btn_remove)

        self._btn_up = QPushButton("Up")
        self._btn_up.clicked.connect(self._move_op_up)
        btn_row.addWidget(self._btn_up)

        self._btn_down = QPushButton("Down")
        self._btn_down.clicked.connect(self._move_op_down)
        btn_row.addWidget(self._btn_down)

        self._btn_clear_queue = QPushButton("Clear")
        self._btn_clear_queue.clicked.connect(self._clear_queue)
        btn_row.addWidget(self._btn_clear_queue)

        queue_layout.addLayout(btn_row)
        content_layout.addWidget(queue_group)

        # ── Destination well selector ────────────────────────────
        dest_group = QGroupBox("Destination")
        dest_group.setStyleSheet(self._group_style())
        dest_layout = QHBoxLayout(dest_group)
        dest_layout.addWidget(QLabel("Dest well:"))
        self._dest_well_combo = QComboBox()
        self._dest_well_combo.setPlaceholderText("Select destination well")
        self._dest_well_combo.setMinimumWidth(120)
        dest_layout.addWidget(self._dest_well_combo)
        dest_layout.addStretch()
        content_layout.addWidget(dest_group)

        content_layout.addStretch()

        scroll.setWidget(content)
        layout.addWidget(scroll, 1)

        # ── Send to execution ────────────────────────────────────
        send_bar = QFrame()
        send_bar.setStyleSheet(f"""
            QFrame {{
                background-color: {COLORS['surface0']};
                border: 1px solid {COLORS['surface1']};
                border-radius: 6px;
            }}
        """)
        send_layout = QHBoxLayout(send_bar)
        send_layout.setContentsMargins(12, 8, 12, 8)

        self._lbl_queue_count = QLabel("Queue: 0 operations")
        self._lbl_queue_count.setStyleSheet(f"color: {COLORS['subtext0']};")
        send_layout.addWidget(self._lbl_queue_count)
        send_layout.addStretch()

        self._btn_send = QPushButton("Send to Execution  ▶")
        self._btn_send.setStyleSheet(f"""
            QPushButton {{
                background-color: {COLORS['green']};
                color: {COLORS['crust']};
                font-weight: bold;
                padding: 8px 24px;
                border-radius: 6px;
                font-size: 11pt;
            }}
            QPushButton:hover {{
                background-color: {COLORS['blue']};
            }}
        """)
        self._btn_send.clicked.connect(self._send_to_execution)
        send_layout.addWidget(self._btn_send)

        layout.addWidget(send_bar)

    # ── Config panels ────────────────────────────────────────────

    def _build_spheroid_config(self) -> QWidget:
        """Build spheroid pickup config panel."""
        panel = QGroupBox("Spheroid Pickup Settings")
        panel.setStyleSheet(self._group_style())
        lay = QVBoxLayout(panel)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Diameter (µm):"))
        self._sp_diameter = QDoubleSpinBox()
        self._sp_diameter.setRange(10, 5000)
        self._sp_diameter.setValue(200)
        self._sp_diameter.setSingleStep(10)
        row1.addWidget(self._sp_diameter)

        row1.addWidget(QLabel("Safety factor:"))
        self._sp_safety = QDoubleSpinBox()
        self._sp_safety.setRange(1.0, 10.0)
        self._sp_safety.setValue(1.5)
        self._sp_safety.setSingleStep(0.1)
        row1.addWidget(self._sp_safety)
        lay.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Bore:"))
        self._sp_bore = QComboBox()
        self._sp_bore.addItems(["P1", "P2", "P3"])
        row2.addWidget(self._sp_bore)

        row2.addWidget(QLabel("Pickup speed (µL/s):"))
        self._sp_pickup_speed = QDoubleSpinBox()
        self._sp_pickup_speed.setRange(0.01, 50)
        self._sp_pickup_speed.setValue(1.0)
        row2.addWidget(self._sp_pickup_speed)

        row2.addWidget(QLabel("Release speed (µL/s):"))
        self._sp_release_speed = QDoubleSpinBox()
        self._sp_release_speed.setRange(0.01, 50)
        self._sp_release_speed.setValue(1.0)
        row2.addWidget(self._sp_release_speed)
        lay.addLayout(row2)

        # Volume preview
        self._lbl_volume = QLabel("Computed volume: —")
        self._lbl_volume.setStyleSheet(f"color: {COLORS['peach']};")
        lay.addWidget(self._lbl_volume)

        # Update volume on diameter/safety change
        self._sp_diameter.valueChanged.connect(self._update_volume_preview)
        self._sp_safety.valueChanged.connect(self._update_volume_preview)
        self._update_volume_preview()

        return panel

    def _build_trypsin_config(self) -> QWidget:
        """Build trypsin cell pickup config panel."""
        panel = QGroupBox("Trypsin Cell Pickup Settings")
        panel.setStyleSheet(self._group_style())
        lay = QVBoxLayout(panel)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Trypsin volume (µL):"))
        self._tp_volume = QDoubleSpinBox()
        self._tp_volume.setRange(0.1, 100)
        self._tp_volume.setValue(5.0)
        row1.addWidget(self._tp_volume)

        row1.addWidget(QLabel("Dwell time (s):"))
        self._tp_dwell = QDoubleSpinBox()
        self._tp_dwell.setRange(1, 3600)
        self._tp_dwell.setValue(120)
        row1.addWidget(self._tp_dwell)
        lay.addLayout(row1)

        row2 = QHBoxLayout()
        self._tp_single = QCheckBox("Single bore mode")
        self._tp_single.setChecked(True)
        self._tp_single.toggled.connect(self._on_trypsin_mode_changed)
        row2.addWidget(self._tp_single)

        row2.addWidget(QLabel("Trypsin bore:"))
        self._tp_tryp_bore = QComboBox()
        self._tp_tryp_bore.addItems(["P1", "P2", "P3"])
        row2.addWidget(self._tp_tryp_bore)

        row2.addWidget(QLabel("Extraction bore:"))
        self._tp_ext_bore = QComboBox()
        self._tp_ext_bore.addItems(["P1", "P2", "P3"])
        self._tp_ext_bore.setEnabled(False)
        row2.addWidget(self._tp_ext_bore)
        lay.addLayout(row2)

        row3 = QHBoxLayout()
        row3.addWidget(QLabel("Extraction volume (µL):"))
        self._tp_ext_vol = QDoubleSpinBox()
        self._tp_ext_vol.setRange(0.1, 200)
        self._tp_ext_vol.setValue(6.0)
        row3.addWidget(self._tp_ext_vol)

        row3.addWidget(QLabel("Trypsin well:"))
        self._tp_tryp_well = QComboBox()
        self._tp_tryp_well.setPlaceholderText("Source well")
        row3.addWidget(self._tp_tryp_well)
        lay.addLayout(row3)

        return panel

    def _build_fluorescent_config(self) -> QWidget:
        """Build fluorescent tagging config panel."""
        panel = QGroupBox("Fluorescent Tagging Settings")
        panel.setStyleSheet(self._group_style())
        lay = QVBoxLayout(panel)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Number of bores:"))
        self._fl_num_bores = QSpinBox()
        self._fl_num_bores.setRange(1, 3)
        self._fl_num_bores.setValue(1)
        row1.addWidget(self._fl_num_bores)

        row1.addWidget(QLabel("Dwell time (s):"))
        self._fl_dwell = QDoubleSpinBox()
        self._fl_dwell.setRange(1, 7200)
        self._fl_dwell.setValue(300)
        row1.addWidget(self._fl_dwell)
        lay.addLayout(row1)

        # Waste bore mode
        row2 = QHBoxLayout()
        self._fl_waste_mode = QCheckBox("Use waste bore (skip waste well travel)")
        row2.addWidget(self._fl_waste_mode)
        row2.addWidget(QLabel("Waste bore:"))
        self._fl_waste_bore = QComboBox()
        self._fl_waste_bore.addItems(["P1", "P2", "P3"])
        self._fl_waste_bore.setEnabled(False)
        self._fl_waste_mode.toggled.connect(self._fl_waste_bore.setEnabled)
        row2.addWidget(self._fl_waste_bore)
        lay.addLayout(row2)

        # Dye configuration (simplified — 3 rows, one per bore)
        for i in range(3):
            bore_row = QHBoxLayout()
            bore_row.addWidget(QLabel(f"Bore {i+1}:"))

            bore_combo = QComboBox()
            bore_combo.addItems(["P1", "P2", "P3"])
            bore_combo.setCurrentIndex(i)
            bore_row.addWidget(bore_combo)

            bore_row.addWidget(QLabel("Dye:"))
            dye_name = QComboBox()
            dye_name.setEditable(True)
            dye_name.addItems(["DAPI", "GFP", "mCherry", "Cy5", "Custom"])
            bore_row.addWidget(dye_name)

            bore_row.addWidget(QLabel("Well:"))
            dye_well = QComboBox()
            dye_well.setPlaceholderText("Dye well")
            bore_row.addWidget(dye_well)

            bore_row.addWidget(QLabel("Vol (µL):"))
            dye_vol = QDoubleSpinBox()
            dye_vol.setRange(0.01, 100)
            dye_vol.setValue(2.0)
            bore_row.addWidget(dye_vol)

            lay.addLayout(bore_row)
            setattr(self, f"_fl_bore_{i}", bore_combo)
            setattr(self, f"_fl_dye_{i}", dye_name)
            setattr(self, f"_fl_well_{i}", dye_well)
            setattr(self, f"_fl_vol_{i}", dye_vol)

        # Service wells
        service_row = QHBoxLayout()
        for label, attr in [("Waste:", "_fl_waste_well"),
                            ("Buffer:", "_fl_buffer_well"),
                            ("Wash:", "_fl_wash_well")]:
            service_row.addWidget(QLabel(label))
            combo = QComboBox()
            combo.setPlaceholderText("Well")
            service_row.addWidget(combo)
            setattr(self, attr, combo)
        lay.addLayout(service_row)

        return panel

    # ── Mode switching ───────────────────────────────────────────

    def _on_mode_changed(self, button_id: int, checked: bool):
        if checked:
            self._config_stack.setCurrentIndex(button_id)

    def _on_trypsin_mode_changed(self, single: bool):
        self._tp_ext_bore.setEnabled(not single)

    def _get_current_mode(self) -> OperationType:
        idx = self._mode_group.checkedId()
        return [OperationType.SPHEROID_PICKUP,
                OperationType.TRYPSIN_CELL_PICKUP,
                OperationType.FLUORESCENT_TAGGING][idx]

    # ── Volume preview ───────────────────────────────────────────

    def _update_volume_preview(self):
        cfg = SpheroidPickupConfig(
            spheroid_diameter_um=self._sp_diameter.value(),
            safety_factor=self._sp_safety.value(),
        )
        vol = cfg.compute_volume_uL()
        self._lbl_volume.setText(f"Computed volume: {vol:.4f} µL "
                                 f"({vol * 1000:.2f} nL)")

    # ── Config builders ──────────────────────────────────────────

    def _build_spheroid_config_obj(self) -> SpheroidPickupConfig:
        return SpheroidPickupConfig(
            spheroid_diameter_um=self._sp_diameter.value(),
            safety_factor=self._sp_safety.value(),
            pickup_bore=self._sp_bore.currentText(),
            pickup_speed_uL_s=self._sp_pickup_speed.value(),
            release_speed_uL_s=self._sp_release_speed.value(),
        )

    def _build_trypsin_config_obj(self) -> TrypsinPickupConfig:
        return TrypsinPickupConfig(
            trypsin_volume_uL=self._tp_volume.value(),
            dwell_time_s=self._tp_dwell.value(),
            single_bore=self._tp_single.isChecked(),
            trypsin_bore=self._tp_tryp_bore.currentText(),
            extraction_bore=self._tp_ext_bore.currentText(),
            extraction_volume_uL=self._tp_ext_vol.value(),
            trypsin_well=self._tp_tryp_well.currentText(),
            dest_well=self._dest_well_combo.currentText(),
        )

    def _build_fluorescent_config_obj(self) -> FluorescentTaggingConfig:
        num = self._fl_num_bores.value()
        dyes = []
        for i in range(num):
            dyes.append(DyeConfig(
                bore=getattr(self, f"_fl_bore_{i}").currentText(),
                dye_well=getattr(self, f"_fl_well_{i}").currentText(),
                dye_name=getattr(self, f"_fl_dye_{i}").currentText(),
                volume_uL=getattr(self, f"_fl_vol_{i}").value(),
            ))
        return FluorescentTaggingConfig(
            num_bores=num,
            dye_configs=dyes,
            dwell_time_s=self._fl_dwell.value(),
            waste_bore=self._fl_waste_bore.currentText() if self._fl_waste_mode.isChecked() else None,
            use_waste_bore_mode=self._fl_waste_mode.isChecked(),
            waste_well=self._fl_waste_well.currentText(),
            buffer_well=self._fl_buffer_well.currentText(),
            wash_well=self._fl_wash_well.currentText(),
        )

    def _get_config_for_mode(self):
        mode = self._get_current_mode()
        if mode == OperationType.SPHEROID_PICKUP:
            return self._build_spheroid_config_obj()
        elif mode == OperationType.TRYPSIN_CELL_PICKUP:
            return self._build_trypsin_config_obj()
        else:
            return self._build_fluorescent_config_obj()

    # ── Queue management ─────────────────────────────────────────

    def _add_operations_from_targets(self):
        """Create operations for all selected targets."""
        selected = [t for t in self._targets if t.selected]
        if not selected:
            logger.warning("No selected targets to add")
            return

        mode = self._get_current_mode()
        config = self._get_config_for_mode()
        dest_well = self._dest_well_combo.currentText()

        for target in selected:
            # Create destination target (same well center for now)
            dest = None
            if dest_well:
                dest = PickPlaceTarget(
                    target_id=f"DEST_{target.target_id}",
                    x_um=0, y_um=0,  # Will be resolved from well positions
                    well_name=dest_well,
                )

            op = PickPlaceOperation(
                op_id=PickPlaceOperation.make_id(),
                op_type=mode,
                source_target=target,
                dest_target=dest,
                config=config,
            )
            self._queue.add(op)

        self._update_queue_list()
        logger.info(f"Added {len(selected)} {mode.value} operations to queue")

    def _remove_selected_op(self):
        item = self._queue_list.currentItem()
        if item:
            op_id = item.data(Qt.UserRole)
            self._queue.remove(op_id)
            self._update_queue_list()

    def _move_op_up(self):
        row = self._queue_list.currentRow()
        if row > 0:
            item = self._queue_list.currentItem()
            if item:
                op_id = item.data(Qt.UserRole)
                self._queue.reorder(op_id, row - 1)
                self._update_queue_list()
                self._queue_list.setCurrentRow(row - 1)

    def _move_op_down(self):
        row = self._queue_list.currentRow()
        if row < self._queue_list.count() - 1:
            item = self._queue_list.currentItem()
            if item:
                op_id = item.data(Qt.UserRole)
                self._queue.reorder(op_id, row + 1)
                self._update_queue_list()
                self._queue_list.setCurrentRow(row + 1)

    def _clear_queue(self):
        self._queue.clear()
        self._update_queue_list()

    def _update_queue_list(self):
        """Rebuild the queue list widget."""
        self._queue_list.clear()
        for op in self._queue.operations:
            icon = {"spheroid_pickup": "🔵",
                    "trypsin_cell_pickup": "🟢",
                    "fluorescent_tagging": "🟣"}.get(op.op_type.value, "⚪")
            status_icon = {"pending": "○", "running": "▶",
                           "completed": "✓", "failed": "✗",
                           "skipped": "—"}.get(op.status.value, "?")
            src = op.source_target.target_id
            dst = op.dest_target.target_id if op.dest_target else "—"
            label = f"{status_icon} {icon} {op.op_type.value}: {src} → {dst}"
            item = QListWidgetItem(label)
            item.setData(Qt.UserRole, op.op_id)
            self._queue_list.addItem(item)

        self._lbl_queue_count.setText(
            f"Queue: {len(self._queue)} operations")

    def _send_to_execution(self):
        """Emit the queue for execution."""
        if len(self._queue) == 0:
            logger.warning("Cannot send empty queue")
            return
        self.queue_ready.emit(self._queue)
        logger.info(f"Queue sent: {len(self._queue)} operations")

    # ── External interface ───────────────────────────────────────

    def update_available_targets(self, targets: list[PickPlaceTarget]):
        """Update the available targets from the target selection page."""
        self._targets = list(targets)

    def set_well_list(self, wells: list[str]):
        """Update well combo boxes."""
        for combo in [self._dest_well_combo, self._tp_tryp_well]:
            combo.clear()
            combo.addItems(wells)
        for i in range(3):
            combo = getattr(self, f"_fl_well_{i}", None)
            if combo:
                combo.clear()
                combo.addItems(wells)
        if hasattr(self, '_fl_waste_well'):
            for combo in [self._fl_waste_well, self._fl_buffer_well, self._fl_wash_well]:
                combo.clear()
                combo.addItems(wells)

    def set_hardware_config(self, config):
        self._hardware_config = config

    def get_page_title(self) -> str:
        return "Operation Queue"

    def get_context_widget(self) -> None:
        return None  # No context panel — all config is inline

    def on_status_update(self):
        pass  # No real-time updates needed

    # ── Style ────────────────────────────────────────────────────

    def _group_style(self) -> str:
        return f"""
            QGroupBox {{
                border: 1px solid {COLORS['surface1']};
                border-radius: 8px;
                color: {COLORS['text']};
                font-size: 11pt;
                font-weight: 600;
                margin-top: 12px;
                padding-top: 16px;
            }}
            QGroupBox::title {{
                color: {COLORS['blue']};
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 4px;
            }}
        """
