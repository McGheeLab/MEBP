import sys, csv, time, uuid, threading
import numpy as np
from scipy.interpolate import interp1d, CubicSpline
import matplotlib.pyplot as plt  # if you later wish to embed plots
from PySide6.QtWidgets import (
    QApplication, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QPushButton, QGroupBox, QFormLayout,
    QTextEdit, QFrame
)
from PySide6.QtCore import Qt, QTimer, QPointF, QSizeF
from PySide6.QtGui import QColor, QPainter, QPen, QBrush

###############################################################################
#  UI Classes                                                                #

class ControlTab(QWidget):
    def __init__(self, printmanager, parent=None):
        super().__init__(parent)
        self.printmanager = printmanager
        self.initUI()
    
    def initUI(self):
        layout = QHBoxLayout(self)
        # Left Column: List of PrintFiles (from well_queue)
        left_layout = QVBoxLayout()
        left_label = QLabel("Print Files")
        self.print_list = QListWidget()
        self.print_list.itemSelectionChanged.connect(self.on_print_selected)
        left_layout.addWidget(left_label)
        left_layout.addWidget(self.print_list)
        layout.addLayout(left_layout)
        
        # Middle Column: Syringes and Ink Wells
        middle_layout = QVBoxLayout()
        self.syringe_group = QGroupBox("Syringes")
        self.syringe_form = QFormLayout(self.syringe_group)
        self.syringe_labels = {}
        for key in ["p1", "p2", "p3"]:
            label = QLabel("Volume: N/A")
            label.setAutoFillBackground(True)
            self.syringe_form.addRow(f"Syringe {key.upper()}:", label)
            self.syringe_labels[key] = label
        middle_layout.addWidget(self.syringe_group)
        
        self.inkwell_group = QGroupBox("Ink Wells")
        self.inkwell_form = QFormLayout(self.inkwell_group)
        self.inkwell_list = QListWidget()
        self.inkwell_form.addRow(self.inkwell_list)
        middle_layout.addWidget(self.inkwell_group)
        layout.addLayout(middle_layout)
        
        # Right Column: Next action and control buttons
        right_layout = QVBoxLayout()
        self.next_action_label = QLabel("Next Action: Idle")
        right_layout.addWidget(self.next_action_label)
        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start")
        self.pause_btn = QPushButton("Pause")
        self.stop_btn = QPushButton("Stop")
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.pause_btn)
        btn_layout.addWidget(self.stop_btn)
        right_layout.addLayout(btn_layout)
        layout.addLayout(right_layout)
        
        # Connect control buttons:
        self.start_btn.clicked.connect(self.start_print)
        self.pause_btn.clicked.connect(self.toggle_pause)
        self.stop_btn.clicked.connect(lambda: self.printmanager.handle_control_print("stop"))
    
    def start_print(self):
        # The start button now calls process_print_queue and is disabled after being pressed.
        self.start_btn.setDisabled(True)
        threading.Thread(target=self.printmanager.process_print_queue, daemon=True).start()
    
    def toggle_pause(self):
        # Toggle between pause and resume.
        if self.printmanager.print_status == "paused":
            self.printmanager.handle_control_print("resume")
            self.pause_btn.setText("Pause")
        elif self.printmanager.print_status == "started":
            self.printmanager.handle_control_print("pause")
            self.pause_btn.setText("Resume")
    
    def on_print_selected(self):
        selected_items = self.print_list.selectedItems()
        if selected_items:
            self.current_print_uid = selected_items[0].data(Qt.UserRole)
        else:
            self.current_print_uid = None
        if hasattr(self, 'print_selected_callback'):
            self.print_selected_callback(self.current_print_uid)
    
    def updateUI(self):
        # Update print file list from printmanager.well_queue.
        self.print_list.clear()
        for pf in self.printmanager.well_queue:
            item = QListWidgetItem(pf.name)
            item.setData(Qt.UserRole, pf.uid)
            item.setBackground(QColor(pf.color))
            self.print_list.addItem(item)
        # Update syringe info from printmanager.syringes.
        for key, syringe in self.printmanager.syringes.items():
            label = self.syringe_labels.get(key)
            if label:
                volume_text = f"Volume: {syringe.current_volume if syringe.current_volume is not None else 'N/A'} / {syringe.max_volume if syringe.max_volume is not None else 'N/A'}"
                label.setText(volume_text)
                label.setStyleSheet(f"background-color: {syringe.color};")
        # Update inkwell list from printmanager.ink_wells.
        self.inkwell_list.clear()
        for key, inkwell in self.printmanager.ink_wells.items():
            text = f"{inkwell.well_location} - {inkwell.cell_type} - Volume: {inkwell.volume}"
            item = QListWidgetItem(text)
            item.setBackground(QColor(inkwell.color))
            self.inkwell_list.addItem(item)
        self.next_action_label.setText(f"Next Action: {self.printmanager.print_status}")

class WaypointsViewsTab(QWidget):
    def __init__(self, printmanager, parent=None):
        super().__init__(parent)
        self.printmanager = printmanager
        self.initUI()
    
    def initUI(self):
        self.main_layout = QVBoxLayout(self)
        header_layout = QHBoxLayout()
        self.total_time_label = QLabel("Total Print Time: N/A")
        self.elapsed_time_label = QLabel("Elapsed Time: N/A")
        self.volume_stats_label = QLabel("Volume Stats: N/A")
        self.status_label = QLabel("Status: N/A")
        header_layout.addWidget(self.total_time_label)
        header_layout.addWidget(self.elapsed_time_label)
        header_layout.addWidget(self.volume_stats_label)
        header_layout.addWidget(self.status_label)
        self.main_layout.addLayout(header_layout)
        self.waypoint_view = QTextEdit()
        self.waypoint_view.setReadOnly(True)
        self.main_layout.addWidget(QLabel("Waypoint File (Realtime)"))
        self.main_layout.addWidget(self.waypoint_view)
        views_layout = QHBoxLayout()
        self.top_view_widget = TopViewWidget()
        self.side_view_widget = SideViewWidget()
        self.iso_view_widget = IsoViewWidget()
        views_layout.addWidget(self.top_view_widget)
        views_layout.addWidget(self.side_view_widget)
        views_layout.addWidget(self.iso_view_widget)
        self.main_layout.addLayout(views_layout)
    
    def updateUI(self):
        pf = self.printmanager.activeprint
        status = self.printmanager.print_status
        self.status_label.setText(f"Status: {status}")
        if pf is None:
            self.waypoint_view.setPlainText("No active print file.")
            self.top_view_widget.pf = None
            self.top_view_widget.update()
            self.side_view_widget.pf = None
            self.side_view_widget.update()
            self.iso_view_widget.pf = None
            self.iso_view_widget.update()
            self.total_time_label.setText("Total Print Time: N/A")
            self.elapsed_time_label.setText("Elapsed Time: N/A")
            self.volume_stats_label.setText("Volume Stats: N/A")
            return

        if status == "started":
            if pf.print_start_time is None:
                pf.print_start_time = time.time()
            elapsed_time = time.time() - pf.print_start_time
        else:
            elapsed_time = 0

        total_print_time = pf.waypoints[-1]['t'] if pf.waypoints else 0
        num_wp = len(pf.waypoints)
        current_index = 0
        if total_print_time > 0 and status == "started":
            current_index = int((elapsed_time / total_print_time) * (num_wp - 1))
            current_index = min(max(current_index, 0), num_wp - 1)

        self.total_time_label.setText(f"Total Print Time: {total_print_time:.1f} sec")
        self.elapsed_time_label.setText(f"Elapsed Time: {elapsed_time:.1f} sec")
        stats = []
        for key, syringe in self.printmanager.syringes.items():
            curr = syringe.current_volume if syringe.current_volume is not None else 0
            maxi = syringe.max_volume if syringe.max_volume is not None else 0
            stats.append(f"{key.upper()}: {curr}/{maxi}")
        self.volume_stats_label.setText("Volume Stats: " + " | ".join(stats))

        text_lines = []
        for i, wp in enumerate(pf.waypoints):
            color = "black"
            if i < current_index:
                color = "green"
            elif i == current_index:
                color = "red"
            text_lines.append(f'<span style="color: {color};">Waypoint {i+1}: x={wp["x"]}, y={wp["y"]}, z={wp["z"]}</span>')
        self.waypoint_view.setHtml("<br>".join(text_lines))

        self.top_view_widget.pf = pf
        self.top_view_widget.current_waypoint_index = current_index
        self.top_view_widget.update()
        self.side_view_widget.pf = pf
        self.side_view_widget.current_waypoint_index = current_index
        self.side_view_widget.update()
        self.iso_view_widget.pf = pf
        self.iso_view_widget.current_waypoint_index = current_index
        self.iso_view_widget.update()

class PrintMonitorWidget(QWidget):
    def __init__(self, printmanager, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Print Monitor")
        self.resize(900, 600)
        self.printmanager = printmanager
        self.initUI()
        self.setupTimer()
    
    def initUI(self):
        main_layout = QVBoxLayout(self)
        self.tab_widget = QTabWidget(self)
        main_layout.addWidget(self.tab_widget)
        self.control_tab = ControlTab(self.printmanager, self)
        self.waypoints_views_tab = WaypointsViewsTab(self.printmanager, self)
        self.control_tab.print_selected_callback = self.on_print_selected
        self.tab_widget.addTab(self.control_tab, "Control")
        self.tab_widget.addTab(self.waypoints_views_tab, "Waypoints & Views")
    
    def on_print_selected(self, uid):
        # This callback can be used to inform other tabs if needed.
        pass
    
    def setupTimer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateUI)
        self.timer.start(500)
    
    def updateUI(self):
        self.control_tab.updateUI()
        self.waypoints_views_tab.updateUI()

class TopViewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.pf = None
        self.current_waypoint_index = 0
        self.setMinimumHeight(250)
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        margin = 10
        if self.pf is None or not self.pf.waypoints:
            painter.drawText(self.rect(), Qt.AlignCenter, "No Data")
            return
        xs = [wp['x'] for wp in self.pf.waypoints]
        ys = [wp['y'] for wp in self.pf.waypoints]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        width = self.width() - 2*margin
        height = self.height() - 2*margin
        dx = max_x - min_x if max_x - min_x != 0 else 1
        dy = max_y - min_y if max_y - min_y != 0 else 1
        scale = min(width/dx, height/dy)
        def transform(x, y):
            tx = margin + (x - min_x) * scale
            ty = self.height() - margin - (y - min_y) * scale
            return QPointF(tx, ty)
        pen = QPen(Qt.black, 2)
        painter.setPen(pen)
        points = [transform(x, y) for x, y in zip(xs, ys)]
        for i in range(len(points)-1):
            painter.drawLine(points[i], points[i+1])
        radius = 5
        for i, pt in enumerate(points):
            if i < self.current_waypoint_index:
                color = QColor("green")
            elif i == self.current_waypoint_index:
                color = QColor("red")
            else:
                color = QColor("black")
            painter.setBrush(QBrush(color))
            painter.drawEllipse(pt, radius, radius)

class SideViewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.pf = None
        self.current_waypoint_index = 0
        self.setMinimumHeight(250)
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        margin = 10
        if self.pf is None or not self.pf.waypoints:
            painter.drawText(self.rect(), Qt.AlignCenter, "No Data")
            return
        xs = [wp['x'] for wp in self.pf.waypoints]
        zs = [wp['z'] for wp in self.pf.waypoints]
        min_x, max_x = min(xs), max(xs)
        min_z, max_z = min(zs), max(zs)
        width = self.width() - 2*margin
        height = self.height() - 2*margin
        dx = max_x - min_x if max_x - min_x != 0 else 1
        dz = max_z - min_z if max_z - min_z != 0 else 1
        scale = min(width/dx, height/dz)
        def transform(x, z):
            tx = margin + (x - min_x) * scale
            tz = self.height() - margin - (z - min_z) * scale
            return QPointF(tx, tz)
        pen = QPen(Qt.black, 2)
        painter.setPen(pen)
        points = [transform(x, z) for x, z in zip(xs, zs)]
        for i in range(len(points)-1):
            painter.drawLine(points[i], points[i+1])
        radius = 5
        for i, pt in enumerate(points):
            if i < self.current_waypoint_index:
                color = QColor("green")
            elif i == self.current_waypoint_index:
                color = QColor("red")
            else:
                color = QColor("black")
            painter.setBrush(QBrush(color))
            painter.drawEllipse(pt, radius, radius)

class IsoViewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.pf = None
        self.current_waypoint_index = 0
        self.setMinimumHeight(250)
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        margin = 10
        if self.pf is None or not self.pf.waypoints:
            painter.drawText(self.rect(), Qt.AlignCenter, "No Data")
            return
        pts = []
        for wp in self.pf.waypoints:
            iso_x = wp['x'] - wp['y']
            iso_y = (wp['x'] + wp['y'])/2 - wp['z']
            pts.append((iso_x, iso_y))
        xs = [pt[0] for pt in pts]
        ys = [pt[1] for pt in pts]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        width = self.width() - 2*margin
        height = self.height() - 2*margin
        dx = max_x - min_x if max_x - min_x != 0 else 1
        dy = max_y - min_y if max_y - min_y != 0 else 1
        scale = min(width/dx, height/dy)
        def transform(x, y):
            tx = margin + (x - min_x) * scale
            ty = self.height() - margin - (y - min_y) * scale
            return QPointF(tx, ty)
        pen = QPen(Qt.black, 2)
        painter.setPen(pen)
        points = [transform(x, y) for x, y in pts]
        for i in range(len(points)-1):
            painter.drawLine(points[i], points[i+1])
        radius = 5
        for i, pt in enumerate(points):
            if i < self.current_waypoint_index:
                color = QColor("green")
            elif i == self.current_waypoint_index:
                color = QColor("red")
            else:
                color = QColor("black")
            painter.setBrush(QBrush(color))
            painter.drawEllipse(pt, radius, radius)
