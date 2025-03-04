import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageQt
from PIL.ImageQt import toqimage

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QComboBox, QFormLayout, QSlider, QFileDialog,
    QStackedWidget
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap

# Use the Qt5Agg backend classes for embedding.
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT as NavigationToolbar

# ------------------------------------------------------------------------------
# Toolpath_From_ImageStack class (unchanged from your version)
class Toolpath_From_ImageStack:
    def __init__(self,
                 threshold=128,
                 tool_diameter=300.0,
                 path_overlap=0.8,
                 pixels_per_micron=0.09,
                 base_feedrate=1000.0,
                 corner_slowdown_factor=0.5,
                 corner_angle_threshold=120.0,
                 flow_factor=0.01,
                 initial_z=0.0,
                 z_increment=10.0):
        self.threshold = threshold
        self.tool_diameter = tool_diameter
        self.path_overlap = path_overlap
        self.pixels_per_micron = pixels_per_micron
        self.base_feedrate = base_feedrate
        self.corner_slowdown_factor = corner_slowdown_factor
        self.corner_angle_threshold = corner_angle_threshold
        self.flow_factor = flow_factor
        self.z_pos = initial_z
        self.z_increment = z_increment

        self.main_images = []
        self.pump_images_layers = []
        self.pump_colors = []
        self.waypoints = []
        self.pump_states_all = []
        
    def load_layer_images(self, main_image_path, pump_image_paths):
        """Load a single layer of images: one main and multiple pump images."""
        main_img = Image.open(main_image_path).convert('L')
        main_pixels = np.array(main_img)
        if len(self.main_images) == 0:
            self.height, self.width = main_pixels.shape
        else:
            if main_pixels.shape != (self.height, self.width):
                raise ValueError("All images must have the same dimensions.")
        
        pump_pixels_list = []
        for p_path in pump_image_paths:
            p_img = Image.open(p_path).convert('L')
            p_pixels = np.array(p_img)
            if p_pixels.shape != (self.height, self.width):
                raise ValueError("Pump image size must match the main image size.")
            pump_pixels_list.append(p_pixels)
        
        self.main_images.append(main_pixels)
        self.pump_images_layers.append(pump_pixels_list)
        
    def set_pump_colors(self, colors):
        self.pump_colors = colors

    def _microns_per_pixel(self):
        return 1.0 / self.pixels_per_micron

    def _distance(self, p1, p2):
        return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

    def _angle_between_vectors(self, v1, v2):
        dot = v1[0]*v2[0] + v1[1]*v2[1]
        mag1 = math.sqrt(v1[0]**2+v1[1]**2)
        mag2 = math.sqrt(v2[0]**2+v2[1]**2)
        if mag1*mag2 == 0:
            return 0.0
        cos_angle = dot/(mag1*mag2)
        cos_angle = max(min(cos_angle,1),-1)
        return math.degrees(math.acos(cos_angle))

    def _get_pump_states_at_pixel(self, pump_images, x_px, y_px):
        x_px = min(max(x_px, 0), self.width-1)
        y_px = min(max(y_px, 0), self.height-1)
        return [p_img[y_px, x_px] <= self.threshold for p_img in pump_images]

    def _get_pump_states_mic(self, pump_images, x_mic, y_mic):
        microns_per_pixel = self._microns_per_pixel()
        x_px = int(round(x_mic / microns_per_pixel))
        y_px = int(round(y_mic / microns_per_pixel))
        return self._get_pump_states_at_pixel(pump_images, x_px, y_px)

    def _generate_layer_toolpath(self, main_pixels, pump_images):
        step_over = self.tool_diameter * self.path_overlap
        pixels_per_pass = max(int(math.ceil(step_over * self.pixels_per_micron)), 1)
        microns_per_pixel = self._microns_per_pixel()

        path_points = []
        direction = 1
        y_px = 0

        while y_px < self.height:
            current_line = main_pixels[y_px, :]
            black_indices = np.where(current_line <= self.threshold)[0]
            if black_indices.size == 0:
                y_px += pixels_per_pass
                direction *= -1
                continue

            segments = []
            start_idx = black_indices[0]
            for i in range(1, len(black_indices)):
                if black_indices[i] > black_indices[i-1] + 1:
                    segments.append((start_idx, black_indices[i-1]))
                    start_idx = black_indices[i]
            segments.append((start_idx, black_indices[-1]))

            if direction == -1:
                segments = segments[::-1]
                segments = [(seg_end, seg_start) for (seg_start, seg_end) in segments]

            current_y_microns = y_px * microns_per_pixel

            for (seg_start_px, seg_end_px) in segments:
                if direction == 1:
                    px_range = range(seg_start_px, seg_end_px + 1)
                else:
                    px_range = range(seg_start_px, seg_end_px - 1, -1)
                px_list = list(px_range)

                prev_states = self._get_pump_states_at_pixel(pump_images, px_list[0], y_px)
                first_x = px_list[0] * microns_per_pixel
                if not path_points or path_points[-1] != (first_x, current_y_microns):
                    path_points.append((first_x, current_y_microns))

                for idx in range(1, len(px_list)):
                    cur_px = px_list[idx]
                    current_states = self._get_pump_states_at_pixel(pump_images, cur_px, y_px)
                    if current_states != prev_states:
                        prev_px = px_list[idx-1]
                        prev_x = prev_px * microns_per_pixel
                        path_points.append((prev_x, current_y_microns))
                        cur_x = cur_px * microns_per_pixel
                        path_points.append((cur_x, current_y_microns))
                        prev_states = current_states

                last_px = px_list[-1]
                last_x = last_px * microns_per_pixel
                if not path_points or path_points[-1] != (last_x, current_y_microns):
                    path_points.append((last_x, current_y_microns))

            y_px += pixels_per_pass
            direction *= -1

        return path_points

    def _compute_waypoints(self, path_points, pump_images):
        if len(path_points) == 0:
            return [], []

        times = [0.0]
        pump_displacements = [0.0 for _ in pump_images]
        waypoints = []
        init_states = self._get_pump_states_mic(pump_images, path_points[0][0], path_points[0][1])
        waypoints.append((path_points[0][0], path_points[0][1], self.z_pos, *pump_displacements, 0.0))
        pump_states = [init_states]

        for i in range(1, len(path_points)):
            p_prev = path_points[i-1]
            p_cur = path_points[i]
            dist = self._distance(p_prev, p_cur)
            if i < len(path_points)-1:
                p_next = path_points[i+1]
                v_in = (p_cur[0]-p_prev[0], p_cur[1]-p_prev[1])
                v_out = (p_next[0]-p_cur[0], p_next[1]-p_cur[1])
                corner_angle = self._angle_between_vectors(v_in, v_out)
                feedrate = self.base_feedrate * self.corner_slowdown_factor if corner_angle < self.corner_angle_threshold else self.base_feedrate
            else:
                feedrate = self.base_feedrate

            dt = dist / feedrate
            t_new = times[-1] + dt
            times.append(t_new)
            current_states = self._get_pump_states_mic(pump_images, p_cur[0], p_cur[1])
            pump_states.append(current_states)
            for j, on_state in enumerate(current_states):
                if on_state:
                    pump_displacements[j] += dist * self.flow_factor
            waypoints.append((p_cur[0], p_cur[1], self.z_pos, *pump_displacements, t_new))

        return waypoints, pump_states

    def _reorder_path_to_closest(self, path_points, last_endpoint):
        if not path_points or last_endpoint is None:
            return path_points
        distances = [self._distance(last_endpoint, p) for p in path_points]
        min_idx = np.argmin(distances)
        return path_points[min_idx:] + path_points[:min_idx]

    def generate_layers_toolpath(self, main_image_paths, pump_image_paths_list, pump_colors, z_increment=10.0):
        self.pump_colors = pump_colors
        self.z_increment = z_increment
        self.waypoints.clear()
        self.pump_states_all.clear()
        self.main_images.clear()
        self.pump_images_layers.clear()
        self.z_pos = 0.0

        last_endpoint = None
        for layer_idx, (m_path, p_paths) in enumerate(zip(main_image_paths, pump_image_paths_list)):
            self.main_images = []
            self.pump_images_layers = []
            self.load_layer_images(m_path, p_paths)
            path_points = self._generate_layer_toolpath(self.main_images[-1],
                                                        self.pump_images_layers[-1])
            if layer_idx % 2 == 1:
                path_points = path_points[::-1]
            if layer_idx > 0 and last_endpoint is not None and path_points:
                path_points = self._reorder_path_to_closest(path_points, last_endpoint)
            layer_waypoints, layer_pump_states = self._compute_waypoints(path_points, self.pump_images_layers[-1])
            if layer_idx == 0:
                self.waypoints = layer_waypoints
                self.pump_states_all = layer_pump_states
            else:
                if self.waypoints and layer_waypoints:
                    time_offset = self.waypoints[-1][-1]
                    adjusted_waypoints = []
                    for wp in layer_waypoints:
                        new_wp = list(wp)
                        new_wp[-1] += time_offset
                        adjusted_waypoints.append(tuple(new_wp))
                    self.waypoints.extend(adjusted_waypoints)
                    self.pump_states_all.extend(layer_pump_states)
                else:
                    self.waypoints.extend(layer_waypoints)
                    self.pump_states_all.extend(layer_pump_states)
            if self.waypoints:
                last_endpoint = (self.waypoints[-1][0], self.waypoints[-1][1])
            self.z_pos += self.z_increment

    def save_waypoints(self, output_file):
        with open(output_file, 'w') as f:
            for wp in self.waypoints:
                f.write(",".join(map(str, wp)) + "\n")

# ------------------------------------------------------------------------------
# ImageStackToolpathWidget: now also includes an image-stack preview (4 images horizontally)
class ImageStackToolpathWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.toolpath_generator = None

        # Parameter input form.
        form_layout = QFormLayout()
        self.threshold_edit = QLineEdit("128")
        self.tool_diameter_edit = QLineEdit("300.0")
        self.path_overlap_edit = QLineEdit("0.8")
        self.pixels_per_micron_edit = QLineEdit("0.09")
        self.base_feedrate_edit = QLineEdit("1000.0")
        self.corner_slowdown_edit = QLineEdit("0.5")
        self.corner_angle_edit = QLineEdit("120.0")
        self.flow_factor_edit = QLineEdit("0.01")
        self.initial_z_edit = QLineEdit("0.0")
        self.z_increment_edit = QLineEdit("10.0")
        form_layout.addRow("Threshold:", self.threshold_edit)
        form_layout.addRow("Tool Diameter:", self.tool_diameter_edit)
        form_layout.addRow("Path Overlap:", self.path_overlap_edit)
        form_layout.addRow("Pixels per Micron:", self.pixels_per_micron_edit)
        form_layout.addRow("Base Feedrate:", self.base_feedrate_edit)
        form_layout.addRow("Corner Slowdown Factor:", self.corner_slowdown_edit)
        form_layout.addRow("Corner Angle Threshold:", self.corner_angle_edit)
        form_layout.addRow("Flow Factor:", self.flow_factor_edit)
        form_layout.addRow("Initial Z:", self.initial_z_edit)
        form_layout.addRow("Z Increment:", self.z_increment_edit)

        # Buttons for image loading, generating toolpath, and exporting.
        self.load_main_images_btn = QPushButton("Load Main Images")
        self.load_pump_images_btn = QPushButton("Load Pump Images")
        self.generate_toolpath_btn = QPushButton("Generate Toolpath")
        self.export_waypoints_btn = QPushButton("Export Waypoints (.csv)")

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.load_main_images_btn)
        button_layout.addWidget(self.load_pump_images_btn)
        button_layout.addWidget(self.generate_toolpath_btn)
        button_layout.addWidget(self.export_waypoints_btn)

        # Image-stack preview: 4 images in a horizontal layout with a vertical slider.
        self.preview_image_labels = []
        preview_layout = QHBoxLayout()
        for _ in range(4):
            lbl = QLabel("No Image")
            lbl.setFixedSize(200, 200)
            lbl.setStyleSheet("border: 1px solid black;")
            lbl.setAlignment(Qt.AlignCenter)
            self.preview_image_labels.append(lbl)
            preview_layout.addWidget(lbl)
        self.preview_slider = QSlider(Qt.Vertical)
        self.preview_slider.setMinimum(0)
        self.preview_slider.setValue(0)
        self.preview_slider.valueChanged.connect(self.update_preview)
        preview_container = QHBoxLayout()
        preview_container.addLayout(preview_layout)
        preview_container.addWidget(self.preview_slider)

        main_layout = QVBoxLayout()
        main_layout.addLayout(form_layout)
        main_layout.addLayout(button_layout)
        main_layout.addLayout(preview_container)
        self.setLayout(main_layout)

        # Connect signals.
        self.load_main_images_btn.clicked.connect(self.load_main_images)
        self.load_pump_images_btn.clicked.connect(self.load_pump_images)
        self.generate_toolpath_btn.clicked.connect(self.generate_toolpath)
        self.export_waypoints_btn.clicked.connect(self.export_waypoints)

        # File storage.
        self.main_image_paths_list = []  # stack of main images
        self.pump_image_paths = []       # pump images for one layer

    def load_main_images(self):
        files, _ = QFileDialog.getOpenFileNames(self, "Select Main Images (Image Stack)", "", "Image Files (*.png *.jpg *.bmp)")
        if files:
            self.main_image_paths_list = files
            print("Main images loaded:", files)
            if len(self.main_image_paths_list) > 4:
                self.preview_slider.setMaximum(len(self.main_image_paths_list) - 4)
            else:
                self.preview_slider.setMaximum(0)
            self.update_preview()

    def load_pump_images(self):
        files, _ = QFileDialog.getOpenFileNames(self, "Select Pump Images", "", "Image Files (*.png *.jpg *.bmp)")
        if files:
            self.pump_image_paths = files
            print("Pump images loaded:", files)
            self.update_preview()

    def pil_to_qimage(self, pil_image):
        """
        Convert a PIL image to QImage.
        """
        # Ensure image is in RGBA mode.
        pil_image = pil_image.convert("RGBA")
        w, h = pil_image.size
        # Get raw image data.
        data = pil_image.tobytes("raw", "RGBA")
        # Create QImage from raw data.
        qimage = QImage(data, w, h, QImage.Format_RGBA8888)
        return qimage
    
    def update_preview(self):
        # Update the first label with the main image slice.
        if self.main_image_paths_list:
            try:
                # Use the slider value if you have a stack; otherwise default to 0.
                main_index = self.preview_slider.value() if len(self.main_image_paths_list) > 1 else 0
                image = Image.open(self.main_image_paths_list[main_index])
                image.thumbnail((200, 200))
                qimage = self.pil_to_qimage(image)
                pix = QPixmap.fromImage(qimage)
                self.preview_image_labels[0].setPixmap(pix)
            except Exception as e:
                print(f"Error loading main image at index {main_index}: {e}")
                self.preview_image_labels[0].setText("Error loading")
                self.preview_image_labels[0].setPixmap(QPixmap())
        else:
            self.preview_image_labels[0].setText("No Main Image")
            self.preview_image_labels[0].setPixmap(QPixmap())
            
        # Update labels 1 to 3 with the pump images.
        for i in range(1, 4):
            pump_index = i - 1  # since pump images list is 0-indexed.
            if pump_index < len(self.pump_image_paths):
                try:
                    pump_image = Image.open(self.pump_image_paths[pump_index])
                    pump_image.thumbnail((200, 200))
                    qimage = self.pil_to_qimage(pump_image)
                    pix = QPixmap.fromImage(qimage)
                    self.preview_image_labels[i].setPixmap(pix)
                except Exception as e:
                    print(f"Error loading pump image at index {pump_index}: {e}")
                    self.preview_image_labels[i].setText("Error loading")
                    self.preview_image_labels[i].setPixmap(QPixmap())
            else:
                self.preview_image_labels[i].setText("No Image")
                self.preview_image_labels[i].setPixmap(QPixmap())
                
    def generate_toolpath(self):
        try:
            threshold = int(self.threshold_edit.text())
            tool_diameter = float(self.tool_diameter_edit.text())
            path_overlap = float(self.path_overlap_edit.text())
            pixels_per_micron = float(self.pixels_per_micron_edit.text())
            base_feedrate = float(self.base_feedrate_edit.text())
            corner_slowdown = float(self.corner_slowdown_edit.text())
            corner_angle = float(self.corner_angle_edit.text())
            flow_factor = float(self.flow_factor_edit.text())
            initial_z = float(self.initial_z_edit.text())
            z_increment = float(self.z_increment_edit.text())
        except Exception as e:
            print("Error parsing parameters:", e)
            return

        self.toolpath_generator = Toolpath_From_ImageStack(
            threshold=threshold,
            tool_diameter=tool_diameter,
            path_overlap=path_overlap,
            pixels_per_micron=pixels_per_micron,
            base_feedrate=base_feedrate,
            corner_slowdown_factor=corner_slowdown,
            corner_angle_threshold=corner_angle,
            flow_factor=flow_factor,
            initial_z=initial_z,
            z_increment=z_increment
        )
        if self.main_image_paths_list and self.pump_image_paths:
            try:
                # For demonstration, we use the first image of the stack as one layer.
                self.toolpath_generator.load_layer_images(self.main_image_paths_list[0], self.pump_image_paths)
                self.toolpath_generator.set_pump_colors([(1, 0, 0), (0, 1, 0), (0, 0, 1)])
                path_points = self.toolpath_generator._generate_layer_toolpath(
                    self.toolpath_generator.main_images[-1],
                    self.toolpath_generator.pump_images_layers[-1]
                )
                waypoints, pump_states = self.toolpath_generator._compute_waypoints(
                    path_points, self.toolpath_generator.pump_images_layers[-1]
                )
                self.toolpath_generator.waypoints = waypoints
                self.toolpath_generator.pump_states_all = pump_states
                print("Toolpath generated with {} waypoints.".format(len(waypoints)))
            except Exception as e:
                print("Error generating toolpath:", e)
        else:
            print("Please load both a main image stack and pump images.")

    def export_waypoints(self):
        if self.toolpath_generator and self.toolpath_generator.waypoints:
            file_path, _ = QFileDialog.getSaveFileName(self, "Export Waypoints", "", "CSV Files (*.csv)")
            if file_path:
                self.toolpath_generator.save_waypoints(file_path)
                print("Waypoints exported to", file_path)
        else:
            print("No toolpath generated to export.")

# ------------------------------------------------------------------------------
# WaypointGeneratorWidget: placeholder for Type 2.
class WaypointGeneratorWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Waypoint Generator from Shapes (Type 2)"))
        layout.addWidget(QLabel("Widgets for adding spheres, cubes, cylinders, etc. go here."))
        self.setLayout(layout)

# ------------------------------------------------------------------------------
# Preview3DWidget: encapsulates a 3D matplotlib canvas with its navigation toolbar and a slider.
class Preview3DWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.figure = plt.figure(figsize=(10, 8))
        self.canvas = FigureCanvas(self.figure)
        #self.toolbar = NavigationToolbar(self.canvas, self)
        self.ax = self.figure.add_subplot(111, projection='3d')
        #layout.addWidget(self.toolbar)
        #layout.addWidget(self.canvas)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(100)
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self.update_plot)
        layout.addWidget(QLabel("Select Z-slice:"))
        layout.addWidget(self.slider)
        self.setLayout(layout)
        self.waypoints = []

    def set_waypoints(self, waypoints):
        self.waypoints = waypoints
        self.update_plot()

    def update_plot(self):
        z_slice = self.slider.value()
        self.ax.clear()
        # Use the waypoints if available; otherwise, create dummy data.
        waypoints = self.waypoints if self.waypoints else []
        if not waypoints:
            xs = np.linspace(0, 100, 50)
            ys = np.sin(xs/10)*50 + 50
            zs = np.full_like(xs, z_slice)
            waypoints = list(zip(xs, ys, zs))
        selected = [wp for wp in waypoints if int(wp[2]) == z_slice]
        others = [wp for wp in waypoints if int(wp[2]) != z_slice]
        if selected:
            xs = [wp[0] for wp in selected]
            ys = [wp[1] for wp in selected]
            zs = [wp[2] for wp in selected]
            self.ax.scatter(xs, ys, zs, c='red', s=40, label=f"Z = {z_slice}")
        if others:
            xs = [wp[0] for wp in others]
            ys = [wp[1] for wp in others]
            zs = [wp[2] for wp in others]
            self.ax.scatter(xs, ys, zs, c='blue', alpha=0.2, label="Other Z")
        self.ax.set_xlabel("X (microns)")
        self.ax.set_ylabel("Y (microns)")
        self.ax.set_zlabel("Z (microns)")
        self.ax.set_title("Waypoint File Preview (3D)")
        self.ax.legend()
        self.canvas.draw()

# ------------------------------------------------------------------------------
# Main window containing two tabs.
class MakePrintWidget(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Toolpath Generator")
        self.resize(1200, 800)

        self.tab_widget = QTabWidget()
        self.setCentralWidget(self.tab_widget)

        # Tab 1: Toolpath Generation Options.
        tab1 = QWidget()
        tab1_layout = QVBoxLayout()
        self.toolpath_type_combo = QComboBox()
        self.toolpath_type_combo.addItem("Toolpath from ImageStack (Type 1)")
        self.toolpath_type_combo.addItem("Waypoint Generator from Shapes (Type 2)")
        self.toolpath_type_combo.currentIndexChanged.connect(self.switch_toolpath_type)
        tab1_layout.addWidget(self.toolpath_type_combo)
        self.stacked_widget = QStackedWidget()
        self.image_stack_widget = ImageStackToolpathWidget()
        self.waypoint_generator_widget = WaypointGeneratorWidget()
        self.stacked_widget.addWidget(self.image_stack_widget)       # index 0
        self.stacked_widget.addWidget(self.waypoint_generator_widget)  # index 1
        tab1_layout.addWidget(self.stacked_widget)
        tab1.setLayout(tab1_layout)
        self.tab_widget.addTab(tab1, "Toolpath Generator")

        # Tab 2: Preview (3D).
        self.preview_widget = Preview3DWidget()
        tab2 = QWidget()
        tab2_layout = QVBoxLayout()
        tab2_layout.addWidget(self.preview_widget)
        tab2.setLayout(tab2_layout)
        self.tab_widget.addTab(tab2, "Preview")

        # Update preview when switching to the preview tab.
        self.tab_widget.currentChanged.connect(self.on_tab_changed)

    def switch_toolpath_type(self, index):
        self.stacked_widget.setCurrentIndex(index)

    def on_tab_changed(self, index):
        # When switching to the Preview tab (index 1), update the preview with generated waypoints.
        if index == 1:
            if (self.image_stack_widget.toolpath_generator and
                self.image_stack_widget.toolpath_generator.waypoints):
                self.preview_widget.set_waypoints(self.image_stack_widget.toolpath_generator.waypoints)

# ------------------------------------------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MakePrintWidget()
    window.show()
    sys.exit(app.exec())
