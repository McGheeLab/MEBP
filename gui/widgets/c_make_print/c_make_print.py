import sys, math, numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageQt
from PIL.ImageQt import toqimage

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QComboBox, QFormLayout, QSlider, QFileDialog,
    QStackedWidget, QDoubleSpinBox, QSpinBox, QGroupBox, QCheckBox, QStyledItemDelegate,
    QMessageBox, QScrollArea, QListWidget, QListWidgetItem, QColorDialog, QMenu
)
from PySide6.QtCore import Qt, QRectF, QPointF, QSizeF, Signal, QEvent
from PySide6.QtGui import QImage, QPixmap, QPainter, QColor, QPen


# ------------------------------------------------------------------------------
# Toolpath_From_ImageStack class (modified to support multi‐frame TIFF stacks)
class Toolpath_From_ImageStack:
    def __init__(self,
                 threshold=128,
                 tool_diameter=300.0,  # now in mm (was in µm)
                 path_overlap=0.8,
                 pixels_per_mm=90.0,   # default now in pixels per mm (was pixels per micron)
                 base_feedrate=1000.0,
                 corner_slowdown_factor=0.5,
                 corner_angle_threshold=120.0,
                 flow_factor=0.01,
                 initial_z=0.0,
                 z_increment=10.0,
                 max_flow_rates=[100.0, 100.0, 100.0]):  # one per pump
        self.threshold = threshold
        self.tool_diameter = tool_diameter  # now in mm
        self.path_overlap = path_overlap
        self.pixels_per_mm = pixels_per_mm    # conversion factor: pixels per mm
        self.base_feedrate = base_feedrate
        self.corner_slowdown_factor = corner_slowdown_factor
        self.corner_angle_threshold = corner_angle_threshold
        self.flow_factor = flow_factor
        self.z_pos = initial_z
        self.z_increment = z_increment
        self.max_flow_rates = max_flow_rates
        self.main_images = []          # List of 2D numpy arrays (one per layer)
        self.pump_images_layers = []   # List of [channel1, channel2, channel3] arrays per layer
        self.pump_colors = []          # To be set externally (e.g. [(1,0,0),(0,1,0),(0,0,1)])
        self.waypoints = []
        self.pump_states_all = []

    def set_pump_colors(self, colors):
        """Store the base pump colors (e.g. [(1,0,0), (0,1,0), (0,0,1)])."""
        self.pump_colors = colors
        
    def load_layer_images(self, main_image_path, pump_image_inputs):
        """
        Load all layers from the given main image file and pump image file(s).
        If main_image_path is a multi-frame TIFF, each frame becomes a layer.
        """
        # --- Load main image layers ---
        main_img = Image.open(main_image_path)
        main_layers = []
        try:
            n_frames = main_img.n_frames
        except AttributeError:
            n_frames = 1
        for i in range(n_frames):
            main_img.seek(i)
            if main_img.mode != 'L':
                frame = main_img.convert('L')
            else:
                frame = main_img.copy()
            main_layers.append(np.array(frame))
        self.height, self.width = main_layers[0].shape
        for layer in main_layers:
            if layer.shape != (self.height, self.width):
                raise ValueError("All main image layers must have the same dimensions.")
            self.main_images.append(layer)

        # --- Load pump image layers (unchanged) ---
        pump_layers = []
        if len(pump_image_inputs) == 1:
            pump_img = Image.open(pump_image_inputs[0])
            try:
                n_frames = pump_img.n_frames
            except AttributeError:
                n_frames = 1
            for i in range(n_frames):
                pump_img.seek(i)
                if pump_img.mode != "RGB":
                    pump_img_rgb = pump_img.convert("RGB")
                else:
                    pump_img_rgb = pump_img.copy()
                r, g, b = pump_img_rgb.split()
                r_arr = np.array(r)
                g_arr = np.array(g)
                b_arr = np.array(b)
                if r_arr.shape != (self.height, self.width):
                    raise ValueError("Pump image layer dimensions must match main image layers.")
                pump_layers.append([r_arr, g_arr, b_arr])
        else:
            if len(pump_image_inputs) != 3:
                raise ValueError("Provide either one pump file or three pump files.")
            channel_layers = []
            for p_path in pump_image_inputs:
                p_img = Image.open(p_path)
                frames = []
                try:
                    n_frames = p_img.n_frames
                except AttributeError:
                    n_frames = 1
                for i in range(n_frames):
                    p_img.seek(i)
                    if p_img.mode != "L":
                        frame = p_img.convert("L")
                    else:
                        frame = p_img.copy()
                    frames.append(np.array(frame))
                channel_layers.append(frames)
            if not (len(channel_layers[0]) == len(channel_layers[1]) == len(channel_layers[2])):
                raise ValueError("Pump image stacks must have the same number of frames.")
            n_frames = len(channel_layers[0])
            for i in range(n_frames):
                layer = [channel_layers[0][i], channel_layers[1][i], channel_layers[2][i]]
                if layer[0].shape != (self.height, self.width):
                    raise ValueError("Pump image layer dimensions must match main image layers.")
                pump_layers.append(layer)
        if len(pump_layers) == 1 and len(self.main_images) > 1:
            pump_layers = pump_layers * len(self.main_images)
        elif len(pump_layers) != len(self.main_images):
            raise ValueError("Number of pump layers does not match number of main image layers.")
        for layer in pump_layers:
            self.pump_images_layers.append(layer)

    def _get_pump_states_at_pixel(self, pump_images, x_px, y_px):
        x_px = min(max(x_px, 0), self.width - 1)
        y_px = min(max(y_px, 0), self.height - 1)
        flows = []
        for i, p_img in enumerate(pump_images):
            intensity = p_img[y_px, x_px]
            flow = self.max_flow_rates[i] * intensity / 255.0
            flows.append(flow)
        return flows

    def _get_pump_states_mm(self, pump_images, x_mm, y_mm):
        mm_per_pixel = 1.0 / self.pixels_per_mm
        x_px = int(round(x_mm / mm_per_pixel))
        y_px = int(round(y_mm / mm_per_pixel))
        return self._get_pump_states_at_pixel(pump_images, x_px, y_px)

    def _mm_per_pixel(self):
        return 1.0 / self.pixels_per_mm

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

    def _generate_layer_toolpath(self, main_pixels, pump_images):
        step_over = self.tool_diameter * self.path_overlap
        pixels_per_pass = max(int(math.ceil(step_over * self.pixels_per_mm)), 1)
        mm_per_pixel = self._mm_per_pixel()

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

            current_y_mm = y_px * mm_per_pixel

            for (seg_start_px, seg_end_px) in segments:
                if direction == 1:
                    px_range = range(seg_start_px, seg_end_px + 1)
                else:
                    px_range = range(seg_start_px, seg_end_px - 1, -1)
                px_list = list(px_range)

                prev_states = self._get_pump_states_at_pixel(pump_images, px_list[0], y_px)
                first_x = px_list[0] * mm_per_pixel
                if not path_points or path_points[-1] != (first_x, current_y_mm):
                    path_points.append((first_x, current_y_mm))

                for idx in range(1, len(px_list)):
                    cur_px = px_list[idx]
                    current_states = self._get_pump_states_at_pixel(pump_images, cur_px, y_px)
                    if current_states != prev_states:
                        prev_px = px_list[idx-1]
                        prev_x = prev_px * mm_per_pixel
                        path_points.append((prev_x, current_y_mm))
                        cur_x = cur_px * mm_per_pixel
                        path_points.append((cur_x, current_y_mm))
                        prev_states = current_states

                last_px = px_list[-1]
                last_x = last_px * mm_per_pixel
                if not path_points or path_points[-1] != (last_x, current_y_mm):
                    path_points.append((last_x, current_y_mm))

            y_px += pixels_per_pass
            direction *= -1

        return path_points

    def _compute_waypoints(self, path_points, pump_images):
        if len(path_points) == 0:
            return [], []

        times = [0.0]
        pump_displacements = [0.0 for _ in pump_images]
        waypoints = []
        init_states = self._get_pump_states_mm(pump_images, path_points[0][0], path_points[0][1])
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
            current_states = self._get_pump_states_mm(pump_images, p_cur[0], p_cur[1])
            pump_states.append(current_states)
            for j, pump_val in enumerate(current_states):
                if pump_val > 0:
                    pump_displacements[j] += dist * self.flow_factor
            waypoints.append((p_cur[0], p_cur[1], self.z_pos, *pump_displacements, t_new))

        return waypoints, pump_states

    def _reorder_path_to_closest(self, path_points, last_endpoint):
        if not path_points or last_endpoint is None:
            return path_points
        distances = [self._distance(last_endpoint, p) for p in path_points]
        min_idx = np.argmin(distances)
        return path_points[min_idx:] + path_points[:min_idx]

    def generate_layers_toolpath(self, main_image_paths, pump_image_paths_list, pump_colors, z_increment=10.0, max_flow_rates=[100.0, 100.0, 100.0]):
        self.pump_colors = pump_colors
        self.z_increment = z_increment
        self.max_flow_rates = max_flow_rates
        self.waypoints.clear()
        self.pump_states_all.clear()
        self.main_images.clear()
        self.pump_images_layers.clear()
        self.z_pos = 0.0

        for m_path, p_paths in zip(main_image_paths, pump_image_paths_list):
            self.load_layer_images(m_path, p_paths)
        last_endpoint = None
        for layer_idx in range(len(self.main_images)):
            main_layer = self.main_images[layer_idx]
            pump_layer = self.pump_images_layers[layer_idx]
            path_points = self._generate_layer_toolpath(main_layer, pump_layer)
            if layer_idx % 2 == 1:
                path_points = path_points[::-1]
            if layer_idx > 0 and last_endpoint is not None and path_points:
                path_points = self._reorder_path_to_closest(path_points, last_endpoint)
            layer_waypoints, layer_pump_states = self._compute_waypoints(path_points, pump_layer)
            if layer_idx == 0:
                self.waypoints = layer_waypoints
                self.pump_states_all = layer_pump_states
            else:
                time_offset = self.waypoints[-1][-1]
                adjusted_waypoints = []
                for wp in layer_waypoints:
                    new_wp = list(wp)
                    new_wp[-1] += time_offset
                    adjusted_waypoints.append(tuple(new_wp))
                self.waypoints.extend(adjusted_waypoints)
                self.pump_states_all.extend(layer_pump_states)
            if self.waypoints:
                last_endpoint = (self.waypoints[-1][0], self.waypoints[-1][1])
            self.z_pos += self.z_increment

        # Center the entire toolpath about (0,0)
        if self.waypoints:
            xs = [wp[0] for wp in self.waypoints]
            ys = [wp[1] for wp in self.waypoints]
            center_x = (min(xs) + max(xs)) / 2.0
            center_y = (min(ys) + max(ys)) / 2.0
            self.waypoints = [(wp[0] - center_x, wp[1] - center_y, *wp[2:]) for wp in self.waypoints]

    def save_waypoints(self, output_file):
        with open(output_file, 'w') as f:
            for wp in self.waypoints:
                f.write(",".join(map(str, wp)) + "\n")


# ------------------------------------------------------------------------------
class ImageStackToolpathWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.toolpath_generator = None
        self.total_frames = 1  
        self.layer_label = QLabel("Frame: 1 / 1")
        
        form_layout = QFormLayout()

        # New real-world size parameters
        self.real_width_edit = QLineEdit("1.0")
        self.add_parameter_row(form_layout, "Real World Width (mm):", self.real_width_edit,
                          "Physical print width in mm (default=1.0)")
        self.real_height_edit = QLineEdit("1.0")
        self.add_parameter_row(form_layout, "Real World Height (mm):", self.real_height_edit,
                          "Physical print height in mm (default=1.0)")
        # Linked field: pixels per mm (updated from pixels per micron)
        self.pixels_per_mm_edit = QLineEdit("90.0")
        self.add_parameter_row(form_layout, "Pixels per mm:", self.pixels_per_mm_edit,
                          "Auto-adjusted from image and real dimensions")
        
        # Other parameters with descriptions.
        self.threshold_edit = QLineEdit("128")
        self.add_parameter_row(form_layout, "Threshold:", self.threshold_edit,
                          "Grayscale threshold for path activation")
        # Update tool diameter description to be in mm.
        self.tool_diameter_edit = QLineEdit("300.0")
        self.add_parameter_row(form_layout, "Tool Diameter (mm):", self.tool_diameter_edit,
                          "Extrusion tool diameter in mm")
        self.path_overlap_edit = QLineEdit("0.8")
        self.add_parameter_row(form_layout, "Path Overlap:", self.path_overlap_edit,
                          "Fraction of tool diameter for adjacent paths (0-1)")
        self.base_feedrate_edit = QLineEdit("1000.0")
        self.add_parameter_row(form_layout, "Base Feedrate (mm/min):", self.base_feedrate_edit,
                          "Tool travel speed in mm/min")
        self.corner_slowdown_edit = QLineEdit("0.5")
        self.add_parameter_row(form_layout, "Corner Slowdown Factor:", self.corner_slowdown_edit,
                          "Speed reduction multiplier at corners")
        self.corner_angle_edit = QLineEdit("120.0")
        self.add_parameter_row(form_layout, "Corner Angle Threshold:", self.corner_angle_edit,
                          "Angle (°) below which to slow down")
        self.flow_factor_edit = QLineEdit("0.01")
        self.add_parameter_row(form_layout, "Flow Factor:", self.flow_factor_edit,
                          "Extrusion multiplier for material flow")
        self.initial_z_edit = QLineEdit("0.0")
        self.add_parameter_row(form_layout, "Initial Z:", self.initial_z_edit,
                          "Starting layer height")
        self.z_increment_edit = QLineEdit("10.0")
        self.add_parameter_row(form_layout, "Z Increment:", self.z_increment_edit,
                          "Height increase per layer")

        # Max Flow Rate fields (calculated automatically).
        self.max_flow_rate_spin1 = QDoubleSpinBox()
        self.max_flow_rate_spin1.setMinimum(0.0)
        self.max_flow_rate_spin1.setMaximum(10000.0)
        self.max_flow_rate_spin1.setValue(100.0)
        self.add_parameter_row(form_layout, "Max Flow Pump 1 (uL/min):", self.max_flow_rate_spin1,
                          "Calculated from feedrate and tool diameter")
        self.max_flow_rate_spin2 = QDoubleSpinBox()
        self.max_flow_rate_spin2.setMinimum(0.0)
        self.max_flow_rate_spin2.setMaximum(10000.0)
        self.max_flow_rate_spin2.setValue(100.0)
        self.add_parameter_row(form_layout, "Max Flow Pump 2 (uL/min):", self.max_flow_rate_spin2,
                          "Calculated from feedrate and tool diameter")
        self.max_flow_rate_spin3 = QDoubleSpinBox()
        self.max_flow_rate_spin3.setMinimum(0.0)
        self.max_flow_rate_spin3.setMaximum(10000.0)
        self.max_flow_rate_spin3.setValue(100.0)
        self.add_parameter_row(form_layout, "Max Flow Pump 3 (uL/min):", self.max_flow_rate_spin3,
                          "Calculated from feedrate and tool diameter")

        # Buttons.
        self.load_main_images_btn = QPushButton("Load Main Images")
        self.load_pump_images_btn = QPushButton("Load Pump Images")
        self.generate_toolpath_btn = QPushButton("Generate Toolpath")
        self.export_waypoints_btn = QPushButton("Export Waypoints (.csv)")
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.load_main_images_btn)
        button_layout.addWidget(self.load_pump_images_btn)
        button_layout.addWidget(self.generate_toolpath_btn)
        button_layout.addWidget(self.export_waypoints_btn)

        # Preview area.
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
        slider_layout = QVBoxLayout()
        slider_layout.addWidget(self.layer_label)
        slider_layout.addWidget(self.preview_slider)
        
        preview_container = QHBoxLayout()
        preview_container.addLayout(preview_layout)
        preview_container.addLayout(slider_layout)

        main_layout = QVBoxLayout()
        main_layout.addLayout(form_layout)
        main_layout.addLayout(button_layout)
        main_layout.addLayout(preview_container)
        self.setLayout(main_layout)

        # Connect signals for buttons.
        self.load_main_images_btn.clicked.connect(self.load_main_images)
        self.load_pump_images_btn.clicked.connect(self.load_pump_images)
        self.generate_toolpath_btn.clicked.connect(self.generate_toolpath)
        self.export_waypoints_btn.clicked.connect(self.export_waypoints)

        # Connect edit fields so that any change triggers a recalculation.
        self.real_width_edit.textChanged.connect(lambda: self.update_all_calculations("width"))
        self.real_height_edit.textChanged.connect(lambda: self.update_all_calculations("height"))
        self.pixels_per_mm_edit.textChanged.connect(lambda: self.update_all_calculations("ppm"))
        self.base_feedrate_edit.textChanged.connect(lambda: self.update_all_calculations(None))
        self.tool_diameter_edit.textChanged.connect(lambda: self.update_all_calculations(None))

        # Store image dimensions when a main image is loaded.
        self.image_pixel_width = None
        self.image_pixel_height = None
        self.main_image_paths_list = []  # list of main image files
        self.pump_image_paths = []       # list of pump image files (or a single file)

    def update_all_calculations(self, source):
        """Update linked dimensions and recalc calculated values."""
        self.update_linked_dimensions(source)
        self.update_max_flow_rate()

    def update_linked_dimensions(self, source):
        """
        For the three linked fields (real width, real height, and pixels per mm),
        update the other two based on the field that is currently being edited.
        The calculations use the loaded image dimensions.
        """
        if self.image_pixel_width is None or self.image_pixel_height is None:
            return

        if source == "width":
            try:
                real_width = round(float(self.real_width_edit.text()), 3)
            except Exception:
                return
            # Calculate new pixels per mm.
            new_ppm = round(self.image_pixel_width / real_width, 3)
            new_real_height = round(real_width * (self.image_pixel_height / self.image_pixel_width), 3)
            self.pixels_per_mm_edit.blockSignals(True)
            self.pixels_per_mm_edit.setText(str(new_ppm))
            self.pixels_per_mm_edit.blockSignals(False)
            self.real_height_edit.blockSignals(True)
            self.real_height_edit.setText(str(new_real_height))
            self.real_height_edit.blockSignals(False)
        elif source == "height":
            try:
                real_height = round(float(self.real_height_edit.text()), 3)
            except Exception:
                return
            new_ppm = round(self.image_pixel_height / real_height, 3)
            new_real_width = round(real_height * (self.image_pixel_width / self.image_pixel_height), 3)
            self.pixels_per_mm_edit.blockSignals(True)
            self.pixels_per_mm_edit.setText(str(new_ppm))
            self.pixels_per_mm_edit.blockSignals(False)
            self.real_width_edit.blockSignals(True)
            self.real_width_edit.setText(str(new_real_width))
            self.real_width_edit.blockSignals(False)
        elif source == "ppm":
            try:
                ppm = round(float(self.pixels_per_mm_edit.text()), 3)
            except Exception:
                return
            new_real_width = round(self.image_pixel_width / ppm, 3)
            new_real_height = round(self.image_pixel_height / ppm, 3)
            self.real_width_edit.blockSignals(True)
            self.real_width_edit.setText(str(new_real_width))
            self.real_width_edit.blockSignals(False)
            self.real_height_edit.blockSignals(True)
            self.real_height_edit.setText(str(new_real_height))
            self.real_height_edit.blockSignals(False)
        else:
            try:
                ppm = float(self.pixels_per_mm_edit.text())
            except Exception:
                return
            new_real_width = round(self.image_pixel_width / ppm, 3)
            new_real_height = round(self.image_pixel_height / ppm, 3)
            self.real_width_edit.blockSignals(True)
            self.real_width_edit.setText(str(new_real_width))
            self.real_width_edit.blockSignals(False)
            self.real_height_edit.blockSignals(True)
            self.real_height_edit.setText(str(new_real_height))
            self.real_height_edit.blockSignals(False)

    def update_max_flow_rate(self):
        """Recalculate the max flow rate based on the base feedrate and tool diameter."""
        try:
            base_feedrate = float(self.base_feedrate_edit.text())
            tool_diameter = float(self.tool_diameter_edit.text())
        except Exception:
            return
        # Now tool_diameter is already in mm.
        computed_max_flow = round(base_feedrate * (math.pi / 4) * (tool_diameter ** 2), 3)
        for spin_box in (self.max_flow_rate_spin1, self.max_flow_rate_spin2, self.max_flow_rate_spin3):
            spin_box.blockSignals(True)
            spin_box.setValue(computed_max_flow)
            spin_box.blockSignals(False)

    def load_main_images(self):
        files, _ = QFileDialog.getOpenFileNames(
            self, "Select Main Images (Image Stack)", "", "Image Files (*.png *.jpg *.bmp *.tif)"
        )
        if files:
            self.main_image_paths_list = files
            print("Main images loaded:", files)
            try:
                img = Image.open(files[0])
                self.image_pixel_width, self.image_pixel_height = img.size
                if hasattr(img, "n_frames") and img.n_frames > 1:
                    self.total_frames = img.n_frames
                else:
                    self.total_frames = 1
                self.update_all_calculations(None)
            except Exception as e:
                self.show_error("Error reading image: " + str(e))
                self.total_frames = 1
            self.preview_slider.setMaximum(self.total_frames - 1)
            self.layer_label.setText(f"Frame: 1 / {self.total_frames}")
            self.update_preview()
            
    def show_error(self, message):
        QMessageBox.critical(self, "Error", message)

    def update_preview(self):
        if self.main_image_paths_list:
            try:
                if len(self.main_image_paths_list) == 1:
                    main_index = self.preview_slider.value()
                    image = Image.open(self.main_image_paths_list[0])
                    try:
                        if image.n_frames > 1:
                            image.seek(main_index)
                    except AttributeError:
                        pass
                else:
                    main_index = self.preview_slider.value() if len(self.main_image_paths_list) > 1 else 0
                    image = Image.open(self.main_image_paths_list[main_index])
                image.thumbnail((200, 200))
                qimage = self.pil_to_qimage(image)
                pix = QPixmap.fromImage(qimage)
                self.preview_image_labels[0].setPixmap(pix)
                self.layer_label.setText(f"Frame: {main_index+1} / {self.total_frames}")
            except Exception as e:
                self.show_error(f"Error loading main image at index {main_index}: {e}")
                self.preview_image_labels[0].setText("Error loading")
                self.preview_image_labels[0].setPixmap(QPixmap())
        else:
            self.preview_image_labels[0].setText("No Main Image")
            self.preview_image_labels[0].setPixmap(QPixmap())
        
        pump_imgs = []
        if len(self.pump_image_paths) == 1:
            try:
                img = Image.open(self.pump_image_paths[0])
                try:
                    if img.n_frames > 1:
                        img.seek(self.preview_slider.value())
                except AttributeError:
                    pass
                img = img.convert("RGB")
                img.thumbnail((200, 200))
                r, g, b = img.split()
                pump_imgs = [r, g, b]
            except Exception as e:
                self.show_error("Error splitting pump image: " + str(e))
        else:
            for p in self.pump_image_paths:
                try:
                    pump_img = Image.open(p).convert("L")
                    pump_img.thumbnail((200, 200))
                    pump_imgs.append(pump_img)
                except Exception as e:
                    self.show_error("Error loading pump image: " + str(e))
        for i in range(1, 4):
            if i-1 < len(pump_imgs):
                qimg = self.pil_to_qimage(pump_imgs[i-1])
                pix = QPixmap.fromImage(qimg)
                self.preview_image_labels[i].setPixmap(pix)
            else:
                self.preview_image_labels[i].setText("No Image")
                self.preview_image_labels[i].setPixmap(QPixmap())

    def load_pump_images(self):
        files, _ = QFileDialog.getOpenFileNames(self, "Select Pump Images", "", "Image Files (*.png *.jpg *.bmp *.tif)")
        if files:
            self.pump_image_paths = files if len(files) > 1 else files
            print("Pump images loaded:", self.pump_image_paths)
            self.update_preview()

    def pil_to_qimage(self, pil_image):
        pil_image = pil_image.convert("RGBA")
        w, h = pil_image.size
        data = pil_image.tobytes("raw", "RGBA")
        qimage = QImage(data, w, h, QImage.Format_RGBA8888)
        return qimage

    def generate_toolpath(self):
        try:
            threshold = int(self.threshold_edit.text())
            tool_diameter = float(self.tool_diameter_edit.text())  # in mm now
            path_overlap = float(self.path_overlap_edit.text())
            pixels_per_mm = float(self.pixels_per_mm_edit.text())
            base_feedrate = float(self.base_feedrate_edit.text())
            corner_slowdown = float(self.corner_slowdown_edit.text())
            corner_angle = float(self.corner_angle_edit.text())
            flow_factor = float(self.flow_factor_edit.text())
            initial_z = float(self.initial_z_edit.text())
            z_increment = float(self.z_increment_edit.text())
        except Exception as e:
            self.show_error("Error parsing parameters: " + str(e))
            return

        # Compute the maximum volumetric flow rate (uL/min) based on the assumption that
        # the needle extrudes a cylinder whose diameter is that of the tool.
        # Here tool_diameter is already in mm.
        computed_max_flow = base_feedrate * (math.pi / 4) * (tool_diameter ** 2)
        self.max_flow_rate_spin1.setValue(computed_max_flow)
        self.max_flow_rate_spin2.setValue(computed_max_flow)
        self.max_flow_rate_spin3.setValue(computed_max_flow)
        max_flow_rates = [
            self.max_flow_rate_spin1.value(),
            self.max_flow_rate_spin2.value(),
            self.max_flow_rate_spin3.value()
        ]

        self.toolpath_generator = Toolpath_From_ImageStack(
            threshold=threshold,
            tool_diameter=tool_diameter,
            path_overlap=path_overlap,
            pixels_per_mm=pixels_per_mm,
            base_feedrate=base_feedrate,
            corner_slowdown_factor=corner_slowdown,
            corner_angle_threshold=corner_angle,
            flow_factor=flow_factor,
            initial_z=initial_z,
            z_increment=z_increment,
            max_flow_rates=max_flow_rates
        )
        if self.main_image_paths_list and self.pump_image_paths:
            try:
                self.toolpath_generator.generate_layers_toolpath(
                    [self.main_image_paths_list[0]],
                    [self.pump_image_paths],
                    [(1, 0, 0), (0, 1, 0), (0, 0, 1)],
                    z_increment=z_increment,
                    max_flow_rates=max_flow_rates
                )
                print("Toolpath generated with {} waypoints.".format(len(self.toolpath_generator.waypoints)))
            except Exception as e:
                self.show_error("Error generating toolpath: " + str(e))
        else:
            self.show_error("Please load both a main image stack and pump images.")

    def export_waypoints(self):
        if self.toolpath_generator and self.toolpath_generator.waypoints:
            file_path, _ = QFileDialog.getSaveFileName(self, "Export Waypoints", "", "CSV Files (*.csv)")
            if file_path:
                try:
                    self.toolpath_generator.save_waypoints(file_path)
                    print("Waypoints exported to", file_path)
                except Exception as e:
                    self.show_error("Error exporting waypoints: " + str(e))
        else:
            self.show_error("No toolpath generated to export.")

    def add_parameter_row(self, form_layout, label_text, widget, description):
        widget.setFixedWidth(100)
        h_layout = QHBoxLayout()
        h_layout.addWidget(widget)
        desc_label = QLabel(description)
        h_layout.addWidget(desc_label)
        form_layout.addRow(label_text, h_layout)



# WaypointGeneratorWidget: placeholder for Type 2.
class WaypointGeneratorWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Waypoint Generator from Shapes (Type 2)"))
        layout.addWidget(QLabel("Widgets for adding spheres, cubes, cylinders, etc. go here."))
        self.setLayout(layout)

# Preview3DWidget remains unchanged from previous version.
class Preview3DCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.waypoints = []         # List of (x, y, z, ...)
        self.pump_states_all = []   # Each state is a list of 3 effective flow values
        self.pump_colors = []       # Expected base colors [(1,0,0), (0,1,0), (0,0,1)]
        self.max_flow_rates = [100.0, 100.0, 100.0]
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.zoom = 1.0
        self.rotation_x = 0.0
        self.rotation_y = 0.0
        self.last_mouse_pos = None
        self._right_press_pos = None
        self._right_dragging = False
        self.focus_layer_index = 0

        self.layer_indices = []  

        self.setContextMenuPolicy(Qt.NoContextMenu)
        self.setMouseTracking(True)

    def set_toolpath(self, waypoints, pump_states_all, pump_colors, max_flow_rates):
        self.waypoints = waypoints
        self.pump_states_all = pump_states_all
        self.pump_colors = pump_colors
        self.max_flow_rates = max_flow_rates
        self.layer_indices = sorted(set(round(wp[2], 5) for wp in self.waypoints))
        self.focus_layer_index = 0
        self.fit_to_view()
        self.update()

    def set_focus_layer(self, index):
        self.focus_layer_index = index
        self.update()

    def raw_transform_point(self, x, y, z):
        rx = self.rotation_x
        ry = self.rotation_y
        x1 = x
        y1 = y * math.cos(rx) - z * math.sin(rx)
        z1 = y * math.sin(rx) + z * math.cos(rx)
        x2 = x1 * math.cos(ry) + z1 * math.sin(ry)
        y2 = y1
        return x2, y2

    def transform_point(self, x, y, z):
        tx, ty = self.raw_transform_point(x, y, z)
        proj_x = tx * self.zoom + self.offset_x
        proj_y = ty * self.zoom + self.offset_y
        return proj_x, proj_y

    def fit_to_view(self):
        if not self.waypoints or self.width() <= 0 or self.height() <= 0:
            return
        projected = [self.raw_transform_point(wp[0], wp[1], wp[2]) for wp in self.waypoints]
        xs = [pt[0] for pt in projected]
        ys = [pt[1] for pt in projected]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        bbox_width = max_x - min_x
        bbox_height = max_y - min_y
        if bbox_width == 0 or bbox_height == 0:
            return
        margin = 0.9
        scale_x = (self.width() * margin) / bbox_width
        scale_y = (self.height() * margin) / bbox_height
        self.zoom = min(scale_x, scale_y)
        self.offset_x = (self.width() - bbox_width * self.zoom) / 2 - min_x * self.zoom
        self.offset_y = (self.height() - bbox_height * self.zoom) / 2 - min_y * self.zoom

    def get_segment_color(self, state):
        if not self.max_flow_rates or len(self.max_flow_rates) < 3:
            return QColor(255, 255, 255)
        try:
            r = int(state[0] / self.max_flow_rates[0] * 255) if self.max_flow_rates[0] != 0 else 0
            g = int(state[1] / self.max_flow_rates[1] * 255) if self.max_flow_rates[1] != 0 else 0
            b = int(state[2] / self.max_flow_rates[2] * 255) if self.max_flow_rates[2] != 0 else 0
        except Exception:
            r = g = b = 0
        r = min(max(r, 0), 255)
        g = min(max(g, 0), 255)
        b = min(max(b, 0), 255)
        return QColor(r, g, b)

    def paintEvent(self, event):
        
        alpha_factor = 0.2
        
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        focus_z = None
        if self.layer_indices:
            focus_z = self.layer_indices[self.focus_layer_index]
        tol = 1e-3
        if self.waypoints:
            transformed_points = [self.transform_point(wp[0], wp[1], wp[2]) for wp in self.waypoints]
            for i in range(len(transformed_points)-1):
                p1 = transformed_points[i]
                p2 = transformed_points[i+1]
                base_color = self.get_segment_color(self.pump_states_all[i]) if i < len(self.pump_states_all) else QColor(0, 0, 0)
                z_val = self.waypoints[i][2]
                alpha = 255 if (focus_z is not None and abs(z_val - focus_z) < tol) else int(255 * alpha_factor)
                color = QColor(base_color)
                color.setAlpha(alpha)
                pen = QPen(color)
                pen.setWidth(2)
                painter.setPen(pen)
                painter.drawLine(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1]))
            for i, pt in enumerate(transformed_points):
                base_color = self.get_segment_color(self.pump_states_all[i]) if i < len(self.pump_states_all) else QColor(0, 0, 0)
                z_val = self.waypoints[i][2]
                if focus_z is not None and abs(z_val - focus_z) < tol:
                    alpha = 255
                    radius = 4
                else:
                    alpha = 0
                    radius = 2
                color = QColor(base_color)
                color.setAlpha(alpha)
                color_outline = QColor(0, 0, 0, alpha)
                pen = QPen(color_outline)
                painter.setPen(pen)
                painter.setBrush(color)
                painter.drawEllipse(int(pt[0]-radius), int(pt[1]-radius), radius*2, radius*2)
        else:
            painter.drawText(self.rect(), Qt.AlignCenter, "No waypoints to display")
        self.drawRosette(painter)

    def drawRosette(self, painter):
        painter.save()
        margin = 10
        rosette_size = 80
        center = QPointF(margin + rosette_size/2, self.height()-margin-rosette_size/2)
        radius = rosette_size/2 - 5
        pen = QPen(QColor(0,0,0))
        pen.setWidth(1)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(center, radius, radius)
        def rosette_vector(dx, dy, dz, length):
            rx = self.rotation_x
            ry = self.rotation_y
            x1 = dx
            y1 = dy * math.cos(rx) - dz * math.sin(rx)
            z1 = dy * math.sin(rx) + dz * math.cos(rx)
            x2 = x1 * math.cos(ry) + z1 * math.sin(ry)
            y2 = y1
            mag = math.hypot(x2, y2)
            if mag == 0:
                return 0, 0
            return (x2/mag*length, y2/mag*length)
        arrow_length = radius - 5
        vx, vy = rosette_vector(1, 0, 0, arrow_length)
        pen.setColor(QColor(255,0,0))
        painter.setPen(pen)
        painter.drawLine(center, QPointF(center.x()+vx, center.y()-vy))
        pen.setColor(QColor(0,255,0))
        painter.setPen(pen)
        painter.drawLine(center, QPointF(center.x(), center.y()-arrow_length))
        vx, vy = rosette_vector(0, 0, 1, arrow_length)
        pen.setColor(QColor(0,0,255))
        painter.setPen(pen)
        painter.drawLine(center, QPointF(center.x()+vx, center.y()-vy))
        painter.restore()

    def mousePressEvent(self, event):
        self.last_mouse_pos = event.position()
        if event.button() == Qt.RightButton:
            self._right_press_pos = event.position()
            self._right_dragging = False

    def mouseMoveEvent(self, event):
        if self.last_mouse_pos is None:
            self.last_mouse_pos = event.position()
            return
        delta = event.position() - self.last_mouse_pos
        buttons = event.buttons()
        if buttons & Qt.LeftButton:
            self.offset_x += delta.x()
            self.offset_y += delta.y()
        if buttons & Qt.RightButton:
            if self._right_press_pos is not None:
                if (event.position() - self._right_press_pos).manhattanLength() > 5:
                    self._right_dragging = True
            self.rotation_y += delta.x() * 0.01
            self.rotation_x += delta.y() * 0.01
        self.last_mouse_pos = event.position()
        self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            if not self._right_dragging:
                self.showContextMenu(event.globalPos())
            self._right_press_pos = None
            self._right_dragging = False
        self.last_mouse_pos = event.position()

    def wheelEvent(self, event):
        zoom_factor = 1.0 + event.angleDelta().y() / 1200.0
        self.zoom *= zoom_factor
        self.update()

    def showContextMenu(self, global_pos):
        menu = QMenu(self)
        front_action = menu.addAction("Front")
        top_action = menu.addAction("Top")
        left_action = menu.addAction("Left")
        right_action = menu.addAction("Right")
        isometric_action = menu.addAction("Isometric")
        bottom_action = menu.addAction("Bottom")
        action = menu.exec_(global_pos)
        if action == front_action:
            self.rotation_x = 0.0; self.rotation_y = 0.0
        elif action == top_action:
            self.rotation_x = -math.pi/2; self.rotation_y = 0.0
        elif action == left_action:
            self.rotation_x = 0.0; self.rotation_y = math.pi/2
        elif action == right_action:
            self.rotation_x = 0.0; self.rotation_y = -math.pi/2
        elif action == isometric_action:
            self.rotation_x = math.radians(-35); self.rotation_y = math.radians(45)
        elif action == bottom_action:
            self.rotation_x = math.pi/2; self.rotation_y = 0.0
        self.fit_to_view()
        self.update()

class Preview3DContainer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.canvas = Preview3DCanvas(self)
        self.layer_slider = QSlider(Qt.Vertical)
        self.layer_slider.setMinimum(0)
        self.layer_slider.setMaximum(0)
        self.layer_slider.setValue(0)
        self.layer_slider.valueChanged.connect(self.on_slider_changed)
        self.layer_label = QLabel("Layer: 1 / 1")
        self.layer_label.setAlignment(Qt.AlignRight)
        main_vlay = QVBoxLayout()
        top_hlay = QHBoxLayout()
        top_hlay.addStretch()
        top_hlay.addWidget(self.layer_label)
        main_vlay.addLayout(top_hlay)
        bottom_hlay = QHBoxLayout()
        bottom_hlay.addWidget(self.canvas, stretch=1)
        bottom_hlay.addWidget(self.layer_slider)
        main_vlay.addLayout(bottom_hlay)
        self.setLayout(main_vlay)

    def on_slider_changed(self, value):
        self.canvas.set_focus_layer(value)
        if self.canvas.layer_indices:
            self.layer_label.setText(f"Layer: {value+1} / {len(self.canvas.layer_indices)}")
        else:
            self.layer_label.setText("Layer: - / -")

    def set_toolpath(self, waypoints, pump_states_all, pump_colors, max_flow_rates):
        self.canvas.set_toolpath(waypoints, pump_states_all, pump_colors, max_flow_rates)
        if self.canvas.layer_indices:
            self.layer_slider.setMinimum(0)
            self.layer_slider.setMaximum(len(self.canvas.layer_indices) - 1)
            self.layer_slider.setValue(0)
            self.layer_label.setText(f"Layer: 1 / {len(self.canvas.layer_indices)}")
        else:
            self.layer_slider.setMaximum(0)
            self.layer_label.setText("Layer: - / -")

# ------------------------------------------------------------------------------
class MakePrintWidget(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Toolpath Generator")
        self.resize(1200, 800)

        self.tab_widget = QTabWidget()
        self.setCentralWidget(self.tab_widget)

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

        self.preview_widget = Preview3DContainer()
        tab2 = QWidget()
        tab2_layout = QVBoxLayout()
        tab2_layout.addWidget(self.preview_widget)
        tab2.setLayout(tab2_layout)
        self.tab_widget.addTab(tab2, "Preview")

        self.tab_widget.currentChanged.connect(self.on_tab_changed)

    def switch_toolpath_type(self, index):
        self.stacked_widget.setCurrentIndex(index)

    def on_tab_changed(self, index):
        if index == 1:
            gen = self.image_stack_widget.toolpath_generator
            if gen and gen.waypoints:
                self.preview_widget.set_toolpath(
                    gen.waypoints,
                    gen.pump_states_all,
                    [(1, 0, 0), (0, 1, 0), (0, 0, 1)],
                    gen.max_flow_rates
                )

# ------------------------------------------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MakePrintWidget()
    window.show()
    sys.exit(app.exec())
