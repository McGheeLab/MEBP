import sys
import cv2
import numpy as np
import time

from qt_core import *

# ------------------- Global Calibration Parameters -------------------
gamma = 1.0       # Gamma correction
brightness = 0    # Brightness offset
contrast = 1.0    # Contrast multiplier

# Desired width/height for camera frames.
frame_width = 320
frame_height = 240

def get_teslong_camera_indices():
    """
    Discover camera indices that match "Teslong Camera".
    """
    indices = []
    try:
        from pygrabber.dshow_graph import FilterGraph
        graph = FilterGraph()
        devices = graph.get_input_devices()
        for i, dev in enumerate(devices):
            if "Teslong Camera" in dev:
                indices.append(i)
    except Exception as e:
        # Fallback: try indices 0 to 9
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                indices.append(i)
                cap.release()
    return indices

def adjust_image(img, gamma_val, brightness_val, contrast_val):
    """
    Adjust brightness, contrast and gamma of the image.
    """
    adjusted = cv2.convertScaleAbs(img, alpha=contrast_val, beta=brightness_val)
    if gamma_val <= 0:
        gamma_val = 0.1
    invGamma = 1.0 / gamma_val
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in range(256)]).astype("uint8")
    adjusted = cv2.LUT(adjusted, table)
    return adjusted

def overlay_grid(img):
    """
    Overlays a 3x3 grid on the frame.
    """
    h, w, _ = img.shape
    cell_w = w / 3
    cell_h = h / 3
    for i in range(3):
        for j in range(3):
            pt1 = (int(j * cell_w), int(i * cell_h))
            pt2 = (int((j + 1) * cell_w), int((i + 1) * cell_h))
            color = (0, 0, 255) if (i == 1 and j == 1) else (255, 255, 255)
            cv2.rectangle(img, pt1, pt2, color, 2)
    return img

def combine_frames(caps):
    """
    Reads one frame from each camera, applies adjustments, overlays a grid,
    and concatenates them horizontally.
    """
    global gamma, brightness, contrast
    frames = []
    for cap in caps:
        ret, frame = cap.read()
        if not ret or frame is None:
            frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
        else:
            frame = cv2.resize(frame, (frame_width, frame_height))
        frame_adjusted = adjust_image(frame, gamma, brightness, contrast)
        frame_with_grid = overlay_grid(frame_adjusted)
        frames.append(frame_with_grid)
    if not frames:
        return np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
    return frames[0] if len(frames) == 1 else np.hstack(frames)

class VideoThread(QThread):
    """
    Thread that continuously reads frames and emits the updated image.
    """
    changePixmap = Signal(QImage)

    def __init__(self, caps, parent=None):
        super().__init__(parent)
        self.caps = caps
        self._running = True

    def run(self):
        while self._running:
            composite = combine_frames(self.caps)
            rgb = cv2.cvtColor(composite, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.changePixmap.emit(qt_image)
            self.msleep(30)

    def stop(self):
        self._running = False
        self.wait()
        for cap in self.caps:
            cap.release()

class CameraWidget(QWidget):
    """
    A widget that contains camera controls and displays the video feed.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Instance-level calibration values (to mirror globals)
        self.gamma = 1.0
        self.brightness = 0
        self.contrast = 1.0
        
        # Video feed control variables
        self.feed_running = False
        self.caps = None
        self.video_thread = None

        # Layouts for organizing controls and display
        main_layout = QVBoxLayout(self)
        controls_layout = QHBoxLayout()
        sliders_layout = QVBoxLayout()

        # Start/Stop Button
        self.start_stop_button = QPushButton("Start")
        self.start_stop_button.clicked.connect(self.toggle_feed)
        controls_layout.addWidget(self.start_stop_button)

        # Gamma slider
        gamma_label = QLabel("Gamma")
        self.gamma_slider = QSlider(Qt.Horizontal)
        self.gamma_slider.setMinimum(1)
        self.gamma_slider.setMaximum(30)
        self.gamma_slider.setValue(int(self.gamma * 10))
        self.gamma_slider.valueChanged.connect(self.update_calibration)
        sliders_layout.addWidget(gamma_label)
        sliders_layout.addWidget(self.gamma_slider)

        # Brightness slider
        brightness_label = QLabel("Brightness")
        self.brightness_slider = QSlider(Qt.Horizontal)
        self.brightness_slider.setMinimum(-100)
        self.brightness_slider.setMaximum(100)
        self.brightness_slider.setValue(self.brightness)
        self.brightness_slider.valueChanged.connect(self.update_calibration)
        sliders_layout.addWidget(brightness_label)
        sliders_layout.addWidget(self.brightness_slider)

        # Contrast slider
        contrast_label = QLabel("Contrast")
        self.contrast_slider = QSlider(Qt.Horizontal)
        self.contrast_slider.setMinimum(1)
        self.contrast_slider.setMaximum(30)
        self.contrast_slider.setValue(int(self.contrast * 10))
        self.contrast_slider.valueChanged.connect(self.update_calibration)
        sliders_layout.addWidget(contrast_label)
        sliders_layout.addWidget(self.contrast_slider)

        # Group the sliders into a group box
        settings_group = QGroupBox("Camera Settings")
        settings_group.setLayout(sliders_layout)
        controls_layout.addWidget(settings_group)

        # Add the controls layout to the main layout
        main_layout.addLayout(controls_layout)

        # Video display label
        self.video_label = QLabel("Video Feed")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        main_layout.addWidget(self.video_label)

    def toggle_feed(self):
        """
        Start or stop the video thread and camera captures.
        """
        if not self.feed_running:
            indices = get_teslong_camera_indices()
            if not indices:
                print("No Teslong Cameras found.")
                return
            self.caps = [cv2.VideoCapture(idx) for idx in indices]
            self.video_thread = VideoThread(self.caps)
            self.video_thread.changePixmap.connect(self.setImage)
            self.video_thread.start()
            self.feed_running = True
            self.start_stop_button.setText("Stop")
        else:
            if self.video_thread is not None:
                self.video_thread.stop()
                self.video_thread = None
            self.feed_running = False
            self.start_stop_button.setText("Start")

    def setImage(self, image):
        """
        Update the video label with the new frame.
        """
        pixmap = QPixmap.fromImage(image)
        scaled_pixmap = pixmap.scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.video_label.setPixmap(scaled_pixmap)

    def update_calibration(self):
        """
        Update calibration parameters from slider values.
        """
        self.gamma = self.gamma_slider.value() / 10.0
        self.brightness = self.brightness_slider.value()
        self.contrast = self.contrast_slider.value() / 10.0
        # Update the globals used in combine_frames:
        global gamma, brightness, contrast
        gamma = self.gamma
        brightness = self.brightness
        contrast = self.contrast

    def closeEvent(self, event):
        if self.video_thread is not None:
            self.video_thread.stop()
        event.accept()

# Optional: allow testing the CameraWidget on its own.
if __name__ == "__main__":
    app = QApplication(sys.argv)
    #widget = CameraWidget()
    #widget.show()
    #sys.exit(app.exec())
