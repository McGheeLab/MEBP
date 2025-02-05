import sys
import cv2
import numpy as np
import time

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton, QLabel, QVBoxLayout,
    QHBoxLayout, QSlider, QGroupBox, QGridLayout, QFrame
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap

# ------------------- Global Calibration Parameters -------------------
# These values will be updated dynamically by the sliders (gamma, brightness, contrast).
gamma = 1.0       # Adjusts the gamma correction (non-linear luminosity adjustment)
brightness = 0    # Adjusts brightness offset
contrast = 1.0    # Adjusts contrast multiplier

# Desired width/height for camera frames before merging them side by side.
frame_width = 320
frame_height = 240

def get_teslong_camera_indices():
    """
    Tries to discover camera indices that match the string "Teslong Camera".
    On Windows, attempts to use pygrabber for enumerating devices.
    If unavailable or fails, it scans basic indices (0 to 9) to see which are usable.
    Returns:
        A list of integer indices corresponding to connected Teslong cameras.
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
        # Fallback to manually checking camera indices
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                indices.append(i)
                cap.release()
    return indices

def adjust_image(img, gamma_val, brightness_val, contrast_val):
    """
    Applies brightness and contrast (using cv2.convertScaleAbs),
    then applies gamma correction to the image.
    Args:
        img (numpy array): Original frame from the camera.
        gamma_val (float): Gamma slider value > 0. (non-linear brightness)
        brightness_val (int): Shift in brightness.
        contrast_val (float): Multiplier for contrast.
    Returns:
        A new adjusted image (numpy array).
    """
    # First step: brightness/contrast adjustment
    adjusted = cv2.convertScaleAbs(img, alpha=contrast_val, beta=brightness_val)
    # Prevent division by zero if gamma_val is invalid
    if gamma_val <= 0:
        gamma_val = 0.1
    # Second step: gamma correction lookup table
    invGamma = 1.0 / gamma_val
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in range(256)]).astype("uint8")
    adjusted = cv2.LUT(adjusted, table)
    return adjusted

def overlay_grid(img):
    """
    Overlays a 3x3 grid on the frame. The center square is highlighted in red,
    and other squares are outlined in white.
    Args:
        img (numpy array): Frame on which the grid is drawn.
    Returns:
        The image with grid lines drawn.
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
    Reads one frame from each camera capture in 'caps', resizes it,
    applies global calibration adjustments (gamma, brightness, contrast),
    overlays a grid, and then concatenates all the frames horizontally.
    Args:
        caps (list): List of cv2.VideoCapture objects.
    Returns:
        A single merged frame (numpy array), or a blank image if no frames exist.
    """
    global gamma, brightness, contrast
    frames = []

    # For each camera, read, resize, adjust, and overlay the grid
    for cap in caps:
        ret, frame = cap.read()
        if not ret or frame is None:
            # If frame is empty/not retrieved, just use a black placeholder
            frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
        else:
            frame = cv2.resize(frame, (frame_width, frame_height))

        frame_adjusted = adjust_image(frame, gamma, brightness, contrast)
        frame_with_grid = overlay_grid(frame_adjusted)
        frames.append(frame_with_grid)

    if len(frames) == 0:
        return np.zeros((frame_height, frame_width, 3), dtype=np.uint8)

    if len(frames) == 1:
        return frames[0]

    # Concatenate all frames side by side
    composite = np.hstack(frames)
    return composite

class VideoThread(QThread):
    """
    A QThread that continuously reads frames from multiple cameras and merges them.
    Emits the final image (in QImage format) via changePixmap signal.
    """
    changePixmap = pyqtSignal(QImage)

    def __init__(self, caps, parent=None):
        """
        Initializes the thread with a list of camera captures.
        Args:
            caps (list): cv2.VideoCapture objects for each camera.
        """
        super(VideoThread, self).__init__(parent)
        self.caps = caps
        self._running = True  # Control variable to stop the loop

    def run(self):
        """
        Main thread loop. Continuously combines frames from each camera,
        converts them to QImage, and sends the signal for display.
        """
        while self._running:
            composite = combine_frames(self.caps)
            # Convert BGR -> RGB for QImage
            rgb = cv2.cvtColor(composite, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            # Construct QImage from raw data
            qt_image = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.changePixmap.emit(qt_image)
            self.msleep(30)  # Wait ~30 ms (about 30 FPS)

    def stop(self):
        """
        Stops the thread cleanly, then releases all camera resources.
        """
        self._running = False
        self.wait()  # Wait until run() is fully done
        # Release each camera
        for cap in self.caps:
            cap.release()

class MainWindow(QMainWindow):
    """
    Main application window that sets up a UI with:
      - Start/Stop button
      - Sliders for gamma, brightness, contrast
      - Label for showing the merged video.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Teslong Camera Feeds - PyQt5")

        # Central widget has a horizontal layout: left side controls, right side video
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.hbox = QHBoxLayout(self.central_widget)

        # ------------------- Left Panel (Controls) -------------------
        self.controls_widget = QWidget()
        self.controls_layout = QVBoxLayout(self.controls_widget)

        self.start_stop_button = QPushButton("Start")
        self.start_stop_button.clicked.connect(self.toggle_feed)
        self.controls_layout.addWidget(self.start_stop_button)

        # Gamma slider
        self.controls_layout.addWidget(QLabel("Gamma"))
        self.gamma_slider = QSlider(Qt.Horizontal)
        self.gamma_slider.setMinimum(1)   # Slider value mapped to gamma
        self.gamma_slider.setMaximum(30)
        self.gamma_slider.setValue(int(gamma * 10))
        self.gamma_slider.valueChanged.connect(self.update_calibration)
        self.controls_layout.addWidget(self.gamma_slider)

        # Brightness slider
        self.controls_layout.addWidget(QLabel("Brightness"))
        self.brightness_slider = QSlider(Qt.Horizontal)
        self.brightness_slider.setMinimum(-100)
        self.brightness_slider.setMaximum(100)
        self.brightness_slider.setValue(brightness)
        self.brightness_slider.valueChanged.connect(self.update_calibration)
        self.controls_layout.addWidget(self.brightness_slider)

        # Contrast slider
        self.controls_layout.addWidget(QLabel("Contrast"))
        self.contrast_slider = QSlider(Qt.Horizontal)
        self.contrast_slider.setMinimum(1)   # Slider value mapped to contrast
        self.contrast_slider.setMaximum(30)
        self.contrast_slider.setValue(int(contrast * 10))
        self.contrast_slider.valueChanged.connect(self.update_calibration)
        self.controls_layout.addWidget(self.contrast_slider)

        # Simple button to demonstrate an action
        self.set_button = QPushButton("Set")
        self.set_button.clicked.connect(self.set_action)
        self.controls_layout.addWidget(self.set_button)

        # Push controls to the top of the panel
        self.controls_layout.addStretch()

        # ------------------- Right Panel (Video Display) -------------------
        self.video_label = QLabel("Video Feed")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 480)

        # Add both panels to the main horizontal layout
        self.hbox.addWidget(self.controls_widget)
        self.hbox.addWidget(self.video_label, stretch=1)

        # ------------------- Thread & Camera Setup -------------------
        self.video_thread = None
        self.caps = None
        self.feed_running = False

    def toggle_feed(self):
        """
        Starts or stops the camera thread:
          - If stopped, discovers Teslong camera indices and begins the thread.
          - If running, stops the thread and resets cameras.
        """
        if not self.feed_running:
            # Find Teslong camera(s)
            indices = get_teslong_camera_indices()
            if len(indices) == 0:
                print("No Teslong Cameras found.")
                return
            # Create captures for each discovered camera
            self.caps = [cv2.VideoCapture(idx) for idx in indices]
            # Start the video thread
            self.video_thread = VideoThread(self.caps)
            self.video_thread.changePixmap.connect(self.setImage)
            self.video_thread.start()
            self.feed_running = True
            self.start_stop_button.setText("Stop")
        else:
            # Stop the video feed if it's running
            if self.video_thread is not None:
                self.video_thread.stop()
                self.video_thread = None
            self.feed_running = False
            self.start_stop_button.setText("Start")

    def setImage(self, image):
        """
        Receives a QImage from the thread, converts it to a QPixmap,
        scales it to the label's size, and applies it to self.video_label.
        Args:
            image (QImage): The live frame already converted to RGB.
        """
        pixmap = QPixmap.fromImage(image)
        scaled_pixmap = pixmap.scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.video_label.setPixmap(scaled_pixmap)

    def update_calibration(self):
        """
        Reads slider positions and updates global gamma, brightness, contrast.
        These new values are then used for subsequent frames in combine_frames().
        """
        global gamma, brightness, contrast
        gamma = self.gamma_slider.value() / 10.0
        brightness = self.brightness_slider.value()
        contrast = self.contrast_slider.value() / 10.0

    def set_action(self):
        """
        Simple placeholder method that prints 'action' to demonstrate a button's effect.
        """
        print("action")

    def closeEvent(self, event):
        """
        Overrides the window close event to ensure the background thread is stopped.
        """
        if self.video_thread is not None:
            self.video_thread.stop()
        event.accept()

# ------------------- Main Application Entry -------------------
if __name__ == "__main__":
    # Create and launch the application
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
