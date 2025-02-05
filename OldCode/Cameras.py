import cv2
import threading
import tkinter as tk
from tkinter import Toplevel, Scale, HORIZONTAL, Button, Label
from PIL import Image, ImageTk
import numpy as np
import time

# Global calibration parameters
gamma = 1.0       # Default gamma value
brightness = 0    # Default brightness offset
contrast = 1.0    # Default contrast multiplier

# Desired frame size for each camera image (will be resized to this)
frame_width = 320
frame_height = 240

def get_teslong_camera_indices():
    """
    Returns a list of camera indices whose device name contains "Teslong Camera".
    On Windows, it uses pygrabber to list devices; if that fails it simply
    tries opening a few indices.
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
        print("pygrabber not available or error enumerating devices, scanning indices. Error:", e)
        # Fallback: try a few indices
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                indices.append(i)
                cap.release()
    return indices

def combine_frames(caps):
    """
    Reads one frame from each VideoCapture object in `caps`,
    resizes it to a fixed size, and tiles them in a grid.
    If a camera fails to produce a frame, a black image is substituted.
    """
    frames = []
    for cap in caps:
        ret, frame = cap.read()
        if not ret or frame is None:
            # Use a black image if the capture fails
            frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
        else:
            frame = cv2.resize(frame, (frame_width, frame_height))
        frames.append(frame)
        
    if len(frames) == 0:
        return np.zeros((frame_height, frame_width, 3), dtype=np.uint8)

    # Decide grid dimensions (roughly square)
    n = len(frames)
    rows = int(np.ceil(np.sqrt(n)))
    cols = int(np.ceil(n / rows))

    # Create a blank image to hold all frames
    combined_img = np.zeros((rows * frame_height, cols * frame_width, 3), dtype=np.uint8)
    for idx, frame in enumerate(frames):
        r = idx // cols
        c = idx % cols
        combined_img[r*frame_height:(r+1)*frame_height,
                     c*frame_width:(c+1)*frame_width] = frame
    return combined_img

def adjust_image(img, gamma_val, brightness_val, contrast_val):
    """
    First adjusts brightness and contrast, then applies gamma correction.
    (Note: brightness is added and contrast multiplies pixel values.)
    """
    # Apply contrast and brightness adjustment.
    adjusted = cv2.convertScaleAbs(img, alpha=contrast_val, beta=brightness_val)
    # Avoid division by zero for gamma
    if gamma_val <= 0:
        gamma_val = 0.1
    invGamma = 1.0 / gamma_val
    table = np.array([((i / 255.0) ** invGamma) * 255
                      for i in np.arange(256)]).astype("uint8")
    adjusted = cv2.LUT(adjusted, table)
    return adjusted

def overlay_grid(img):
    """
    Draws a 3x3 grid over the image. The center cell gets a red outline,
    while the other cells are outlined in white.
    """
    h, w, _ = img.shape
    cell_w = w / 3
    cell_h = h / 3
    for i in range(3):
        for j in range(3):
            pt1 = (int(j * cell_w), int(i * cell_h))
            pt2 = (int((j + 1) * cell_w), int((i + 1) * cell_h))
            # Center cell gets a red outline (BGR: 0,0,255)
            if i == 1 and j == 1:
                color = (0, 0, 255)
            else:
                color = (255, 255, 255)
            cv2.rectangle(img, pt1, pt2, color, 2)
    return img

# Global variable for the latest frame to display
latest_frame = None
frame_lock = threading.Lock()

def video_thread():
    """
    This thread continuously grabs frames from all camera feeds, combines them,
    applies the adjustments and grid overlay, and saves the result as a Tk image.
    """
    global latest_frame, gamma, brightness, contrast
    while not stop_event.is_set():
        combined = combine_frames(caps)
        adjusted = adjust_image(combined, gamma, brightness, contrast)
        with_grid = overlay_grid(adjusted)
        # Convert from BGR to RGB for PIL
        rgb = cv2.cvtColor(with_grid, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb)
        tk_img = ImageTk.PhotoImage(image=pil_img)
        with frame_lock:
            latest_frame = tk_img
        time.sleep(0.03)  # roughly 30 FPS

def update_label():
    """
    Periodically updates the Tkinter label with the latest frame image.
    """
    with frame_lock:
        if latest_frame is not None:
            video_label.config(image=latest_frame)
            video_label.image = latest_frame
    root.after(30, update_label)

def open_calibration_window():
    """
    Opens a small window with three sliders (gamma, brightness, contrast)
    and a "Set" button. When "Set" is pressed the new values are stored.
    """
    calib_win = Toplevel(root)
    calib_win.title("Calibration")

    tk.Label(calib_win, text="Gamma").pack()
    gamma_scale = Scale(calib_win, from_=0.1, to=3.0, resolution=0.1, orient=HORIZONTAL)
    gamma_scale.set(gamma)
    gamma_scale.pack()

    tk.Label(calib_win, text="Brightness").pack()
    brightness_scale = Scale(calib_win, from_=-100, to=100, orient=HORIZONTAL)
    brightness_scale.set(brightness)
    brightness_scale.pack()

    tk.Label(calib_win, text="Contrast").pack()
    contrast_scale = Scale(calib_win, from_=0.1, to=3.0, resolution=0.1, orient=HORIZONTAL)
    contrast_scale.set(contrast)
    contrast_scale.pack()

    def set_values():
        global gamma, brightness, contrast
        gamma = gamma_scale.get()
        brightness = brightness_scale.get()
        contrast = contrast_scale.get()
        calib_win.destroy()

    set_button = Button(calib_win, text="Set", command=set_values)
    set_button.pack(pady=5)

def close_app():
    """
    Stops the video thread, releases all camera captures, and closes the window.
    """
    stop_event.set()
    for cap in caps:
        cap.release()
    root.destroy()

# ----------------------- Main GUI Setup -----------------------
root = tk.Tk()
root.title("Teslong Camera Feeds")

# Label to show the video image
video_label = Label(root)
video_label.pack()

# Frame for the control buttons
btn_frame = tk.Frame(root)
btn_frame.pack(pady=5)

# Button to close the camera feed and exit
close_button = Button(btn_frame, text="Close", command=close_app)
close_button.pack(side=tk.LEFT, padx=5)

# Button to open the calibration window (for adjusting gamma, brightness, contrast)
calibrate_button = Button(btn_frame, text="Calibrate", command=open_calibration_window)
calibrate_button.pack(side=tk.LEFT, padx=5)

# ----------------------- Setup Cameras and Thread -----------------------

# Get indices of all Teslong Cameras
indices = get_teslong_camera_indices()
if len(indices) == 0:
    print("No Teslong Cameras found.")
    exit(1)

# Open a VideoCapture for each camera index found
caps = [cv2.VideoCapture(idx) for idx in indices]

# Create an event to signal the video thread to stop
stop_event = threading.Event()

# Start the video capture thread (daemon so it closes with the app)
t = threading.Thread(target=video_thread, daemon=True)
t.start()

# Schedule regular updates of the Tk image
update_label()

# Start the GUI event loop
root.mainloop()
