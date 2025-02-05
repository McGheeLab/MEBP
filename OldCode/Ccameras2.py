import cv2
import threading
import tkinter as tk
from tkinter import Toplevel, Scale, Button, Label, HORIZONTAL, Frame
from PIL import Image, ImageTk
import numpy as np
import time

# ------------------- Global Calibration Parameters -------------------
gamma = 1.0       # Gamma correction factor
brightness = 0    # Brightness offset
contrast = 1.0    # Contrast multiplier

# Base resolution for each camera feed before any scaling
frame_width = 320
frame_height = 240

# Global variable to hold the latest composite frame (BGR numpy array)
latest_composite = None
frame_lock = threading.Lock()

# ------------------- Device Selection -------------------
def get_teslong_camera_indices():
    """
    Returns a list of camera indices whose device name contains "Teslong Camera".
    It first tries using pygrabber (on Windows) and falls back to scanning a few indices.
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
        print("pygrabber not available or error enumerating devices. Scanning indices. Error:", e)
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                indices.append(i)
                cap.release()
    return indices

# NOTE on USB Bus limitations:
# OpenCV does not provide a cross-platform method to check which USB bus a camera is attached to.
# On some systems, simultaneous capture from cameras on the same USB bus may fail.
# This code does not “fix” that limitation.

# ------------------- Image Processing Functions -------------------
def adjust_image(img, gamma_val, brightness_val, contrast_val):
    """
    Adjusts the image by modifying contrast/brightness and then applying gamma correction.
    """
    adjusted = cv2.convertScaleAbs(img, alpha=contrast_val, beta=brightness_val)
    if gamma_val <= 0:
        gamma_val = 0.1
    invGamma = 1.0 / gamma_val
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(256)]).astype("uint8")
    adjusted = cv2.LUT(adjusted, table)
    return adjusted

def overlay_grid(img):
    """
    Draws a 3x3 grid on the image. The center cell is outlined in red; the other cells in white.
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
    Reads one frame from each VideoCapture in 'caps', resizes, applies calibration adjustments,
    overlays a grid on each feed, and then horizontally concatenates the processed frames.
    """
    global gamma, brightness, contrast
    frames = []
    for cap in caps:
        ret, frame = cap.read()
        if not ret or frame is None:
            frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
        else:
            frame = cv2.resize(frame, (frame_width, frame_height))
        # Adjust brightness, contrast, gamma
        frame_adjusted = adjust_image(frame, gamma, brightness, contrast)
        # Overlay grid on each individual frame
        frame_with_grid = overlay_grid(frame_adjusted)
        frames.append(frame_with_grid)
    
    if len(frames) == 0:
        return np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
    
    # Concatenate frames side by side.
    if len(frames) > 1:
        composite = np.hstack(frames)
    else:
        composite = frames[0]
    return composite

# ------------------- Video Capture Thread -------------------
def video_thread():
    """
    Continuously grabs frames from all cameras, processes them, and updates the global
    composite image. Runs in a separate thread.
    """
    global latest_composite
    while not stop_event.is_set():
        composite = combine_frames(caps)
        with frame_lock:
            latest_composite = composite
        time.sleep(0.03)  # roughly 30 FPS

# ------------------- GUI Update Function -------------------
def update_label():
    """
    Converts the latest composite frame to a PhotoImage and resizes it to fill the video display.
    The resizing is done using OpenCV which is faster than PIL's resizing.
    """
    with frame_lock:
        if latest_composite is not None:
            # Use the dimensions of the video_frame (not the entire window) to leave space for buttons.
            w = video_frame.winfo_width()
            h = video_frame.winfo_height()
            if w > 0 and h > 0:
                # Convert BGR to RGB and then resize using OpenCV.
                rgb = cv2.cvtColor(latest_composite, cv2.COLOR_BGR2RGB)
                resized = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_AREA)
                pil_img = Image.fromarray(resized)
                tk_img = ImageTk.PhotoImage(pil_img)
                video_label.config(image=tk_img)
                video_label.image = tk_img
    root.after(30, update_label)

# ------------------- Calibration Window -------------------
def open_calibration_window():
    """
    Opens a window with sliders for gamma, brightness, and contrast.
    Changes update dynamically as the sliders are moved.
    Also includes a Set button that prints "action" to the console.
    """
    calib_win = Toplevel(root)
    calib_win.title("Calibration")

    Label(calib_win, text="Gamma").pack()
    gamma_scale = Scale(calib_win, from_=0.1, to=3.0, resolution=0.1, orient=HORIZONTAL)
    gamma_scale.set(gamma)
    gamma_scale.pack()

    Label(calib_win, text="Brightness").pack()
    brightness_scale = Scale(calib_win, from_=-100, to=100, orient=HORIZONTAL)
    brightness_scale.set(brightness)
    brightness_scale.pack()

    Label(calib_win, text="Contrast").pack()
    contrast_scale = Scale(calib_win, from_=0.1, to=3.0, resolution=0.1, orient=HORIZONTAL)
    contrast_scale.set(contrast)
    contrast_scale.pack()

    def update_calibration(*args):
        global gamma, brightness, contrast
        gamma = float(gamma_scale.get())
        brightness = int(brightness_scale.get())
        contrast = float(contrast_scale.get())
        # The video thread will pick up these changes immediately.

    # Update calibration dynamically when sliders are moved.
    gamma_scale.config(command=lambda val: update_calibration())
    brightness_scale.config(command=lambda val: update_calibration())
    contrast_scale.config(command=lambda val: update_calibration())

    # A Set button in the calibration window that prints "action"
    set_btn = Button(calib_win, text="Set", command=lambda: print("action"))
    set_btn.pack(pady=5)

# ------------------- Main Window Set Button -------------------
def set_action():
    """
    Prints "action" to the console.
    """
    print("action")

# ------------------- Exit/Cleanup -------------------
def close_app():
    """
    Stops the video thread, releases all cameras, and closes the application.
    """
    stop_event.set()
    for cap in caps:
        cap.release()
    root.destroy()

# ------------------- Main GUI Setup -------------------
root = tk.Tk()
root.title("Teslong Camera Feeds")

# Use grid layout: row 0 for the video display, row 1 for the buttons.
root.rowconfigure(0, weight=1)
root.rowconfigure(1, weight=0)
root.columnconfigure(0, weight=1)

# Video frame container (so that the video area resizes without pushing the buttons)
video_frame = Frame(root)
video_frame.grid(row=0, column=0, sticky="nsew")
video_frame.rowconfigure(0, weight=1)
video_frame.columnconfigure(0, weight=1)

# Label to display the video.
video_label = Label(video_frame)
video_label.grid(row=0, column=0, sticky="nsew")

# Button frame at the bottom.
btn_frame = Frame(root)
btn_frame.grid(row=1, column=0, sticky="ew")

close_button = Button(btn_frame, text="Close", command=close_app)
close_button.pack(side=tk.LEFT, padx=5, pady=5)

calibrate_button = Button(btn_frame, text="Calibrate", command=open_calibration_window)
calibrate_button.pack(side=tk.LEFT, padx=5, pady=5)

set_button = Button(btn_frame, text="Set", command=set_action)
set_button.pack(side=tk.LEFT, padx=5, pady=5)

# ------------------- Initialize Cameras -------------------
indices = get_teslong_camera_indices()
if len(indices) == 0:
    print("No Teslong Cameras found.")
    exit(1)

# Open a VideoCapture for each found camera index.
caps = [cv2.VideoCapture(idx) for idx in indices]

# ------------------- Start Video Capture Thread -------------------
stop_event = threading.Event()
t = threading.Thread(target=video_thread, daemon=True)
t.start()

# Start updating the GUI.
update_label()

# Start the main GUI loop.
root.mainloop()
