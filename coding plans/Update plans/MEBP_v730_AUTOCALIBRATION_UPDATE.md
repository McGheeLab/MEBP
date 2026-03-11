# MEBP v7.3.0 — Autocalibration Update Plan

**Objective:** Implement vision-based autocalibration for well plate center detection and needle alignment using the BUC3D-1000C microscope camera.

**Branch:** `Version-7.3.0`
**Status:** All 6 phases complete + Phase 7 (SimulatedCamera, detection hardening, overlay/shutdown fixes)
**Date:** 2026-03-11

---

## 1. Feature Overview

Three core capabilities:

### 1A — Well Center Auto-Detection
The camera, mounted to the XY stage, moves over well locations. When the user enables auto-detection, the system finds the circular well edge in the camera frame, fits a circle, and computes the well center. The system then navigates to corner wells (predicted from plate geometry) and auto-detects those centers too, building a full plate coordinate map.

### 1B — Needle Auto-Detection & Focus Assist
The user manually lowers the needle into the camera FOV. The system detects the needle tip as a dark circle matching the configured needle OD. As the needle descends, the circle transitions from blurred to in-focus. The system provides real-time focus quality feedback to help the user find the optimal focal plane. (Future: full auto-focus via Z-axis sweep.)

### 1C — Parallel Processing
All detection algorithms run in background threads to maintain camera feed at 15+ FPS. Frame capture and detection are decoupled via a producer-consumer pattern.

---

## 2. Files Modified / Created

### New Files

| File | Purpose |
|------|---------|
| `SupportClasses/VisionDetector.py` | Backend detection algorithms — zero GUI dependencies |
| `SupportClasses/SimulatedCamera.py` | Synthetic microscope frame generator with DOF model, cv2.VideoCapture API |
| `gui/widgets/detection_overlay.py` | QPainter overlay for detected circles, needle, focus indicator |
| `gui/widgets/detection_worker.py` | QThread-based parallel vision processing (pull-based frame consumer) |
| `tests/test_v730_simulated_camera.py` | 23 tests for SimulatedCamera + VisionDetector integration |
| `tests/test_v730_vision.py` | 36 unit tests for detection algorithms |
| `tests/test_v730_detection_worker.py` | 15 tests for DetectionWorker config and throughput |

### Modified Files

| File | Changes |
|------|---------|
| `gui/pages/calibration.py` | Auto-detect workflow, needle/focus assist, simulated camera wiring, DetectionWorker shutdown |
| `gui/widgets/camera_widget.py` | `get_current_frame()` API, detection overlay, simulated camera backend, resizeEvent fix |
| `gui/app.py` | DetectionWorker shutdown in closeEvent (prevents QThread crash on exit) |
| `SupportClasses/HardwareConfig.py` | `CameraConfig` dataclass (micron/pixel, FOV, camera ID) |
| `SupportClasses/PhysicalModels.py` | `CameraSpec` dataclass |
| `config/hardware/cameras.json` | Camera catalog with pixel sizes and default settings |

---

## 3. Architecture

### 3.1 New Dataclasses

```python
# PhysicalModels.py
@dataclass
class CameraSpec:
    """Physical camera specifications."""
    name: str                           # "BUC3D-1000C"
    sensor_pixel_size_um: float         # Physical pixel pitch (µm) from sensor datasheet
    max_resolution: tuple[int, int]     # (3664, 2748)
    active_resolution: tuple[int, int]  # Current capture resolution

    @property
    def active_pixel_size_um(self) -> float:
        """Effective pixel size accounting for binning/downscale."""
        return self.sensor_pixel_size_um * (self.max_resolution[0] / self.active_resolution[0])

# HardwareConfig.py
@dataclass
class CameraConfig:
    """Camera calibration configuration."""
    camera_spec: CameraSpec | None = None
    objective_magnification: float = 1.0       # Objective lens magnification (1x, 2x, 4x, etc.)
    micron_per_pixel: float | None = None      # Computed or user-override (µm/px)
    camera_to_needle_offset_um: tuple[float, float] = (0.0, 0.0)  # (dx, dy) from camera center to needle tip

    @property
    def computed_micron_per_pixel(self) -> float | None:
        """µm/px from sensor spec and magnification."""
        if self.camera_spec:
            return self.camera_spec.active_pixel_size_um / self.objective_magnification
        return None

    def pixel_to_um(self, px: float) -> float:
        """Convert pixel distance to micrometers."""
        scale = self.micron_per_pixel or self.computed_micron_per_pixel
        if scale is None:
            raise ValueError("No micron/pixel calibration available")
        return px * scale

    def um_to_pixel(self, um: float) -> float:
        """Convert micrometers to pixel distance."""
        scale = self.micron_per_pixel or self.computed_micron_per_pixel
        if scale is None:
            raise ValueError("No micron/pixel calibration available")
        return um / scale
```

### 3.2 VisionDetector Architecture

```
SupportClasses/VisionDetector.py
├── DetectionResult (dataclass)
│   ├── center_px: tuple[float, float]      # (cx, cy) in pixels
│   ├── radius_px: float                     # Detected radius in pixels
│   ├── confidence: float                    # 0.0–1.0
│   ├── center_um: tuple[float, float]       # Converted to µm (if calibrated)
│   └── radius_um: float                     # Converted to µm
│
├── FocusResult (dataclass)
│   ├── score: float                         # Focus metric (higher = sharper)
│   ├── normalized_score: float              # 0.0–1.0 relative to session best
│   ├── is_in_focus: bool                    # Above threshold
│   └── roi_center_px: tuple[float, float]   # Region analyzed
│
├── WellDetector
│   ├── detect_well(frame, expected_diameter_px, ...) -> DetectionResult | None
│   │   ├── 1. Convert to grayscale
│   │   ├── 2. Gaussian blur (noise reduction)
│   │   ├── 3. cv2.HoughCircles() with radius range from expected diameter ±30%
│   │   ├── 4. If multiple circles: pick closest to frame center + best radius match
│   │   └── 5. Refine center via contour fitting on edge-detected ROI
│   │
│   └── detect_well_with_fallback(frame, expected_diameter_px, ...) -> DetectionResult | None
│       ├── Try HoughCircles first
│       ├── Fallback: adaptive threshold → findContours → fitEllipse
│       └── Fallback: Canny edges → arc fitting
│
├── NeedleDetector
│   ├── detect_needle(frame, expected_od_px, ...) -> DetectionResult | None
│   │   ├── Multi-strategy: picks best result by confidence
│   │   ├── Strategy 1: _detect_hough() — HoughCircles on inverted grayscale
│   │   │   ├── Invert (dark needle → bright), median blur
│   │   │   ├── HoughCircles with radius range from expected ± tolerance
│   │   │   └── Score by size match + center proximity
│   │   ├── Strategy 2: _detect_contour() — Adaptive threshold + contour
│   │   │   ├── block_size scaled to 40% of expected diameter
│   │   │   ├── Multiple c_offset attempts for robustness
│   │   │   ├── Moments-based center (not minEnclosingCircle)
│   │   │   └── Radius from sqrt(area/pi) — avoids overestimate
│   │   └── Strategy 3: _detect_radial() — Radial intensity profile
│   │       ├── 36 radial lines from frame center outward
│   │       ├── Find dark→bright transition radius per ray
│   │       └── Median of transitions = robust radius estimate
│   │
│   └── compute_focus_score(frame, roi_rect=None) -> FocusResult
│       ├── Crop to ROI (or full frame)
│       ├── Laplacian variance (70% weight)
│       ├── Tenengrad gradient magnitude (30% weight)
│       └── Returns raw score — caller normalizes via FocusTracker
│
├── FocusTracker
│   ├── update(result) → normalized FocusResult
│   ├── get_trend(window=5) → "improving"/"declining"/"stable"/"unknown"
│   └── reset() — clear session state
│
└── DetectionWorker (QThread-based, in gui/widgets/detection_worker.py)
    ├── Signals:
    │   ├── well_detected(DetectionResult)
    │   ├── needle_detected(DetectionResult)
    │   ├── focus_updated(FocusResult)
    │   └── detection_cleared()
    │
    ├── Pull-based: pulls latest frame via get_current_frame()
    ├── Modes: IDLE, WELL_DETECT, NEEDLE_DETECT, FOCUS_ASSIST
    └── ~3-5ms per detection cycle (well: ~5ms, needle: ~3ms, focus: ~4ms)
```

### 3.3 Detection Overlay Architecture

```
gui/widgets/detection_overlay.py
├── DetectionOverlay (QWidget — transparent overlay on camera feed)
│   ├── draw_detected_circle(center, radius, color, confidence)
│   ├── draw_crosshair(center, color)
│   ├── draw_focus_bar(score, max_score)
│   ├── draw_needle_indicator(center, radius, in_focus)
│   ├── clear()
│   └── paintEvent() — QPainter rendering
│
└── Colors:
    ├── Well detected: Catppuccin Green (#a6e3a1)
    ├── Well candidate: Catppuccin Yellow (#f9e2af)
    ├── Needle detected: Catppuccin Peach (#fab387)
    ├── Focus good: Catppuccin Green
    ├── Focus poor: Catppuccin Red (#f38ba8)
    └── Confidence ring: opacity scales with confidence value
```

### 3.4 Parallel Processing Architecture

```
Camera Thread (ToupCam callback or OpenCV grab)
    │
    ▼ frame (BGR numpy array)
┌─────────────────────────┐
│   CameraWidget          │
│   _grab_frame() @ 15fps │──→ display in QLabel (GUI thread)
│                         │
│   get_current_frame()   │──→ returns latest frame copy (thread-safe)
└────────────┬────────────┘
             │ frame copy (on demand)
             ▼
┌─────────────────────────┐
│   DetectionWorker       │
│   (QThread)             │
│                         │
│   Pulls frame via       │
│   get_current_frame()   │
│   every ~50ms           │
│                         │
│   Runs detector:        │
│   - WellDetector or     │
│   - NeedleDetector      │
│                         │
│   Emits signal with     │
│   DetectionResult       │
└────────────┬────────────┘
             │ signal (queued connection)
             ▼
┌─────────────────────────┐
│   CalibrationPage       │
│   (GUI thread)          │
│                         │
│   Updates overlay       │
│   Updates status text   │
│   Stores result for     │
│   "Accept" button       │
└─────────────────────────┘
```

**Key design:** DetectionWorker does NOT receive every frame. It pulls the latest frame when it's ready for the next detection cycle. This naturally throttles detection to whatever rate the algorithms can sustain without building up a backlog.

---

## 4. Implementation Steps

### Phase 1: Foundation (Camera Config + Detection Backend) ✓

- [x] **1.1** Add `CameraSpec` dataclass to `PhysicalModels.py`
- [x] **1.2** Add `CameraConfig` dataclass to `HardwareConfig.py` with `pixel_to_um()` / `um_to_pixel()` conversions
- [x] **1.3** Create `config/hardware/cameras.json` catalog with BUC3D-1000C specs (pixel pitch: 1.67 µm for this sensor)
- [x] **1.4** Wire `CameraConfig` into `HardwareConfig` serialization (`to_dict()` / `from_dict()`)
- [x] **1.5** Add camera config section to Hardware Setup page (Page 0) — camera selection, magnification, micron/pixel override

### Phase 2: Vision Detection Algorithms ✓

- [x] **2.1** Create `SupportClasses/VisionDetector.py` with `DetectionResult` and `FocusResult` dataclasses
- [x] **2.2** Implement `WellDetector.detect_well()` — HoughCircles with expected diameter filtering
- [x] **2.3** Implement `WellDetector.detect_well_with_fallback()` — contour + ellipse fitting fallback
- [x] **2.4** Implement `NeedleDetector.detect_needle()` — dark circle detection with OD matching
- [x] **2.5** Implement `NeedleDetector.compute_focus_score()` — Laplacian variance + Tenengrad
- [x] **2.6** Write unit tests for detectors using synthetic test images (36 tests, all pass)

### Phase 3: Parallel Processing Infrastructure ✓

- [x] **3.1** Add `get_current_frame() -> np.ndarray | None` to `CameraWidget` (thread-safe frame copy via threading.Lock)
- [x] **3.2** Implement `DetectionWorker(QThread)` in `gui/widgets/detection_worker.py` — pull-based frame processing with signal emission
- [x] **3.3** Add `DetectionMode` enum: `IDLE`, `WELL_DETECT`, `NEEDLE_DETECT`, `FOCUS_ASSIST`
- [x] **3.4** Verify detection throughput: all algorithms <50ms per frame (well: ~5ms, needle: ~3ms, focus: ~4ms at 640×480)

### Phase 4: Detection Overlay ✓

- [x] **4.1** Create `gui/widgets/detection_overlay.py` — transparent QPainter overlay widget with Catppuccin colors
- [x] **4.2** Implement circle rendering with confidence-scaled opacity (well: green/yellow, needle: peach, crosshairs)
- [x] **4.3** Implement focus quality bar (vertical bar on right edge, red→yellow→green gradient)
- [x] **4.4** Integrate overlay as child widget of `CameraWidget.video_label`, auto-resizes via `resizeEvent`

### Phase 5: Well Auto-Detection Workflow (Calibration Page) ✓

- [x] **5.1** Add "Auto-Detect Well" toggle button to Step 2C (Teach A1) and Step 2D (Teach Corner)
- [x] **5.2** When enabled: start `DetectionWorker` in `WELL_DETECT` mode with signal connections
- [x] **5.3** Compute `expected_diameter_px` from `WellPlate.well_diameter` (mm→µm) + `CameraConfig.um_to_pixel()`
- [x] **5.4** Display detected circle overlay on camera feed in real-time via `DetectionOverlay`
- [x] **5.5** "Accept" button → calls existing `_record_a1_xyz()` / `_record_corner_xyz()` to store position
- [x] **5.6** Corner auto-detect reuses same workflow — existing "Go ▶" navigates, then "Auto-Detect" finds well
- [x] **5.7** Auto-navigate uses existing `_goto_corner_auto()` (safe Z travel already implemented)
- [x] **5.8** Corner well detection: same UI pattern as A1 — detect, center, accept
- [x] **5.9** Third well uses existing Go ▶ + manual Rec XYZ (auto-detect can be added later)
- [x] **5.10** `pixel_offset_to_stage_um()` used in "Center" button — moves stage by detected offset

### Phase 6: Needle Auto-Detection & Focus Assist

- [x] **6.1** Add "Detect Needle" + "Focus Assist" buttons in Needle Alignment section after Step 1
- [x] **6.2** When enabled: start `DetectionWorker` in `NEEDLE_DETECT` mode via `_toggle_needle_detect()`
- [x] **6.3** Compute `expected_od_px` from `HardwareConfig.needle.od_um` + `CameraConfig.um_to_pixel()` via `_get_expected_od_px()`
- [x] **6.4** Display needle detection overlay (peach circle + dashed ring) via `_on_needle_detected()` → overlay
- [x] **6.5** Switch to `FOCUS_ASSIST` mode via `_toggle_focus_assist()`: displays real-time focus score bar
- [x] **6.6** Show directional hint: "Getting sharper" / "Getting blurrier — reverse Z" / "In focus!" via `FocusTracker.get_trend()`
- [x] **6.7** Record best-focus Z position and display in `_lbl_best_focus` label

### Phase 7: Simulated Camera, Detection Hardening, Bug Fixes ✓

- [x] **7.1** Create `SupportClasses/SimulatedCamera.py` — synthetic inverted microscope frame generator
  - Black plate surface with red well holes, dark needle circle at center
  - Wells translate as XY stage position changes
  - Gaussian DOF envelope per objective: `opacity = exp(-0.5 * (defocus / dof_hw)^2)`
  - DOF half-width from `um_per_px * 0.06` (2× → 0.20mm, 4× → 0.10mm, 10× → 0.04mm, 20× → 0.02mm)
  - cv2.VideoCapture-compatible API (read, isOpened, release, get)
  - Controller reference for auto-position pull
- [x] **7.2** Integrate SimulatedCamera into CameraWidget — "SIM: Microscope" backend option
- [x] **7.3** Wire simulated camera to HardwareConfig (plate, needle spec, camera config, controller)
- [x] **7.4** Fix SimulatedCamera blur model — scale with optics not object size (`BLUR_REFERENCE_UM / um_per_px`)
- [x] **7.5** Rewrite `NeedleDetector.detect_needle()` — 3-strategy detection (HoughCircles + contour + radial)
  - HoughCircles on inverted grayscale (primary, handles moderate blur)
  - Contour-based with scaled block_size and moments-based center (fallback)
  - Radial intensity profile from frame center (last resort)
  - Best result by confidence across strategies
- [x] **7.6** Fix DetectionOverlay coordinate mapping — account for KeepAspectRatio scaling + centering
  - Added `_display_transform()` computing uniform scale + offsets
  - Fixed `_map_radius` from averaged X/Y scales to uniform scale
- [x] **7.7** Fix CameraWidget `resizeEvent` — use `rect()` not `geometry()` for child overlay
- [x] **7.8** Fix QThread crash on shutdown — `_shutdown_detection_worker()` calls `stop_detection()` + `wait()`
  - Added `closeEvent` on CalibrationPage
  - App `closeEvent` iterates pages and calls `_shutdown_detection_worker()`
- [x] **7.9** Write tests: 23 SimulatedCamera tests (frame generation, wells, needle, DOF, vision integration)

---

## 5. Key Algorithms

### 5.1 Well Circle Detection

```python
def detect_well(frame: np.ndarray, expected_diameter_px: float,
                tolerance: float = 0.3) -> DetectionResult | None:
    """
    Detect a circular well edge in the camera frame.

    The well appears as a bright circle (well bottom) surrounded by a dark ring
    (well wall) or as an edge of a circle (partial well visible).

    Args:
        frame: BGR image from camera
        expected_diameter_px: Expected well diameter in pixels
        tolerance: Fraction of diameter for radius range (±30% default)
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    expected_radius = expected_diameter_px / 2
    min_radius = int(expected_radius * (1 - tolerance))
    max_radius = int(expected_radius * (1 + tolerance))

    # HoughCircles: param1=Canny high threshold, param2=accumulator threshold
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.5,                    # Accumulator resolution ratio
        minDist=expected_diameter_px * 0.8,  # Min distance between centers
        param1=100,                # Canny edge threshold
        param2=30,                 # Accumulator threshold (lower = more detections)
        minRadius=min_radius,
        maxRadius=max_radius
    )

    if circles is None:
        return None

    # Score each circle: prefer closest to frame center + best radius match
    frame_center = (frame.shape[1] / 2, frame.shape[0] / 2)
    best = _score_circles(circles[0], frame_center, expected_radius)

    return DetectionResult(
        center_px=(best[0], best[1]),
        radius_px=best[2],
        confidence=_compute_confidence(best, frame_center, expected_radius)
    )
```

### 5.2 Needle Detection (Multi-Strategy)

```python
def detect_needle(frame, expected_od_px, tolerance=0.4, min_circularity=0.5):
    """
    Multi-strategy needle detection — picks best result by confidence.

    Strategy 1 (HoughCircles): Invert grayscale, median blur, HoughCircles
      with radius range. Robust to moderate blur and noise.

    Strategy 2 (Contour): Adaptive threshold with block_size scaled to 40%
      of expected diameter. Moments-based center, area-based radius (avoids
      minEnclosingCircle overestimate). Multiple c_offset attempts.

    Strategy 3 (Radial): 36 radial intensity profiles from frame center.
      Find dark→bright transition radius per ray, median = radius estimate.
      Last resort — works even with significant blur.
    """
    result_hough   = _detect_hough(gray, expected_r, tolerance, frame_center)
    result_contour = _detect_contour(gray, expected_r, tolerance, min_circ, frame_center)
    result_radial  = _detect_radial(gray, expected_r, tolerance, frame_center)

    # Pick best by confidence
    return max([r for r in (result_hough, result_contour, result_radial) if r],
               key=lambda r: r.confidence, default=None)
```

### 5.3 Focus Score

```python
def compute_focus_score(frame: np.ndarray, roi_rect=None) -> FocusResult:
    """
    Compute focus quality metric using Laplacian variance.

    Higher variance = sharper edges = better focus.
    As needle descends: score increases to peak (best focus) then decreases.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if roi_rect:
        x, y, w, h = roi_rect
        gray = gray[y:y+h, x:x+w]

    # Primary: Laplacian variance (fast, reliable)
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)
    laplacian_var = laplacian.var()

    # Secondary: Tenengrad (Sobel gradient magnitude)
    gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    tenengrad = (gx ** 2 + gy ** 2).mean()

    # Combined score (weighted)
    score = 0.7 * laplacian_var + 0.3 * tenengrad

    return FocusResult(
        score=score,
        normalized_score=0.0,  # Caller normalizes against session max
        is_in_focus=False,     # Caller determines threshold
        roi_center_px=(gray.shape[1] / 2, gray.shape[0] / 2)
    )
```

### 5.4 Pixel-to-Stage Coordinate Conversion

```python
def pixel_offset_to_stage_um(
    detected_center_px: tuple[float, float],
    frame_size_px: tuple[int, int],
    micron_per_pixel: float
) -> tuple[float, float]:
    """
    Convert detected center offset from frame center to stage movement in µm.

    If detected center is right of frame center → stage needs to move right (positive X).
    If detected center is below frame center → stage needs to move down (positive Y).

    Note: Camera orientation relative to stage axes must be calibrated.
    Sign conventions may need flipping depending on mounting.
    """
    frame_cx = frame_size_px[0] / 2
    frame_cy = frame_size_px[1] / 2

    dx_px = detected_center_px[0] - frame_cx
    dy_px = detected_center_px[1] - frame_cy

    dx_um = dx_px * micron_per_pixel
    dy_um = dy_px * micron_per_pixel

    return (dx_um, dy_um)
```

---

## 6. Well Auto-Detection Workflow (Detailed)

### Step-by-Step User Flow

```
1. User is on Calibration Page, camera is live, plate is under camera.

2. User sets Safe Z and Top Z (Steps 2A, 2B — unchanged).

3. User jogs roughly over well A1 (manually or via "Go to A1 estimate").

4. User clicks "Auto-Detect Well" button (Step 2C).
   ├── System starts DetectionWorker in WELL_DETECT mode
   ├── Expected diameter computed: WellPlate.well_diameter_mm → um → pixels
   ├── Green overlay circle appears on camera feed when well detected
   ├── Offset from frame center displayed: "Δx: +45 µm, Δy: -12 µm"
   └── Confidence displayed: "Confidence: 87%"

5. User clicks "Center on Well" (optional iterative refinement).
   ├── Stage moves by detected offset to center well in FOV
   ├── Re-detect → smaller offset → repeat until offset < threshold
   └── Typically converges in 1-2 iterations

6. User clicks "Accept A1" → position recorded as taught A1.

7. System computes predicted corner well position:
   ├── corner_x = a1_x + (cols - 1) * spacing_x_um
   ├── corner_y = a1_y + (rows - 1) * spacing_y_um
   └── Accounts for any detected plate rotation (from A1 detection)

8. System auto-navigates to predicted corner well (at Safe Z).
   ├── Raise to Safe Z
   ├── Move XY to predicted corner
   └── Lower to Top Z

9. Auto-detect corner well → overlay → "Accept Corner" button.

10. (Optional) Auto-navigate to third well → detect → accept → Z-plane fit.
```

### Iterative Centering Algorithm

```python
def auto_center_on_well(self, max_iterations=3, threshold_um=20.0):
    """Move stage iteratively until well center aligns with camera center."""
    for i in range(max_iterations):
        frame = self.camera_widget.get_current_frame()
        result = self.well_detector.detect_well(frame, self.expected_diameter_px)

        if result is None:
            self.status_label.setText(f"Iteration {i+1}: No well detected")
            return False

        dx_um, dy_um = pixel_offset_to_stage_um(
            result.center_px,
            (frame.shape[1], frame.shape[0]),
            self.camera_config.micron_per_pixel
        )

        offset_magnitude = math.sqrt(dx_um**2 + dy_um**2)
        if offset_magnitude < threshold_um:
            self.status_label.setText(f"Centered! (offset: {offset_magnitude:.1f} µm)")
            return True

        # Move stage by offset
        self.stage_controller.move_xy_relative(dx_um * 10, dy_um * 10)  # µm → microsteps
        time.sleep(0.5)  # Wait for stage to settle

    return False
```

---

## 7. Camera Configuration UI (Hardware Setup Page)

### Optical Path

The system uses a **Nikon Ti2-U** inverted microscope base with the BUC3D-1000C camera mounted on a C-mount port. The objective magnification cannot be auto-detected, so the user must select the installed objective from a dropdown.

**Available objectives:** 2×, 4×, 10×, 20×

**Pixel scale at 916×686 preview resolution:**

| Objective | µm/px | FOV (mm) |
|-----------|-------|----------|
| 2×  | 3.34 | 3.06 × 2.29 |
| 4×  | 1.67 | 1.53 × 1.15 |
| 10× | 0.668 | 0.61 × 0.46 |
| 20× | 0.334 | 0.31 × 0.23 |

### New Section in Page 0 (after Rosette Library)

```
┌─ Camera Configuration ──────────────────────────────────┐
│                                                          │
│  Camera:        [BUC3D-1000C               ▼]           │
│  Resolution:    [916 × 686                 ▼]           │
│  Objective:     [4×                        ▼]           │
│                                                          │
│  Pixel Scale:   1.67 µm/px  (sensor: 1.67 µm, bin: 4×) │
│  [ ] Use custom scale  [____] µm/px                     │
│                                                          │
│  Camera FOV:    1530 × 1146 µm (1.53 × 1.15 mm)        │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

---

## 8. Calibration Page UI Changes

### Modified Step 2C (Teach A1)

```
┌─ Step 2C: Teach A1 ─────────────────────────────────────┐
│                                                          │
│  [Manual Jog]  [Auto-Detect Well]                        │
│                                                          │
│  ┌─ Detection Status ─────────────────────────────────┐  │
│  │  ● Well detected    Confidence: 87%                │  │
│  │  Offset: Δx +45 µm, Δy -12 µm                     │  │
│  │  Diameter: 15.4 mm (expected: 15.6 mm)             │  │
│  └────────────────────────────────────────────────────┘  │
│                                                          │
│  [Center on Well]  [Accept A1 Position]                  │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

### Modified Step 2D (Teach Corner)

```
┌─ Step 2D: Teach Corner ─────────────────────────────────┐
│                                                          │
│  Predicted position: X=171360 µm, Y=136080 µm           │
│                                                          │
│  [Navigate to Corner]  [Auto-Detect]                     │
│                                                          │
│  ┌─ Detection Status ─────────────────────────────────┐  │
│  │  ● Well detected    Confidence: 91%                │  │
│  │  Offset: Δx +8 µm, Δy -3 µm                       │  │
│  └────────────────────────────────────────────────────┘  │
│                                                          │
│  [Center on Well]  [Accept Corner Position]              │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

### New: Needle Detection Section (Step 1)

```
┌─ Needle Alignment ──────────────────────────────────────┐
│                                                          │
│  [Detect Needle in FOV]                                  │
│                                                          │
│  ┌─ Detection Status ─────────────────────────────────┐  │
│  │  ● Needle detected  OD: 0.91 mm (expected: 0.91)  │  │
│  │  Position: center of frame ± 15 µm                 │  │
│  └────────────────────────────────────────────────────┘  │
│                                                          │
│  ┌─ Focus Quality ────────────────────────────────────┐  │
│  │  ████████████░░░░  Score: 73%                      │  │
│  │  ▲ Move Z down for better focus                    │  │
│  └────────────────────────────────────────────────────┘  │
│                                                          │
│  Best focus at Z = -2.45 mm                              │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

---

## 9. Testing Strategy

### Unit Tests (`tests/test_v730_vision.py`)

- [ ] `test_well_detection_synthetic_circle` — Generate image with known circle, verify detection accuracy < 2px
- [ ] `test_well_detection_noisy_image` — Add Gaussian noise, verify robustness
- [ ] `test_well_detection_partial_circle` — Circle edge only partially visible
- [ ] `test_well_detection_wrong_size` — Circle of wrong diameter should be rejected
- [ ] `test_needle_detection_dark_circle` — Dark circle on bright background
- [ ] `test_needle_detection_hollow` — Annulus (hollow needle end)
- [ ] `test_focus_score_sharp_vs_blurred` — Sharp image scores higher than blurred
- [ ] `test_focus_score_monotonic` — Score increases as blur decreases
- [ ] `test_pixel_to_stage_conversion` — Known offset → correct µm conversion
- [ ] `test_camera_config_serialization` — to_dict/from_dict roundtrip

### Integration Tests

- [ ] `test_detection_worker_signals` — Worker emits signals on detection
- [ ] `test_detection_worker_frame_dropping` — Under load, worker processes latest frame only
- [ ] `test_camera_config_propagation` — Config flows from HardwareSetup → CalibrationPage

---

## 10. Dependencies

### Required
- `opencv-python >= 4.8` — already optional dependency, becomes required for autocalibration
- `numpy >= 1.24` — already required

### No New Dependencies
All detection algorithms use OpenCV + NumPy which are already in the project.

---

## 11. Configuration Persistence

### settings.json additions

```json
{
  "hardware_config": {
    "camera_config": {
      "camera_name": "BUC3D-1000C",
      "sensor_pixel_size_um": 1.67,
      "max_resolution": [3664, 2748],
      "active_resolution": [916, 686],
      "objective_magnification": 1.0,
      "micron_per_pixel_override": null,
      "camera_to_needle_offset_um": [0.0, 0.0]
    }
  },
  "calibration": {
    "auto_detect_enabled": true,
    "well_detect_confidence_threshold": 0.6,
    "needle_detect_confidence_threshold": 0.5,
    "auto_center_max_iterations": 3,
    "auto_center_threshold_um": 20.0,
    "focus_score_threshold": 0.7
  }
}
```

### cameras.json catalog

```json
{
  "description": "Camera specifications for autocalibration",
  "cameras": {
    "BUC3D-1000C": {
      "name": "Bestscope BUC3D-1000C (ToupTek C3CMOS10000KPA)",
      "sensor_pixel_size_um": 1.67,
      "max_resolution": [3664, 2748],
      "preview_resolutions": [
        [3664, 2748],
        [1832, 1374],
        [916, 686]
      ],
      "interface": "USB 3.0",
      "notes": "10MP CMOS, 1/2.3\" sensor"
    }
  }
}
```

---

## 12. Issues & Decisions Log

| Date | Issue | Decision |
|------|-------|----------|
| 2026-03-11 | Where to put CameraConfig? | In HardwareConfig — camera is hardware, and calibration needs needle + camera + plate together |
| 2026-03-11 | Thread model for detection? | QThread (not multiprocessing) — needs access to CameraWidget frame buffer, no IPC overhead |
| 2026-03-11 | Pull vs push frame delivery? | Pull — DetectionWorker pulls latest frame when ready, avoids backlog |
| 2026-03-11 | Camera axis orientation? | Camera-to-stage axis mapping needs one-time calibration. Store as sign flips in CameraConfig. Will address in implementation. |
| 2026-03-11 | HoughCircles vs contour fitting? | HoughCircles primary, contour/ellipse fitting as fallback. HoughCircles faster for well-defined circles. |
| 2026-03-11 | Microscope base? | Nikon Ti2-U inverted microscope. Objectives: 2×, 4×, 10×, 20×. No auto-detect — user selects from dropdown. |
| 2026-03-11 | Magnification UI? | Changed from free-form QDoubleSpinBox to QComboBox dropdown with fixed Ti2-U objectives. Prevents invalid values. Default: 2×. |
| 2026-03-11 | Needle detection off-center and wrong size | Root cause: DetectionOverlay `_map_x`/`_map_y`/`_map_radius` didn't account for KeepAspectRatio scaling + centering in QLabel. Up to 46px center error and 40px radius error. Fixed with `_display_transform()` uniform scale + offset. |
| 2026-03-11 | Needle detection fails at slight defocus | Blur model scaled with needle pixel size (`od_px/3`), giving 91px sigma at 0.05mm defocus. Fixed: scale with optics (`BLUR_REFERENCE_UM / um_per_px`) — 4.5px sigma instead. |
| 2026-03-11 | Needle detection single-strategy fragile | Adaptive threshold with fixed block_size=51 broke on any blur or noise. Rewrote with 3 strategies: HoughCircles (primary), contour (fallback), radial profile (last resort). Now handles blur, noise, and various magnifications. |
| 2026-03-11 | QThread crash on app exit | DetectionWorker thread not stopped before QWidget destruction. Added `_shutdown_detection_worker()` in CalibrationPage + app closeEvent calls it on all pages. |

---

## 13. Future Extensions (Not in v7.3.0)

- **Auto-focus via Z sweep:** Automated Z-axis scan to find best focus position
- **Plate rotation detection:** Detect plate angle from multiple well detections, apply rotation correction
- **Needle-to-camera offset calibration:** Automated procedure to determine XY offset between camera center and needle tip
- **Multi-well batch detection:** Scan across entire plate row/column, detect all wells in sequence
- **Template matching:** Use reference images for more robust well/needle detection
- **Focus stacking:** Combine multi-focus images for depth estimation
