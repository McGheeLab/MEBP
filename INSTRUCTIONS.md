# MEBP Bioprinter — Operating Instructions

**Version 7.1** — Complete guide for setting up and operating the Multi-Extruder Bioprinting Platform.

---

## Table of Contents

1. [First-Time Setup](#1-first-time-setup)
2. [Connecting Hardware](#2-connecting-hardware)
3. [Dashboard Overview](#3-dashboard-overview)
4. [Manual Jogging](#4-manual-jogging)
5. [Calibration Procedure](#5-calibration-procedure)
6. [Creating a Print Job](#6-creating-a-print-job)
7. [Running a Print](#7-running-a-print)
8. [Print Monitor](#8-print-monitor)
9. [Multi-Material Printing](#9-multi-material-printing)
10. [Print Recording & Replay](#10-print-recording--replay)
11. [Settings & Configuration](#11-settings--configuration)
12. [Xbox Controller Setup](#12-xbox-controller-setup)
13. [Keyboard Shortcuts](#13-keyboard-shortcuts)
14. [Troubleshooting](#14-troubleshooting)
15. [Safety Guidelines](#15-safety-guidelines)

---

## 1. First-Time Setup

### Installing Dependencies

Open a terminal and run:

```bash
pip install PySide6 pyserial pygame numpy
```

For camera support during calibration (optional):

```bash
pip install opencv-python
```

### Launching the Application

**Simulation mode** (no hardware required — good for learning the interface):

```bash
python main.py
```

**With real hardware** (connect XY stage and ZP board first):

```bash
python main.py --real-xy --real-zp
```

**With verbose logging** (shows all serial communication):

```bash
python main.py --real-xy --real-zp --verbose
```

The application window opens with the Dashboard page. The sidebar on the left provides navigation between pages.

### Understanding the Interface

The interface has four main regions:

- **Left sidebar** — Page navigation icons. Click an icon to switch pages. Click ≡ to expand the sidebar and see page names.
- **Context panel** — Slides out from the left with page-specific settings. Click ☰ in the top-right corner to toggle it, or click ✕ to close.
- **Main content area** — The active page content.
- **Bottom status bar** — Always-visible readouts: XY position, Z/pump positions, speed, safety status.

Connection status dots in the top bar show which hardware is connected:
- 🟢 Green = connected
- 🔴 Red/gray = disconnected

---

## 2. Connecting Hardware

### Automatic Connection

1. Navigate to the **Dashboard** page (📊).
2. Open the context panel (☰ button or it opens automatically).
3. Click **Connect XY** to connect the Prior ProScan stage.
4. Click **Connect ZP** to connect the Marlin 3D printer board.
5. The connection dots in the top bar turn green when successful.

MEBP automatically scans serial ports and identifies the correct hardware by querying firmware version strings.

### Controller Auto-Detection (v7.1)

If you have a ProScan II instead of a ProScan III:

1. Go to the **Settings** page (⚙️).
2. In the Controller card, select **Auto-Detect** from the dropdown.
3. Click the **Auto-Detect** button.
4. MEBP will probe the connected stage and identify the controller model.
5. Click **Apply** to save.

### Manual Port Selection

If auto-detection fails:

1. Go to **Settings** → Connection card.
2. Click **Refresh Ports** to scan available serial ports.
3. Note the port names (e.g., COM3, /dev/ttyUSB0).
4. The correct ports will be used automatically on the next connect attempt.

### Simulation Mode

You can mix real and simulated hardware:

- In **Settings**, check/uncheck **Simulate XY** or **Simulate ZP**.
- Changes require an application restart to take effect.
- Simulated stages respond to all commands with realistic physics simulation.

---

## 3. Dashboard Overview

The Dashboard provides a read-only overview of the entire system:

- **XY Position** — Current stage position in microsteps, with zero-referenced coordinates.
- **ZP Position** — Z needle height and P1/P2/P3 syringe pump positions in mm.
- **Speeds** — Current XY, Z, and pump speed multipliers.
- **Zero Reference** — The calibrated zero point (set during calibration).
- **Safety Limits** — Current software endstop values and enable status.
- **Print History** — Statistics from past prints (count, success rate, total time).

### Context Panel

The Dashboard context panel provides:

- **Connect/Disconnect** buttons for XY, ZP, and Xbox controller.
- **Xbox Mapping Editor** button (opens a dialog to customize controller mapping).
- **Position Log** count and export buttons (CSV/JSON).
- **Print History** export and clear buttons.

---

## 4. Manual Jogging

The Jog Control page (🕹️) allows manual movement of all axes.

### Direction Pad

The on-screen direction pad has arrow buttons for XY movement. Each click moves the stage by the selected step size.

**XY Step Sizes**: 10, 50, 100, 500, 1000, 5000, 10000 microsteps

Click a step size button to select it, then use the direction pad.

### Z and Pump Controls

- **Z Up/Down** — Move the needle vertically.
- **P1/P2/P3 Dispense/Aspirate** — Move individual syringe pumps (dispense = push fluid out; aspirate = draw fluid in).

**Z/Pump Step Sizes**: 0.01, 0.05, 0.1, 0.5, 1.0, 5.0 mm

### Speed Sliders (Context Panel)

Open the context panel to access speed multipliers:

- **XY Speed** — Controls stage travel speed.
- **Z Speed** — Controls needle vertical speed.
- **Pump Speed** — Controls syringe pump feed rate.

### Quick Actions (Context Panel)

- **Home All** — Move XY to zero and Z to zero.
- **Zero All** — Set the current position as the new zero reference.
- **E-Stop** — Emergency stop (sends M112 to Marlin board).

---

## 5. Calibration Procedure

The Calibration page (📐) guides you through a 3-step process to establish the coordinate system.

### Step 1: Zero Needle

1. Using the Jog page or Xbox controller, position the needle tip at the desired zero point.
2. This should be a known reference position — typically the center of well A1 at the plate surface.
3. Click **Set Zero** to establish this position as the coordinate origin.
4. All subsequent positions will be relative to this point.

### Step 2: Teach Plate

1. Click **Start Plate Teaching**.
2. Jog the needle to well **A1** center. Click **Record A1**.
3. Jog to the **diagonal corner** well (e.g., H12 for a 96-well plate). Click **Record Corner**.
4. MEBP computes the plate orientation, scale, and rotation from these two points.
5. Review the computed values (scale factor, rotation angle) to verify they make sense.

### Step 3: Validate

1. If a camera is connected, the live feed appears with a crosshair overlay.
2. MEBP moves to several computed well positions so you can visually verify alignment.
3. Use the camera controls in the context panel to adjust brightness, zoom, and crosshair.
4. If alignment looks correct, click **Save Calibration** in the context panel.

### Tips for Good Calibration

- Use a fine needle (25G or smaller) for precise positioning.
- Ensure the plate is firmly seated and level.
- Always zero with the needle touching the plate surface, not hovering above it.
- After calibration, test by jogging to a few computed well positions.
- Calibration is saved to settings.json and persists across restarts.

---

## 6. Creating a Print Job

The Print Setup page (🖨️) provides three ways to create a print job.

### Option A: Load from File

1. Click the **File** tab in the main content area.
2. Click **Browse** and select a `.json` or `.gcode` file.
3. The 2D preview canvas shows the loaded path.
4. Adjust print settings in the context panel (travel height, print speed, etc.).

### Option B: Well Plate + Pattern

1. Click the **Well Plate** tab.
2. Select a plate format (6, 12, 24, 48, 96, or 384 wells).
3. Select which wells to print in (click individual wells or select rows/columns).
4. Choose a pattern: line, meander, spiral, or grid.
5. Configure pattern parameters (length, spacing, angle).
6. The preview shows the pattern positioned in each selected well.

### Option C: Pattern Generator

1. Click the **Pattern** tab.
2. Configure a standalone geometric pattern.
3. Set the origin position and scale.

### Print Settings (Context Panel)

Open the context panel to configure:

| Setting | Description | Typical Value |
|---------|-------------|---------------|
| Travel Z Height | Z height during travel moves | 5.0 mm |
| Print Z Height | Z height during printing | 0.1 mm |
| Layer Height | Z increment per layer | 0.1 mm |
| Number of Layers | Total layer count | 1–10 |
| XY Feedrate | Stage speed during travel | 1000 units/s |
| Print Feedrate | Stage speed during printing | 200 units/s |
| Pump | Active pump (P1/P2/P3) | P1 |
| Flow Rate | Extrusion per mm of travel | 0.01 |
| Retract Amount | Pump retraction after path segment | 0.05 mm |
| Prime Amount | Pump prime before path segment | 0.03 mm |
| Settle Delay | Wait time after travel moves | 0.0 s |

---

## 7. Running a Print

### Starting a Print

1. Ensure the job is loaded and settings are configured.
2. In the context panel, click **Start Print**.
3. The progress bar shows execution progress.
4. The 2D canvas updates to show the current position.

### Pause / Resume

- Click **Pause** to halt execution after the current command completes.
- Click **Resume** to continue from where it paused.
- Progress is automatically saved during pause for crash recovery.

### Abort

- Click **Abort** to stop the print.
- The needle automatically raises to the travel Z height for safety.
- Progress is saved so you can potentially resume later.

### Print Resume (Crash Recovery)

If the application closes unexpectedly during a print:

1. On next startup, a dialog asks "Resume Print?"
2. Click **Yes** to continue from the last saved checkpoint.
3. Click **No** to discard the saved progress.

### Print Queue

To run multiple jobs sequentially:

1. Load a job and click **Add to Queue**.
2. Repeat for additional jobs.
3. Drag to reorder jobs in the queue list.
4. Click **Start Queue** to execute all jobs in sequence.
5. If any job fails, the queue stops (safety precaution for bioprinting).

### Exporting

- **Export G-code** — Saves the current job as a `.gcode` file for use with external tools.
- **Save JSON** — Saves the current job in MEBP's native JSON format.

---

## 8. Print Monitor

The Print Monitor page (📈) provides real-time visualization during printing.

### Plate Overview

The miniature plate view shows per-well progress:

- 🟩 **Green** — Well completed
- 🟨 **Yellow** — Currently printing
- ⬜ **Gray** — Pending
- 🟥 **Red** — Error

Click any well to see details.

### Trajectory View

The main trajectory view shows the needle path in real-time:

- **Solid line** — Completed path (where the needle has been)
- **Dashed line** — Upcoming path (where the needle will go)
- **✛ Crosshair** — Current needle position

### Syringe Status

Visual syringe indicators show:

- Current fill level for each pump
- Which pump is active (flow animation)
- Needle gauge and dimensions

### Progress Panel

Real-time statistics:

- Job name, current well, layer, and step
- Elapsed time and estimated time remaining
- Tracking error (average deviation from planned path)
- Controller type (PID or Kalman)

### Pause / Abort from Monitor

The monitor page has its own Pause and Abort buttons that control the active print — you don't need to switch back to Print Setup.

---

## 9. Multi-Material Printing

MEBP supports printing with up to 3 different materials (inks) using the 3 syringe pumps.

### Per-Pump Retract/Prime

Each pump can have independent retraction and priming amounts:

1. In Print Setup context panel, expand the retraction/prime section.
2. Set amounts for P1, P2, and P3 independently.
3. These are applied automatically during pump switches.

### Pump Sequence Mode

Assign different pumps to different wells:

1. Enable multi-material mode in the context panel.
2. Set a pump sequence (e.g., `P1, P2, P1, P2`).
3. The sequence cycles through wells in order.

### Per-Layer Pump Assignment

Assign different pumps to different layers:

1. In multi-material mode, switch to "per-layer" assignment.
2. Map each layer number to a pump (e.g., Layer 1 → P1, Layer 2 → P2).

### Automatic Ink Change (v7.1)

When switching pumps, MEBP checks if the new pump's loaded ink matches what's required:

- If the ink matches, it proceeds normally.
- If the ink differs, it automatically runs a **service sequence**: waste → wash → buffer → ink pickup.
- This ensures clean transitions between materials.

---

## 10. Print Recording & Replay

### Automatic Recording (v7.1)

Every print is automatically recorded. The recording captures:

- Planned position at each waypoint (where the needle should be)
- Actual position at each waypoint (where the needle actually is)
- Timestamps for temporal analysis
- Segment identification (travel vs print vs retract)

### Viewing Recordings

1. Navigate to the **Print Monitor** page (📈).
2. Open the context panel.
3. The **Print Recordings** section shows a list of all recorded prints.
4. Click **Refresh** to update the list.
5. Click a recording to see its summary (duration, sample count, tracking error).

### Replay Overlay

1. Select a recording from the list.
2. Click **Load Replay**.
3. The actual path from the recording is overlaid on the trajectory view.
4. Tracking error statistics are displayed (average and maximum deviation).
5. Click **Clear** to remove the overlay.

### Deleting Recordings

Select a recording and click **Delete Recording** to remove it permanently.

---

## 11. Settings & Configuration

### Safety Limits

In Settings → Safety Limits:

- Set per-axis minimum and maximum values for XY, Z, and each pump.
- Use **Set from Current** to capture the current position as a limit.
- Set maximum feedrates for Z and pumps.
- Toggle safety limits on/off globally.

Safety limits are automatically applied to all movement commands. If a command would exceed a limit, it is clamped and a warning is logged.

### Controller Selection (v7.1)

In Settings → Controller:

- Select a controller protocol from the dropdown (ProScan II, ProScan III, or Auto-Detect).
- Click **Auto-Detect** to identify the connected controller automatically.
- Click **Test Rate** to measure command round-trip times.
- Rate test results show average, minimum, and maximum response times.

### Polling Intervals

- **Position polling**: How often positions are read from hardware (default: 300ms).
- **Watchdog interval**: How often serial port health is checked (default: 3s).

### Simulation Mode

- Check **Simulate XY** or **Simulate ZP** to use software simulators instead of hardware.
- Changes require an application restart.
- Useful for development, testing, and training.

---

## 12. Xbox Controller Setup

### Connecting

1. Plug in an Xbox controller via USB.
2. Go to Dashboard and open the context panel.
3. Click **Connect Xbox**.
4. The Xbox connection dot turns green if successful.

### Button Mapping

Default mapping:

| Input | Action |
|-------|--------|
| Left stick | XY stage jogging |
| Right stick (up/down) | Z needle jogging |
| Right stick (left/right) | Pump 1 jogging |
| D-pad up/down | Increment/decrement Z speed |
| D-pad left/right | Increment/decrement XY speed |
| A button (0) | Set zero position |
| LB button (4) | Decrease Z speed |
| RB button (5) | Increase Z speed |

### Customizing Mapping

1. Click **Xbox Mapping Editor** in the Dashboard context panel.
2. The editor dialog shows three tabs: Buttons, Axes, D-Pad.
3. Use the dropdowns to assign commands to each input.
4. Click **Save** to apply.
5. The mapping file is hot-reloaded every 5 seconds — no restart needed.

---

## 13. Keyboard Shortcuts

These shortcuts work on any page:

| Key | Action |
|-----|--------|
| ← → ↑ ↓ | Jog XY stage by current step size |
| Page Up | Jog Z up |
| Page Down | Jog Z down |
| Home | Move all axes to zero reference |
| Escape | **Emergency Stop** (sends M112) |

Keyboard shortcuts are disabled when a text input field is focused.

---

## 14. Troubleshooting

### "XY stage not found" on connect

- Ensure the ProScan controller is powered on and connected via RS-232.
- Check that the correct COM port appears in Settings → Serial Ports.
- Try **Refresh Ports** in Settings.
- If using a USB-to-serial adapter, ensure drivers are installed.
- Try Auto-Detect in Settings → Controller to identify the correct protocol.

### "ZP stage not found" on connect

- Ensure the Marlin board is powered on and connected via USB.
- The board must respond to `M115` (firmware version query).
- Check that no other application (e.g., Pronterface, Cura) has the port open.

### Xbox controller not detected

- Ensure the controller is plugged in before clicking Connect.
- Only Xbox-compatible controllers are supported (via pygame).
- On Linux, you may need the `xboxdrv` package.

### Print fails with "Safety limit clamped"

- Your print path extends beyond the configured safety limits.
- Go to Settings → Safety Limits and expand the range, or adjust your print coordinates.
- Check that the zero reference is set correctly.

### Positions show "—" (no data)

- The stage is not connected. Check connection status dots.
- The position poller may have stalled. Try disconnecting and reconnecting.

### Application crashes on startup

- Delete `settings.json` to reset to defaults.
- Check the console log for error messages.
- Ensure all required Python packages are installed.

---

## 15. Safety Guidelines

### Before Every Print Session

1. Verify safety limits are enabled (green shield icon in the status bar).
2. Check that the needle is clear of the plate before homing.
3. Confirm the zero reference is set correctly.
4. Test movement with small step sizes before running full jobs.

### During Printing

- Monitor the Print Monitor page for tracking errors and progress.
- Keep the Emergency Stop shortcut (Escape key) accessible.
- Do not disconnect hardware during a print.
- If the needle contacts the plate unexpectedly, press Escape immediately.

### After Printing

- Review the print recording for tracking error analysis.
- Run a service sequence (waste → wash) if changing inks.
- Retract syringe pumps to relieve pressure.

### Flow Rate Safety (v7.1)

MEBP enforces maximum flow rates based on needle gauge to prevent excessive pressure:

| Needle Gauge | Max Flow Rate |
|-------------|---------------|
| 20G | 2000 µL/min |
| 22G | 1000 µL/min |
| 25G | 500 µL/min |
| 27G | 200 µL/min |
| 30G | 50 µL/min |
| 32G | 20 µL/min |

If a print job requests a flow rate exceeding the limit for the installed needle, it will be automatically clamped.

### Emergency Procedures

1. **Escape key** — Sends M112 emergency stop to Marlin (stops all motors immediately).
2. **Abort button** — Stops the current print and raises the needle to travel height.
3. **Power off** — As a last resort, power off the Marlin board. MEBP will detect the disconnect.
4. On next startup, MEBP offers to resume the interrupted print or discard progress.
