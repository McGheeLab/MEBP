# MEBP v7.2.6 — Print Results Page Coding Plan

## Version: 7.2.6 | Date: March 2026
## Scope: New "Print Results" diagnostic page in the print workflow

---

## 1. Executive Summary

Add a **Print Results** page (📋) at index 6, between Print Monitor and Settings.
This page provides comprehensive post-print diagnostics:

- **Path comparison**: Ideal (planned) vs actual trajectory overlaid in XY/XZ/YZ projections
- **Playback simulation**: Animated replay with scrub bar, speed control, and segment filtering
- **Diagnostic statistics**: Tracking errors, path lengths, timing, per-segment breakdown
- **Time-series charts**: Error vs time, position delta, velocity, pump activity
- **Export**: Save reports and data

---

## 2. Architecture Overview

### 2.1 New File

| File | Purpose | Est. Lines |
|------|---------|-----------|
| `gui/pages/print_results.py` | PrintResultsPage widget with all sub-widgets | ~900 |

### 2.2 Modified File

| File | Changes |
|------|---------|
| `gui/app.py` | Register page at index 6, shift Settings to 7, wire signals/recorder, update all index refs |

### 2.3 Data Source

All data comes from `PrintRecorder` recordings (JSON meta + CSV data):

**From `*_meta.json`:**
- `job_name`, `timestamp`, `status`, `duration_s`, `num_samples`
- `workspace` dict (needle, plate, pumps, inks)
- `well_setup` dict
- `summary` dict with tracking error stats, path length

**From `*_data.csv` (each row = 1 sample):**
- `t` — time from print start (seconds)
- `planned_x, planned_y, planned_z` — commanded positions (mm)
- `actual_x, actual_y, actual_z` — measured positions (mm)
- `planned_p1..p3, actual_p1..p3` — pump positions (mm)
- `tracking_error_xy, tracking_error_z` — computed errors (mm)
- `segment_id` — which object/segment
- `is_travel, is_retract` — move type flags

---

## 3. Page Layout

```
┌─────────────────────────────────────────────────────────────────────┐
│  Print Results — [Recording Selector ▾]  [◀ Prev] [Next ▶]        │
├───────────────────────────────┬─────────────────────────────────────┤
│  XY Path Comparison           │  Error Heatmap / XZ Path           │
│  ┌─────────────────────────┐  │  ┌───────────────────────────────┐ │
│  │ Planned ---- Actual ──  │  │  │  XZ projection or             │ │
│  │ (error-colored)         │  │  │  Error vs Time chart           │ │
│  │ ✛ playback cursor       │  │  │  (switchable via tab)          │ │
│  └─────────────────────────┘  │  └───────────────────────────────┘ │
├───────────────────────────────┴─────────────────────────────────────┤
│  Playback: [|◀] [▶/❚❚] [▶|] [⟲]  Speed: [1x ▾]  ═══●═══  00:12  │
├─────────────────────────────────────────────────────────────────────┤
│  Statistics                                                          │
│  ┌──────────────┬──────────────┬──────────────┬────────────────────┐│
│  │ Tracking Err │ Path Length  │ Timing       │ Per-Segment Table  ││
│  │ XY mean: 12µm│ Planned: 45mm│ Total: 34s   │ Seg | Err | Len   ││
│  │ XY max:  89µm│ Actual: 46mm │ Print: 28s   │ 0   | 15µm| 12mm  ││
│  │ Z mean:  3µm │ Deviation: 2%│ Travel: 6s   │ 1   | 8µm | 33mm  ││
│  │ Z max:   15µm│              │              │                    ││
│  └──────────────┴──────────────┴──────────────┴────────────────────┘│
└─────────────────────────────────────────────────────────────────────┘

Context Panel:
┌────────────────────┐
│ Recording Info     │
│ Job: Scaffold_v1   │
│ Date: 2026-03-09   │
│ Status: completed  │
│ Samples: 1,234     │
│ Duration: 34.2s    │
├────────────────────┤
│ Display Filters    │
│ [✓] Print moves    │
│ [✓] Travel moves   │
│ [ ] Retract moves  │
│ Segments: [All ▾]  │
├────────────────────┤
│ Workspace Info     │
│ Needle: 30G        │
│ Plate: 24-well     │
│ Pumps: P1, P2      │
├────────────────────┤
│ [📤 Export CSV]    │
│ [📊 Export Report] │
│ [🔄 Refresh List]  │
└────────────────────┘
```

---

## 4. Detailed Component Specifications

### 4.1 PathComparisonWidget (custom QWidget with QPainter)

- Draws planned path as blue dashed line
- Draws actual path colored by tracking error magnitude:
  - Green (< 20µm) → Yellow (20-50µm) → Red (> 50µm)
- Travel moves drawn as thin gray dotted lines (toggleable)
- Playback cursor: crosshair at current sample position
- Mouse hover shows coordinates + error at nearest sample
- Zoom (scroll wheel) and pan (middle-click drag)
- Switchable between XY, XZ, YZ projections via buttons

### 4.2 PlaybackController (QWidget bar)

- Play/Pause button (toggles state)
- Stop button (resets to start)
- Speed selector: 0.25x, 0.5x, 1x, 2x, 5x, 10x
- QSlider scrub bar (0 to num_samples)
- Time label showing current time / total time
- QTimer drives animation at ~30fps, advancing by speed factor
- Emits `sample_changed(int)` signal → updates all views

### 4.3 StatisticsPanel (QWidget with grouped labels)

Four stat groups:
1. **Tracking Error**: XY mean/max/RMS, Z mean/max/RMS (all in µm)
2. **Path Length**: Planned total, actual total, deviation %, per-axis
3. **Timing**: Total duration, print time, travel time, retract time, idle time
4. **Per-Segment Table**: QTableWidget with columns: Segment ID, Sample Count,
   Mean Error (µm), Max Error (µm), Path Length (mm), Duration (s)

### 4.4 ErrorTimeSeriesWidget (custom QWidget with QPainter)

- X-axis: time (seconds)
- Y-axis: tracking error (µm)
- Two lines: XY error, Z error
- Vertical playback cursor line
- Segment boundaries shown as thin vertical lines
- Hover tooltip with exact values

### 4.5 Context Panel

- Recording selector (same list as Print Monitor, loaded from PrintRecorder)
- Display filter checkboxes
- Segment filter combo
- Workspace info labels (from recording metadata)
- Export buttons

---

## 5. Coding Sessions

### Session 1: Create `gui/pages/print_results.py` (this chat)

New file with:
- PathComparisonWidget (QPainter-based path overlay)
- PlaybackController (transport bar with QTimer)
- StatisticsPanel (grouped stat labels + per-segment table)
- ErrorTimeSeriesWidget (QPainter chart)
- PrintResultsPage (top-level page assembling all sub-widgets)
- Context panel builder
- Recording loading from PrintRecorder
- Page interface methods (get_page_title, get_context_widget, etc.)

### Session 2: Create patch script for `gui/app.py` (this chat)

Patch script `patches/v726/patch_s1_print_results.py`:

**Changes to app.py:**

A) **Add import** for PrintResultsPage
   ```python
   from gui.pages.print_results import PrintResultsPage
   ```

B) **Add sidebar menu button** "btn_results" with 📋 icon
   Insert after btn_monitor in the menu_items list:
   ```python
   ("btn_results",  "📋", "Print Results"),
   ```

C) **Update page instantiation** in _create_pages:
   Insert after PrintMonitorPage (index 5):
   ```python
   PrintResultsPage(self.controller, self.settings),    # 6
   ```
   Settings shifts from index 6 → 7.

D) **Update btn_map** in _on_menu_click:
   ```python
   "btn_results":   6,
   "btn_settings":  7,   # was 6
   ```

E) **Update _navigate_to** title/context arrays:
   Insert "Print Results" at index 6, "Results" at index 6

F) **Update _update_page_gating**:
   Settings guard changes from `i == 6` to `i == 7`

G) **Update all hardcoded page index references**:
   - `self._page_widgets[4]` → Print Setup (unchanged)
   - `self._page_widgets[5]` → Print Monitor (unchanged)
   - `self._page_widgets[6]` → NOW Print Results (was Settings)
   - `self._page_widgets[7]` → NOW Settings
   - `self._switch_page(5)` → Print Monitor (unchanged)

H) **Wire recorder to results page**:
   ```python
   results_page = pages[6]
   if hasattr(results_page, 'set_recorder'):
       results_page.set_recorder(self.recorder)
   ```

I) **Wire print completion signal**:
   When PrintManager state → COMPLETED, auto-load latest recording into results page.

J) **Add "View Results" auto-navigate**:
   On print completion, optionally switch to Print Results page.

---

## 6. Signal Flow

```
PrintManager state → COMPLETED
  │
  ▼
app.py._on_print_completed()
  ├── results_page.load_latest_recording()
  └── (optional) _navigate_to(6)  # auto-switch to results
```

```
PrintResultsPage context panel
  │ recording selected from list
  ▼
_load_recording(meta_path)
  ├── PrintRecorder.load_recording(path) → (meta, samples)
  ├── path_widget.set_data(samples, meta)
  ├── error_chart.set_data(samples)
  ├── stats_panel.set_data(samples, meta)
  └── playback.set_total_samples(len(samples))

PlaybackController.sample_changed(idx)
  ├── path_widget.set_cursor(idx)
  └── error_chart.set_cursor(idx)
```

---

## 7. Risk Assessment

| Risk | Mitigation |
|------|-----------|
| Large file may hit create_file limit | Use bash_tool heredoc for creation |
| Page index shift breaks existing wiring | Patch uses regex to find exact patterns, not line numbers |
| PrintRecorder.load_recording returns different format | Use defensive dict.get() everywhere |
| QPainter performance with large datasets | Downsample for display (max 5000 points), full data for stats |
| No recordings exist yet | Show friendly "No recordings" placeholder |

---

## 8. Verification Steps

After applying:
1. `python3 -c "import ast; ast.parse(open('gui/pages/print_results.py').read())"`
2. `python3 -c "import ast; ast.parse(open('gui/app.py').read())"`
3. `python main.py` — verify page appears in sidebar, no crash
4. Navigate to Print Results — verify placeholder shown
5. If recordings exist, load one — verify path display
6. Test playback controls
7. Verify Settings page still works at new index 7
