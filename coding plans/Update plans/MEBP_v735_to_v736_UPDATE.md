# MEBP v7.3.5 → v7.3.6 Update Plan

## Objective

Make the GUI scale correctly on large / high-DPI screens (1440p, 4K, Retina, ultrawide).
Previously the UI was locked to 96 DPI and used hardcoded pixel dimensions everywhere,
causing elements to appear tiny on large displays.

Additionally: package the application as a standalone executable via PyInstaller.

---

## Files Modified

### New Files
| File | Purpose |
|------|---------|
| `gui/scaling.py` | DPI-aware scaling utility — `s(px)`, `sf(pt)`, `scale_factor()` |
| `MEBP.spec` | PyInstaller spec file for building standalone executable |
| `build_exe.py` | Build script — `python build_exe.py [--clean] [--debug]` |
| `.gitignore` | Ignore build artifacts, runtime files, IDE configs |

### Core Changes
| File | Change |
|------|--------|
| `main.py` | Replace `QT_FONT_DPI=96` lock with `QT_AUTO_SCREEN_SCALE_FACTOR=1`; add `freeze_support()` for PyInstaller |
| `gui/styles.py` | Convert `DARK_THEME` from static string to `build_theme(scale)` function; all px/pt values parameterized |
| `gui/app.py` | Import scaling, apply `build_theme(k)` at startup, wrap all pixel/font dimensions with `s()`/`scaled_font_size()` |
| `gui/ui_functions.py` | Scale menu width (60/200px), context panel width (260px) |

### Page-Level Scaling
| File | Key Changes |
|------|-------------|
| `gui/pages/jog_control.py` | Button sizes, field widths |
| `gui/pages/calibration.py` | Camera controls, needle detection buttons, Z-teach buttons |
| `gui/pages/print_monitor.py` | Plate overview, syringe display, progress bars |
| `gui/pages/hardware_setup.py` | Dialogs, tables, input fields, color pickers |
| `gui/pages/dashboard.py` | Card layouts, font sizes |
| `gui/pages/print_results.py` | Segment table, playback controls |
| `gui/pages/print_setup.py` | Layout dimensions |
| `gui/pages/settings_page.py` | Form fields |
| `gui/pages/print_objects.py` | Print file path widths |
| `gui/pages/print_well_setup.py` | Well setup dimensions |
| `gui/pages/helper_functions.py` | Image toolpath controls |
| `gui/pages/mode_page.py` | Mode page icon column |
| `gui/pages/printing_mode.py` | Sub-page container |
| `gui/pages/pick_place_mode.py` | Sub-page container |
| `gui/pages/pp_target_selection.py` | Target selection controls |
| `gui/pages/pp_operation_queue.py` | Queue list, combo widths |
| `gui/pages/pp_execution.py` | Execution monitor |

### Widget-Level Scaling
| File | Key Changes |
|------|-------------|
| `gui/widgets/jog_button_array.py` | Button sizes (44→s(44)), combo widths |
| `gui/widgets/camera_widget.py` | Camera view dimensions |
| `gui/widgets/detection_overlay.py` | Overlay element sizes |
| `gui/widgets/jog_well_plate.py` | Well plate navigator |

---

## Implementation Steps

- [x] Create `gui/scaling.py` with `s()`, `sf()`, `scale_factor()`, `scaled_font_size()`
- [x] Update `main.py` — remove 96 DPI lock, enable native Qt6 HiDPI
- [x] Update `gui/styles.py` — convert to `build_theme(k)` with parameterized dimensions
- [x] Update `gui/app.py` — wire scaling, wrap all hardcoded sizes
- [x] Update `gui/ui_functions.py` — scale menu/context panel widths
- [x] Update `gui/widgets/jog_button_array.py` — scale button sizes
- [x] Update all page files — wrap hardcoded sizes with `s()`
- [x] Create `MEBP.spec` and `build_exe.py` for PyInstaller packaging
- [x] Create `.gitignore` for build artifacts
- [ ] Test on multiple screen resolutions
- [ ] Test PyInstaller build launches correctly

---

## Testing Notes

### DPI Scaling
- Set `MEBP_UI_SCALE=1.5` environment variable to simulate 4K on a 1080p screen
- Verify all buttons, inputs, and labels scale proportionally
- Check that no text is clipped or overflows containers
- Test menu expand/collapse animation at different scales

### PyInstaller
- Run `python build_exe.py --clean`
- Verify `dist/MEBP/MEBP` launches in simulation mode
- Verify config files are bundled (`config/controllers/`, `config/hardware/`)
- Test headless mode: `./dist/MEBP/MEBP --simulate-xy --simulate-zp --headless`

---

## Design Decisions

1. **Scale factor floor = 1.0**: The `scale_factor()` function returns `max(1.0, dpi/96)` so the UI never shrinks below its 96-DPI design size, only grows.

2. **`build_theme(k)` vs regex replacement**: Chose parameterized f-string generation over post-hoc regex replacement of px/pt values in QSS. The f-string approach is explicit, type-safe, and won't accidentally scale color hex values.

3. **`--onedir` PyInstaller mode**: Chosen over `--onefile` because the app writes user data (settings.json, print_records/, sim state) that must persist between runs. `--onedir` keeps the bundle writable and persistent.

4. **Excluding transitive deps**: TensorFlow (765 MB), PyTorch (284 MB), and other unused packages were excluded from the PyInstaller bundle, reducing size from 2.9 GB to ~470 MB.
