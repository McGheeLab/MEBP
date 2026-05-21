# MEBP v7.4.0-a — Architecture Reference (Delta)

**Multi-Extrusion Bioprinting Platform**
**Version 7.4.0-a | May 2026**

> **This document is a delta against [`ARCHITECTURE_V737.md`](ARCHITECTURE_V737.md).**
> Sections not mentioned here are unchanged from v7.3.7. v7.4.0-a is the
> foundation sub-version of a larger UX-focused v7.4.0 release; the substantive
> workflow restructure lands in v7.4.0-b and the onboarding wizard in v7.4.0-c.

---

## 1. What's New in v7.4.0-a

A small foundation pass that fixes the most user-painful persistence bug,
introduces a reusable component library, adds visual primitives (page
transitions + global loading banner) and standardizes hardcoded paddings
through the existing DPI scaling system. No structural page reorganization
yet — that comes in -b.

### Summary of changes

| Area | What changed |
|------|--------------|
| Calibration persistence | Auto-save now fires from a debounced timer on every `_emit_calibration_data_changed`, not just post-mosaic. Safe Z, Top Z, taught 3-point positions, plate format are persisted within 500 ms of any change. |
| Component library | New `gui/widgets/components.py`: `Card`, `StatusBadge`, `SectionHeader`, `FormRow`, `WizardStep`, `LoadingBanner`. Pages will adopt these in v7.4.0-b instead of rolling their own. |
| Page transitions | New `gui/widgets/page_transition.py::fade_swap()` — softly fades between pages, skips camera-bearing pages to avoid black-frame artifacts. |
| Global loading banner | `MainWindow.show_loading(msg)` / `hide_loading()` mounts a yellow strip with braille spinner above the content stack. |
| QSS scaling | New `gui/scaling.py::sp(px)` shorthand returning `"{N}px"`. Hardcoded paddings/radii swept across `jog_control`, `hardware_setup`, `calibration`, `print_setup`, `print_workspace`, `print_well_setup`, `helper_functions`, `settings_page`. |
| Version string | `MainWindow` title now reads `MEBP Bioprinter — v7.4.0-a`. |

### Deferred to v7.4.0-b / -c

- Hardware Setup decomposition into sub-pages (mode-page pattern).
- Settings/Hardware boundary cleanup (axis flip, ZP feedrates, safety limits move to HW).
- Cameras relocated from HW Setup to Calibration.
- First-run onboarding wizard + help-mode toggle.
- Remaining padding sweep across `print_objects`, `print_results`, `pp_execution`, `print_monitor`, `pp_operation_queue`, `pp_target_selection`, `pp_operation_setup`, `well_preview`.

---

## 2. New / Changed Files

```
gui/
├── scaling.py                              # +sp(px) helper
├── app.py                                  # +LoadingBanner mount, fade_swap navigation, version bump
├── pages/
│   ├── calibration.py                      # +debounced auto-save in _emit_calibration_data_changed
│   ├── hardware_setup.py                   # scaled paddings
│   ├── jog_control.py                      # scaled paddings
│   ├── print_setup.py                      # scaled paddings + scaled font sizes
│   ├── print_workspace.py                  # scaled paddings + scaling import added
│   ├── print_well_setup.py                 # scaled paddings + COLORS-based hex replacement
│   ├── helper_functions.py                 # scaled paddings
│   └── settings_page.py                    # scaled paddings
└── widgets/
    ├── components.py                       # NEW — reusable UI primitives
    └── page_transition.py                  # NEW — fade_swap helper
```

---

## 3. Component Library — `gui/widgets/components.py`

Designed for use by v7.4.0-b and -c. No page currently consumes them; they are
shipped early so the foundation is in place when the restructure begins.

### Card

Titled container with optional collapse behavior. Uses `COLORS['surface0']`
background, `COLORS['surface1']` border, `s(8)` corner radius, `s(12)` padding.

```python
card = Card("Pump Channels")
card.add_widget(pump_widget)

collapsible = Card("Advanced", collapsible=True)
collapsible.add_widget(advanced_form)
collapsible.set_collapsed(True)
```

### StatusBadge

Colored pill label with five variants: `ok`, `warn`, `err`, `info`, `pending`.
Background tinted (38/255 alpha) on the foreground color; border + text fully
opaque.

```python
badge = StatusBadge("Connected", variant="ok")
badge.set_status("err", "Disconnected")
```

### SectionHeader

Wraps the existing `build_page_header_style(scale)` from `gui/styles.py` so
pages stop importing the bare `PAGE_HEADER_STYLE` constant.

### FormRow

Label + field + optional inline help text. Help is hidden by default;
`set_help_visible(True)` reveals it. The progressive-disclosure target for the
v7.4.0-c "Help" toggle.

```python
row = FormRow("Syringe volume", spin,
              help_text="Total syringe volume in µL.")
row.set_help_visible(global_help_mode_enabled)
```

### WizardStep

Header (`Step N of M`, big title) + body slot + Prev/Next/Finish controls.
Signals: `prev_clicked`, `next_clicked`, `completed`. `is_last=True` swaps
the Next button to a green Finish button that emits `completed` instead.
Backbone for the v7.4.0-c onboarding wizard.

### LoadingBanner

Thin colored strip mounted in `MainWindow` between the top bar and the page
stack. Uses a 10-frame braille Unicode animation (`⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏`) ticked by a
`QTimer` at 90 ms — no `QMovie` or GIF asset needed. Exposed via
`MainWindow.show_loading(msg)` / `hide_loading()`.

---

## 4. Page Transitions — `gui/widgets/page_transition.py`

Single helper:

```python
fade_swap(stack: QStackedWidget, new_index: int, ms: int = 120) -> None
```

Implementation uses `QGraphicsOpacityEffect` + `QPropertyAnimation`. The
destination page is walked for any known camera-widget class
(`CameraWidget`, `CameraFeedView`, `TargetOverlayCameraView`) — if any is
present, the animation is skipped and the stack swaps instantly. This avoids
black-frame artifacts from `QGraphicsOpacityEffect` interfering with raw
QPainter camera streams.

`gui/app.py::_navigate_to` now calls `fade_swap(self._page_stack, index)`
instead of `self._page_stack.setCurrentIndex(index)`.

---

## 5. Calibration Auto-Save (the real bug from v7.3.7)

### What was wrong

`_save_calibration` existed at `gui/pages/calibration.py:4257` and worked
correctly. But auto-save was only triggered from one location (post-mosaic,
line 3708). The five other callers of `_emit_calibration_data_changed` —
which fire on safe-Z teach, top-Z teach, 3-point teach, manual XY teach, Z
plane fit — only emitted the signal and never persisted. Users who taught
a Safe Z and then quit without clicking the manual Save button lost the
teach point.

### What v7.4.0-a does

A new `QTimer` (`self._autosave_timer`, 500 ms single-shot) is constructed
in `CalibrationPage.__init__`, hooked to `_save_calibration`, and `start()`
is called inside `_emit_calibration_data_changed`. Repeated emits during a
mosaic scan coalesce into a single write 500 ms after the last change —
no disk thrashing.

```python
# CalibrationPage.__init__
self._autosave_timer = QTimer(self)
self._autosave_timer.setSingleShot(True)
self._autosave_timer.setInterval(500)
self._autosave_timer.timeout.connect(self._save_calibration)

# _emit_calibration_data_changed
self.calibration_data_changed.emit()
if self.settings is not None and hasattr(self, '_autosave_timer'):
    self._autosave_timer.start()
```

---

## 6. Updated Design Principles

(Additions to the V737 list — others unchanged.)

10. **Reusable UI primitives** — `gui/widgets/components.py` provides Card,
    StatusBadge, SectionHeader, FormRow, WizardStep, and LoadingBanner. New
    page code should consume these instead of constructing equivalent
    widgets from scratch. Existing pages will migrate to them across
    v7.4.0-b/c as part of the workflow restructure.

11. **Calibration persistence is automatic** — Any change that calls
    `_emit_calibration_data_changed` triggers a 500 ms-debounced write to
    `settings.json`. Manual Save / Load remains available for explicit
    export/import of calibration snapshots.

---

## 7. Sections Unchanged from V737

The following sections of `ARCHITECTURE_V737.md` apply verbatim to v7.4.0-a:

- §2 Technology Stack
- §3 Folder Structure (with additions noted in §2 above)
- §4 Backend Modules (`SupportClasses/`)
- §5 Hardware Communication
- §6 Print Pipeline
- §7 Calibration (apart from the persistence detail in §5 above)
- §8 Print Plan of Action
- §9 Print Trajectory Planning
- §10 Print Recorder / History
- §11 Camera Subsystem
- §12 Threading Model
- §13 Settings Persistence
- §14 Theme & Scaling (apart from the new `sp(px)` helper)

When v7.4.0-b ships, a full ARCHITECTURE_V740B.md will replace this delta
with a complete updated architecture reference.
