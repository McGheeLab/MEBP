# MEBP v7.5.x — Fix "multiple pages have a white background"

## Objective

Operator (2026-07-24): "multiple pages have a white background now, check for
this in needle calibration." Several pages/panels render with a white
background instead of the Catppuccin-Mocha dark theme.

## Root cause (investigation)

The dark theme is a **QSS only** (`gui/styles.py::build_theme`, applied via
`MainWindow.setStyleSheet` in `gui/app.py:151`). Two facts combine into the bug:

1. **No dark palette is set anywhere.** `main.py` does `app.setStyle("Fusion")`
   but never `app.setPalette(...)` (grep of the whole repo: zero `setPalette`).
   Fusion follows the **OS/system palette** when none is set — so the palette
   `Base`/`Window` roles are whatever the OS theme is (white on a light-mode OS,
   or after a Qt dark-mode-detection change).
2. **The QSS doesn't cover every surface.** The base `QWidget` rule sets no
   `background-color`, and there are **no `QScrollArea` / `QGraphicsView` rules**
   at all. So those widgets' viewports — and transparent panels (the whole left
   context panel is `background-color: transparent`, `#contextScrollArea
   QWidget { transparent }`) that bottom out at the palette — fall back to the
   palette `Base`.

Result: when the OS palette `Base` is light, **every widget the QSS doesn't
explicitly paint renders white** — QScrollArea viewports (most page bodies),
QGraphicsView viewports, and transparent panels. Confirmed contributing detail:
5 `QGraphicsView` subclasses set **no** background brush
(`_CalibrationPlateView`, `_CalibrationYZView` in `calibration.py`,
`WellPlateView`, `ProjectionPane`, `ProjectionView`) while 5 siblings DO
(`_MappingView`, `MosaicRegistrationView`, `_ZoomImageView`, `WellPreviewView`,
`PlateDesignerCanvas` → `setBackgroundBrush(QColor(COLORS["base"]))`). The
inconsistency was masked as long as the OS palette Base happened to be dark.

**Not a regression from the recent camera/mapping work** — `gui/styles.py`,
`main.py`, and the calibration page's styling are all unchanged (git). The
trigger is an external OS/Qt palette shift exposing the latent fragility.

## Fix

Pin a dark palette on the `QApplication` so the app's dark theme does **not**
depend on the OS theme — the standard practice for a Fusion + dark-QSS app. New
`main.py::_apply_dark_palette(app)` (called right after `app.setStyle(fusion)`)
sets a Catppuccin-Mocha `QPalette`: `Base #181825` (the viewport background),
`Window #1e1e2e`, `Text #cdd6f4`, `Button #313244`, `Highlight #cba6f7`,
disabled text `#6c7086`, etc. The QSS still overrides per-widget wherever it
sets an explicit background, so styled widgets are unchanged; only the
previously-white uncovered viewports/panels now default to the theme's dark
surfaces — across **all** pages, including needle calibration.

## Files Modified

| File | Change |
|------|--------|
| `main.py` | `_apply_dark_palette(app)` helper + call after `setStyle(fusion)` |

## Testing

- `python -m py_compile main.py` OK; headless palette check confirms
  `Base == #181825`, `Window == #1e1e2e`, `Text == #cdd6f4`,
  `Highlight == #cba6f7` after `_apply_dark_palette`.
- Tests create their own `QApplication` (no palette) and don't assert
  rendering, so they're unaffected.
- **Needs visual confirmation on the running app (ME3B V1):** needle
  calibration + the other affected pages now show the dark background;
  text-entry/combo/graphics views read correctly on dark.

## Optional follow-up (not required by the palette fix)

For belt-and-suspenders/consistency, add
`setBackgroundBrush(QColor(COLORS["base"]))` to the 5 `QGraphicsView` classes
that lack it so they're dark independent of the palette (matching their 5
siblings). The palette fix already makes them dark, so this is cosmetic
consistency only.

## Status

- [x] Root-cause the white background (no dark palette + uncovered viewports)
- [x] Pin a dark `QPalette` in `main.py`
- [x] Verify Base/Window/Text resolve dark headlessly
- [ ] Operator visual confirmation on the running app
