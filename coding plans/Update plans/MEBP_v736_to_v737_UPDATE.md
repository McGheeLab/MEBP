# MEBP v7.3.6 → v7.3.7 Update Plan

## Objective

Fix import error preventing application startup, and fix icon/font sizing on mode page sidebar buttons and print setup tab bar so all emoji icons properly fit their buttons at any DPI.

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/pages/print_well_setup.py` | Fix ImportError: `SECTION_TITLE_STYLE` and `CONTEXT_SECTION_LABEL_STYLE` imported from `gui.scaling` (wrong module) → `gui.styles` (correct) |
| `gui/pages/mode_page.py` | Fix hardcoded `font-size: 16px` in sidebar button QSS → DPI-scaled `sf(14)pt`; add emoji font-family fallbacks for macOS; scale border-radius, border width, and sidebar margins; remove dead `_make_sub_icon()` and unused imports |
| `gui/pages/print_setup.py` | Fix hardcoded tab bar `padding: 8px 16px`, `margin-right: 2px`, `border-radius: 6px` → DPI-scaled values via `_sc()`; add explicit scaled font declaration |

## Implementation Steps

- [x] Fix `print_well_setup.py` import — move `SECTION_TITLE_STYLE`, `CONTEXT_SECTION_LABEL_STYLE` from `gui.scaling` import to `gui.styles` import
- [x] Fix `mode_page.py` sidebar button styling:
  - [x] Import `sf` from `gui.scaling`
  - [x] Replace hardcoded `font-size: 16px` with `font: {sf(14)}pt` using emoji font-family fallbacks
  - [x] Scale `border-radius` and `border` width with `s()`
  - [x] Scale sidebar content margins `(4, 8, 4, 8)` → `(s(4), s(8), s(4), s(8))`
  - [x] Remove redundant `setFont()` call (overridden by QSS)
  - [x] Remove dead `_make_sub_icon()` function and unused `QFont`, `QIcon`, `QPixmap`, `QPainter`, `QColor`, `QSizePolicy`, `scaled_font_size` imports
- [x] Fix `print_setup.py` tab bar styling:
  - [x] Import `sf as _sf` from `gui.scaling`
  - [x] Scale tab padding, margin, and border-radius with `_sc()`
  - [x] Add explicit scaled font declaration with emoji fallbacks

## Testing Notes

- Verified all modified modules import cleanly (`python -c "from gui.pages.mode_page import ModePage; ..."`)
- Visual verification: mode page sidebar buttons (Printing: 🖨️📈📋🧰, Pick & Place: ⚙️🎯▶️) should render emoji icons centered and proportionally sized within buttons at any DPI
- Print setup tab bar text should be legible and properly padded at any DPI
- Application starts without ImportError

## Issues & Decisions

- **`_make_sub_icon()` was dead code** — defined in `mode_page.py` but never called anywhere. Buttons use `QPushButton(icon_text)` with text directly, not QIcon. Removed to reduce confusion.
- **QSS overrides `setFont()`** — the `setFont(QFont("Segoe UI Emoji", scaled_font_size(14)))` call on sidebar buttons was being completely overridden by the QSS `font-size: 16px`. Removed the setFont call and fixed the QSS to use scaled sizing.
- **macOS emoji rendering** — "Segoe UI Emoji" doesn't exist on macOS, so the font was falling back to system default with unpredictable sizing. Added `"Apple Color Emoji"` and `"Noto Color Emoji"` fallbacks in the font-family chain, matching the pattern already used in `styles.py` for the main sidebar buttons.
