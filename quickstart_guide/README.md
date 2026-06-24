# MEBP Quick Start Guide

`MEBP_Quick_Start_Guide.pdf` is an operator-facing orientation guide covering:

1. **Getting hardware set up** — connect/simulate stages, axis map, Z setup, pump
   plunger calibration, safety envelope, inks, needle, cameras.
2. **Calibration** — needle location, Z reference heights, plate location, plate-Z auto-cal.
3. **Quick Print** — object/well selection plus *every* optional ⚙ Settings field.
4. **Cell Targeting & Removal** — trypsinize-in-place then relocate.
5. **Spheroid Pick & Place** — aspirate spheroids and deposit at a target.
6. **Cell Staining (Cell Labeling)** — deposit a stain, incubate, aspirate to waste.

Each section follows one standard template (numbered chip header, overview,
"Where to find it", "Before you start", a screenshot, step-by-step, an
exhaustive Settings & controls table, and Safety / Tips callouts).

## Files

| File | Purpose |
|------|---------|
| `MEBP_Quick_Start_Guide.pdf` | **The deliverable.** |
| `build_guide.py` | The reusable PDF template + flow-layout engine (`GuideDoc`) and the section assembler. Run to regenerate the PDF. |
| `guide_content.json` | The guide text/settings (one object per guide; the data the template renders). |
| `capture_screenshots.py` | Renders the real app (simulated hardware, `WA_DontShowOnScreen`) and saves a PNG per page/sub-page. |
| `screenshots/` | Captured UI screenshots. |
| `assets/` | Hand-captured screenshots (e.g. the needle-location side-camera steps). |

## Regenerate

From the repo root (uses the real Qt platform — the offscreen plugin loads no
fonts, so text would otherwise render as tofu boxes):

```bash
# 1. (optional) re-capture screenshots from the live UI
python quickstart_guide/capture_screenshots.py

# 2. rebuild the PDF from guide_content.json + screenshots
python quickstart_guide/build_guide.py
```

To edit wording or add/remove a setting, edit `guide_content.json` and re-run
step 2. To change layout/branding, edit `GuideDoc` / `render_guide` in
`build_guide.py`. To map which screenshots appear in a section, edit the
`FIGURES` dict in `build_guide.py`.

Screenshots are taken against **simulated** hardware, so values shown in them
(positions, pump fill, etc.) are illustrative; the live app shows the real
stage, plate and camera feeds.
