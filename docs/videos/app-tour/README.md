# MEBP — "Where Everything Lives" (app tour)

A ~94-second orientation video for new operators: what the six sidebar pages
are for, and what order to learn them in. Built with
[HyperFrames](https://github.com/heygen-com/hyperframes), which renders an HTML
composition to a deterministic MP4.

This is a **trial run** — one video, to judge whether the format is worth
investing in.

## Prerequisites

Node 22+ and FFmpeg (`winget install OpenJS.NodeJS.LTS Gyan.FFmpeg`).

## Commands

```bash
npm run check     # lint + runtime + layout + motion + WCAG contrast
npm run dev       # Studio preview (long-running server)
npm run render    # MP4 to out/
```

Render explicitly:

```bash
npx hyperframes render --quality high --output out/mebp-app-tour.mp4
npx hyperframes snapshot --at 10,23,36,53,63,75,85,91   # eyeball frames
```

## Files

| Path | Notes |
|------|-------|
| `index.html` | The composition — all 9 scenes and the single paused GSAP timeline. |
| `screens/*.png` | **Generated.** Real app captures. Do not hand-edit. |
| `styles/tour.css` | Layout, spotlight and callout styling. |
| `styles/mocha.css` | **Generated.** Do not hand-edit. |

## The screenshots are real, and regenerable

`screens/` holds actual captures of the running app, produced by
[`tools_capture_app_screens.py`](../../../tools_capture_app_screens.py):

```bash
python tools_capture_app_screens.py            # from the repo root
python tools_capture_app_screens.py --list     # what it captures
```

It boots a full `MainWindow` with **simulated** hardware, walks the navigation,
and grabs each screen with `QWidget.grab()` — the same primitive
`quick_print_report.py` uses for its HTML export. Re-run it after a UI change
and the video updates with it. Two non-obvious constraints are baked in:

- It runs against a **copy** of `settings.json`. The app auto-saves (the
  calibration page has a ~500 ms debounced write), so a scripted boot pointed
  at the real file could corrupt a taught calibration.
- It does **not** use `QT_QPA_PLATFORM=offscreen`. That plugin has no font
  database here, so layout and icons render correctly while every glyph comes
  out as tofu — a silent failure. The real platform plugin is used and the
  window is parked off-screen instead.

## How the overlays work

Each teaching step is one `.spot` div. Its `box-shadow: 0 0 0 9999px rgba(...)`
dims everything *outside* the ring, so a single element is both the highlight
and the scrim — there is no separate overlay to keep in sync.

Spotlight geometry is in **percentages of the frame**, so it survives a change
of capture resolution. Steps cross-fade strictly sequentially: two spotlights
visible at once would double-darken the scrim. Sizes are static because
`width`/`height` are not on the animatable-property allowlist.

⚠ **Verify placement after re-capturing.** A spotlight pointing at the wrong
control is worse than none. Snapshot the step midpoints and look:

```bash
npx hyperframes snapshot --at 16.5,31,37,47,62,70,82
```

This already caught one error — Print Builder's sub-nav is a *vertical icon
column*, not the horizontal tab strip every other mode page uses, so the
first-pass spotlight framed empty canvas.

## The palette is generated, not copied

`styles/mocha.css` is produced from the app's own theme by
[`tools_export_theme_tokens.py`](../../../tools_export_theme_tokens.py), whose
single source of truth is `COLORS` in `gui/styles.py`. No hex value is
hand-written anywhere in this project. After a theme change:

```bash
python tools_export_theme_tokens.py     # from the repo root
```

## Every on-screen claim is sourced

The video names pages, tabs and tiles. Those names come from **live code**, not
from `README.md` (whose Usage section still documents the retired 7-page nav).
The mapping is in the `SOURCES` comment at the top of `index.html`. Re-check it
whenever navigation changes — a tutorial that names a tab wrong is worse than no
tutorial.

Two claims were corrected during the build after checking the source, and are
worth not regressing:

- Page gating leaves **Hardware Setup *and* Settings** always enabled
  (`gui/app.py:1878`); only pages 2–5 lock. An earlier cut said "the rest stay
  closed".
- Hardware Setup is **not** inert. `stage_panel._on_jog_array_z` jogs Z with
  `bypass_safety=True`, and `microscope_setup_panel._go_filter` /
  `_go_objective` rotate the turrets. An earlier cut said "nothing on this page
  moves the machine" — false, and unsafe to tell an operator.

The acronym is deliberately not expanded on the title card: `CLAUDE.md` says
*Microscope-Enabled Bioprinting Platform*, `README.md` says *Multi-Extrusion
Bioprinting Platform*. That conflict is unresolved in the repo.

## Authoring notes

- One paused GSAP timeline on `window.__timelines["main"]`, built synchronously.
- Every tween is `fromTo` with `immediateRender: false`, so nothing mutates a
  later scene's elements at page load and each frame is reproducible from its
  time alone regardless of seek order.
- The font is named `Roboto` directly. Naming a system font (e.g. Segoe UI)
  makes the renderer alias it to Roboto anyway, so preview and render disagree.
- `check` reports one standing warning — `composition_file_too_large`. Splitting
  into sub-compositions is the framework's suggestion; it was not done here
  because sub-composition mounts add real failure modes for a single-file trial.
