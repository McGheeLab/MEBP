# app-tour — HyperFrames project

The MEBP "Where Everything Lives" orientation video.

**Read `../DIRECTING.md` first** — it holds the collaboration contract (the
four review gates), the recording workflow, and the conventions this project
follows. `README.md` here covers this project specifically.

## Commands

```bash
npm run check     # required gate: lint + runtime + layout + motion + contrast
npm run dev       # Studio preview (long-running server — run in background)
npm run render    # MP4 to out/
```

## Pinned CLI

`package.json` pins an exact `hyperframes@X.Y.Z` so this project re-renders
identically over time. That pin does not advance on its own. To move it up:

```bash
npx hyperframes@latest upgrade --project . --check   # show the delta
npx hyperframes@latest upgrade --project .           # apply, then re-run check
```

Note the documented commands in the READMEs invoke `npx hyperframes …`
unpinned, which can resolve to a *newer* CLI than the npm scripts use. Prefer
`npm run …` when the version matters.

## Generated files — do not hand-edit

- `styles/mocha.css` → `python tools_export_theme_tokens.py` (repo root)
- `screens/*.png` → `python tools_capture_app_screens.py` (repo root)
