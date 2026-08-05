# Tutorial Video Pipeline — a portable playbook

How to build a repeatable "teach people to use this software" video pipeline in
any application, with a human directing and an agent producing.

Copy this file into the target repo and work through Part 6. It is
self-contained; nothing below depends on the project it was written in.

Built and proven on a PySide6 desktop app (a lab bioprinter controller). The
method is stack-agnostic; the reference implementation is Qt, and every
component has adaptation notes for web, Electron, and native.

---

## What you end up with

Four generated artifacts, none hand-maintained:

| Artifact | Generated from | Why generated |
|---|---|---|
| Theme tokens (CSS) | The app's own palette constant | A hand-copied hex drifts the moment the theme changes |
| Screenshots | The running app, driven programmatically | A stale screenshot teaches a UI that no longer exists |
| Spotlight geometry | The widget the operator actually clicked | Eyeballing coordinates against a screenshot is the #1 error source |
| Beat sheet | The recorded session | The reviewable artifact, produced in seconds not hours |

Plus a written collaboration contract, so the human and the agent stop
duplicating each other's work.

**Stack:** [HyperFrames](https://github.com/heygen-com/hyperframes) renders HTML
to deterministic MP4. Requires **Node 22+ and FFmpeg**.

```bash
npx skills add heygen-com/hyperframes --full-depth   # agent skill — read it first
npx hyperframes init my-video --non-interactive --example blank
```

> Read the installed `hyperframes-core` skill before writing composition HTML.
> Its authoring contract is much stricter than the README implies, and three of
> its rules will silently break a first attempt (see Part 4).

---

# Part 1 — The collaboration contract

The single highest-leverage artifact. Write it before writing code.

## The division

**The human owns the story.** What task, for whom, what they must be able to do
afterwards — and the half that is *not* in the source: why a step matters, what
goes wrong when it is skipped, what you would say to someone standing next to
you.

**The agent owns accuracy and craft.** Every on-screen claim traced to a source
line before anything is built; layout, geometry, motion, checks, render.

This split is empirical, not philosophical. In the first build the agent wrote
two claims that read perfectly and were both false — each contradicted by code
the docs did not mention. Equally, nothing in the source said which step people
actually get wrong in practice.

## The four gates

Cheapest first. The point is to make it free to change your mind early.

| Gate | Artifact | Human does | Cost to change |
|---|---|---|---|
| **1 · Beats** | `STORYBOARD.md` | Delete, reorder, rewrite rows | ~1 min |
| **2 · Sketches** | Wireframes (real words, no styling) | Confirm layout and copy | ~5 min |
| **3 · Preview** | Editor timeline | Scrub; **retime and edit copy directly** | ~10 min |
| **4 · Render** | MP4 | Watch | ~50 min if the story was wrong |

**Rule: never render without gate 3.** The first build rendered three times
because factual errors surfaced late. That is the specific waste this prevents.

Gate 3 must be real control, not a rubber stamp — `npx hyperframes preview`
opens an editor where the human retimes clips and edits copy themselves.

## Intake, per video

Four things in one message:

1. **One sentence.** *"After watching, [who] can [do what]."* If you cannot
   write it, the video has no spine — stop here.
2. **A tape** (recorded session) or a list of screens.
3. **The domain reality.** Why it matters, what breaks, the thing you always
   end up telling people.
4. **Words.** Terms to use, terms to avoid.

## Sourcing discipline (non-negotiable)

Every factual claim on screen is traced to a source line **before** anything is
built, and the mapping lives in a `SOURCES` comment at the top of the
composition:

```html
<!-- ══ SOURCES ═══════════════════════════════════════════════════
     Anchored by SYMBOL first, line second — line numbers drift.
     nav order .............. `titles = [...]` in navigateTo, src/app.ts ~1991
     page gating ............ `updatePageGating`, src/app.ts ~1895
                              (page 1 AND settings stay enabled; only 2–5 lock)
     tab order + rationale .. `addTab` calls + comments,
                              src/calibration.ts 10994-11028
     ⚠ Do NOT source from README — its usage section is stale.
     ═══════════════════════════════════════════════════════════════ -->
```

Three rules that fall out of this, learned the hard way:

- **Never source from the README.** It documented a navigation structure that
  had been replaced. Docs rot; code does not.
- **A safety-adjacent claim must be verified twice.** An early cut said
  "nothing on this page moves the machine." The page had a jog control that
  explicitly bypassed soft limits. Telling an operator a live page is inert is
  worse than saying nothing.
- **Anchor on symbol names, not bare line numbers.** Two citations in the first
  build were off by 17 lines — wrong at authoring time, not drifted — so the
  "re-walk the SOURCES table" verification step landed on unrelated code and
  silently passed. A symbol name degrades gracefully; a line number rots into
  a confident lie.

---

# Part 2 — Architecture

```
  app source ──► theme export ──────► tokens.css      (generated)
                                          │
  running app ─► capture harness ───► screens/*.png   (generated)
       │                                  │
       └──────► action recorder ──► tape.jsonl        (generated)
                                          │
                              tape → storyboard tool
                                          │
                          ┌───────────────┴───────────────┐
                    STORYBOARD.md                  NARRATION.md
                     (gate 1: agent)              (gate 1: human)
                                  └──────┬────────┘
                                    composition.html
                                          │
                              check → snapshot → preview → render
```

Five components. Build them in this order; each is useful alone.

| # | Component | Effort | Value alone |
|---|---|---|---|
| A | Theme token export | ~30 min | Video matches the app, forever |
| B | Screen capture harness | ~2 h | Any "where things are" video |
| C | Action recorder | ~4 h | Any "how to do this" video |
| D | Tape → storyboard | ~1 h | Turns a recording into a review gate |
| E | Composition patterns | ~2 h | The video itself |

---

# Part 3 — Implementation

## A · Theme token export

Emit the app's palette as CSS custom properties. **Never hand-copy a colour.**

```python
# tools_export_theme_tokens.py
"""Export the app palette as CSS custom properties so material rendered
outside the app matches it and cannot drift."""
import importlib.util, os

def load_colors(path):
    # Load BY PATH, not as a package import — keeps this free of GUI/framework
    # imports regardless of what the package __init__ grows later.
    spec = importlib.util.spec_from_file_location("_theme", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.COLORS

def to_css(colors):
    lines = ["/* GENERATED — DO NOT EDIT. Source: <theme file>. */", ":root {"]
    for key in sorted(colors):
        if isinstance(colors[key], str) and colors[key].startswith("#"):
            lines.append("  --t-%s: %s;" % (key.replace("_", "-"), colors[key]))
    return "\n".join(lines + ["}", ""])
```

**Adapt:** JS/TS — import the theme object and serialise. CSS-in-JS — the
tokens may already be custom properties; symlink or copy at build time.
Native — export from the resource bundle.

**Gotcha:** if the theme file imports the UI framework, load it by path or
extract the constant, so this stays runnable in CI without a display.

---

## B · Screen capture harness

Drive the app programmatically and grab each screen.

```python
# tools_capture_app_screens.py  (Qt reference)
SCREENS = [
    ("hardware-device", 0, "Setup / Device",  lambda w: _sub(w, 0, 0)),
    ("calibration",     1, "Calibration",     lambda w: _tab(w, 2)),
    ("jog",             2, "Jog",             lambda w: None),
]

def main():
    os.environ.pop("QT_QPA_PLATFORM", None)   # see gotcha 1 below
    os.environ["UI_SCALE"] = "1.0"            # reproducible across DPI

    settings = copy_to_tempdir(REAL_SETTINGS)  # see gotcha 2 below
    app, win = boot_app_in_simulation(settings)
    win.resize(1920, 1080)
    win.move(-4000, -4000)                     # off the visible desktop
    win.show(); settle(app)

    for slug, page, _desc, navigate in SCREENS:
        try:
            win.navigate_to(page); settle(app)
            navigate(win);         settle(app)
            win.grab().save(f"screens/{slug}.png")
        except Exception as exc:               # one bad screen ≠ dead run
            print("FAILED", slug, exc)
```

### Three gotchas that cost real time

**1 · Headless rendering may have no fonts.** Qt's `offscreen` platform plugin
rendered layout, icons and theme perfectly — and **every text glyph as tofu**.
A silent failure: it looks fine until you try to read it. Fix: use the real
platform plugin and park the window off-screen. Check the equivalent for your
stack (headless Chrome needs fonts installed in the container).

**2 · Point the harness at a COPY of user settings.** Apps auto-save. This one
had a debounced write on a calibration page, so a scripted boot against the
real file could corrupt a taught hardware calibration. Copy to a temp dir and
run against that. Always.

**3 · Test the "impossible" assumption.** The test suite asserted twice that a
full main-window boot was "intractable headless." It booted first try. Spend
ten minutes proving it before designing around it.

**4 · Startup modals deadlock an unattended run — invisibly.** A first-run
wizard or a "your calibration is stale" box calls `exec()`, which opens its own
event loop that `processEvents()` never returns from. With the window parked
off-screen there is nothing to dismiss and nothing to see; the harness simply
hangs forever. Suppress the known ones up front *and* run a reaper timer that
closes any modal that appears anyway. This only bites on a **fresh** machine —
the author's box has the settings that keep those dialogs quiet, so it ships
broken and fails for everyone else.

**5 · Verify every navigation.** Guarded hops (`hasattr`, index checks) that
"fail safe" actually fail *silently*: the harness grabs whatever page was
already showing and saves it under the new page's filename, then reports
success. Have each screen declare the title it expects and read it back.

**Adapt:** web/Electron — Playwright `page.screenshot()`, navigate by route.
Native — the platform screenshot API plus a UI-automation driver.

**Capture at 1.5–2× and downscale.** Native 2880×1620 downsamples to a crisper
1080p than rendering at 1080p directly.

---

## C · Action recorder

The component that makes "how to do X" videos cheap. It records what the
operator *does*, and — critically — **the rectangle of the control they
clicked**.

That rectangle is the whole point. Spotlights get placed from instrumentation
instead of estimated by eye. In the first build, an eyeballed spotlight framed
empty canvas because one page used a vertical nav where every other page used a
horizontal one.

```python
# action_recorder.py  (Qt reference)
MARK_KEY = Qt.Key_F9

class ActionRecorder(QObject):
    """Inert until start(). Installs an app-wide event filter; removes it on
    stop(), so a normal session carries no overhead."""

    def eventFilter(self, obj, event):
        if not self._armed:
            return False
        try:
            if event.type() == QEvent.MouseButtonPress:
                self._record_click(obj, event)
            elif event.type() == QEvent.KeyPress and event.key() == MARK_KEY:
                self._mark_last()
        except Exception:
            log.debug("capture failed", exc_info=True)   # never break the app
        return False                                      # never consume

    def _record_click(self, obj, event):
        target = _meaningful(obj)          # walk up to a widget that names itself
        step = {
            "i": len(self._steps) + 1,
            "t": round(time.time() - self._t0, 3),
            "widget": target.__class__.__name__,
            "object_name": target.objectName(),
            "label": _widget_label(target),          # text / title / tooltip
            "rect_pct": self._rect_pct(target),      # ← the important field
            "page": self._page_title(),
            "marked": False,
        }
        step["screen"] = self._grab(step["i"])       # BEFORE the click lands
        self._steps.append(step)

    def _rect_pct(self, w):
        """Rect as % of the window — resolution-independent, which is what a
        spotlight needs."""
        win = self._window
        tl = w.mapTo(win, w.rect().topLeft())
        return {"left":   100.0 * tl.x() / win.width(),
                "top":    100.0 * tl.y() / win.height(),
                "width":  100.0 * w.width() / win.width(),
                "height": 100.0 * w.height() / win.height()}

    def _mark_last(self):
        if self._steps:
            self._steps[-1]["marked"] = True     # flags; does NOT add a step
```

### Five design rules

1. **Capture the screen *before* the click is delivered.** A beat then reads
   "here is the screen, here is what to click" — and each step's shot doubles
   as the previous step's result.
2. **The mark key flags the last step; it never records one.** The operator
   demonstrates naturally and editorialises with one key, instead of directing
   while working.
3. **Never consume the event, ever.** Always return "not handled." Recording
   must be unobservable in app behaviour.
4. **Swallow every capture failure.** A telemetry bug must not be the reason
   the app misbehaves.
5. **Store raw truth; presentation is a separate layer.** Labels came back as
   bare emoji from an icon-only nav. The recorder kept them; the *builder*
   falls back to `objectName`. Do not clean data at capture time.

**Arming:** a toolbar toggle is most discoverable but adds a permanent control
to the UI — in a safety-critical app, style it quiet when idle and unmistakable
when armed, and warn against leaving it running. A global hotkey or a CLI flag
avoids the UI change entirely. Pick deliberately.

**Cost:** a window grab per click, so expect a slight hitch. Document that.

**Adapt:** web — a capture-phase `click` listener plus
`getBoundingClientRect()` normalised by viewport; screenshot via
`html2canvas` or CDP. Electron — same, plus `capturePage()`. Native — the
accessibility API usually gives element bounds for free.

---

## D · Tape → storyboard

Turns a recording into the gate-1 artifact. Emits four things and,
deliberately, **not the composition** — the whole point is a checkpoint before
building.

```
STORYBOARD.md    beat table — delete/reorder/rewrite rows   (review gate 1)
NARRATION.md     one blank section per beat — the human's half
screens/         captured frames for the kept beats
spotlights.json  clicked-widget rects as % of frame
```

```python
def _display(step):
    """Readable name. Recorder stores raw truth; presentation is this layer.
    An icon-only control's label reads as nothing, so fall back to the name
    the code gives it."""
    label = step.get("label") or ""
    if any(c.isalpha() and ord(c) < 128 for c in label):
        return label
    return step.get("object_name") or step.get("widget") or "?"

beats = [s for s in steps if s["marked"]] or steps   # marked, else everything
```

`STORYBOARD.md` opens with the sentence that must be filled in:

```markdown
**This video teaches _[who]_ that _[one sentence]_.**  ← fill this in

| # | Where | What you clicked | Beat says… | Screen |
|---|-------|------------------|-----------|--------|
| 1 | Calibration | btn_mosaic_scan  | _(to write)_ | `01-btn-mosaic-scan.png` |
```

`NARRATION.md` is the human's half — the recording knows *what* was clicked,
never *why* it mattered.

---

## E · Composition patterns

### The spotlight

One element is both the highlight ring **and** the scrim:

```css
.spot {
  position: absolute;                 /* geometry in % — survives a resolution change */
  border: 3px solid var(--t-accent);
  border-radius: 12px;
  box-shadow: 0 0 0 9999px rgba(0, 0, 0, 0.82);   /* dims everything OUTSIDE */
  opacity: 0;
}
```

```html
<div class="spot" style="left:27%; top:9.6%; width:37.4%; height:3.4%"></div>
```

Fed straight from `spotlights.json`. Two constraints:

- **Size is static.** `width`/`height` are not on the animatable allowlist, so
  steps **cross-fade** between separate spotlight elements rather than one
  resizing.
- **Fades must be sequential, never overlapping.** Two visible box-shadows
  double-darken the scrim, which reads as a flash.

### Timeline

```js
const tl = gsap.timeline({ paused: true });        // built synchronously

// immediateRender:false — nothing mutates a later scene's elements at page
// load, and every frame is reproducible from its time alone regardless of
// seek order. Use fromTo (fully specified) rather than to/from.
tl.fromTo(sel, {opacity: 0, y: 22}, {opacity: 1, y: 0, duration: .5,
                                     immediateRender: false}, t);

window.__timelines["main"] = tl;                   // key === composition id
```

```js
function step(spotSel, callSel, tIn, tOut) {       // one teaching beat
  fade(spotSel, tIn, 0, 1, .4);
  rise(callSel, tIn + .15);
  fade(spotSel, tOut, 1, 0, .3);
  fade(callSel, tOut, 1, 0, .3);
}
```

---

# Part 4 — HyperFrames gotchas

Each of these cost time. In rough order of how likely they are to bite.

| # | Symptom | Cause / fix |
|---|---|---|
| 1 | Lint: `root_composition_missing_data_start` | Root needs `data-start="0"` alongside id/width/height/duration |
| 2 | Lint: `gsap_css_transform_conflict` | Never pair a CSS initial `transform` with a GSAP tween on the same property — set the initial state in `fromTo` |
| 3 | Later scenes unanimated or broken | Later-scene clips aren't in the DOM at load. Use `immediateRender: false`; never `gsap.set()` a later scene at load |
| 4 | Frame renders **black** although preview is fine | A full-screen fill on the composition root. Put it on a full-bleed child |
| 5 | Media renders **blank** | Duplicate `<img>`/`<video>` — same src/start/duration, or duplicate ids. Give every asset a unique id and distinct source |
| 6 | Preview and render use different fonts | The renderer aliases system fonts to its bundled font. Name the bundled font (e.g. `Roboto`) directly |
| 7 | Contrast warnings | It checks WCAG AA numerically. One pairing measured **4.49:1 against a 4.5 threshold**. Fix by colour, not by eye |
| 8 | Text overlaps itself | No `<br>` in body text — let it wrap via `max-width` |
| 9 | Capture fails: "zero duration" | Root `data-duration`, or a finite animation the runtime can measure |
| 10 | Nondeterministic frames | No clocks, unseeded random, network fetches, input state, or `repeat: -1` |
| 11 | Occlusion / contrast noise on deliberately covered layers | Mark them `data-layout-allow-occlusion` |

**Animatable allowlist:** `opacity`, `x`, `y`, `scale`, `rotation`, `color`,
`backgroundColor`, `borderRadius`, transforms. Never `width`, `height`, `top`,
`left`, `display`, or raw `visibility`. Never tween a `.clip` element itself —
the framework owns clip visibility.

**Loop:**

```bash
npx hyperframes lint                    # fast, while authoring
npx hyperframes check                   # required gate: lint + runtime + layout + motion + contrast
npx hyperframes snapshot --at 16,31,47  # LOOK at these
npx hyperframes preview                 # human gate 3
npx hyperframes render --quality high --output out.mp4
ffprobe -v error -show_format out.mp4   # verify independently
```

**Snapshot the step midpoints and actually look at them.** This is the step
that catches a spotlight pointing at the wrong control — which a passing
`check` will never tell you.

**Costs at 1080p** (~95 s video, 3 workers): check ~30 s, snapshot ~20 s,
render ~50 s. Rendering is cheap. *Deciding wrong* is expensive — which is why
the gates sit before the render, not after.

---

# Part 5 — Verifying the output

Do not trust that it worked because the command exited zero.

1. `ffprobe` the file — duration must equal the root `data-duration`, frame
   count must equal duration × fps.
2. **Extract a frame from the MP4 itself** (not the snapshot pipeline) and
   look at it. This proves the render path, and confirms late text edits
   actually landed:
   ```bash
   ffmpeg -y -ss 27.5 -i out.mp4 -frames:v 1 verify.png
   ```
3. Re-walk the `SOURCES` table and confirm every on-screen name still matches
   the code at the cited line.

---

# Part 6 — Porting checklist

Day one, in order. Stop at any point and you still have something useful.

- [ ] Install Node 22+, FFmpeg, and the HyperFrames agent skill. **Read
      `hyperframes-core` before writing composition HTML.**
- [ ] Write the collaboration contract (Part 1) into `docs/videos/DIRECTING.md`.
      Adapt the gate table; keep "never render without gate 3."
- [ ] **A** · Theme export. Verify no hex is hand-written anywhere.
- [ ] **B** · Capture harness. Prove text renders — check for tofu before
      building anything on top. Confirm it runs against a settings copy.
- [ ] Build one "where things are" video. Ship it. Learn the tooling on a
      low-stakes piece.
- [ ] **C** · Action recorder. Verify: filter removed on stop, events never
      consumed, mark flags the last step, capture failure cannot propagate.
- [ ] **D** · Tape → storyboard. Run it on a real recording end to end.
- [ ] Build one "how to do X" video from a real tape.
- [ ] Freeze the approved run as a recipe so the next one starts from proven
      structure. The freeze script ships with the agent skills, so it only
      exists if you installed them into this repo
      (`npx skills add heygen-com/hyperframes --full-depth`) — otherwise ask
      the agent to do it in a session where they are loaded.

## Conventions to carry over

- Generated files carry a `GENERATED — DO NOT EDIT` header naming their source.
- Geometry in percentages, never pixels.
- Commit: composition, generated CSS, screens. Ignore: `node_modules/`,
  `out/`, `snapshots/`.
- Tests for the recorder specifically: inert-until-armed, filter-removed-on-
  stop, never-consumes-events, mark-flags-last, failure-cannot-propagate,
  truncated-tape-still-readable.

## The three things that actually matter

Everything above is mechanism. These are the load-bearing ideas:

1. **Instrument the geometry.** The clicked rectangle removes the largest
   source of wrong output. Never eyeball coordinates against a screenshot.
2. **Gate before you render, not after.** Wrong content discovered at gate 4
   costs ~60× what it costs at gate 1.
3. **Source every claim from code, and treat safety claims as adversarial.**
   Two claims that read perfectly were false, and one of them would have told
   an operator a live control was inert.
