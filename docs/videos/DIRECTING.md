# Directing MEBP tutorial videos

You direct. I produce. This is the contract so neither of us burns effort on
work the other was going to throw away.

Videos are built with [HyperFrames](https://github.com/heygen-com/hyperframes)
(HTML → MP4). Prerequisites: Node 22+ and FFmpeg.

---

## The division

**You own the story.** What task, for whom, and what they must be able to do
afterwards. And the half that is not in the source code: *why* a step matters,
what goes wrong when it is skipped, what you would say to someone standing next
to you at the bench.

**I own accuracy and craft.** Every on-screen claim traced to a source line
before anything is built; layout, spotlight geometry, motion, checks, render.

That split is not arbitrary. In the first tour I wrote two claims that read
perfectly and were both false — "the rest stay closed" (Settings stays
enabled too) and "nothing on this page moves the machine" (the Device jog pad
bypasses soft limits). Neither was catchable from the docs; both came out of
the source. Equally, nothing in the source says which step people get wrong in
practice. That is yours.

---

## The four gates

Cheapest first. Kill an idea at gate 1 for a minute of my time, or at gate 4
for an hour of both of ours.

| Gate | Artifact | What you do | Cost to change |
|---|---|---|---|
| **1 · Beats** | `STORYBOARD.md` | Delete, reorder, rewrite rows | ~1 min |
| **2 · Sketches** | Wireframe board in Studio | Confirm the layout and the words | ~5 min |
| **3 · Preview** | Studio timeline | Scrub it. **Edit text and timing yourself.** | ~10 min |
| **4 · Render** | MP4 | Watch it | ~50 min if the story was wrong |

**I do not render without gate 3.** In the first session I rendered three times
because I found factual errors late. That is the waste this exists to prevent.

Gate 3 is real control, not a rubber stamp — `npx hyperframes preview` opens
Studio, where you can retime clips and edit copy directly. Changes you make
there are changes to the project.

---

## What I need from you, per video

Four things, one message:

1. **One sentence.** *"After watching, a new operator can re-anchor the plate
   map after a remount."* If we cannot write this, the video has no spine.
2. **A tape** (record it — see below) or a list of screens.
3. **The lab reality.** Why it matters, what breaks, the thing you always end
   up telling people.
4. **Words.** Terms to use, terms to avoid.

## What you get from me

- A beat sheet within minutes of the tape, not a finished video days later.
- Every claim sourced. If I cannot find it in the code, I ask instead of
  guessing.
- Spotlights placed from the widget you actually clicked, not eyeballed.
- A flag whenever the recording shows something the code contradicts — that
  usually means a real bug, and it is worth more than the video.

## After the first approved video

I freeze it as a HyperFrames **recipe**, so the next one of the same kind
starts from the proven structure instead of a blank page:

```bash
node .agents/skills/media-use/scripts/recipe.mjs freeze --name mebp-walkthrough
```

---

## Recording a walkthrough

**● REC** in the top bar, next to Help.

1. Click **● REC**. It turns red and counts steps.
2. Do the task, at normal pace. Every click is captured with a screenshot.
3. Press **F9** right after any step that matters — that is you saying
   *"this one is a teaching beat."* Everything else is kept as context.
4. Click **● REC** again. The console prints the tape path.

Then:

```bash
python tools_build_tour_from_tape.py logs/sessions/<stamp>/tape.jsonl
```

You get `STORYBOARD.md` (gate 1), `NARRATION.md` (one blank section per beat,
for the *why*), `screens/`, and `spotlights.json`. Fill in the two markdown
files and hand them back.

### Notes on recording

- **You cannot mark too much.** Unmarked steps are still recorded; F9 only
  chooses the spine. Re-run the tool with `--all-steps` to see everything.
- **Narrate afterwards, not during.** `NARRATION.md` has a section per beat.
  Hands stay on the machine while recording.
- **It captures the screen *before* each click** — so a beat reads "here is the
  screen, here is what to click", and each step's shot doubles as the previous
  step's result.
- **It never alters behaviour.** The event filter always passes events through
  and is installed only while armed. A capture failure is swallowed and logged;
  the recorder must never be the reason the app misbehaves.
- **Recording costs a window grab per click**, so expect a slight hitch on each
  one. Do not leave it armed during a real print.
- **Closing the app mid-recording still writes the tape.**

### If you are not recording

`tools_capture_app_screens.py` grabs whole pages without any interaction —
useful for tours of *where things are* rather than *how to do something*. It
runs against a copy of `settings.json` and never touches your real calibration.

---

## Conventions worth keeping

- `styles/mocha.css` in each project is **generated** from `gui/styles.py`
  by `tools_export_theme_tokens.py`. Never hand-write a colour.
- Spotlight geometry is in **percentages of the frame**, so it survives a
  change of capture resolution.
- Snapshot the step midpoints and *look* before rendering. This already caught
  a spotlight framing empty canvas, because Print Builder's sub-nav is a
  vertical icon column rather than the horizontal strip every other mode page
  uses.
- Committed: composition, generated CSS, screens. Ignored: `node_modules/`,
  `out/`, `snapshots/`.
