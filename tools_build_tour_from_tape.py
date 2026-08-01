# -*- coding: utf-8 -*-
"""tools_build_tour_from_tape.py - turn a recorded walkthrough (logs/sessions/
<stamp>/tape.jsonl, written by the top-bar REC button) into a HyperFrames
tutorial project.

Usage:  python tools_build_tour_from_tape.py <tape.jsonl> [-o docs/videos/<name>]
        python tools_build_tour_from_tape.py <tape.jsonl> --all-steps

Emits, into the output directory:

  STORYBOARD.md   the beat sheet - REVIEW GATE 1. One row per beat; edit or
                  delete rows here before anything gets built.
  NARRATION.md    a numbered worksheet, one blank line per beat, for writing
                  what each step is FOR. This is the half only the operator
                  can supply; the recorder captures what was clicked, never
                  why it mattered.
  screens/        the captured frames for the kept beats.
  spotlights.json the clicked widget rectangles, as % of frame.

By default only steps you flagged with F9 become beats; --all-steps keeps
every click. It deliberately does NOT write index.html - the composition is
built after the beat sheet and narration come back approved, which is the
whole point of having a gate."""
import argparse
import io
import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def _load(tape_path):
    from gui.action_recorder import read_tape
    return read_tape(tape_path)


def _has_letters(text):
    return any(c.isalpha() and ord(c) < 128 for c in (text or ""))


def _display(step):
    """Readable name for a clicked widget.

    The recorder stores raw truth; presentation is this tool's job. An
    icon-only control (the collapsed nav rail is pure emoji) has a label that
    reads as nothing, so fall through to its objectName — which is how the
    code names it, and what the director will recognise."""
    label = step.get("label") or ""
    if _has_letters(label):
        return label
    return step.get("object_name") or label or step.get("widget") or "?"


def _slug(step):
    base = _display(step)
    keep = [c.lower() if (c.isalnum() and ord(c) < 128) else "-" for c in base]
    out = "".join(keep).strip("-")
    while "--" in out:
        out = out.replace("--", "-")
    return (out or "step")[:44]


def _where(step):
    page, sub = step.get("page", ""), step.get("sub_page", "")
    if page and sub and sub != page:
        return "%s / %s" % (page, sub)
    return page or sub or "?"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("tape", help="path to tape.jsonl")
    ap.add_argument("-o", "--out", default=None,
                    help="output project dir (default: docs/videos/<session>)")
    ap.add_argument("--all-steps", action="store_true",
                    help="keep every click, not just F9-marked beats")
    args = ap.parse_args(argv)

    tape_path = os.path.abspath(args.tape)
    if not os.path.exists(tape_path):
        raise SystemExit("no such tape: %s" % tape_path)
    session_dir = os.path.dirname(tape_path)
    session_name = os.path.basename(session_dir) or "walkthrough"

    header, steps = _load(tape_path)
    if not steps:
        raise SystemExit("tape has no steps: %s" % tape_path)

    marked = [s for s in steps if s.get("marked")]
    if args.all_steps or not marked:
        beats, why = steps, ("--all-steps" if args.all_steps else
                             "no F9-marked steps in this tape")
        if not marked and not args.all_steps:
            print("NOTE: %s - keeping all %d steps. Prune STORYBOARD.md by "
                  "hand, or re-record using F9 to mark the beats."
                  % (why, len(steps)))
    else:
        beats = marked

    out = os.path.abspath(args.out or os.path.join(
        HERE, "docs", "videos", session_name))
    screens_dir = os.path.join(out, "screens")
    os.makedirs(screens_dir, exist_ok=True)

    spotlights, rows = [], []
    for n, step in enumerate(beats, start=1):
        shot = step.get("screen") or ""
        src = os.path.join(session_dir, shot) if shot else ""
        dest_name = "%02d-%s.png" % (n, _slug(step))
        if src and os.path.exists(src):
            shutil.copy2(src, os.path.join(screens_dir, dest_name))
        else:
            dest_name = ""
        spotlights.append({
            "beat": n,
            "screen": dest_name,
            "rect_pct": step.get("rect_pct"),
            "target": _display(step),
            "widget": step.get("widget", ""),
            "where": _where(step),
            "t": step.get("t"),
        })
        rows.append((n, _where(step), _display(step), dest_name))

    with io.open(os.path.join(out, "spotlights.json"), "w",
                 encoding="utf-8") as fh:
        json.dump({"session": session_name,
                   "window": header.get("window"),
                   "beats": spotlights}, fh, indent=2)

    # ── STORYBOARD.md — review gate 1 ───────────────────────────
    sb = [
        "# Storyboard — %s" % session_name,
        "",
        "Recorded %s · %d step(s) captured, %d kept as beats."
        % (header.get("created", "?"), len(steps), len(beats)),
        "",
        "> **This is review gate 1.** Change it here — reordering a row costs",
        "> seconds; changing your mind after the render costs an hour.",
        "> Delete beats you do not want. Reorder freely. Then fill in",
        "> `NARRATION.md` and hand both back.",
        "",
        "**This video teaches _[who]_ that _[one sentence]_.**  ← fill this in",
        "",
        "| # | Where | What you clicked | Beat says… | Screen |",
        "|---|-------|------------------|-----------|--------|",
    ]
    for n, where, target, shot in rows:
        sb.append("| %d | %s | %s | _(to write)_ | `%s` |"
                  % (n, where, target, shot or "—"))
    sb += [
        "",
        "## Notes",
        "",
        "- Spotlight geometry is taken from the widget you actually clicked",
        "  (`spotlights.json`), so highlights land on the real control rather",
        "  than being eyeballed against a screenshot.",
        "- Every factual claim added to a beat gets traced to a source line",
        "  before the composition is built.",
        "",
    ]
    with io.open(os.path.join(out, "STORYBOARD.md"), "w",
                 encoding="utf-8") as fh:
        fh.write("\n".join(sb))

    # ── NARRATION.md — the operator's half ──────────────────────
    nar = [
        "# Narration — %s" % session_name,
        "",
        "One line per beat: **why** this step, and what goes wrong without it.",
        "The recording knows what was clicked; only you know what it was for.",
        "",
        "Useful things to say: what you are looking for on screen, what a good",
        "result looks like, what you would warn a new operator about here.",
        "",
    ]
    for n, where, target, _shot in rows:
        nar += ["## %d · %s — %s" % (n, where, target), "", "", ""]
    with io.open(os.path.join(out, "NARRATION.md"), "w",
                 encoding="utf-8") as fh:
        fh.write("\n".join(nar))

    print("beats:      %d (of %d captured steps)" % (len(beats), len(steps)))
    print("screens:    %s" % screens_dir)
    print("storyboard: %s" % os.path.join(out, "STORYBOARD.md"))
    print("narration:  %s" % os.path.join(out, "NARRATION.md"))
    print("\nNext: prune STORYBOARD.md, fill NARRATION.md, hand both back.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
