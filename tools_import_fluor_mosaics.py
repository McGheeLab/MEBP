"""tools_import_fluor_mosaics.py — fold another machine folder's fluorescence
mosaics into this machine's store, without losing either side.

WHY THIS EXISTS
---------------
Before v7.21.6 a channel image was named ``<plate>_<well>_<channel>.png`` — one
slot per (plate, well, channel) — so re-scanning a well overwrote the previous
capture's pixels and replaced its metadata record. On ME3B_01 the only reason
any pre-v7.21.6 capture survived is an accident: the machine id changed
(``ME3B_2`` → ``ME3B_01``), which forked the whole per-machine folder and left
the older snapshot sitting untouched in ``config/hardware/ME3B_2/``.

This tool moves that snapshot into the live store as ARCHIVED captures:

  * a (plate, well, channel) the live store does NOT have  → imported ACTIVE
    (it becomes visible in the app again)
  * a (plate, well, channel) the live store DOES have      → imported into that
    channel's ``history`` (the current capture stays active — importing an older
    image over a newer one is exactly the overwrite this release removes)

Every imported file is COPIED under a v7.21.6 timestamped name derived from the
source record's own capture date, so it can never collide with a live one. The
source folder is left completely untouched, so a bad run costs nothing.

USAGE
-----
    python tools_import_fluor_mosaics.py --from ME3B_2 --dry-run
    python tools_import_fluor_mosaics.py --from ME3B_2
    python tools_import_fluor_mosaics.py --from ME3B_2 --history-only

    --from            source machine folder name, or a path to a
                      fluorescence_mosaics.json / the folder holding one
    --into            destination machine folder (default: the active machine)
    --dry-run         report what would happen; write nothing
    --history-only    never import as ACTIVE — archive everything, so what the
                      app displays today cannot change
    --active-anyway   also let an imported capture REPLACE a live active one
                      when it is genuinely newer (off by default)

⚠ An imported ``extent_um`` is the absolute stage µm of its own capture session.
If the plate has been re-seated or re-taught since, an old mosaic will overlay
at its capture-time position, not today's — the same caveat that already applies
to any older capture in the store (this store has no re-anchor concept). The
pixels are correct; the georeference is as good as the calibration it was taken
under.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from SupportClasses import MachineConfig  # noqa: E402
from SupportClasses.FluorescenceMosaicStore import (  # noqa: E402
    FluorescenceMosaicStore, capture_stamp, channel_image_name, well_key,
)

HW = Path(__file__).resolve().parent / "config" / "hardware"


def resolve_source(spec: str) -> Path:
    """Accept a machine-folder name, a folder, or the json path itself."""
    cands = [Path(spec), Path(spec) / "fluorescence_mosaics.json",
             HW / spec / "fluorescence_mosaics.json", HW / spec]
    for c in cands:
        if c.is_file():
            return c
        if c.is_dir() and (c / "fluorescence_mosaics.json").is_file():
            return c / "fluorescence_mosaics.json"
    raise SystemExit(f"no fluorescence_mosaics.json found for --from {spec!r}\n"
                     f"  looked in: {', '.join(str(c) for c in cands)}")


def stamp_for(record: dict, fallback_mtime: float) -> str:
    """A filename stamp for an imported record, from what it actually knows.

    Priority: its own ``captured_at`` → its ``date`` (midday, so the stamp does
    not imply a precision the record does not have) → the file's mtime. A wrong
    stamp here is only a filename; the record's own ``date`` field is what any
    reader reports, and that is carried across verbatim.
    """
    ca = str(record.get("captured_at") or "")
    if ca:
        try:
            return capture_stamp(datetime.fromisoformat(ca))
        except ValueError:
            pass
    d = str(record.get("date") or "")
    if d:
        try:
            return datetime.fromisoformat(d).strftime("%Y%m%d-120000")
        except ValueError:
            pass
    return capture_stamp(datetime.fromtimestamp(fallback_mtime))


def newer(a: dict, b: dict) -> bool:
    """True when record ``a`` is strictly newer than ``b`` (unknown → False)."""
    ka = str(a.get("captured_at") or a.get("date") or "")
    kb = str(b.get("captured_at") or b.get("date") or "")
    return bool(ka and kb and ka > kb)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from", dest="src", required=True)
    ap.add_argument("--into", dest="dst", default=None)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--history-only", action="store_true")
    ap.add_argument("--active-anyway", action="store_true")
    args = ap.parse_args()

    src_json = resolve_source(args.src)
    src_dir = src_json.parent
    if args.dst:
        os.environ["MEBP_MACHINE_ID"] = args.dst
    store = FluorescenceMosaicStore()
    dst_json, dst_dir = store._path, store._img_dir
    if src_json.resolve() == dst_json.resolve():
        raise SystemExit("source and destination are the same store")

    print(f"source      : {src_json}")
    print(f"destination : {dst_json}")
    print(f"machine     : {MachineConfig.machine_id()}")
    print(f"mode        : {'DRY RUN — nothing will be written' if args.dry_run else 'WRITING'}"
          f"{'  (history-only)' if args.history_only else ''}\n")

    def already_imported(plate, wname, ch, tag: str) -> bool:
        """True when a previous run of this tool already brought ``tag`` in.

        Makes the tool safely RE-RUNNABLE — which matters, because the app
        rewrites this json from memory on every capture, so an import can be
        reverted underneath you and need repeating. Without this a repeat run
        would copy all 33 images again under fresh names.
        """
        live = store._channel_meta(plate, wname, ch)
        if live is None:
            return False
        for rec in [live] + [h for h in (live.get("history") or [])
                             if isinstance(h, dict)]:
            if str(rec.get("imported_from") or "") == tag:
                return True
        return False

    src = json.loads(src_json.read_text(encoding="utf-8"))
    plan: list[tuple] = []
    for wk, well in (src.get("wells") or {}).items():
        plate = well.get("plate_key") or wk.split("|")[0]
        wname = well.get("well_name") or wk.split("|")[-1]
        for ch, rec in (well.get("channels") or {}).items():
            for cand in [rec] + [h for h in (rec.get("history") or [])
                                 if isinstance(h, dict)]:
                rel = str(cand.get("image") or "")
                img = src_dir / rel.replace("fluor_mosaics/", "fluor_mosaics" + os.sep)
                if not rel or not img.is_file():
                    plan.append(("MISSING", plate, wname, ch, cand, None, rel))
                    continue
                if already_imported(plate, wname, ch,
                                    f"{src_dir.name}/{rel.split('/')[-1]}"):
                    plan.append(("SKIP-DONE", plate, wname, ch, cand, None, rel))
                    continue
                live = store._channel_meta(plate, wname, ch)
                if live is None and not args.history_only:
                    action = "ACTIVE"
                elif (live is not None and not args.history_only
                      and args.active_anyway and newer(cand, live)):
                    action = "ACTIVE(newer)"
                else:
                    action = "HISTORY"
                plan.append((action, plate, wname, ch, cand, img, rel))

    counts: dict[str, int] = {}
    for action, plate, wname, ch, rec, img, rel in plan:
        counts[action] = counts.get(action, 0) + 1
        size = f"{img.stat().st_size / 1e6:5.1f}MB" if img else "  ——  "
        print(f"  {action:13s} {plate:20s} {wname:4s} {ch:14s} "
              f"{str(rec.get('date') or '?'):10s} {size}  {rel.split('/')[-1]}")

    print("\n" + "  ".join(f"{k}={v}" for k, v in sorted(counts.items())))
    if args.dry_run:
        print("\nDry run — no files copied, no metadata written.")
        return 0

    dst_dir.mkdir(parents=True, exist_ok=True)
    backup = dst_json.with_suffix(
        dst_json.suffix + f".bak-preimport-{capture_stamp()}")
    shutil.copy2(dst_json, backup) if dst_json.is_file() else None
    if dst_json.is_file():
        print(f"\nmetadata backup: {backup.name}")

    copied = failed = 0
    for action, plate, wname, ch, rec, img, rel in plan:
        if img is None:
            continue
        stamp = stamp_for(rec, img.stat().st_mtime)
        name = store._unique_image_name(
            plate, wname, ch,
            datetime.strptime(stamp, "%Y%m%d-%H%M%S"))
        try:
            shutil.copy2(img, dst_dir / name)
        except OSError as exc:
            print(f"  !! copy failed {img.name}: {exc}")
            failed += 1
            continue
        new_rec = {k: v for k, v in rec.items() if k != "history"}
        new_rec["image"] = f"fluor_mosaics/{name}"
        # The processed sibling is NOT carried over: it is a derived artefact
        # whose name is paired to the raw stem, and the raw is re-processable.
        new_rec.pop("processed_image", None)
        new_rec.pop("processing", None)
        new_rec["imported_from"] = f"{src_dir.name}/{rel.split('/')[-1]}"
        if action.startswith("ACTIVE"):
            existing = store._channel_meta(plate, wname, ch)
            wells = store._data.setdefault("wells", {})
            entry = wells.setdefault(well_key(plate, wname), {
                "plate_key": str(plate), "well_name": str(wname), "channels": {}})
            entry.setdefault("channels", {})
            if existing is not None:
                new_rec["history"] = ([{k: v for k, v in existing.items()
                                        if k != "history"}]
                                      + list(existing.get("history") or []))
            entry["channels"][str(ch)] = new_rec
            if not entry.get("objective") and rec.get("objective"):
                entry["objective"] = rec["objective"]
            store._save_meta()
        else:
            if store._channel_meta(plate, wname, ch) is None:
                # history-only mode with nothing live to attach to: seed the
                # channel from this record so the image is still reachable.
                wells = store._data.setdefault("wells", {})
                entry = wells.setdefault(well_key(plate, wname), {
                    "plate_key": str(plate), "well_name": str(wname),
                    "channels": {}})
                entry.setdefault("channels", {})[str(ch)] = new_rec
                store._save_meta()
            else:
                store.add_history_entry(plate, wname, ch, new_rec)
        copied += 1

    print(f"\ncopied {copied} image(s) into {dst_dir}"
          + (f", {failed} failed" if failed else ""))
    print("source folder left untouched.")
    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
