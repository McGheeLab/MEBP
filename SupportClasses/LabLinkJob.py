"""
LabLinkJob.py — turn a MEBP imaging output into a LabLink job pair.

v7.17. A LabLink job is **two files sharing a stem**, both uploaded to the
session's input channel::

    mebp_A1_20260808T142530_7f3a.nd3        the image
    mebp_A1_20260808T142530_7f3a.job.json   the sidecar

The pairing is on the stem, and the suffix is `.job.json` rather than `.json`
so a bare `x.json` sitting beside `x.nd3` can never be silently adopted as one.

**Why the sidecar exists** (`lablink/docs/IMAGE-JOB-FORMAT.md` §1): the
analysis derives real parameters from optical metadata — a deconvolution PSF
is computed per channel from the objective's NA and the channel's emission
wavelength — so an image without it does not fail, it produces *different
numbers*. Measured on real data: supplying NA and emission moved a segmented
object count from **2855 to 2660**, with no warning from any layer. Worse, a
container does not merely omit those fields, it **invents** two of them (a
TIFF reports its own bit depth rather than the sensor's, and names the channel
`Ch0`). So the sidecar **overrides** the image; it does not fill gaps.

**The sidecar is derived by reading the written file back**, never from the
in-memory values that produced it. A sidecar that describes something other
than the bytes being sent is the one failure this format exists to prevent,
and re-reading is what makes desync impossible rather than merely unlikely.

`.nd3` became readable by the hub on 2026-08-08: the ND2 Studios engine
ingests it natively (`nodelab_v2/nd3_ingest.py` in that repo) and every
`nd2studios` recipe's `inputs[].match` includes `*.nd3`. `ND3_NOT_YET_ACCEPTED`
stays exported (now False) so a UI that checked it keeps working — flip it
back only if a hub is deployed from a lablink older than commit 7123061 plus
the `*.nd3` recipe change.
"""

from __future__ import annotations

import json
import logging
import re
import secrets
import shutil
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

SIDECAR_FORMAT = "lablink.imagejob/1"
SIDECAR_SUFFIX = ".job.json"

#: See the module docstring. False since 2026-08-08: the hub's recipes accept
#: `*.nd3` and the ND2 Studios engine reads the container natively.
ND3_NOT_YET_ACCEPTED = False

#: Upload-name rule, verbatim from the hub (`protocol.NAME_RE`). Duplicated
#: here ONLY so a name can be validated with no import of the vendored client;
#: a test asserts the two are identical, so they cannot drift.
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._ -]{0,127}$")
WINDOWS_RESERVED = (
    {"CON", "PRN", "AUX", "NUL"}
    | {f"COM{i}" for i in range(1, 10)}
    | {f"LPT{i}" for i in range(1, 10)}
)

#: Budget for the stem. The reply side appends a suffix and possibly the
#: produced file's own stem against the SAME 128-char ceiling, and an output
#: whose name overflows is silently not returned — so leave it room.
MAX_STEM = 60


class LabLinkJobError(Exception):
    """The job cannot be built. Carries an operator-readable reason."""


@dataclass
class JobPair:
    """One image + its sidecar, ready to upload."""

    image_path: Path
    sidecar_path: Path
    stem: str
    sidecar: dict = field(default_factory=dict)
    #: Fields a recipe may require that we could not supply. Never guessed.
    missing: tuple = ()

    def files(self) -> tuple:
        return (self.image_path, self.sidecar_path)


# ── Naming ────────────────────────────────────────────────────────

def sanitize_hint(value) -> str:
    """A short `[A-Za-z0-9]`-only fragment, or `""`.

    ⚠ Deliberately lossy AND deliberately not load-bearing. It exists so a
    human can recognise a filename on the hub; nothing parses it back. The
    identity of a job lives in the stored job record, never in its name —
    which is why a many-to-one squash here is safe, where the same squash in
    `FluorescenceMosaicStore._safe_token` is the trap the v7.12 notes record.
    """
    return re.sub(r"[^A-Za-z0-9]", "", str(value or ""))[:16]


#: Bytes of randomness in a stem. ⚠ MEASURED, not chosen for looks: the
#: timestamp resolves to one second, so everything inside that second is
#: distinguished by the nonce alone, and a repeat is a 409 — permanently
#: non-retryable, i.e. the exact failure the unique name exists to prevent.
#: Expected collisions over 10 000 same-second names: 2 bytes -> 763,
#: 3 -> 3.0, 4 -> 0.012. Two bytes was the first cut and its own test caught
#: it at 729 collisions.
NONCE_BYTES = 4


def build_stem(prefix: str, hint: str = "", *, now: Optional[datetime] = None,
               nonce: str = "") -> str:
    """A stem that is unique per acquisition and legal as an upload name.

    ``{prefix}_{hint}_{stamp}_{nonce}``.

    ⚠ Uniqueness is not cosmetic. The hub treats same-name-different-content
    as **409 Conflict**, which is 4xx and therefore never retried — so a name
    derived from MEBP's own deterministic store filenames (which a re-scan
    overwrites) would upload the first scan of a well and permanently reject
    every one after it. `QUICKSTART.md` prescribes exactly this remedy:
    *"Give every file a unique name — put a timestamp in it. This single habit
    avoids the whole problem."*
    """
    clean_prefix = re.sub(r"[^A-Za-z0-9]", "", str(prefix or "")) or "mebp"
    stamp = (now or datetime.now()).strftime("%Y%m%dT%H%M%S")
    rand = nonce or secrets.token_hex(NONCE_BYTES)
    parts = [clean_prefix[:16]]
    hint = sanitize_hint(hint)
    if hint:
        parts.append(hint)
    parts += [stamp, rand]
    stem = "_".join(parts)
    if len(stem) > MAX_STEM:          # drop the human affordance, never the id
        stem = "_".join([clean_prefix[:16], stamp, rand])
    return stem


def validate_upload_name(name: str) -> str:
    """Return *name* or raise with the reason. Refuses; never sanitizes.

    Called on the RENDERED name at config-edit time as well as at send time,
    so a name the hub would reject cannot reach the wire.
    """
    if not isinstance(name, str) or not name:
        raise LabLinkJobError("an upload name is required")
    if "/" in name or "\\" in name:
        raise LabLinkJobError(f"{name!r} must be a single path segment")
    if not NAME_RE.match(name):
        raise LabLinkJobError(
            f"{name!r} is not a legal LabLink upload name — letters, digits, "
            f"dot, underscore, space and hyphen only, starting with a letter "
            f"or digit, 128 characters max")
    if name[-1] in (".", " "):
        raise LabLinkJobError(f"{name!r} must not end with a dot or a space")
    if name.split(".")[0].upper() in WINDOWS_RESERVED:
        raise LabLinkJobError(f"{name!r} is a reserved Windows device name")
    return name


# ── Sidecar ───────────────────────────────────────────────────────

def _positive(value):
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return f if f > 0 else None


def build_sidecar(nd3_path, *, optics_lookup=None, objective_na=None,
                  bit_depth=None, recipe: str = "", knobs=None,
                  original_name: str = "", note: str = "") -> tuple:
    """Read *nd3_path* back and describe it. Returns ``(sidecar, missing)``.

    ``optics_lookup`` maps a channel name to ``{"emission_nm", "excitation_nm"}``
    — in production, ``MicroscopeConfigStore.filter_optics_for``.

    Refuses rather than guesses:

    * no ``scale.um_per_px`` on any image → refused (`pixel_size_um` is
      required and everything spatial derives from it);
    * images disagreeing on pixel size → refused. The sidecar carries ONE
      `pixel_size_um`; picking a channel's silently would put every µm² figure
      in the results wrong while still looking plausible;
    * sensor bit depth unknown → refused unless passed. The array's dtype is
      the CONTAINER depth — exactly the invented value the sidecar exists to
      override — so it is never a fallback.

    ``missing`` names recipe-dependent fields we could not supply. They are
    left OUT of the sidecar rather than filled with nominal values: the hub
    answers `missing_metadata` naming them, which a client can usually repair,
    whereas a plausible wrong number cannot be detected by anyone.
    """
    from SupportClasses import ND3

    nd3_path = Path(nd3_path)
    with ND3.open_nd3(nd3_path) as reader:
        ids = list(reader.image_ids())
        if not ids:
            raise LabLinkJobError(f"{nd3_path.name} contains no images")
        dataset = dict(reader.dataset_meta or {})
        pitches, found, acquisitions = [], [], []
        for order, image_id in enumerate(ids):
            img = reader.image(image_id)
            meta = dict(img.meta or {})
            acquisitions.append(dict(meta.get("acquisition") or {}))
            pitch = _positive((meta.get("scale") or {}).get("um_per_px"))
            if pitch is None:
                raise LabLinkJobError(
                    f"image {image_id!r} in {nd3_path.name} has no "
                    f"scale.um_per_px — LabLink requires pixel_size_um, and "
                    f"everything spatial derives from it")
            pitches.append(pitch)
            entries = list(img.channels or [])
            if not entries:
                # A single-channel container may name it on the acquisition.
                name = str((meta.get("acquisition") or {}).get("channel")
                           or dataset.get("channel") or "").strip()
                if name:
                    entries = [{"name": name}]
            for entry in entries:
                name = str(entry.get("name") or "").strip()
                if not name:
                    continue
                # ⚠ ORDER IS LOAD-BEARING. The sidecar's `channels` array is
                # "one entry per channel, IN ACQUISITION ORDER … it is how
                # channel indices resolve" — but ND3Reader.image_ids() returns
                # sorted() ids, i.e. ALPHABETICAL. Left alone, a DAPI/FITC/Cy5
                # well would be declared Cy5, DAPI, FITC and a recipe indexing
                # by position would analyse the wrong channel and produce a
                # result that looks entirely normal. `channel_number` is the
                # microscope's filter position (DAPI=1 … Bright Field=5), the
                # only acquisition-order fact the container carries; ids are
                # the tie-break so the result is at least deterministic.
                number = entry.get("channel_number")
                try:
                    number = int(number)
                except (TypeError, ValueError):
                    number = None
                found.append((number, order, name, dict(entry)))

    # None sorts last: a channel with no recorded number cannot be placed, and
    # trailing is the least destructive place to put it.
    found.sort(key=lambda t: (t[0] is None, t[0] if t[0] is not None else 0, t[1]))
    channels = [(name, entry) for _, _, name, entry in found]

    spread = max(pitches) - min(pitches)
    if spread > 1e-9 and spread / max(pitches) > 1e-6:
        raise LabLinkJobError(
            f"{nd3_path.name} holds images at different pixel sizes "
            f"({min(pitches):.6g}-{max(pitches):.6g} µm/px) but the sidecar "
            f"carries a single pixel_size_um. Send them as separate jobs — a "
            f"single value here would make every µm² figure in the results "
            f"wrong while still looking plausible.")

    if not channels:
        raise LabLinkJobError(
            f"{nd3_path.name} declares no channel name. LabLink requires the "
            f"REAL name (a placeholder like Ch0 is the trap the sidecar "
            f"exists to prevent).")

    acq = acquisitions[0] if acquisitions else {}
    sensor_bits = bit_depth
    if sensor_bits is None:
        m = re.match(r"^\s*(\d+)", str(acq.get("bit_depth", "")))
        if m:
            sensor_bits = int(m.group(1))
    if sensor_bits is None:
        raise LabLinkJobError(
            f"the sensor bit depth for {nd3_path.name} is unknown — pass "
            f"bit_depth= explicitly. The array's dtype is the CONTAINER "
            f"depth, the invented value the sidecar exists to override, so it "
            f"is deliberately not used as a fallback.")

    image_block: dict = {
        "pixel_size_um": float(pitches[0]),
        "bit_depth": int(sensor_bits),
    }
    mag = acq.get("magnification") or acq.get("objective", "")
    m = re.match(r"^\s*(\d+(?:\.\d+)?)\s*[xX×]", str(mag))
    if m:
        image_block["objective_magnification"] = float(m.group(1))

    missing = []
    na = _positive(objective_na if objective_na is not None
                   else acq.get("numerical_aperture"))
    if na is not None:
        image_block["objective_na"] = na
    else:
        missing.append("objective_na")

    seen, entries = set(), []
    for name, file_entry in channels:
        if name in seen:
            continue
        seen.add(name)
        entry = {"name": name}
        optics = (optics_lookup(name) if callable(optics_lookup) else {}) or {}
        for src, dst in (("emission_nm", "emission_nm"),
                         ("excitation_nm", "excitation_nm")):
            nm = _positive(optics.get(src))
            if nm is None:
                # ND3_SPEC §14: an ND3 channel entry MAY carry its filter
                # wavelengths, and this bridge passes them through when
                # present. The lookup (the microscope's own config) stays
                # authoritative; the file is the fallback — both are recorded
                # facts, never nominal fluorophore-table values.
                nm = _positive(file_entry.get(src))
            if nm is not None:
                entry[dst] = nm
        if "emission_nm" not in entry:
            missing.append(f"channel_emission_nm ({name})")
        entries.append(entry)
    image_block["channels"] = entries

    sidecar: dict = {"format": SIDECAR_FORMAT, "image": image_block}

    # ⚠ `job` is OPTIONAL and the API's recipe/knobs are authoritative. If the
    # two disagree on any value the hub REFUSES the job rather than letting one
    # silently win, so this block is written only when the caller passes the
    # exact values it will also send on the wire.
    if recipe or knobs is not None:
        job: dict = {}
        if recipe:
            job["recipe"] = str(recipe)
        if knobs is not None:
            job["knobs"] = knobs
        sidecar["job"] = job

    source = {}
    if original_name:
        source["original_name"] = str(original_name)
    if note or dataset.get("notes"):
        source["note"] = str(note or dataset.get("notes"))
    acquired = dataset.get("created_iso") or acq.get("timestamp_iso")
    if acquired:
        source["acquired"] = str(acquired)
    if source:
        sidecar["source"] = source

    return sidecar, tuple(missing)


def write_job_pair(nd3_path, out_dir, stem: str, sidecar: dict, *,
                   move: bool = False) -> JobPair:
    """Place ``<stem>.nd3`` + ``<stem>.job.json`` in *out_dir*.

    The image is copied (or moved) rather than referenced, so what gets
    uploaded is an immutable snapshot: MEBP's own stores use deterministic
    filenames that a re-scan overwrites, and `FluorescenceMosaicStore` writes
    with a bare `cv2.imwrite` (no tmp+replace), so a live path can be torn or
    swapped between enqueue and send.
    """
    nd3_path, out_dir = Path(nd3_path), Path(out_dir)
    image_name = validate_upload_name(f"{stem}.nd3")
    sidecar_name = validate_upload_name(f"{stem}{SIDECAR_SUFFIX}")
    out_dir.mkdir(parents=True, exist_ok=True)

    image_path = out_dir / image_name
    if move:
        shutil.move(str(nd3_path), str(image_path))
    else:
        shutil.copy2(str(nd3_path), str(image_path))

    sidecar_path = out_dir / sidecar_name
    tmp = sidecar_path.with_name(sidecar_path.name + ".tmp")
    tmp.write_text(json.dumps(sidecar, indent=2, sort_keys=True),
                   encoding="utf-8")
    tmp.replace(sidecar_path)
    return JobPair(image_path=image_path, sidecar_path=sidecar_path,
                   stem=stem, sidecar=sidecar)


def describe_missing(missing) -> str:
    """One operator-readable sentence, or ``""`` when nothing is missing."""
    if not missing:
        return ""
    return (
        "This job omits " + ", ".join(missing) + ". A recipe with a "
        "deconvolution step derives from those and will refuse with "
        "missing_metadata. Fill the filter cube's wavelengths in on "
        "Hardware Setup → Microscope; MEBP does not guess them, because a "
        "nominal value changes the result silently.")
