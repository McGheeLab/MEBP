"""
NeedleFocusTemplateStore.py — in-focus needle reference images + the needle's
offset from the microscope camera centre.

v7.5.x: during the plate-Z touch-off the operator clicks the needle tip in the
live microscope view at the moment the needle is confirmed sitting on the glass.
Two different things are learned from that one gesture:

1. **An in-focus needle template.** The image patch around the click, captured
   with the needle AT the plate bottom. At later wells the software template-
   matches the live frame against these patches and compares the ROI focus score,
   so it can tell the operator "this looks like the in-focus needle from the last
   well" instead of leaving them to judge it by eye. Purely ADVISORY — the
   operator still presses Set; nothing here moves the stage or records a Z.

2. **The needle's offset from the camera centre**, in stage µm. Every live-view
   click→target path in the app currently assumes the needle sits exactly under
   the camera crosshair. It does not, so each of those targets is off by this
   vector. Measuring it once makes the correction available (see
   ``StageController.needle_target_xy_for_feature_um``); APPLYING it to the
   existing workflows is deliberately a separate change, because click-to-CENTRE
   (imaging, mosaic) must NOT apply it while click-to-PICK must.

KEY: ``"<camobj>|<needle_type>:<bore_um>"``
    A template is a picture of THIS needle at THIS magnification, so it depends
    on the camera + objective (``camobj``, from
    ``CalibrationPage._ploc_camera_objective_key()``) and on the needle — a 22 G
    and a 30 G look nothing alike. It does NOT depend on the plate: reusing the
    reference across plates is the whole point. ``needle_tip_length_mm`` is stored
    as metadata rather than folded into the key because it does not change how the
    needle LOOKS, but it does invalidate any Z anchored to it.

MULTIPLE CAPTURES PER KEY. Each well touch appends a capture rather than
replacing the last one, and verification scores against the BEST match across all
of them — which is what makes the check robust to illumination and focus drift
across the plate. The reported offset is the running mean, with the spread kept so
a mis-click can be surfaced instead of quietly biasing it.

Files:
    config/hardware/needle_focus_templates.json         — metadata per key
    config/hardware/needle_focus_templates/<key>_<n>.png — the patches (BGR)

Zero GUI dependencies (json + OpenCV only). Mirrors ``ReanchorFeatureStore``.
Set ``MEBP_NEEDLE_TEMPLATE_DIR`` to redirect both files (test isolation).
"""

from __future__ import annotations

import json
import logging
import math
import os
import re
from datetime import datetime
from pathlib import Path
from typing import Optional

from SupportClasses.MachineConfig import resolve_machine_path

logger = logging.getLogger(__name__)

try:
    import cv2
    _CV2 = True
except ImportError:   # pragma: no cover - cv2 always present in this project
    cv2 = None
    _CV2 = False

_DEFAULT_PATH = resolve_machine_path("needle_focus_templates.json")
_IMG_SUBDIR = "needle_focus_templates"
# Migrated at import time (see MosaicStore.py's _DEFAULT_IMG_DIR comment).
_DEFAULT_IMG_DIR = resolve_machine_path(_IMG_SUBDIR)

# Keep a bounded history per key: enough to cover a many-well calibration while
# never letting the store grow without limit across repeated re-calibrations.
MAX_CAPTURES_PER_KEY = 12


def _safe_key(key) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(key)) or "needle"


def template_key(camobj: str, needle_type: str | None = None,
                 bore_um: float | None = None) -> str:
    """Build the store key. See the module docstring for why these three."""
    nt = str(needle_type or "needle")
    bore = f"{float(bore_um):.0f}" if bore_um else "0"
    return f"{camobj or 'cam'}|{nt}:{bore}"


class NeedleFocusTemplateStore:
    """In-focus needle patches + needle↔camera-centre offset, per camera/needle."""

    def __init__(self, path: Path | None = None):
        if path is None:
            env = os.environ.get("MEBP_NEEDLE_TEMPLATE_DIR")
            path = (Path(env) / "needle_focus_templates.json" if env
                    else _DEFAULT_PATH)
        self._path = Path(path)
        self._img_dir = (_DEFAULT_IMG_DIR if self._path == _DEFAULT_PATH
                          else self._path.parent / _IMG_SUBDIR)
        self._data: dict = {"version": "1.0", "templates": {}}
        self._load()

    # ── persistence ───────────────────────────────────────────────────

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            with open(self._path, encoding="utf-8") as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                self._data.update(loaded)
            if not isinstance(self._data.get("templates"), dict):
                self._data["templates"] = {}
        except Exception as exc:
            logger.warning(
                f"NeedleFocusTemplateStore: failed to load {self._path}: {exc}")
            self._data = {"version": "1.0", "templates": {}}

    def _save_meta(self) -> None:
        """Atomic write — a half-written index would lose every template."""
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = f"{self._path}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
        except Exception as exc:
            logger.error(f"NeedleFocusTemplateStore: failed to save: {exc}")

    # ── write ─────────────────────────────────────────────────────────

    def add_capture(self, key, patch_bgr, *,
                    center_offset_px: tuple,
                    center_offset_um: tuple,
                    um_per_px: float,
                    frame_wh: tuple | None = None,
                    focus_score: float | None = None,
                    z_zref_mm: float | None = None,
                    stage_um: tuple | None = None,
                    well: str = "",
                    needle_type: str | None = None,
                    needle_bore_um: float | None = None,
                    needle_tip_length_mm: float | None = None,
                    camobj: str = "",
                    microscope_focus_um: float | None = None,
                    needle_z_user_mm: float | None = None,
                    ground_truth: bool = False,
                    adopted_by: str | None = None,
                    auto_focus_um: float | None = None,
                    margin_um: float | None = None) -> bool:
        """Append one in-focus needle capture. Returns False if nothing stored.

        ``center_offset_um`` is the NEEDLE's position minus the CAMERA CENTRE's
        position, in stage µm — see :meth:`needle_center_offset_um`.

        v7.10 — ``microscope_focus_um`` / ``needle_z_user_mm`` record the OPTICAL
        datum: the motorised microscope's focus-axis reading and the needle's own
        height, both sampled at the instant the tip is confirmed sitting on the
        plate bottom. Their difference ties the scope's focus axis to the needle
        Z axis (see :meth:`focus_to_needle_z_mm`), which is what lets a refocus
        predict a needle height instead of the operator re-touching off.

        Both are optional and default to ``None``: a rig with no motorised focus
        still gets a fully useful capture (the template patch and the needle↔
        camera offset), and every capture written before v7.10 simply lacks the
        keys — the readers skip those rather than inventing a value.

        v7.11 — the training fields. ``ground_truth`` is set ONLY by an explicit
        operator confirmation, and only such captures feed :meth:`focus_bias_um`
        and the template bank; an unconfirmed capture is still stored, so a run
        is never lost, but it does not teach anything. ``adopted_by`` is
        ``"auto"`` or ``"operator"`` and ``auto_focus_um`` is what the estimator
        proposed — recorded even when the operator overrode it, because the
        DIFFERENCE is the entire training signal. A systematic difference means
        the gradient metric peaks off the true tip focus for this needle type,
        which is one measurable, correctable number; without storing both there
        is nothing to measure it from.

        All four are conditional-emit, so a capture that supplies none of them
        round-trips byte-identically to a pre-v7.11 entry.
        """
        if not _CV2 or patch_bgr is None or getattr(patch_bgr, "size", 0) == 0:
            return False
        if not _is_xy(center_offset_um) or not _is_xy(center_offset_px):
            return False
        k = str(key)
        entry = self._data.setdefault("templates", {}).setdefault(
            k, {"captures": []})
        caps = entry.setdefault("captures", [])
        n = _next_index(caps)
        fname = f"{_safe_key(k)}_{n}.png"
        try:
            self._img_dir.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(self._img_dir / fname), patch_bgr):
                return False
        except Exception as exc:
            logger.error(f"NeedleFocusTemplateStore.add_capture: write: {exc}")
            return False

        try:
            ph, pw = patch_bgr.shape[:2]
        except Exception:
            ph = pw = 0
        caps.append({
            "n": n,
            "image": f"{_IMG_SUBDIR}/{fname}",
            "patch_wh": [int(pw), int(ph)],
            "center_offset_px": [float(center_offset_px[0]),
                                 float(center_offset_px[1])],
            "center_offset_um": [float(center_offset_um[0]),
                                 float(center_offset_um[1])],
            "um_per_px": float(um_per_px or 0.0),
            "frame_wh": ([int(frame_wh[0]), int(frame_wh[1])]
                         if _is_xy(frame_wh) else None),
            "focus_score": (None if focus_score is None else float(focus_score)),
            "z_zref_mm": (None if z_zref_mm is None else float(z_zref_mm)),
            "stage_um": ([float(stage_um[0]), float(stage_um[1])]
                         if _is_xy(stage_um) else None),
            "well": str(well or ""),
            # v7.10 optical datum. Stored as two RAW numbers rather than their
            # difference on purpose: if the focus axis turns out to be inverted
            # or differently scaled on some scope, a stored difference would be
            # unrecoverable whereas these can be re-derived by hand.
            "microscope_focus_um": (None if microscope_focus_um is None
                                    else float(microscope_focus_um)),
            "needle_z_user_mm": (None if needle_z_user_mm is None
                                 else float(needle_z_user_mm)),
            "date": datetime.now().isoformat(timespec="seconds"),
        })
        # v7.11 training fields — conditional-emit so a capture that supplies
        # none of them is byte-identical to a pre-v7.11 entry.
        cap = caps[-1]
        if ground_truth:
            cap["ground_truth"] = True
        if adopted_by:
            cap["adopted_by"] = str(adopted_by)
        if auto_focus_um is not None:
            try:
                cap["auto_focus_um"] = float(auto_focus_um)
            except (TypeError, ValueError):
                pass
        if margin_um is not None:
            try:
                cap["margin_um"] = float(margin_um)
            except (TypeError, ValueError):
                pass
        # Trim oldest first, and drop their images so the directory cannot grow
        # without bound across repeated recalibrations.
        while len(caps) > MAX_CAPTURES_PER_KEY:
            self._unlink_capture(caps.pop(0))

        entry["needle_type"] = (str(needle_type) if needle_type is not None
                                else entry.get("needle_type"))
        entry["needle_bore_um"] = (float(needle_bore_um)
                                   if needle_bore_um is not None
                                   else entry.get("needle_bore_um"))
        entry["needle_tip_length_mm"] = (
            float(needle_tip_length_mm) if needle_tip_length_mm is not None
            else entry.get("needle_tip_length_mm"))
        entry["camobj"] = str(camobj or entry.get("camobj") or "")
        self._save_meta()
        return True

    def clear(self, key) -> None:
        """Forget every capture for a key (and delete its images)."""
        k = str(key)
        entry = self._data.get("templates", {}).pop(k, None)
        if entry:
            for cap in entry.get("captures", []) or []:
                self._unlink_capture(cap)
            self._save_meta()

    def _unlink_capture(self, cap: dict) -> None:
        rel = (cap or {}).get("image")
        if not rel:
            return
        try:
            (self._path.parent / rel).unlink(missing_ok=True)
        except Exception:
            pass

    # ── read ──────────────────────────────────────────────────────────

    def has(self, key) -> bool:
        return bool(self.captures(key))

    def get(self, key) -> Optional[dict]:
        """The raw entry for a key, or None."""
        return self._data.get("templates", {}).get(str(key))

    def captures(self, key) -> list[dict]:
        """All stored captures for a key whose image still exists on disk."""
        entry = self.get(key) or {}
        out = []
        for cap in entry.get("captures", []) or []:
            rel = cap.get("image")
            if rel and (self._path.parent / rel).exists():
                out.append(cap)
        return out

    def load_patches(self, key) -> list[tuple[dict, "object"]]:
        """``[(capture_meta, patch_bgr), …]`` for every readable capture.

        Verification scores against ALL of these and keeps the best match, which
        is what makes it tolerant of illumination and focus drift across a plate.
        """
        if not _CV2:
            return []
        out = []
        for cap in self.captures(key):
            try:
                img = cv2.imread(str(self._path.parent / cap["image"]))
            except Exception:
                img = None
            if img is not None:
                out.append((cap, img))
        return out

    def needle_center_offset_um(self, key) -> Optional[tuple]:
        """Mean needle offset from the camera centre (stage µm), or None.

        SIGN CONTRACT — the whole point of having one function own this:

            offset = (stage position of the NEEDLE)
                     - (stage position of the CAMERA CROSSHAIR)

        so the only two things a consumer ever does with it are::

            needle_stage_xy = current_xy + offset          # where is the needle?
            target_xy       = feature_xy - offset          # put the needle there

        Distinct from ``CalibrationPage._needle_origin_um``, which is a zero-ref
        PARK POSITION under the side-camera crosshairs — a different concept in a
        different frame. Do not conflate them.
        """
        caps = self.captures(key)
        if not caps:
            return None
        xs = [c["center_offset_um"][0] for c in caps
              if _is_xy(c.get("center_offset_um"))]
        ys = [c["center_offset_um"][1] for c in caps
              if _is_xy(c.get("center_offset_um"))]
        if not xs:
            return None
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    def needle_center_offset_spread_um(self, key) -> Optional[float]:
        """Max distance of any capture's offset from the mean (µm), or None.

        A spread much larger than the µm/px is the signature of a mis-click, so
        the UI can flag it rather than letting one bad click bias the mean.
        """
        caps = self.captures(key)
        mean = self.needle_center_offset_um(key)
        if mean is None or len(caps) < 2:
            return None
        worst = 0.0
        for c in caps:
            o = c.get("center_offset_um")
            if _is_xy(o):
                worst = max(worst, math.dist((o[0], o[1]), mean))
        return worst

    def _focus_datum_pairs(self, key) -> list:
        """``[(microscope_focus_um, needle_z_user_mm), ...]`` for captures that
        carry both. Pre-v7.10 captures lack the keys and are skipped."""
        out = []
        for c in self.captures(key):
            f, z = c.get("microscope_focus_um"), c.get("needle_z_user_mm")
            if isinstance(f, (int, float)) and isinstance(z, (int, float)):
                if math.isfinite(float(f)) and math.isfinite(float(z)):
                    out.append((float(f), float(z)))
        return out

    def focus_to_needle_z_mm(self, key) -> Optional[float]:
        """Mean ``K = needle_z_user_mm − microscope_focus_um/1000`` (mm), or None.

        The invariant that turns the microscope's focus axis into a Z metrology
        tool: with the scope focused on any plane, the needle height that reaches
        that plane is ``needle_z_user ≈ K + focus_um/1000``.

        Returns None when no capture carries the datum — which is the case for
        every pre-v7.10 file and for any rig without a motorised focus axis, so
        callers must treat None as "not measured", never as zero.
        """
        pairs = self._focus_datum_pairs(key)
        if not pairs:
            return None
        return sum(z - f / 1000.0 for f, z in pairs) / len(pairs)

    def focus_to_needle_z_spread_mm(self, key) -> Optional[float]:
        """Max |K_i − mean K| across captures (mm), or None with < 2 samples.

        A large spread means the focus axis disagrees with the needle Z axis in
        scale or sign — the failure this datum could otherwise hide. Surface it;
        do not average it away.
        """
        pairs = self._focus_datum_pairs(key)
        mean = self.focus_to_needle_z_mm(key)
        if mean is None or len(pairs) < 2:
            return None
        return max(abs((z - f / 1000.0) - mean) for f, z in pairs)

    def reference_focus_score(self, key) -> Optional[float]:
        """Median in-focus score across captures, or None.

        Median rather than mean: one blurry capture should not drag the reference
        the live comparison is made against.
        """
        vals = sorted(c["focus_score"] for c in self.captures(key)
                      if c.get("focus_score") is not None)
        if not vals:
            return None
        m = len(vals) // 2
        return vals[m] if len(vals) % 2 else (vals[m - 1] + vals[m]) / 2.0

    def ground_truth_captures(self, key) -> list:
        """Only the captures an operator explicitly confirmed.

        The bank is worth exactly what its labels are worth. A capture the
        operator never confirmed may be centred on a reflection, a neighbouring
        bore or the well wall, and averaging those into the reference is how a
        "trained" model gets quietly worse than no model.
        """
        return [c for c in self.captures(key) if c.get("ground_truth") is True]

    def focus_bias_um(self, key) -> Optional[float]:
        """Median ``operator_focus − auto_focus`` (µm) over confirmed captures.

        The one learnable parameter here. ``compute_focus_score`` is a gradient
        metric on a 3-D specular tip: it can peak reproducibly a few µm off the
        plane the operator judges to be the true tip focus, and if it does so
        CONSISTENTLY for a needle type that is a correctable bias, not noise.

        Median over ground truth only, and ``None`` below two samples — one
        disagreement is an anecdote. None means "not measured": callers must not
        read it as zero, since a zero bias is itself a claim.
        """
        deltas = []
        for c in self.ground_truth_captures(key):
            auto = c.get("auto_focus_um")
            got = c.get("microscope_focus_um")
            if (isinstance(auto, (int, float)) and isinstance(got, (int, float))
                    and math.isfinite(float(auto)) and math.isfinite(float(got))):
                deltas.append(float(got) - float(auto))
        if len(deltas) < 2:
            return None
        deltas.sort()
        m = len(deltas) // 2
        return (deltas[m] if len(deltas) % 2
                else 0.5 * (deltas[m - 1] + deltas[m]))

    def summary(self, key) -> str:
        """One-line operator-facing description of what is stored."""
        caps = self.captures(key)
        if not caps:
            return "no in-focus needle reference stored"
        off = self.needle_center_offset_um(key)
        s = f"{len(caps)} reference image(s)"
        if off is not None:
            s += f" · needle offset ({off[0]:+.0f}, {off[1]:+.0f}) µm"
        spread = self.needle_center_offset_spread_um(key)
        if spread is not None:
            s += f" · spread {spread:.0f} µm"
        return s

    def keys(self) -> list[str]:
        return list((self._data.get("templates") or {}).keys())


def _is_xy(v) -> bool:
    return isinstance(v, (list, tuple)) and len(v) >= 2


def _next_index(caps: list) -> int:
    used = {int(c.get("n", 0) or 0) for c in (caps or [])}
    n = 0
    while n in used:
        n += 1
    return n


# ── process-wide singleton ────────────────────────────────────────────

_STORE: NeedleFocusTemplateStore | None = None


def get_store(path: Path | None = None) -> NeedleFocusTemplateStore:
    """Lazily-created shared store (re-created when an explicit path is given)."""
    global _STORE
    if _STORE is None or path is not None:
        _STORE = NeedleFocusTemplateStore(path)
    return _STORE
