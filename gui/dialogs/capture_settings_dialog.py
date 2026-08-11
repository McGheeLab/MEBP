"""
capture_settings_dialog.py — where captures go and what they are called.

v7.14, built to the operator's ask: "in the settings, there should be a file
destination picker and the name picker with optional meta data for the date,
channel, objective (10x NA 0.3), etc."

v7.15 — SPLIT IN TWO, on the operator's report: *"the settings for the snapshot
should be separate from the video settings, it's confusing when they are the
same."* Right-click 📷 opens :class:`ImageCaptureSettingsDialog`, right-click ⏺
opens :class:`VideoRecordingSettingsDialog`.

The KEYS were always separate (``still_*`` vs ``video_*``); only the
presentation was shared, and it was shared in ways that actively misled:

* one preview label rendered BOTH names and merged their unknown-token
  warnings, so it never said which template the bad token was in;
* a video-only validation problem was displayed as a warning under *Images*;
* the "Metadata" box visually governed everything while ``embed_metadata`` /
  ``write_sidecar`` were read only on the still path.

⚠ ``Settings.set_section`` REPLACES a whole section, so each dialog writes back
the full merged dict — editing images must not destroy the video keys.

Every control is still driven by ``CaptureSpec.CAPTURE_DEFAULTS``/``TOKENS``,
so a new setting or token cannot be half-wired. Nothing is persisted until OK
(the rule two other dialogs in this repo had to be fixed to obey).
"""

from __future__ import annotations

import logging

from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFileDialog, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QSpinBox, QVBoxLayout,
)

from gui.styles import COLORS
from gui.scaling import s
from SupportClasses.CaptureSpec import (
    CAPTURE_DEFAULTS, TOKENS, estimated_video_mb_per_min, merged_settings,
    render_template, resolve_output_dir, still_extension, validate_still,
    validate_video, video_extension)

logger = logging.getLogger(__name__)

_STILL_SOURCES = [("display", "Display — exactly what you see (8-bit)"),
                  ("raw", "Raw — 16-bit sensor counts (quantitative)")]
_STILL_FORMATS = [("png", "PNG"), ("tiff", "TIFF (required for raw)")]
_VIDEO_SOURCES = [("display", "Live view as seen (encoded video)"),
                  ("raw_timelapse", "Raw 16-bit time-lapse (TIFF sequence)")]
_CONTAINERS = [("mp4", "MP4"), ("avi", "AVI")]


class _CaptureSettingsBase(QDialog):
    """Destination + file name + OK/Cancel; subclasses add their own kind."""

    #: settings key holding this dialog's filename template
    TEMPLATE_KEY = ""

    def __init__(self, settings=None, camera_manager=None, cam_idx=0,
                 parent=None):
        super().__init__(parent)
        self._settings = settings
        self._mgr = camera_manager
        self._cam_idx = cam_idx
        self.setMinimumWidth(s(560))
        self._build()
        self.set_values(merged_settings(self._stored()))

    def _stored(self):
        try:
            return self._settings.get_section("capture") if self._settings else None
        except Exception:
            return None

    # ── UI scaffold ───────────────────────────────────────────────

    def _build(self):
        root = QVBoxLayout(self)
        root.setSpacing(s(8))
        root.setContentsMargins(s(12), s(12), s(12), s(12))

        dest = QGroupBox("Destination")
        dl = QVBoxLayout(dest)
        row = QHBoxLayout()
        self._dir = QLineEdit()
        self._dir.setPlaceholderText("<MEBP>/captures")
        self._dir.textChanged.connect(self._refresh_preview)
        row.addWidget(self._dir, 1)
        browse = QPushButton("Browse…")
        browse.clicked.connect(self._browse)
        row.addWidget(browse)
        dl.addLayout(row)
        self._by_date = QCheckBox("Put each day's captures in its own folder")
        self._by_date.toggled.connect(self._refresh_preview)
        dl.addWidget(self._by_date)
        root.addWidget(dest)

        name = QGroupBox("File name")
        nl = QVBoxLayout(name)
        form = QFormLayout()
        self._tpl = QLineEdit()
        self._tpl.textChanged.connect(self._refresh_preview)
        form.addRow(self.TEMPLATE_LABEL, self._tpl)
        nl.addLayout(form)
        legend = QLabel("Available: " + "  ".join(
            f"{{{tok}}}" for tok, _ex, _h in TOKENS))
        legend.setWordWrap(True)
        legend.setToolTip("\n".join(f"{{{t}}} — {h} (e.g. {ex})"
                                    for t, ex, h in TOKENS))
        legend.setStyleSheet(f"color: {COLORS['subtext0']};")
        nl.addWidget(legend)
        self._preview = QLabel("")
        self._preview.setWordWrap(True)
        self._preview.setStyleSheet(
            "font-family: Consolas, monospace; "
            f"color: {COLORS.get('green', '#a6e3a1')};")
        nl.addWidget(self._preview)
        root.addWidget(name)

        self._build_kind(root)

        self._note = QLabel("")
        self._note.setWordWrap(True)
        root.addWidget(self._note)

        btn_row = QHBoxLayout()
        reset = QPushButton("Restore defaults")
        reset.clicked.connect(lambda: self.set_values(CAPTURE_DEFAULTS))
        btn_row.addWidget(reset)
        btn_row.addStretch()
        bb = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok
                              | QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        btn_row.addWidget(bb)
        root.addLayout(btn_row)

    def _combo(self, items, on_change=None):
        c = QComboBox()
        for data, label in items:
            c.addItem(label, data)
        c.currentIndexChanged.connect(on_change or self._refresh_preview)
        return c

    def _browse(self):
        start = self._dir.text().strip() or str(resolve_output_dir(
            {"output_dir": "", "subfolder_by_date": False}))
        chosen = QFileDialog.getExistingDirectory(
            self, "Where should captures be saved?", start)
        if chosen:
            self._dir.setText(chosen)

    def _camera_size(self):
        try:
            hw = self._mgr.get_hw_settings(self._cam_idx) or {}
            w, h = hw.get("resolution") or (0, 0)
            if w and h:
                return int(w), int(h)
        except Exception:
            pass
        return 1024, 1024

    # ── Preview + validation (each dialog judges only its own kind) ──

    def _refresh_preview(self, *_a):
        cfg = self.values()
        tokens = {tok: ex for tok, ex, _h in TOKENS}
        try:
            directory = resolve_output_dir(cfg)
            stem, unknown = render_template(cfg[self.TEMPLATE_KEY], tokens)
            text = f"{directory}\n  {stem}{self._extension(cfg)}"
            if unknown:
                text += "\n  ⚠ unknown: " + ", ".join(
                    "{%s}" % b for b in sorted(set(unknown)))
            self._preview.setText(text)
        except Exception as exc:      # pragma: no cover — defensive
            self._preview.setText(f"(preview unavailable: {exc})")
        self._refresh_note(cfg)

    def _refresh_note(self, cfg):
        problems = self._validate(cfg)
        if problems:
            self._note.setText("⚠ " + problems[0])
            self._note.setStyleSheet(
                f"color: {COLORS.get('peach', '#fab387')};")
        else:
            self._note.setText(self._info(cfg))
            self._note.setStyleSheet(f"color: {COLORS['subtext0']};")

    # ── Persistence ───────────────────────────────────────────────

    def values(self) -> dict:
        """The FULL capture section: stored values with this dialog's edits.

        ⚠ Not just this dialog's keys — ``Settings.set_section`` replaces the
        whole section, so returning a partial dict would delete the other
        dialog's settings on OK.
        """
        out = merged_settings(self._stored())
        out.update({
            "output_dir": self._dir.text().strip(),
            "subfolder_by_date": self._by_date.isChecked(),
            self.TEMPLATE_KEY: self._tpl.text(),
        })
        out.update(self._kind_values())
        return out

    def set_values(self, cfg):
        v = merged_settings(cfg)
        self._dir.setText(str(v["output_dir"]))
        self._by_date.setChecked(bool(v["subfolder_by_date"]))
        self._tpl.setText(str(v[self.TEMPLATE_KEY]))
        self._set_kind_values(v)
        self._refresh_preview()

    def accept(self):
        """Persist ONLY on OK — Cancel must leave the settings untouched."""
        if self._settings is not None:
            try:
                self._settings.set_section("capture", self.values())
                self._settings.save()
            except Exception as exc:
                logger.warning(f"capture settings not saved: {exc}")
        super().accept()

    # ── Subclass contract ─────────────────────────────────────────

    def _build_kind(self, root):        raise NotImplementedError
    def _kind_values(self) -> dict:     raise NotImplementedError
    def _set_kind_values(self, v):      raise NotImplementedError
    def _extension(self, cfg) -> str:   raise NotImplementedError
    def _validate(self, cfg) -> list:   raise NotImplementedError
    def _info(self, cfg) -> str:        return ""


class ImageCaptureSettingsDialog(_CaptureSettingsBase):
    """Everything about the 📷 button, and nothing about video."""

    TEMPLATE_KEY = "still_template"
    TEMPLATE_LABEL = "Image name"

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.setWindowTitle("Image capture settings")

    def _build_kind(self, root):
        box = QGroupBox("Image")
        f = QFormLayout(box)
        self._src = self._combo(_STILL_SOURCES, self._on_source)
        f.addRow("Capture", self._src)
        self._fmt = self._combo(_STILL_FORMATS)
        f.addRow("Format", self._fmt)
        self._avg = QSpinBox()
        self._avg.setRange(1, 32)
        self._avg.setToolTip(
            "Average N raw frames per capture — noise falls as √N. Raw only.")
        self._avg.valueChanged.connect(self._refresh_preview)
        f.addRow("Average frames (raw)", self._avg)
        self._full_res = QCheckBox(
            "Capture at full sensor resolution (briefly interrupts the feed)")
        self._full_res.toggled.connect(self._refresh_preview)
        f.addRow("", self._full_res)
        root.addWidget(box)

        meta = QGroupBox("Metadata")
        mf = QFormLayout(meta)
        self._embed = QCheckBox("Store inside the image file")
        self._embed.setToolTip(
            "PNG text chunks / TIFF ImageDescription — ImageJ reads both, and "
            "the TIFF also carries the pixel scale so Set Scale is prefilled.")
        mf.addRow("", self._embed)
        self._sidecar = QCheckBox("Also write a .json file beside it")
        self._sidecar.setToolTip(
            "Always readable, and survives re-saving the image elsewhere.")
        mf.addRow("", self._sidecar)
        self._operator = QLineEdit()
        mf.addRow("Operator", self._operator)
        self._notes = QLineEdit()
        mf.addRow("Notes", self._notes)
        root.addWidget(meta)

    def _on_source(self):
        """Raw REQUIRES TIFF — 16-bit PNG is not reliably readable, so the
        combination is prevented here rather than refused at write time."""
        raw = self._src.currentData() == "raw"
        self._avg.setEnabled(raw)
        self._full_res.setEnabled(raw)
        if raw:
            idx = self._fmt.findData("tiff")
            if idx >= 0:
                self._fmt.setCurrentIndex(idx)
        self._fmt.setEnabled(not raw)
        self._refresh_preview()

    def _kind_values(self):
        return {
            "still_source": self._src.currentData(),
            "still_format": self._fmt.currentData(),
            "still_raw_avg_frames": self._avg.value(),
            "still_full_res": self._full_res.isChecked(),
            "embed_metadata": self._embed.isChecked(),
            "write_sidecar": self._sidecar.isChecked(),
            "operator": self._operator.text().strip(),
            "notes": self._notes.text().strip(),
        }

    def _set_kind_values(self, v):
        for combo, key in ((self._src, "still_source"),
                           (self._fmt, "still_format")):
            idx = combo.findData(v[key])
            combo.setCurrentIndex(idx if idx >= 0 else 0)
        self._avg.setValue(int(v["still_raw_avg_frames"]))
        self._full_res.setChecked(bool(v["still_full_res"]))
        self._embed.setChecked(bool(v["embed_metadata"]))
        self._sidecar.setChecked(bool(v["write_sidecar"]))
        self._operator.setText(str(v["operator"]))
        self._notes.setText(str(v["notes"]))
        self._on_source()

    def _extension(self, cfg):
        return still_extension(cfg)

    def _validate(self, cfg):
        return validate_still(cfg)

    def _info(self, cfg):
        if cfg["still_source"] == "raw":
            n = int(cfg["still_raw_avg_frames"])
            base = "16-bit sensor counts, quantitative."
            return base + (f" Averaging {n} frames — noise falls ~{n ** 0.5:.1f}×."
                           if n > 1 else "")
        return "8-bit, exactly what the live view shows (orientation included)."


class VideoRecordingSettingsDialog(_CaptureSettingsBase):
    """Everything about the ⏺ button, and nothing about stills."""

    TEMPLATE_KEY = "video_template"
    TEMPLATE_LABEL = "Video name"

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.setWindowTitle("Video recording settings")

    def _build_kind(self, root):
        box = QGroupBox("Recording")
        f = QFormLayout(box)
        self._src = self._combo(_VIDEO_SOURCES, self._on_source)
        f.addRow("Record", self._src)
        self._fps = QDoubleSpinBox()
        self._fps.setRange(0.1, 120.0)
        self._fps.setDecimals(1)
        self._fps.setSuffix(" fps")
        self._fps.setToolTip(
            "The PLAYBACK rate. A slower camera repeats frames so one second "
            "of recording is still one second of video.")
        self._fps.valueChanged.connect(self._refresh_preview)
        f.addRow("Frame rate", self._fps)
        self._quality = QSpinBox()
        self._quality.setRange(1, 100)
        self._quality.setToolTip("Honoured by MJPG; mp4v ignores it.")
        self._quality.valueChanged.connect(self._refresh_preview)
        f.addRow("Quality", self._quality)
        self._container = self._combo(_CONTAINERS)
        f.addRow("Container", self._container)
        root.addWidget(box)

        limits = QGroupBox("Stop automatically")
        lf = QFormLayout(limits)
        self._max_s = QSpinBox()
        self._max_s.setRange(0, 86400)
        self._max_s.setSuffix(" s")
        self._max_s.setSpecialValueText("no limit")
        self._max_s.valueChanged.connect(self._refresh_preview)
        lf.addRow("After", self._max_s)
        self._max_gb = QDoubleSpinBox()
        self._max_gb.setRange(0.0, 512.0)
        self._max_gb.setDecimals(1)
        self._max_gb.setSuffix(" GB")
        self._max_gb.setSpecialValueText("no limit")
        self._max_gb.setToolTip(
            "Recording stops once the file reaches this size.")
        self._max_gb.valueChanged.connect(self._refresh_preview)
        lf.addRow("Or at", self._max_gb)
        self._interval = QDoubleSpinBox()
        self._interval.setRange(0.0, 3600.0)
        self._interval.setDecimals(1)
        self._interval.setSuffix(" s")
        lf.addRow("Time-lapse interval", self._interval)
        root.addWidget(limits)

    def _on_source(self):
        raw = self._src.currentData() == "raw_timelapse"
        for w in (self._fps, self._quality, self._container):
            w.setEnabled(not raw)
        self._interval.setEnabled(raw)
        self._refresh_preview()

    def _kind_values(self):
        return {
            "video_source": self._src.currentData(),
            "video_fps": self._fps.value(),
            "video_quality": self._quality.value(),
            "video_container": self._container.currentData(),
            "video_max_seconds": self._max_s.value(),
            # v7.15: this was hard-coded to the default here, so a stored
            # value was silently reset every time OK was pressed.
            "video_max_gb": self._max_gb.value(),
            "raw_timelapse_interval_s": self._interval.value(),
        }

    def _set_kind_values(self, v):
        for combo, key in ((self._src, "video_source"),
                           (self._container, "video_container")):
            idx = combo.findData(v[key])
            combo.setCurrentIndex(idx if idx >= 0 else 0)
        self._fps.setValue(float(v["video_fps"]))
        self._quality.setValue(int(v["video_quality"]))
        self._max_s.setValue(int(v["video_max_seconds"]))
        self._max_gb.setValue(float(v["video_max_gb"]))
        self._interval.setValue(float(v["raw_timelapse_interval_s"]))
        self._on_source()

    def _extension(self, cfg):
        return (".<folder>" if cfg["video_source"] == "raw_timelapse"
                else video_extension(cfg))

    def _validate(self, cfg):
        return validate_video(cfg)

    def _info(self, cfg):
        w, h = self._camera_size()
        if cfg["video_source"] == "raw_timelapse":
            rate = 1.0 / max(0.1, float(cfg["raw_timelapse_interval_s"] or 1))
            mb = estimated_video_mb_per_min(w, h, rate, "raw_timelapse")
            return (f"≈ {mb / 1024:.1f} GB per minute at {w}×{h} — every frame "
                    f"is a real 16-bit capture with its own timestamp; the "
                    f"sequence is a time-lapse, not continuous video.")
        mb = estimated_video_mb_per_min(w, h, cfg["video_fps"], "display",
                                        cfg["video_quality"])
        return (f"≈ {mb:.0f} MB per minute at {w}×{h}. The frame rate is the "
                f"playback rate — a slower camera repeats frames so the video "
                f"still runs at real time.")


#: v7.15 back-compat: the single dialog became two. Anything still importing
#: the old name gets the image one, which is what it previously opened onto.
CaptureSettingsDialog = ImageCaptureSettingsDialog
