"""v7.7 — the Quick Print post-print report.

Before this, a finished Quick Print produced one line of grey text and a filename.
Everything needed for a real report already existed and was simply never joined
up: the JSONL execution log on disk, the feed plan and its simulation in memory,
``XYChallenge``'s scoring, and four capable QPainter widgets sitting in the Full
Print Results page that Quick Print never used because it writes no
``PrintRecorder`` recording.

This panel joins them. It reads the log through ``SupportClasses.PrintLogReader``,
adapts it to the sample shape those widgets already consume, scores it with
``XYChallenge.path_report``, and puts the **prediction beside the measurement** —
the feedback loop that turns "the simulation said 7 µm and the machine did 27 µm"
from a private discovery into something the operator sees every run.

No new dependencies: the figures are the existing widgets, and the HTML export
embeds them as base64 PNGs rendered with ``QWidget.grab()``.
"""

from __future__ import annotations

import base64
import csv
import logging
import os
import re
from datetime import datetime
from typing import Optional

from PySide6.QtCore import QBuffer, QIODevice, Qt, QUrl
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (QFileDialog, QHBoxLayout, QLabel, QPushButton,
                               QScrollArea, QSizePolicy, QTableWidget,
                               QTableWidgetItem, QVBoxLayout, QWidget)

from gui.scaling import s, sf
from gui.styles import COLORS
from gui.widgets.components import Card, StatusBadge
from gui.pages.print_results import (ErrorTimeSeriesWidget, PathComparisonWidget,
                                     PlaybackController, StatisticsPanel)
from SupportClasses import PrintLogReader as PLR

logger = logging.getLogger(__name__)

_SAFE_NAME_RE = re.compile(r"[^A-Za-z0-9._-]+")


def safe_filename(name: str, fallback: str = "print_report") -> str:
    """Reduce an arbitrary label to something safe to put on disk."""
    cleaned = _SAFE_NAME_RE.sub("_", (name or "").strip()).strip("._-")
    return cleaned[:80] or fallback


def _mono(text: str, color_key: str = "text", pt: int = 9) -> QLabel:
    lbl = QLabel(text)
    lbl.setWordWrap(True)
    lbl.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
    lbl.setStyleSheet(f"color: {COLORS[color_key]}; font-size: {sf(pt)}pt; "
                      f"font-family: monospace;")
    return lbl


def _row_label(text: str, color_key: str = "subtext0", pt: int = 9) -> QLabel:
    lbl = QLabel(text)
    lbl.setWordWrap(True)
    lbl.setStyleSheet(f"color: {COLORS[color_key]}; font-size: {sf(pt)}pt;")
    return lbl


def widget_png_base64(widget: QWidget, min_w: int = 900,
                      min_h: int = 300) -> str:
    """Render a widget to a base64 PNG so it can be inlined in the HTML export.

    ``QWidget.grab()`` captures exactly what the operator saw, which keeps the
    exported figure and the on-screen figure from drifting apart — and needs no
    plotting library (matplotlib is deliberately not a GUI dependency here).

    A widget that has never been shown can still be grabbed, but it may not have
    been laid out yet and would produce a degenerate image — so give it a floor
    size first. Only ever resize a widget that is NOT visible, so exporting can
    never perturb what the operator is looking at.
    """
    try:
        if not widget.isVisible() and (widget.width() < min_w
                                       or widget.height() < min_h):
            widget.resize(max(widget.width(), min_w),
                          max(widget.height(), min_h))
        pixmap = widget.grab()
        if pixmap.isNull() or pixmap.width() < 4 or pixmap.height() < 4:
            return ""
        buf = QBuffer()
        buf.open(QIODevice.OpenModeFlag.WriteOnly)
        pixmap.save(buf, "PNG")
        return base64.b64encode(bytes(buf.data())).decode("ascii")
    except Exception as exc:                                # pragma: no cover
        logger.debug("widget capture failed: %s", exc)
        return ""


# ── the comparison that closes the loop ──────────────────────────────

def compare_prediction(actual: dict | None, predicted: dict | None) -> dict:
    """Predicted vs measured, plus the ratio.

    Returns ``{"rows": [(metric, predicted, actual)], "ratio": float|None,
    "interpretation": str}``. The ratio is the honest headline: on ME3B V1 the
    simulator has run 1.5–4.3× optimistic depending on geometry, so a single
    "headroom" constant cannot be trusted as an upper bound — showing the ratio
    per run is how it becomes evidence instead of a guess.
    """
    a = actual or {}
    p = predicted or {}
    rows: list[tuple[str, str, str]] = []

    def _fmt(v, unit="", fmt="{:.0f}"):
        if v is None:
            return "—"
        try:
            return fmt.format(float(v)) + unit
        except (TypeError, ValueError):
            return "—"

    for label, key, unit, fmt in (
            ("p95 deviation", "p95_um", " µm", "{:.0f}"),
            ("rms deviation", "rms_um", " µm", "{:.0f}"),
            ("max deviation", "max_um", " µm", "{:.0f}"),
            ("completion", "completion_frac", "", "{:.3f}"),
            ("wall time", "wall_s", " s", "{:.1f}")):
        rows.append((label, _fmt(p.get(key), unit, fmt),
                     _fmt(a.get(key), unit, fmt)))

    ratio = None
    try:
        pp, ap = float(p.get("p95_um")), float(a.get("p95_um"))
        if pp > 0.5:                 # below this the ratio is numerical noise
            ratio = ap / pp
    except (TypeError, ValueError):
        ratio = None

    if ratio is None:
        interp = ("No prediction to compare against — run in closed-loop "
                  "velocity mode on a characterised machine to get one.")
    elif ratio <= 1.2:
        interp = (f"The machine matched the prediction ({ratio:.1f}×). The model "
                  f"is trustworthy for this geometry.")
    elif ratio <= 2.0:
        interp = (f"The machine was {ratio:.1f}× worse than predicted — within "
                  f"the 2× headroom the printability checker assumes.")
    else:
        interp = (f"The machine was {ratio:.1f}× worse than predicted, ABOVE the "
                  f"2× headroom the printability checker assumes: treat its "
                  f"pass/fail verdicts for this kind of geometry as optimistic.")
    return {"rows": rows, "ratio": ratio, "interpretation": interp}


class QuickPrintReportPanel(QWidget):
    """The Report zone: what actually happened, and how it compared."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._samples: list[dict] = []
        self._meta: dict = {}
        self._log: Optional[PLR.PrintLog] = None
        self._actual: dict = {}
        self._predicted: dict = {}
        self._context: dict = {}
        self._build_ui()
        self.clear()

    # ── construction ─────────────────────────────────────────────
    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(8), s(8), s(8), s(8))
        outer.setSpacing(s(8))

        # Verdict header
        head = QWidget()
        hl = QHBoxLayout(head)
        hl.setContentsMargins(0, 0, 0, 0)
        hl.setSpacing(s(8))
        self._verdict = StatusBadge("no run yet", variant="pending")
        hl.addWidget(self._verdict)
        self._headline = _row_label("", "text", 10)
        hl.addWidget(self._headline, stretch=1)
        self._btn_html = QPushButton("Export HTML…")
        self._btn_html.clicked.connect(self._on_export_html)
        hl.addWidget(self._btn_html)
        self._btn_csv = QPushButton("Export CSV…")
        self._btn_csv.clicked.connect(self._on_export_csv)
        hl.addWidget(self._btn_csv)
        self._btn_log = QPushButton("Open log")
        self._btn_log.clicked.connect(self._on_open_log)
        hl.addWidget(self._btn_log)
        outer.addWidget(head)

        body = QScrollArea()
        body.setWidgetResizable(True)
        body.setFrameShape(QScrollArea.Shape.NoFrame)
        inner = QWidget()
        self._body = QVBoxLayout(inner)
        self._body.setContentsMargins(0, 0, 0, 0)
        self._body.setSpacing(s(8))
        body.setWidget(inner)
        outer.addWidget(body, stretch=1)

        # ── predicted vs actual ──
        self._card_compare = Card("Predicted vs actual")
        self._compare_tbl = self._make_table(
            ["", "simulated", "real"], rows=5)
        self._card_compare.add_widget(self._compare_tbl)
        self._compare_note = _row_label("", "peach")
        self._card_compare.add_widget(self._compare_note)
        self._body.addWidget(self._card_compare)

        # ── the path, playback, error trace, statistics ──
        self._card_path = Card("Path — planned vs actual (coloured by error)",
                               flush=True)
        self._path_view = PathComparisonWidget()
        self._path_view.setMinimumHeight(s(260))
        self._card_path.add_widget(self._path_view)
        self._playback = PlaybackController()
        self._playback.sample_changed.connect(self._on_cursor)
        self._card_path.add_widget(self._playback)
        self._body.addWidget(self._card_path)

        self._card_err = Card("Deviation over time", flush=True)
        self._err_view = ErrorTimeSeriesWidget()
        self._err_view.setMinimumHeight(s(140))
        self._card_err.add_widget(self._err_view)
        self._body.addWidget(self._card_err)

        self._card_stats = Card("Statistics")
        self._stats = StatisticsPanel()
        self._card_stats.add_widget(self._stats)
        self._body.addWidget(self._card_stats)

        # ── feed-plan sections ──
        self._card_sections = Card("Feed-plan sections — what the plan did and "
                                   "what it cost")
        self._sections_tbl = self._make_table(
            ["#", "length mm", "speed mm/s", "lookahead mm", "p95 µm",
             "max µm", "why"], rows=0)
        self._card_sections.add_widget(self._sections_tbl)
        self._restart_note = _row_label("", "subtext0")
        self._card_sections.add_widget(self._restart_note)
        self._body.addWidget(self._card_sections)

        # ── fluidics ──
        self._card_fluid = Card("Fluidics")
        self._fluid_txt = _mono("")
        self._card_fluid.add_widget(self._fluid_txt)
        self._body.addWidget(self._card_fluid)

        # ── machine health ──
        self._card_health = Card("Machine health")
        self._health_txt = _mono("")
        self._card_health.add_widget(self._health_txt)
        self._body.addWidget(self._card_health)

        # ── problems ──
        self._card_problems = Card("Problems")
        self._problems_txt = _mono("")
        self._card_problems.add_widget(self._problems_txt)
        self._body.addWidget(self._card_problems)

        self._body.addStretch(1)

    def _make_table(self, headers: list[str], rows: int) -> QTableWidget:
        t = QTableWidget(rows, len(headers))
        t.setHorizontalHeaderLabels(headers)
        t.verticalHeader().setVisible(False)
        t.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        t.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        t.setAlternatingRowColors(True)
        t.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
        t.setStyleSheet(f"font-size: {sf(9)}pt;")
        t.horizontalHeader().setStretchLastSection(True)
        return t

    # ── population ───────────────────────────────────────────────
    def clear(self) -> None:
        self._samples, self._meta, self._log = [], {}, None
        self._actual, self._predicted = {}, {}
        self._verdict.set_status("pending", "no run yet")
        self._headline.setText(
            "Run a print and its report appears here — the measured path, how it "
            "compared with the prediction, and what the pump and the board did.")
        for card in (self._card_compare, self._card_path, self._card_err,
                     self._card_stats, self._card_sections, self._card_fluid,
                     self._card_health, self._card_problems):
            card.setVisible(False)
        for b in (self._btn_html, self._btn_csv, self._btn_log):
            b.setEnabled(False)

    def load(self, log_path, *, ideal_pts=None, predicted: dict | None = None,
             context: dict | None = None) -> bool:
        """Read ``log_path`` and populate. ``ideal_pts`` (zero-ref mm) overrides
        the path stored in the log — the caller has it in memory right after the
        print and it is exact, where the logged copy may be decimated.

        Returns False when the log yielded nothing usable.
        """
        self._context = dict(context or {})
        self._predicted = dict(predicted or {})
        try:
            log = PLR.read_log(str(log_path))
        except Exception as exc:                            # pragma: no cover
            logger.exception("could not read print log: %s", exc)
            return False
        if not log.events:
            return False
        self._log = log
        self._samples, self._meta = PLR.to_recorder_samples(log, ideal_pts)
        self._actual = self._score(ideal_pts)
        self._render()
        return True

    def _score(self, ideal_pts) -> dict:
        """Score the executed trace against the ideal with the same machinery the
        bench and the geometry panel use, so the numbers are comparable."""
        ideal = ideal_pts if ideal_pts is not None else (
            self._log.ideal_points() if self._log else None)
        if not ideal or not self._meta.get("has_planned"):
            return {}
        try:
            from SupportClasses import XYChallenge as XC
        except Exception:                                   # pragma: no cover
            return {}
        pts = [(sm["actual_x"], sm["actual_y"]) for sm in self._samples
               if not sm["is_travel"]]
        if len(pts) < 3:
            return {}
        summary = self._meta.get("summary", {})
        res_um = float(summary.get("element_um") or 30.0) or 30.0
        try:
            rep = XC.path_report(
                [XC.Sample(x, y, 0.0) for x, y in pts], ideal,
                commanded_speed_mm_s=summary.get("commanded_speed_mm_s") or None,
                resolution_um=res_um,
                status="ok" if self._meta.get("status") == "completed"
                else str(self._meta.get("status")),
                wall_s=summary.get("path_wall_s") or None)
            rep["verdict"] = XC.verdict_for(rep, res_um)
            rep["resolution_um"] = res_um
            return rep
        except Exception as exc:                            # pragma: no cover
            logger.debug("scoring failed: %s", exc)
            return {}

    # ── rendering ────────────────────────────────────────────────
    def _render(self) -> None:
        meta, log = self._meta, self._log
        if log is None:
            return
        for b in (self._btn_html, self._btn_csv):
            b.setEnabled(True)
        self._btn_log.setEnabled(True)

        verdict = str(self._actual.get("verdict") or "")
        status = str(meta.get("status") or "unknown")
        if status == "completed" and verdict == "pass":
            self._verdict.set_status("ok", "PASS")
        elif status == "completed" and verdict == "marginal":
            self._verdict.set_status("warn", "MARGINAL")
        elif status == "completed" and verdict == "fail":
            self._verdict.set_status("err", "FAIL")
        elif status == "completed":
            self._verdict.set_status("ok", "COMPLETED")
        elif status in ("aborted",):
            self._verdict.set_status("warn", "ABORTED")
        else:
            self._verdict.set_status("err", status.upper() or "UNKNOWN")

        res_um = self._actual.get("resolution_um")
        bits = [log.job_name, f"{meta.get('duration_s', 0.0):.1f} s",
                f"mode {meta.get('mode')}"]
        if self._actual.get("p95_um") is not None:
            bits.append(f"p95 {self._actual['p95_um']:.0f} µm"
                        + (f" vs a {res_um:.0f} µm element" if res_um else ""))
        if log.truncated:
            bits.append("⚠ the log is truncated (the run did not close it)")
        self._headline.setText("  ·  ".join(bits))

        self._render_compare()
        self._render_path(meta)
        self._render_sections(log)
        self._render_fluidics(log)
        self._render_health(log, meta)
        self._render_problems(log)

    def _render_compare(self) -> None:
        cmp_ = compare_prediction(self._actual, self._predicted)
        tbl = self._compare_tbl
        rows = cmp_["rows"]
        tbl.setRowCount(len(rows))
        for r, (label, pred, act) in enumerate(rows):
            for c, txt in enumerate((label, pred, act)):
                tbl.setItem(r, c, QTableWidgetItem(txt))
        tbl.resizeColumnsToContents()
        tbl.setFixedHeight(
            tbl.horizontalHeader().height()
            + sum(tbl.rowHeight(r) for r in range(tbl.rowCount())) + s(4))
        self._compare_note.setText(cmp_["interpretation"])
        self._card_compare.setVisible(True)

    def _render_path(self, meta: dict) -> None:
        has_fb = bool(meta.get("has_position_feedback"))
        self._path_view.set_data(self._samples)
        self._err_view.set_data(self._samples)
        self._playback.set_data(self._samples)
        self._stats.set_data(self._samples, meta)
        self._card_path.setVisible(bool(self._samples))
        self._card_stats.setVisible(bool(self._samples))
        # Open-loop reads no position during the path, so there is nothing to
        # compare — say that rather than draw a flat zero-error trace as if the
        # print had been perfect.
        self._card_err.setVisible(has_fb and bool(meta.get("has_planned")))
        if not has_fb:
            self._headline.setText(
                self._headline.text()
                + "  ·  ⚠ open-loop: no position feedback was recorded, so "
                  "deviation cannot be reported")

    def _render_sections(self, log) -> None:
        rows = PLR.section_stats(log)
        self._card_sections.setVisible(bool(rows))
        if not rows:
            return
        tbl = self._sections_tbl
        tbl.setRowCount(len(rows))
        for r, sec in enumerate(rows):
            vals = [
                str(sec["index"]),
                f"{sec['length_mm']:.2f}",
                f"{sec['speed_mm_s']:.2f}",
                f"{sec['lookahead_mm']:.3f}",
                ("—" if sec.get("p95_um") is None else f"{sec['p95_um']:.0f}"),
                ("—" if sec.get("max_um") is None else f"{sec['max_um']:.0f}"),
                ("skipped (shorter than its own arrive tolerance)"
                 if sec["skipped"] else sec["reason"]),
            ]
            for c, txt in enumerate(vals):
                tbl.setItem(r, c, QTableWidgetItem(txt))
        tbl.resizeColumnsToContents()
        tbl.setFixedHeight(
            tbl.horizontalHeader().height()
            + sum(tbl.rowHeight(r) for r in range(tbl.rowCount())) + s(6))

        dec = PLR.restart_decomposition(log)
        if dec and dec.get("near_p95_um") is not None \
                and dec.get("mid_p95_um") is not None:
            self._restart_note.setText(
                f"Error by position: p95 {dec['near_p95_um']:.0f} µm within "
                f"{dec['window_mm']:.1f} mm of a section boundary "
                f"({dec['near_n']} samples) vs {dec['mid_p95_um']:.0f} µm "
                f"mid-section ({dec['mid_n']}). The corner vertices themselves "
                f"are hit — what remains is the transient as each next section "
                f"spins up, so effort spent on restarts beats effort spent on "
                f"the straights.")
        else:
            self._restart_note.setText("")

    def _render_fluidics(self, log) -> None:
        v = PLR.volume_reconciliation(log)
        lines = []
        if v.get("path_length_mm"):
            lines.append(f"path            {v['path_length_mm']:.2f} mm at "
                         f"{v['vol_per_mm_uL']:.5f} µL/mm")
        if v.get("planned_path_uL"):
            lines.append(f"planned         {v['planned_path_uL']:.4f} µL along "
                         f"the path")
        if v.get("deposited_path_uL") is not None:
            dep, plan = v["deposited_path_uL"], v.get("planned_path_uL") or 0.0
            delta = (f"  ({100.0 * dep / plan - 100.0:+.1f} % vs planned)"
                     if plan > 0 else "")
            lines.append(f"commanded       {dep:.4f} µL{delta}")
        if v.get("discrete_dispensed_uL"):
            lines.append(f"prime/dispense  {v['discrete_dispensed_uL']:.4f} µL "
                         f"(discrete actuations)")
        if v.get("discrete_aspirated_uL"):
            lines.append(f"aspirated       {v['discrete_aspirated_uL']:.4f} µL")
        if v.get("pressure_relief_uL"):
            lines.append(f"relief/suckback {v['pressure_relief_uL']:.4f} µL")
        if log.of("abort_requested"):
            lines.append("")
            lines.append("⚠ this run was aborted — a pump move may have been cut "
                         "mid-stroke, so the volume actually dispensed is "
                         "INDETERMINATE. Re-check the syringe fill.")
        self._fluid_txt.setText("\n".join(lines) or "no pump activity recorded")
        self._card_fluid.setVisible(bool(lines))

    def _render_health(self, log, meta: dict) -> None:
        h = log.comm_health()
        lines = []
        if h:
            lines.append(f"ZP link         {h['ok']}/{h['commands']} commands "
                         f"acked, {h['ok_failures']} ack failures, "
                         f"{h['board_resets']} board resets")
            if h.get("disconnected_samples"):
                lines.append(f"⚠ the board reported disconnected in "
                             f"{h['disconnected_samples']} samples")
        pe = log.path_end
        if pe:
            lines.append(f"path            {pe.get('status')} — "
                         f"{PLR.as_float(pe.get('s_mm')):.2f} of "
                         f"{PLR.as_float(pe.get('tot_mm')):.2f} mm in "
                         f"{PLR.as_float(pe.get('wall_s')):.1f} s")
        sm = meta.get("summary", {})
        if sm.get("plan_est_s"):
            est, act = sm["plan_est_s"], sm.get("path_wall_s") or 0.0
            if act > 0:
                lines.append(f"time estimate   {est:.1f} s predicted vs "
                             f"{act:.1f} s actual "
                             f"({100.0 * act / est - 100.0:+.0f} %)")
        if meta.get("n_bad_lines"):
            lines.append(f"⚠ {meta['n_bad_lines']} unreadable log line(s)")
        self._health_txt.setText("\n".join(lines) or "no telemetry recorded")
        self._card_health.setVisible(bool(lines))

    def _render_problems(self, log) -> None:
        probs = PLR.problems(log)
        if not probs:
            self._problems_txt.setText("None — the run was clean.")
            self._card_problems.setVisible(True)
            return
        lines = [f"{p['t']:7.2f}s  {p['kind']:<14} {p['text']}" for p in probs]
        tb = [p for p in probs if p.get("traceback")]
        if tb:
            lines.append("")
            lines.append("traceback (most recent error):")
            lines.append(tb[-1]["traceback"][-1500:])
        self._problems_txt.setText("\n".join(lines))
        self._card_problems.setVisible(True)

    def _on_cursor(self, idx: int) -> None:
        self._path_view.set_cursor(idx)
        self._err_view.set_cursor(idx)

    # ── export ───────────────────────────────────────────────────
    def _default_stem(self) -> str:
        base = os.path.basename(str(self._meta.get("log_path") or ""))
        stem = os.path.splitext(base)[0] or safe_filename(
            self._log.job_name if self._log else "print_report")
        return safe_filename(stem)

    def _on_open_log(self) -> None:
        p = self._meta.get("log_path")
        if not p:
            return
        if not QDesktopServices.openUrl(QUrl.fromLocalFile(str(p))):
            QDesktopServices.openUrl(
                QUrl.fromLocalFile(os.path.dirname(str(p))))

    def _on_export_csv(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "Export report CSV", f"{self._default_stem()}.csv",
            "CSV files (*.csv)")
        if not path:
            return
        try:
            self.write_csv(path)
        except Exception as exc:
            logger.exception("CSV export failed: %s", exc)

    def _on_export_html(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "Export report HTML", f"{self._default_stem()}.html",
            "HTML files (*.html)")
        if not path:
            return
        try:
            self.write_html(path)
            QDesktopServices.openUrl(QUrl.fromLocalFile(path))
        except Exception as exc:
            logger.exception("HTML export failed: %s", exc)

    #: Cap on per-sample CSV rows so an unexpectedly huge log can't produce a
    #: multi-hundred-MB file.
    MAX_CSV_ROWS = 200_000

    def write_csv(self, path: str) -> str:
        """Summary rows followed by the per-sample table."""
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["Metric", "Value"])
            for k, v in self.summary_rows():
                w.writerow([k, v])
            w.writerow([])
            cols = ["t", "planned_x", "planned_y", "actual_x", "actual_y",
                    "tracking_error_xy", "segment_id", "is_travel",
                    "is_retract", "s_mm", "v_meas_mm_s", "deposited_uL"]
            w.writerow(cols)
            for sm in self._samples[:self.MAX_CSV_ROWS]:
                w.writerow([sm.get(c, "") for c in cols])
            if len(self._samples) > self.MAX_CSV_ROWS:
                w.writerow([f"... {len(self._samples) - self.MAX_CSV_ROWS} "
                            f"further rows omitted"])
        return path

    def summary_rows(self) -> list[tuple[str, str]]:
        meta, log = self._meta, self._log
        rows: list[tuple[str, str]] = [
            ("job name", str(log.job_name if log else "")),
            ("status", str(meta.get("status", ""))),
            ("motion mode", str(meta.get("mode", ""))),
            ("duration s", f"{meta.get('duration_s', 0.0):.2f}"),
            ("log", str(meta.get("log_path", ""))),
            ("exported", datetime.now().isoformat(timespec="seconds")),
        ]
        for k in ("p95_um", "rms_um", "max_um", "completion_frac", "verdict",
                  "resolution_um"):
            if self._actual.get(k) is not None:
                v = self._actual[k]
                rows.append((f"actual {k}",
                             f"{v:.4g}" if isinstance(v, (int, float)) else str(v)))
        for k in ("p95_um", "rms_um", "max_um"):
            if self._predicted.get(k) is not None:
                rows.append((f"predicted {k}", f"{self._predicted[k]:.4g}"))
        cmp_ = compare_prediction(self._actual, self._predicted)
        if cmp_["ratio"] is not None:
            rows.append(("actual / predicted p95", f"{cmp_['ratio']:.2f}"))
        for k, v in (meta.get("summary") or {}).items():
            rows.append((k, f"{v:.6g}" if isinstance(v, float) else str(v)))
        for k, v in (self._context or {}).items():
            rows.append((f"context {k}", str(v)))
        return rows

    def write_html(self, path: str) -> str:
        """A single self-contained HTML file — figures inlined as base64 PNGs, so
        it opens anywhere with no assets and no network access."""
        from html import escape
        log = self._log
        cmp_ = compare_prediction(self._actual, self._predicted)

        def table(headers, rows):
            th = "".join(f"<th>{escape(str(h))}</th>" for h in headers)
            trs = "".join(
                "<tr>" + "".join(f"<td>{escape(str(c))}</td>" for c in r)
                + "</tr>" for r in rows)
            return f"<table><thead><tr>{th}</tr></thead><tbody>{trs}</tbody></table>"

        figs = []
        for title, widget, want in (
                ("Path — planned vs actual", self._path_view, bool(self._samples)),
                ("Deviation over time", self._err_view,
                 bool(self._samples) and bool(self._meta.get("has_planned")))):
            # Gate on whether the figure HAS DATA, not on whether it happens to
            # be on screen — exporting from a background zone dropped every
            # figure silently.
            if not want:
                continue
            b64 = widget_png_base64(widget)
            if b64:
                figs.append(f"<h2>{escape(title)}</h2>"
                            f'<img alt="{escape(title)}" '
                            f'src="data:image/png;base64,{b64}">')

        sec_rows = [
            [x["index"], f"{x['length_mm']:.2f}", f"{x['speed_mm_s']:.2f}",
             f"{x['lookahead_mm']:.3f}",
             "—" if x.get("p95_um") is None else f"{x['p95_um']:.0f}",
             "—" if x.get("max_um") is None else f"{x['max_um']:.0f}",
             "skipped" if x["skipped"] else x["reason"]]
            for x in (PLR.section_stats(log) if log else [])]

        probs = PLR.problems(log) if log else []
        prob_rows = [[f"{p['t']:.2f}", p["kind"], p["text"]] for p in probs]

        html = f"""<!doctype html>
<meta charset="utf-8">
<title>{escape(log.job_name if log else 'Print report')} — MEBP print report</title>
<style>
 body {{ font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
        margin: 2rem auto; max-width: 60rem; color: #1e1e2e; line-height: 1.5; }}
 h1 {{ font-size: 1.5rem; margin-bottom: .2rem; }}
 h2 {{ font-size: 1.05rem; margin-top: 1.8rem;
       border-bottom: 1px solid #ccd; padding-bottom: .2rem; }}
 .sub {{ color: #5b6b7a; margin-top: 0; }}
 table {{ border-collapse: collapse; width: 100%; font-size: .9rem;
          margin: .6rem 0; }}
 th, td {{ border: 1px solid #dde; padding: .3rem .5rem; text-align: left;
           vertical-align: top; }}
 th {{ background: #f2f5f8; }}
 img {{ max-width: 100%; border: 1px solid #dde; border-radius: 4px; }}
 pre {{ background: #f7f9fb; padding: .6rem; overflow-x: auto;
        font-size: .82rem; }}
 .note {{ background: #fff6f0; border-left: 3px solid #d97757;
          padding: .5rem .7rem; }}
</style>
<h1>{escape(log.job_name if log else 'Print report')}</h1>
<p class="sub">{escape(str(self._meta.get('status','')))} ·
 {self._meta.get('duration_s', 0.0):.1f} s · mode
 {escape(str(self._meta.get('mode','')))} · exported
 {escape(datetime.now().isoformat(timespec='seconds'))}</p>

<h2>Predicted vs actual</h2>
{table(["", "simulated", "real"], cmp_["rows"])}
<p class="note">{escape(cmp_["interpretation"])}</p>

{''.join(figs)}

<h2>Summary</h2>
{table(["Metric", "Value"], self.summary_rows())}

{"<h2>Feed-plan sections</h2>" + table(
    ["#", "length mm", "speed mm/s", "lookahead mm", "p95 µm", "max µm", "why"],
    sec_rows) if sec_rows else ""}

{"<h2>Problems</h2>" + table(["t (s)", "kind", "what"], prob_rows)
 if prob_rows else "<h2>Problems</h2><p>None — the run was clean.</p>"}

<h2>Fluidics</h2><pre>{escape(self._fluid_txt.text())}</pre>
<h2>Machine health</h2><pre>{escape(self._health_txt.text())}</pre>
"""
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(html)
        return path
