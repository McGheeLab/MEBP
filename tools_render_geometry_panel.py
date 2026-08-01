# -*- coding: utf-8 -*-
"""tools_render_geometry_panel.py - render a geometry-panel JSON
(logs/challenge/geometry_panel_*.json, written by the Geometry Panel dialog
or the calibration bench) to PNG figures next to it.

Usage:  python tools_render_geometry_panel.py [panel.json]
        (no argument = the newest geometry_panel_*.json in logs/challenge)

Outputs <stem>.png (the 6x3 ideal-vs-actual matrix, actual trace coloured
by local deviation, stall points marked) and <stem>_pred_vs_actual.png
(model prediction vs hardware per cell). Requires matplotlib."""
import json
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.lines import Line2D

import glob
import sys

if len(sys.argv) > 1:
    SRC = sys.argv[1]
else:
    _c = sorted(glob.glob(os.path.join("logs", "challenge",
                                       "geometry_panel_*.json")))
    if not _c:
        raise SystemExit("no geometry_panel_*.json found in logs/challenge")
    SRC = _c[-1]
OUT_DIR = os.path.dirname(os.path.abspath(SRC))

RAMP = ["#2e9e83", "#8fb83e", "#d9b93a", "#e08334", "#d94f3d", "#b8305a"]
BOUNDS = [0, 15, 30, 60, 120, 240, 1e9]
CMAP = ListedColormap(RAMP)
NORM = BoundaryNorm(BOUNDS, CMAP.N)

SHAPES = [("Square", "right-angle corners"),
          ("Circle", "constant curvature"),
          ("Star", "5 spikes — sharpest turns"),
          ("Zigzag", "diagonal corners"),
          ("Line-Reversal", "180° retraces (back-trace geometry)"),
          ("Comb", "fine teeth")]
SIZES = [2.0, 5.0, 10.0]
VERDICT_C = {"pass": "#2e9e83", "marginal": "#e08334", "fail": "#b8305a"}


def nearest_dev_um(P, ideal):
    A = np.asarray(ideal[:-1], float)
    B = np.asarray(ideal[1:], float)
    AB = B - A
    L2 = (AB ** 2).sum(axis=1)
    L2[L2 < 1e-18] = 1e-18
    out = np.empty(len(P))
    for i, p in enumerate(P):
        t = np.clip(((p - A) * AB).sum(axis=1) / L2, 0.0, 1.0)
        proj = A + t[:, None] * AB
        out[i] = math.sqrt(((p - proj) ** 2).sum(axis=1).min())
    return out * 1000.0


def draw_cell(ax, cell):
    ideal = np.asarray([(p[0], p[1]) for p in cell["ideal"]], float)
    c0 = (ideal.min(axis=0) + ideal.max(axis=0)) / 2.0
    ideal = ideal - c0
    samples = np.asarray([(s[0], s[1]) for s in cell["samples"]], float) - c0
    stride = max(1, len(samples) // 1600)
    samples = samples[::stride]

    ax.plot(ideal[:, 0], ideal[:, 1], color="#8d9aa8", lw=2.2, alpha=0.85,
            solid_joinstyle="round", zorder=1)
    if len(samples) > 1:
        dev = nearest_dev_um(samples, ideal)
        segs = np.stack([samples[:-1], samples[1:]], axis=1)
        seg_dev = np.maximum(dev[:-1], dev[1:])
        lc = LineCollection(segs, cmap=CMAP, norm=NORM, linewidths=1.3,
                            capstyle="round", zorder=2)
        lc.set_array(seg_dev)
        ax.add_collection(lc)
        ax.plot(*samples[0], "o", ms=4, color="#5c6b7a", zorder=3)
        if cell["actual_reason"] != "arrived":
            ex, ey = samples[-1]
            ax.plot(ex, ey, marker="x", ms=13, mew=2.2, color="#b8305a",
                    zorder=4)
            ax.plot(ex, ey, marker="o", ms=13, mfc="none", mew=1.6,
                    mec="#b8305a", zorder=4)

    pts = np.vstack([ideal, samples]) if len(samples) else ideal
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    span = max((hi - lo).max(), 0.6)
    mid = (lo + hi) / 2.0
    pad = span * 0.58
    ax.set_xlim(mid[0] - pad, mid[0] + pad)
    ax.set_ylim(mid[1] - pad, mid[1] + pad)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_color("#d0d7dd")

    # 1 mm scale bar (axes-relative placement, data-scaled length)
    x0 = mid[0] - pad + span * 0.06
    y0 = mid[1] - pad + span * 0.07
    ax.plot([x0, x0 + 1.0], [y0, y0], color="#5c6b7a", lw=2, zorder=5)
    ax.annotate("1 mm", (x0 + 0.5, y0), xytext=(0, 3),
                textcoords="offset points", ha="center", fontsize=6.5,
                color="#5c6b7a")

    a = cell["actual"]
    p = cell.get("predicted")
    if p is None:
        pred_txt = ""
    else:
        pred = ("stall" if not cell.get("predicted_completed", True)
                else f"{p['p95_um']:.0f}")
        pred_txt = f"  (pred {pred})"
    stall = "   ✚ STALLED" if cell["actual_reason"] != "arrived" else ""
    v = cell["actual_verdict"]
    ax.set_title(f"p95 {a['p95_um']:.0f} µm{pred_txt}  ·  {v.upper()}{stall}",
                 fontsize=7.6, color=VERDICT_C.get(v, "#333"), pad=3)


def main():
    with open(SRC, "r", encoding="utf-8") as f:
        data = json.load(f)
    cells = {(c["shape"], float(c["size_mm"])): c
             for c in data["cells"] if c.get("status") == "ok"}
    ch, tn = data["characteristics"], data["tuning"]

    plt.rcParams.update({"font.family": "DejaVu Sans",
                         "figure.facecolor": "white",
                         "axes.facecolor": "#f7f9fa"})
    fig, axes = plt.subplots(len(SHAPES), len(SIZES),
                             figsize=(11.5, 21.5))
    for r, (shape, note) in enumerate(SHAPES):
        for c, size in enumerate(SIZES):
            ax = axes[r][c]
            cell = cells.get((shape, size))
            if cell is None:
                ax.axis("off")
                continue
            draw_cell(ax, cell)
            if c == 0:
                ax.set_ylabel(f"{shape}\n{note}", fontsize=9.5,
                              labelpad=14, rotation=90)
        for c, size in enumerate(SIZES):
            if r == 0:
                axes[r][c].annotate(f"{size:g} mm", xy=(0.5, 1.14),
                                    xycoords="axes fraction", ha="center",
                                    fontsize=11, fontweight="bold")

    fig.suptitle(
        "XY Geometry Panel — ME3B V1, 2026-07-28 · ideal (grey) vs actual "
        "stage path (coloured by local deviation)\n"
        f"dead time {ch['dead_time_s']*1000:.0f} ms · τ {ch['tau_s']*1000:.0f} ms · "
        f"top {ch['top_speed_um_s']/1000:.2f} mm/s · loop {ch['control_loop_ms']:.1f} ms"
        f" · commanded {data['speed_mm_s']:g} mm/s · "
        + (f"lookahead {tn['lookahead']:g} mm · kp {tn['kp']:g}"
           if "lookahead" in tn else
           f"motion: {tn.get('planner', 'planned')} (feature-aware sections)")
        + f" · element {data['resolution_um']:.0f} µm",
        fontsize=11, y=0.995)

    # deviation colourbar + legend
    sm = plt.cm.ScalarMappable(cmap=CMAP, norm=NORM)
    cax = fig.add_axes([0.32, 0.043, 0.36, 0.007])
    cb = fig.colorbar(sm, cax=cax, orientation="horizontal",
                      boundaries=BOUNDS[:-1] + [360], ticks=[0, 15, 30, 60, 120, 240])
    cb.ax.set_xticklabels(["0", "15", "30\n(element)", "60", "120", "240+"],
                          fontsize=7.5)
    cb.ax.set_title("deviation from ideal (µm)", fontsize=8.5, pad=4,
                    color="#333")
    handles = [
        Line2D([], [], color="#8d9aa8", lw=2.2, label="ideal path"),
        Line2D([], [], marker="o", ls="none", color="#5c6b7a", ms=5,
               label="start"),
        Line2D([], [], marker="x", ls="none", color="#b8305a", ms=9, mew=2,
               label="stall point (run never finished)"),
    ]
    fig.legend(handles=handles, loc="lower left", bbox_to_anchor=(0.04, 0.022),
               fontsize=8, frameon=False, ncol=1)
    ok_cells = list(cells.values())
    n_pass30 = sum(1 for c in ok_cells
                   if c["actual_reason"] == "arrived"
                   and c["actual"]["p95_um"] <= 30
                   and c["actual"]["rms_um"] <= 30)
    lines = [f"{n_pass30}/{len(ok_cells)} cells at ≤ 30 µm from ideal"]
    if any("predicted_verdict" in c for c in ok_cells):
        n_agree = sum(1 for c in ok_cells
                      if c.get("predicted_verdict") == c["actual_verdict"])
        lines.append(f"{n_agree}/{len(ok_cells)} verdicts predicted offline")
    fig.text(0.985, 0.030, "\n".join(lines),
             ha="right", va="bottom", fontsize=8, color="#5c6b7a")
    fig.text(0.5, 0.004,
             f"source: {os.path.basename(SRC)} — hardware runs through "
             "XYPathSimulator.follow_path (same loop as the offline model)",
             ha="center", fontsize=7.2, color="#8a97a3")

    fig.subplots_adjust(left=0.07, right=0.985, top=0.955, bottom=0.068,
                        hspace=0.28, wspace=0.12)
    stem = os.path.splitext(os.path.basename(SRC))[0]
    out = os.path.join(OUT_DIR, stem + ".png")
    fig.savefig(out, dpi=170)
    print("wrote", out)

    # ── companion: predicted vs actual p95 ────────────────────────
    fig2, ax = plt.subplots(figsize=(12.0, 4.8))
    short = {"Line-Reversal": "Line-Rev"}
    labels, pv, av, cols, stalls = [], [], [], [], []
    for shape, _n in SHAPES:
        for size in SIZES:
            cell = cells.get((shape, size))
            if not cell:
                continue
            labels.append(f"{short.get(shape, shape)}\n{size:g} mm")
            pv.append(min(cell["predicted"]["p95_um"], 1400))
            av.append(min(cell["actual"]["p95_um"], 1400))
            cols.append(VERDICT_C.get(cell["actual_verdict"], "#333"))
            stalls.append((not cell["predicted_completed"],
                           cell["actual_reason"] != "arrived"))
    x = np.arange(len(labels))
    ax.bar(x - 0.2, pv, 0.38, color="#9fb3c4",
           label="predicted (offline model)")
    ax.bar(x + 0.2, av, 0.38, color=cols, label="actual (hardware)")
    for xi, (ps, as_) in enumerate(stalls):
        if ps:
            ax.annotate("✚", (xi - 0.2, pv[xi] + 25), ha="center",
                        fontsize=9, color="#5c6b7a")
        if as_:
            ax.annotate("✚", (xi + 0.2, av[xi] + 25), ha="center",
                        fontsize=9, color="#b8305a", fontweight="bold")
    ax.axhline(30, color="#2e9e83", lw=1, ls="--")
    ax.annotate("30 µm element", (0.1, 44), fontsize=8, color="#2e9e83")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=7.4)
    ax.set_ylabel("p95 deviation (µm, clipped at 1400)")
    ax.set_ylim(0, 1520)
    ax.set_title("Predicted vs actual p95 per cell — verdicts matched in "
                 f"{sum(1 for c in ok_cells if c.get('predicted_verdict') == c['actual_verdict'])}"
                 f"/{len(ok_cells)} cells · ✚ = run stalled (never finished; a tiny p95 "
                 "with ✚ means it tracked the line but could not turn around)",
                 fontsize=10)
    ax.legend(fontsize=8.5, frameon=False, loc="upper left",
              bbox_to_anchor=(0.02, 0.98))
    ax.spines[["top", "right"]].set_visible(False)
    fig2.tight_layout()
    out2 = os.path.join(OUT_DIR, stem + "_pred_vs_actual.png")
    fig2.savefig(out2, dpi=170)
    print("wrote", out2)


if __name__ == "__main__":
    main()
