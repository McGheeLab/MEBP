"""
calibration_status_dialog.py — Calibration "usability" pop-up.

v7.5.x: Reports when the XY, Z and pump (P) axes were last calibrated, and warns
the operator to recalibrate when either threshold in the
``CalibrationStatusStore`` is crossed:

* **M** — the XY stage has traveled more than ``xy_recal_travel_mm`` since the
  last XY calibration → *recalibrate X/Y*.
* **H** — more than ``recal_interval_hours`` have elapsed since a calibration →
  *update*.

The evaluation (`evaluate_calibration_status`) is pure so it can be unit-tested
without a GUI; `show_calibration_status` renders it as a ``QMessageBox`` mirroring
the app's other startup prompts.
"""

from __future__ import annotations

import logging
from datetime import datetime
from typing import Optional

logger = logging.getLogger(__name__)

_LABELS = {"xy": "XY", "z": "Z", "p": "Pump (P)"}


def _fmt_ago(hours: Optional[float]) -> str:
    if hours is None:
        return "time unknown"
    if hours < 1.0:
        return f"{hours * 60:.0f} min ago"
    if hours < 48.0:
        return f"{hours:.1f} h ago"
    return f"{hours / 24.0:.1f} days ago"


def _fmt_when(iso: Optional[str]) -> str:
    """Trim a stored ISO stamp to a friendly ``YYYY-MM-DD HH:MM``."""
    if not iso:
        return "never"
    try:
        return datetime.fromisoformat(iso).strftime("%Y-%m-%d %H:%M")
    except (TypeError, ValueError):
        return str(iso)


def evaluate_calibration_status(store, now: Optional[datetime] = None) -> dict:
    """Compute the pop-up contents from a ``CalibrationStatusStore``.

    Returns ``{"attention": bool, "info": [str], "warnings": [str]}``.
    ``attention`` is True when any warning fired (a threshold crossed, or a
    type was never calibrated).
    """
    now = now or datetime.now()
    m_mm, h_hours = store.get_thresholds()
    info: list[str] = []
    warnings: list[str] = []

    for kind in ("xy", "z", "p"):
        at = store.get_calibrated_at(kind)
        label = _LABELS[kind]
        if at is None:
            info.append(f"{label}: never calibrated")
            warnings.append(f"{label} has never been calibrated.")
            continue
        hrs = store.hours_since(kind, now)
        info.append(f"{label}: {_fmt_when(at)} ({_fmt_ago(hrs)})")
        if h_hours > 0 and hrs is not None and hrs > h_hours:
            warnings.append(
                f"Update {label} calibration — last done {_fmt_ago(hrs)} "
                f"(over the {h_hours:g} h limit).")

    travel_since = store.xy_travel_since_cal_um()
    if travel_since is not None:
        mm = travel_since / 1000.0
        info.append(f"XY travel since calibration: {mm:,.0f} mm")
        if m_mm > 0 and mm > m_mm:
            warnings.append(
                f"Recalibrate X/Y — the stage has traveled {mm:,.0f} mm since "
                f"the last XY calibration (over the {m_mm:g} mm limit).")

    return {"attention": bool(warnings), "info": info, "warnings": warnings}


def show_calibration_status(parent, *, store=None, force: bool = True) -> bool:
    """Show the calibration-status pop-up.

    ``force=True`` (a manual "Calibration status…" click) always shows it;
    ``force=False`` (startup) shows it only when attention is needed and
    otherwise returns ``False`` without displaying anything.

    Returns True iff the dialog was shown.
    """
    if store is None:
        try:
            from SupportClasses.CalibrationStatusStore import get_store
            store = get_store()
        except Exception as e:
            logger.debug(f"calibration status: no store ({e})")
            return False

    try:
        status = evaluate_calibration_status(store)
    except Exception as e:
        logger.warning(f"calibration status evaluation failed: {e}")
        return False

    if not force and not status["attention"]:
        return False

    try:
        from PySide6.QtWidgets import QMessageBox
    except Exception:
        return False

    box = QMessageBox(parent)
    box.setWindowTitle("Calibration status")
    if status["attention"]:
        box.setIcon(QMessageBox.Icon.Warning)
        box.setText("Some calibrations may need attention.")
        box.setInformativeText(
            "\n".join("⚠ " + w for w in status["warnings"])
            + "\n\nLast calibrated:\n"
            + "\n".join("• " + line for line in status["info"]))
    else:
        box.setIcon(QMessageBox.Icon.Information)
        box.setText("Calibration is up to date.")
        box.setInformativeText(
            "Last calibrated:\n"
            + "\n".join("• " + line for line in status["info"]))
    box.setStandardButtons(QMessageBox.StandardButton.Ok)
    box.exec()
    return True
