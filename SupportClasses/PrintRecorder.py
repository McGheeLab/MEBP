"""
PrintRecorder.py — Automatic print execution recording for MEBP v7.1.

Records every print execution to timestamped files for historical tracking,
replay visualization, and quality analysis.

Recorded data per print:
- Workspace config (needle, syringe, inks)
- Well assignments and plane fit
- Planned trajectory (all waypoints)
- Actual positions (timestamped from position poller)
- Commands sent and timing
- Tracking errors
- Final status (completed/aborted/error)

File format: JSON + CSV pair
- {timestamp}_meta.json  → workspace, settings, well assignments
- {timestamp}_data.csv   → t, planned_x, planned_y, planned_z,
                            actual_x, actual_y, actual_z,
                            p1, p2, p3, tracking_error

Integration with PrintHistory:
The existing PrintHistory.py tracks summary stats. PrintRecorder adds full
trajectory-level data. They complement each other:
- PrintHistory: "How many prints have we done? Success rate?"
- PrintRecorder: "Show me the actual path of print #47 vs planned."

Session I — Tasks P7.1, P7.2, P7.3.
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# Default output directory for print recordings
DEFAULT_RECORDS_DIR = "print_records"

# CSV column names for the data file
CSV_COLUMNS = [
    "t",
    "planned_x", "planned_y", "planned_z",
    "planned_p1", "planned_p2", "planned_p3",
    "actual_x", "actual_y", "actual_z",
    "actual_p1", "actual_p2", "actual_p3",
    "tracking_error_xy", "tracking_error_z",
    "segment_id", "is_travel", "is_retract",
]


# ═══════════════════════════════════════════════════════════════════
# Data Structures
# ═══════════════════════════════════════════════════════════════════

@dataclass
class RecordingSample:
    """A single timestep of recorded data."""
    t: float = 0.0

    # Planned position (from trajectory)
    planned_x: float = 0.0
    planned_y: float = 0.0
    planned_z: float = 0.0
    planned_p1: float = 0.0
    planned_p2: float = 0.0
    planned_p3: float = 0.0

    # Actual position (from hardware)
    actual_x: float = 0.0
    actual_y: float = 0.0
    actual_z: float = 0.0
    actual_p1: float = 0.0
    actual_p2: float = 0.0
    actual_p3: float = 0.0

    # Tracking errors (computed)
    tracking_error_xy: float = 0.0
    tracking_error_z: float = 0.0

    # Metadata
    segment_id: int = 0
    is_travel: bool = False
    is_retract: bool = False

    def to_csv_row(self) -> str:
        """Format as a CSV row string."""
        return ",".join([
            f"{self.t:.4f}",
            f"{self.planned_x:.4f}", f"{self.planned_y:.4f}", f"{self.planned_z:.4f}",
            f"{self.planned_p1:.4f}", f"{self.planned_p2:.4f}", f"{self.planned_p3:.4f}",
            f"{self.actual_x:.4f}", f"{self.actual_y:.4f}", f"{self.actual_z:.4f}",
            f"{self.actual_p1:.4f}", f"{self.actual_p2:.4f}", f"{self.actual_p3:.4f}",
            f"{self.tracking_error_xy:.4f}", f"{self.tracking_error_z:.4f}",
            str(self.segment_id),
            "1" if self.is_travel else "0",
            "1" if self.is_retract else "0",
        ])


@dataclass
class RecordingInfo:
    """Metadata for a completed recording (for browser listing)."""
    timestamp: str = ""
    job_name: str = ""
    status: str = ""
    duration_s: float = 0.0
    num_samples: int = 0
    meta_path: str = ""
    data_path: str = ""
    plate_format: int = 0
    num_wells: int = 0

    @classmethod
    def from_meta(cls, meta_path: Path) -> RecordingInfo | None:
        """Create RecordingInfo from a metadata JSON file."""
        try:
            with open(meta_path) as f:
                meta = json.load(f)
            data_path = meta_path.with_name(
                meta_path.stem.replace("_meta", "_data")
            ).with_suffix(".csv")
            return cls(
                timestamp=meta.get("timestamp", ""),
                job_name=meta.get("job_name", "unknown"),
                status=meta.get("status", "unknown"),
                duration_s=meta.get("duration_s", 0.0),
                num_samples=meta.get("num_samples", 0),
                meta_path=str(meta_path),
                data_path=str(data_path) if data_path.exists() else "",
                plate_format=meta.get("workspace", {}).get("plate_format", 0),
                num_wells=meta.get("num_wells_printed", 0),
            )
        except Exception as e:
            logger.warning(f"Failed to read recording metadata {meta_path}: {e}")
            return None


# ═══════════════════════════════════════════════════════════════════
# PrintRecorder
# ═══════════════════════════════════════════════════════════════════

class PrintRecorder:
    """
    Records print execution data to timestamped files.

    Usage::

        recorder = PrintRecorder()

        # Start recording when print begins
        recorder.start_recording(
            workspace_dict=workspace.to_dict(),
            well_setup_dict=well_setup.to_dict(),
            job_name="Scaffold_Batch_1",
        )

        # Record samples during execution
        recorder.record_sample(
            t=0.5,
            planned=(1.0, 2.0, 0.1, 0.0, 0.0, 0.0),
            actual_xy=(1.01, 1.99),
            actual_zp=(0.11, 0.0, 0.0, 0.0),
            segment_id=0,
            is_travel=False,
        )

        # Stop recording on completion
        recorder.stop_recording(status="completed")

        # Browse past recordings
        recordings = PrintRecorder.list_recordings()

        # Load a recording for replay
        meta, data = PrintRecorder.load_recording(meta_path)
    """

    def __init__(self, output_dir: str | Path = DEFAULT_RECORDS_DIR):
        self.output_dir = Path(output_dir)
        self._recording = False
        self._start_time: float = 0.0
        self._timestamp_str: str = ""
        self._current_meta: dict[str, Any] = {}
        self._samples: list[RecordingSample] = []
        self._meta_path: Path | None = None
        self._data_path: Path | None = None

    # ── Properties ────────────────────────────────────────────────

    @property
    def is_recording(self) -> bool:
        """True if currently recording."""
        return self._recording

    @property
    def sample_count(self) -> int:
        """Number of samples recorded in current session."""
        return len(self._samples)

    @property
    def elapsed_time(self) -> float:
        """Elapsed time since recording started (seconds)."""
        if not self._recording:
            return 0.0
        return time.monotonic() - self._start_time

    # ── P7.1: Start/Record/Stop ──────────────────────────────────

    def start_recording(
        self,
        workspace_dict: dict | None = None,
        well_setup_dict: dict | None = None,
        job_name: str = "unnamed",
        extra_metadata: dict | None = None,
    ) -> Path:
        """
        Begin recording a new print.

        Args:
            workspace_dict: Serialised WorkspaceConfig (from workspace.to_dict())
            well_setup_dict: Serialised well setup (from WellSetupModel.to_dict())
            job_name: Human-readable print job name
            extra_metadata: Any additional key-value pairs to store

        Returns:
            Path to the output directory for this recording
        """
        if self._recording:
            logger.warning("Already recording — stopping previous recording first")
            self.stop_recording(status="interrupted")

        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Generate timestamp-based filenames
        now = datetime.now(timezone.utc)
        self._timestamp_str = now.strftime("%Y%m%d_%H%M%S")
        prefix = f"{self._timestamp_str}_{_sanitize_name(job_name)}"
        self._meta_path = self.output_dir / f"{prefix}_meta.json"
        self._data_path = self.output_dir / f"{prefix}_data.csv"

        # Build metadata (P7.2)
        self._current_meta = {
            "timestamp": now.isoformat(),
            "timestamp_str": self._timestamp_str,
            "job_name": job_name,
            "status": "recording",
            "start_time_monotonic": time.monotonic(),
            "workspace": workspace_dict or {},
            "well_setup": well_setup_dict or {},
        }
        if extra_metadata:
            self._current_meta["extra"] = extra_metadata

        # Reset sample buffer
        self._samples = []
        self._start_time = time.monotonic()
        self._recording = True

        logger.info(f"PrintRecorder: started recording '{job_name}' → {self._meta_path.name}")
        return self.output_dir

    def record_sample(
        self,
        t: float,
        planned: tuple[float, ...],
        actual_xy: tuple[float, float],
        actual_zp: tuple[float, ...],
        segment_id: int = 0,
        is_travel: bool = False,
        is_retract: bool = False,
    ) -> None:
        """
        Record one timestep of data.

        Args:
            t: Time from print start (seconds)
            planned: (x, y, z, p1, p2, p3) planned positions (mm)
            actual_xy: (x, y) actual XY positions (mm)
            actual_zp: (z, p1, p2, p3) actual Z and pump positions (mm)
            segment_id: Which object/segment this belongs to
            is_travel: True for travel moves
            is_retract: True during retract/prime
        """
        if not self._recording:
            return

        # Unpack planned (ensure at least 6 elements)
        px = planned[0] if len(planned) > 0 else 0.0
        py = planned[1] if len(planned) > 1 else 0.0
        pz = planned[2] if len(planned) > 2 else 0.0
        pp1 = planned[3] if len(planned) > 3 else 0.0
        pp2 = planned[4] if len(planned) > 4 else 0.0
        pp3 = planned[5] if len(planned) > 5 else 0.0

        # Unpack actual
        ax, ay = actual_xy
        az = actual_zp[0] if len(actual_zp) > 0 else 0.0
        ap1 = actual_zp[1] if len(actual_zp) > 1 else 0.0
        ap2 = actual_zp[2] if len(actual_zp) > 2 else 0.0
        ap3 = actual_zp[3] if len(actual_zp) > 3 else 0.0

        # Compute tracking errors
        err_xy = ((px - ax) ** 2 + (py - ay) ** 2) ** 0.5
        err_z = abs(pz - az)

        sample = RecordingSample(
            t=t,
            planned_x=px, planned_y=py, planned_z=pz,
            planned_p1=pp1, planned_p2=pp2, planned_p3=pp3,
            actual_x=ax, actual_y=ay, actual_z=az,
            actual_p1=ap1, actual_p2=ap2, actual_p3=ap3,
            tracking_error_xy=err_xy, tracking_error_z=err_z,
            segment_id=segment_id,
            is_travel=is_travel, is_retract=is_retract,
        )
        self._samples.append(sample)

    def record_sample_from_waypoint(
        self,
        waypoint: Any,
        actual_xy: tuple[float, float],
        actual_zp: tuple[float, ...],
    ) -> None:
        """
        Record a sample using a Waypoint object directly.

        Args:
            waypoint: TrajectoryPlanner.Waypoint instance
            actual_xy: (x, y) actual XY positions (mm)
            actual_zp: (z, p1, p2, p3) actual Z and pump positions (mm)
        """
        self.record_sample(
            t=waypoint.t,
            planned=(waypoint.x, waypoint.y, waypoint.z,
                     waypoint.p1, waypoint.p2, waypoint.p3),
            actual_xy=actual_xy,
            actual_zp=actual_zp,
            segment_id=getattr(waypoint, "segment_id", 0),
            is_travel=getattr(waypoint, "is_travel", False),
            is_retract=getattr(waypoint, "is_retract", False),
        )

    def stop_recording(self, status: str = "completed") -> tuple[Path | None, Path | None]:
        """
        Finalize and save the recording.

        Args:
            status: Final status ("completed", "aborted", "error")

        Returns:
            Tuple of (meta_path, data_path) or (None, None) if not recording
        """
        if not self._recording:
            logger.warning("PrintRecorder: stop_recording called but not recording")
            return None, None

        self._recording = False
        duration = time.monotonic() - self._start_time

        # Update metadata with final info
        self._current_meta["status"] = status
        self._current_meta["duration_s"] = round(duration, 3)
        self._current_meta["num_samples"] = len(self._samples)
        self._current_meta["end_time"] = datetime.now(timezone.utc).isoformat()

        # Compute summary statistics
        self._current_meta["summary"] = self._compute_summary()

        # Remove non-serialisable fields
        self._current_meta.pop("start_time_monotonic", None)

        # Write files
        meta_path = self._write_metadata()
        data_path = self._write_csv_data()

        logger.info(
            f"PrintRecorder: stopped recording '{self._current_meta.get('job_name', '')}' "
            f"— {status}, {len(self._samples)} samples, {duration:.1f}s"
        )

        # Clear buffers
        self._samples = []
        self._current_meta = {}

        return meta_path, data_path

    # ── P7.2: JSON Metadata Writer ───────────────────────────────

    def _write_metadata(self) -> Path | None:
        """Write metadata JSON file."""
        if self._meta_path is None:
            return None
        try:
            with open(self._meta_path, "w") as f:
                json.dump(self._current_meta, f, indent=2, default=str)
            logger.debug(f"PrintRecorder: metadata written to {self._meta_path}")
            return self._meta_path
        except Exception as e:
            logger.error(f"PrintRecorder: failed to write metadata: {e}")
            return None

    # ── P7.3: CSV Data Writer ────────────────────────────────────

    def _write_csv_data(self) -> Path | None:
        """Write trajectory data CSV file."""
        if self._data_path is None or not self._samples:
            return None
        try:
            with open(self._data_path, "w") as f:
                # Header
                f.write(",".join(CSV_COLUMNS) + "\n")
                # Data rows
                for sample in self._samples:
                    f.write(sample.to_csv_row() + "\n")
            logger.debug(
                f"PrintRecorder: {len(self._samples)} samples written to {self._data_path}"
            )
            return self._data_path
        except Exception as e:
            logger.error(f"PrintRecorder: failed to write CSV data: {e}")
            return None

    # ── Summary Statistics ────────────────────────────────────────

    def _compute_summary(self) -> dict:
        """Compute summary statistics from recorded samples."""
        if not self._samples:
            return {"empty": True}

        errors_xy = [s.tracking_error_xy for s in self._samples if not s.is_travel]
        errors_z = [s.tracking_error_z for s in self._samples if not s.is_travel]
        travel_count = sum(1 for s in self._samples if s.is_travel)
        print_count = sum(1 for s in self._samples if not s.is_travel and not s.is_retract)
        retract_count = sum(1 for s in self._samples if s.is_retract)
        segments = set(s.segment_id for s in self._samples)

        summary = {
            "total_samples": len(self._samples),
            "print_samples": print_count,
            "travel_samples": travel_count,
            "retract_samples": retract_count,
            "num_segments": len(segments),
        }

        if errors_xy:
            summary["tracking_error_xy_mean_mm"] = round(
                sum(errors_xy) / len(errors_xy), 4
            )
            summary["tracking_error_xy_max_mm"] = round(max(errors_xy), 4)

        if errors_z:
            summary["tracking_error_z_mean_mm"] = round(
                sum(errors_z) / len(errors_z), 4
            )
            summary["tracking_error_z_max_mm"] = round(max(errors_z), 4)

        # Path length (XY during printing only)
        print_samples = [s for s in self._samples if not s.is_travel and not s.is_retract]
        if len(print_samples) > 1:
            path_len = 0.0
            for i in range(1, len(print_samples)):
                dx = print_samples[i].actual_x - print_samples[i - 1].actual_x
                dy = print_samples[i].actual_y - print_samples[i - 1].actual_y
                path_len += (dx**2 + dy**2) ** 0.5
            summary["actual_print_path_length_mm"] = round(path_len, 3)

        return summary

    # ── Static Methods for Loading / Browsing ────────────────────

    @staticmethod
    def load_recording(meta_path: str | Path) -> tuple[dict, list[dict]]:
        """
        Load a recording for replay or analysis.

        Args:
            meta_path: Path to the *_meta.json file

        Returns:
            Tuple of (metadata_dict, list_of_sample_dicts)

        The data is returned as plain dicts for maximum compatibility.
        Use pandas externally if needed::

            import pandas as pd
            meta, samples = PrintRecorder.load_recording(path)
            df = pd.DataFrame(samples)
        """
        meta_path = Path(meta_path)

        # Load metadata
        with open(meta_path) as f:
            meta = json.load(f)

        # Derive data path
        data_path = meta_path.with_name(
            meta_path.stem.replace("_meta", "_data")
        ).with_suffix(".csv")

        samples: list[dict] = []
        if data_path.exists():
            with open(data_path) as f:
                header = f.readline().strip().split(",")
                for line in f:
                    values = line.strip().split(",")
                    if len(values) != len(header):
                        continue
                    row = {}
                    for col, val in zip(header, values):
                        try:
                            if col in ("segment_id",):
                                row[col] = int(val)
                            elif col in ("is_travel", "is_retract"):
                                row[col] = val == "1"
                            else:
                                row[col] = float(val)
                        except ValueError:
                            row[col] = val
                    samples.append(row)

        return meta, samples

    @staticmethod
    def load_recording_as_dataframe(meta_path: str | Path):
        """
        Load recording data as a pandas DataFrame.

        Returns:
            Tuple of (metadata_dict, pandas.DataFrame)

        Raises:
            ImportError if pandas is not available
        """
        import pandas as pd

        meta_path = Path(meta_path)

        with open(meta_path) as f:
            meta = json.load(f)

        data_path = meta_path.with_name(
            meta_path.stem.replace("_meta", "_data")
        ).with_suffix(".csv")

        if data_path.exists():
            df = pd.read_csv(data_path)
        else:
            df = pd.DataFrame(columns=CSV_COLUMNS)

        return meta, df

    @staticmethod
    def list_recordings(
        records_dir: str | Path = DEFAULT_RECORDS_DIR,
        limit: int = 50,
    ) -> list[RecordingInfo]:
        """
        List available recordings, newest first.

        Args:
            records_dir: Directory to search
            limit: Maximum number of recordings to return

        Returns:
            List of RecordingInfo objects, sorted newest first
        """
        records_dir = Path(records_dir)
        if not records_dir.exists():
            return []

        meta_files = sorted(
            records_dir.glob("*_meta.json"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )

        recordings = []
        for mf in meta_files[:limit]:
            info = RecordingInfo.from_meta(mf)
            if info is not None:
                recordings.append(info)

        return recordings

    @staticmethod
    def delete_recording(meta_path: str | Path) -> bool:
        """
        Delete a recording (both meta and data files).

        Args:
            meta_path: Path to the *_meta.json file

        Returns:
            True if successfully deleted
        """
        meta_path = Path(meta_path)
        data_path = meta_path.with_name(
            meta_path.stem.replace("_meta", "_data")
        ).with_suffix(".csv")

        deleted = False
        for path in (meta_path, data_path):
            if path.exists():
                try:
                    path.unlink()
                    deleted = True
                except Exception as e:
                    logger.error(f"Failed to delete {path}: {e}")

        return deleted


# ═══════════════════════════════════════════════════════════════════
# Utility Functions
# ═══════════════════════════════════════════════════════════════════

def _sanitize_name(name: str) -> str:
    """Sanitize a job name for use in filenames."""
    safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in name)
    return safe[:50]  # Limit length
