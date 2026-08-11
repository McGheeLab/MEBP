"""
LabLinkService.py — push MEBP imaging outputs to a LabLink hub, get results back.

v7.17. One job is one **session**::

    POST /s  ->  upload <stem>.nd3 + <stem>.job.json  ->  POST /s/{id}/cmd
             ->  long-poll to a terminal state  ->  collect artifacts
             ->  DELETE /s/{id}          (in a finally, on every exit path)

**The page is a control surface; this is the owner.** The operator's
requirement is *"when we turn it on with this workflow page, it automatically
sends outputs"* — so sending has to continue when they navigate away, which a
page-owned thread cannot do. Qt-free for the same reason it is a singleton:
`SupportClasses` is backend-only, and observers attach with plain callbacks.

⚠ **Listener callbacks run on a worker thread.** Do not touch Qt from one —
use `gui/widgets/lablink_bridge.py`, which emits a payload-free signal that
Qt delivers on the GUI thread, and re-read `snapshot()` from there.

DISCOVER, DO NOT HARDCODE (`docs/INTEGRATING-WITH-LABLINK.md` §3). Channel
names come from the open response and are never derived; `max_file_bytes`,
`longpoll_max_s` and the session quota come from `/hello` and `POST /s`;
recipes and knob bounds come from `GET /workflows`. Nothing here caches them
beyond one run.

RETRY, IN FULL: **iff `status == 0 or status >= 500`.** A 4xx and a terminal
command `error` both mean the job description is wrong and the same bytes will
fail identically. A full hub answers **503, not 429**, specifically so that
rule stays correct.
"""

from __future__ import annotations

import json
import logging
import queue
import shutil
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

logger = logging.getLogger(__name__)

#: Bounded on purpose. A full queue must drop and SAY so, never block: one of
#: the producers (`stop_recording`) runs on the GUI thread.
MAX_PENDING = 32

#: Added to the hub's `longpoll_max_s` for the socket timeout. A timeout BELOW
#: the hub's long-poll turns a working call into a retry storm.
LONGPOLL_MARGIN_S = 10.0

#: Fallback when /hello does not publish one.
DEFAULT_LONGPOLL_MAX_S = 25.0

JOB_STATES = ("queued", "building", "opening", "uploading", "running",
              "collecting", "done", "failed", "dropped")


@dataclass
class JobSpec:
    """What to send. A plain, inspectable record — never a closure."""

    source: str                       # one of LabLinkConfigStore.SOURCES
    plate_key: str = ""
    well: str = ""
    path: str = ""                    # for captures: the file already written
    note: str = ""
    label: str = ""


@dataclass
class JobRecord:
    """Everything the UI needs to say what happened, and why."""

    job_id: str
    spec: JobSpec
    state: str = "queued"
    detail: str = ""
    session_id: str = ""
    cmd_seq: Optional[int] = None
    recipe: str = ""
    upload_stem: str = ""
    #: The echo of every knob in effect, including recipe defaults and the
    #: ones left derived. `QUICKSTART.md` §10: *"Log this. It is the record of
    #: what actually ran, and the only thing that lets you reproduce a result
    #: later."*
    knobs_used: dict = field(default_factory=dict)
    artifacts: list = field(default_factory=list)
    missing_metadata: tuple = ()
    permanent: bool = False           # a 4xx / terminal error: do not retry
    started: float = 0.0
    finished: float = 0.0

    def as_dict(self) -> dict:
        d = dict(self.__dict__)
        d["spec"] = dict(self.spec.__dict__)
        return json.loads(json.dumps(d, default=str))


def _is_retryable(exc) -> bool:
    """The whole retry policy, from the integration doc §6."""
    status = getattr(exc, "status", None)
    if status is None:
        return False                  # not a LabLinkError: a local fault
    return status == 0 or status >= 500


class LabLinkService:
    """Owns the queue, the worker thread and the job index."""

    def __init__(self, *, store=None, client_factory: Optional[Callable] = None,
                 outbox: Optional[Path] = None, busy_check: Optional[Callable] = None):
        from SupportClasses import LabLinkConfigStore as cfg
        self._cfg = store if store is not None else cfg.get_store()
        self._client_factory = client_factory or self._default_client
        self._outbox = Path(outbox) if outbox is not None else (
            self._cfg.path.parent / "lablink_outbox")
        #: Returns True while the machine must not be disturbed. Hashing 300 MB
        #: and writing a socket alongside a print can starve the serial reader
        #: enough to trip a disconnect watchdog — this repo has three update
        #: plans about that failure class, and it would present as "the
        #: printer randomly disconnects since v7.17".
        self._busy_check = busy_check

        self._q: "queue.Queue" = queue.Queue(maxsize=MAX_PENDING)
        self._jobs: dict = {}
        self._order: list = []
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._listeners: list = []
        self._seq = 0
        self.dropped = 0
        self.last_error = ""

    # ── Lifecycle ─────────────────────────────────────────────────

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._run, name="LabLinkUpload", daemon=True)
            self._thread.start()

    def stop(self, timeout: float = 5.0) -> dict:
        """Stop the worker and report what was NOT sent.

        The operator chose in-session-only retry, which is honest only if the
        loss is stated at the moment it happens.
        """
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
        with self._lock:
            unsent = [j for j in self._jobs.values()
                      if j.state in ("queued", "building", "opening",
                                     "uploading", "running", "collecting")]
            for job in unsent:
                job.state = "dropped"
                job.detail = "MEBP closed before this job finished"
            self._thread = None
        report = {"unsent": len(unsent), "dropped_full": self.dropped}
        self._notify()
        return report

    # ── Observation ───────────────────────────────────────────────

    def add_listener(self, cb) -> None:
        with self._lock:
            if cb not in self._listeners:
                self._listeners.append(cb)

    def remove_listener(self, cb) -> None:
        with self._lock:
            if cb in self._listeners:
                self._listeners.remove(cb)

    def _notify(self) -> None:
        for cb in list(self._listeners):
            try:
                cb(self)
            except Exception:      # a bad listener must not break the worker
                logger.exception("LabLink listener failed")

    def snapshot(self, limit: int = 50) -> dict:
        """A plain deep copy for the UI. Safe to call from any thread."""
        with self._lock:
            recent = [self._jobs[j].as_dict()
                      for j in self._order[-limit:] if j in self._jobs]
            counts: dict = {}
            for job in self._jobs.values():
                counts[job.state] = counts.get(job.state, 0) + 1
        return {"jobs": recent, "counts": counts, "dropped_full": self.dropped,
                "last_error": self.last_error,
                "running": bool(self._thread and self._thread.is_alive()),
                "pending": self._q.qsize()}

    # ── Submission ────────────────────────────────────────────────

    def submit(self, spec: JobSpec) -> Optional[str]:
        """Queue a job. Returns its id, or None if it was not accepted.

        NEVER blocks and never raises — one caller is on the GUI thread.
        """
        try:
            if not self._cfg.source_enabled(spec.source):
                return None
        except Exception:
            logger.exception("LabLink: could not read the config")
            return None
        with self._lock:
            self._seq += 1
            job_id = f"j{self._seq:05d}"
            record = JobRecord(job_id=job_id, spec=spec, started=time.time())
            self._jobs[job_id] = record
            self._order.append(job_id)
        try:
            self._q.put_nowait(job_id)
        except queue.Full:
            self.dropped += 1
            with self._lock:
                record.state = "dropped"
                record.detail = (
                    f"the queue was full ({MAX_PENDING} pending) — this output "
                    f"was not sent")
            logger.warning("LabLink: queue full, dropped %s", spec.source)
            self._notify()
            return None
        self.start()
        self._notify()
        return job_id

    # ── Worker ────────────────────────────────────────────────────

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                job_id = self._q.get(timeout=0.25)
            except queue.Empty:
                continue
            # Hold work back while the machine is doing something that must
            # not be disturbed, rather than dropping it.
            while not self._stop.is_set() and self._is_busy():
                time.sleep(0.5)
            if self._stop.is_set():
                return
            with self._lock:
                job = self._jobs.get(job_id)
            if job is None:
                continue
            try:
                self._process(job)
            except Exception as exc:
                # A raise must END the job visibly, never strand it. The
                # capture recorder was fixed into exactly this shape after an
                # unhandled exception in a daemon thread left a session
                # permanently "recording" with no trace anywhere.
                logger.exception("LabLink job %s failed", job.job_id)
                self._fail(job, f"{type(exc).__name__}: {exc}")
            finally:
                job.finished = time.time()
                self._notify()

    def _is_busy(self) -> bool:
        try:
            return bool(self._busy_check and self._busy_check())
        except Exception:
            return False

    def _set(self, job: JobRecord, state: str, detail: str = "") -> None:
        with self._lock:
            job.state = state
            if detail:
                job.detail = detail
        self._notify()

    def _fail(self, job: JobRecord, detail: str, *, permanent: bool = True) -> None:
        with self._lock:
            job.state = "failed"
            job.detail = detail
            job.permanent = permanent
        self.last_error = detail

    def _default_client(self):
        from SupportClasses.lablink import SessionClient
        from SupportClasses.lablink import protocol as p
        base = self._cfg.base_url()
        token = self._cfg.token()
        if not base or not token:
            raise RuntimeError("LabLink is not configured (needs a URL and a token)")
        # ⚠ ABOVE the hub's long-poll window, not below.
        timeout = getattr(p, "LONGPOLL_MAX_S", DEFAULT_LONGPOLL_MAX_S) + LONGPOLL_MARGIN_S
        return SessionClient(base, token,
                             node_id=self._cfg.node_id() or None,
                             timeout=timeout)

    def _process(self, job: JobRecord) -> None:
        from SupportClasses import LabLinkJob as J

        settings = self._cfg.source(job.spec.source)
        job.recipe = settings.get("recipe", "")
        knobs = settings.get("knobs") or {}

        # ---- build the snapshot + sidecar -------------------------------
        self._set(job, "building")
        pair = self._build_pair(job, settings)
        job.upload_stem = pair.stem
        job.missing_metadata = pair.missing
        if pair.missing:
            logger.info("LabLink %s: %s", job.job_id, J.describe_missing(pair.missing))

        hub = self._client_factory()
        info = hub.hello()
        if not self._check_size(job, pair, info):
            return

        # ---- one session, closed on every exit path ----------------------
        self._set(job, "opening")
        session = hub.open(settings.get("workflow", ""), job.recipe,
                           knobs=knobs or None,
                           label=job.spec.label or self._label(job))
        job.session_id = getattr(session, "id", "")
        try:
            self._set(job, "uploading")
            refs = [session.send_data(p) for p in pair.files()]

            self._set(job, "running")
            # ⚠ The FULL knob set every command: knobs are resolved per command
            # against the recipe's defaults and do NOT inherit from open, so an
            # omitted knob silently reverts.
            result = session.run(inputs=refs, knobs=knobs or None,
                                 cmd_id=job.job_id, check=False)
            job.cmd_seq = getattr(result, "session", None) and job.cmd_seq
            job.knobs_used = dict(getattr(result, "knobs", {}) or {})
            if not result.ok:
                self._fail(job, self._explain(result), permanent=True)
                return

            self._set(job, "collecting")
            job.artifacts = self._collect(job, session, result)
            self._set(job, "done", f"{len(job.artifacts)} artifact(s)")
        finally:
            # Not optional: it frees the warm session slot the hub is holding.
            try:
                session.close()
            except Exception as exc:
                logger.warning("LabLink: could not close session %s: %s",
                               job.session_id, exc)

    def _build_pair(self, job: JobRecord, settings: dict):
        from SupportClasses import LabLinkJob as J
        nd3, original = self._export(job)
        optics = self._optics_lookup()
        sidecar, missing = J.build_sidecar(
            nd3, optics_lookup=optics, original_name=original,
            note=job.spec.note)
        stem = J.build_stem(job.spec.source.split("_")[0] or "mebp",
                            job.spec.well or job.spec.plate_key)
        pair = J.write_job_pair(nd3, self._outbox, stem, sidecar, move=True)
        pair.missing = missing
        return pair

    def _export(self, job: JobRecord):
        """Produce a `.nd3` for this job. Returns (path, original name)."""
        from SupportClasses import ND3Export
        self._outbox.mkdir(parents=True, exist_ok=True)
        tmp = self._outbox / f"_build_{job.job_id}.nd3"
        spec = job.spec
        if spec.source == "fluorescence_well":
            from SupportClasses import FluorescenceMosaicStore as fms
            ND3Export.export_fluorescence_well(
                fms.get_store(), spec.plate_key, spec.well, tmp)
            return tmp, f"{spec.plate_key}|{spec.well}"
        if spec.source == "plate_mosaic":
            from SupportClasses import MosaicStore
            ND3Export.export_plate_mosaic(
                MosaicStore.get_store(), spec.plate_key, tmp)
            return tmp, str(spec.plate_key)
        # Captures are already a file on disk.
        src = Path(spec.path)
        if not src.is_file():
            raise FileNotFoundError(f"{src} no longer exists")
        if src.suffix.lower() == ".nd3":
            shutil.copy2(src, tmp)
            return tmp, src.name
        from SupportClasses import CaptureMetadata
        import cv2
        array = cv2.imread(str(src), cv2.IMREAD_UNCHANGED)
        if array is None:
            raise ValueError(f"{src.name} could not be read")
        meta = CaptureMetadata.read_sidecar(src) if hasattr(
            CaptureMetadata, "read_sidecar") else {}
        ND3Export.export_capture(array, meta, tmp)
        return tmp, src.name

    def _optics_lookup(self):
        try:
            from SupportClasses import MicroscopeConfigStore as mcs
            return mcs.get_store().filter_optics_for
        except Exception:
            return None

    def _check_size(self, job, pair, info) -> bool:
        """Refuse locally rather than spending the transfer to learn.

        The hub's own figure is authoritative and read at run time — the
        operator intends to raise it — and the per-source ceiling is a
        separate, smaller bound for a slow link.
        """
        total = sum(p.stat().st_size for p in pair.files())
        hub_max = int(info.get("max_file_bytes") or 0)
        biggest = max(p.stat().st_size for p in pair.files())
        if hub_max and biggest > hub_max:
            self._fail(job,
                       f"{biggest / 1e6:.0f} MB exceeds this hub's limit of "
                       f"{hub_max / 1e6:.0f} MB (max_file_bytes from /hello). "
                       f"Raise it on the server or send a smaller region.")
            return False
        local_max = self._cfg.max_upload_bytes(job.spec.source)
        if local_max and total > local_max:
            self._fail(job,
                       f"{total / 1e6:.0f} MB exceeds the {local_max / 1e6:.0f} MB "
                       f"ceiling set for {job.spec.source}")
            return False
        return True

    def _label(self, job: JobRecord) -> str:
        bits = [b for b in (job.spec.plate_key, job.spec.well) if b]
        return " ".join(bits) or job.spec.source

    def _explain(self, result) -> str:
        """Show an unrecognised code by its MESSAGE — the set grows."""
        code = getattr(result, "code", "") or ""
        message = getattr(result, "message", "") or ""
        if code == "missing_metadata":
            fields = ((getattr(result, "error", None) or {}).get("fields") or [])
            extra = f" ({', '.join(fields)})" if fields else ""
            return (f"the recipe needs optical metadata this job does not "
                    f"carry{extra}. Fill the filter cube's wavelengths in on "
                    f"Hardware Setup → Microscope. {message}")
        return f"{code}: {message}" if code else (message or "the job failed")

    def _collect(self, job: JobRecord, session, result) -> list:
        """Download what came back. Held artifacts are NOT pulled by default —
        a multi-gigabyte label volume nobody opened should not cross a
        1.5 MB/s link on its own."""
        from SupportClasses import LabLinkJob as J
        dest = self._results_dir() / job.job_id
        dest.mkdir(parents=True, exist_ok=True)
        out = []
        for rec in (getattr(result, "returned", None) or []):
            name = rec.get("name") if isinstance(rec, dict) else str(rec)
            if not name:
                continue
            try:
                # ⚠ Validate a SERVER-SUPPLIED name before it reaches the
                # filesystem. On Windows `Path("C:/x") / "//host/share/f"` is a
                # UNC path, and merely calling .exists() offers your NTLM
                # credentials to whoever answers.
                J.validate_upload_name(name)
                path = session.fetch(name, dest)
                out.append(str(path))
            except Exception as exc:
                logger.warning("LabLink: could not collect %s: %s", name, exc)
        for rec in (getattr(result, "held", None) or []):
            name = rec.get("name") if isinstance(rec, dict) else str(rec)
            logger.info("LabLink %s: %r is held on the hub until pulled",
                        job.job_id, name)
        return out

    def _results_dir(self) -> Path:
        return self._cfg.path.parent / "lablink_results"


# ── Module-level singleton ────────────────────────────────────────

_service: Optional[LabLinkService] = None


def get_service(**kwargs) -> LabLinkService:
    global _service
    if _service is None:
        _service = LabLinkService(**kwargs)
    return _service


def peek_service() -> Optional[LabLinkService]:
    """The service if one exists, else None — never constructs one.

    Used by the producer shim so publishing costs nothing at all when the
    feature has never been turned on.
    """
    return _service


def reset_service() -> None:
    global _service
    if _service is not None:
        try:
            _service.stop(timeout=1.0)
        except Exception:
            pass
    _service = None
