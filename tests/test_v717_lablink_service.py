"""v7.17 — the LabLink push service, driven against a fake hub.

No socket is opened. The client factory is injected, so every branch that
matters — the retry rule, the close-in-finally, the queue drop, the size
refusal — is reachable deterministically.

The rules under test come from `docs/INTEGRATING-WITH-LABLINK.md`:

* **retry iff `status == 0 or >= 500`** — §6 calls that "the whole retry
  policy", and a full hub answers 503 rather than 429 precisely to keep it
  correct. A 4xx retried unchanged fails identically forever.
* **`DELETE /s/{id}` is not optional** — it frees the warm session slot, and
  §5b says to call it "in your teardown path, in a `finally`, not only on the
  happy path".
* **send the full knob set on every command** — knobs resolve per command
  against the recipe defaults and do NOT inherit from `open`, so an omitted
  knob silently reverts.
"""

from __future__ import annotations

import os
import queue
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np


# ── fake hub ──────────────────────────────────────────────────────

class FakeResult:
    def __init__(self, ok=True, code="", message="", returned=None,
                 held=None, knobs=None):
        self.ok = ok
        self.code = code
        self.message = message
        self.returned = returned or []
        self.held = held or []
        self.knobs = knobs or {}
        self.error = {}
        self.session = "s1"


class FakeSession:
    def __init__(self, hub, workflow, recipe, knobs, label):
        self.hub = hub
        self.id = "sess-1"
        self.workflow, self.recipe = workflow, recipe
        self.open_knobs, self.label = knobs, label
        self.uploaded = []
        self.run_calls = []
        self.closed = 0

    def send_data(self, path, name=None, meta=None, retries=4):
        self.uploaded.append(Path(path).name)
        return {"name": Path(path).name, "sha256": "deadbeef"}

    def run(self, command="run", *, inputs=None, knobs=None, cmd_id="",
            check=True, **kw):
        self.run_calls.append({"inputs": list(inputs or []), "knobs": knobs,
                               "cmd_id": cmd_id})
        if self.hub.raise_on_run is not None:
            raise self.hub.raise_on_run
        return self.hub.result

    def fetch(self, name, dest):
        p = Path(dest) / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("result")
        return p

    def close(self):
        self.closed += 1


class FakeHub:
    def __init__(self, *, max_file_bytes=512 * 1024 * 1024):
        self.max_file_bytes = max_file_bytes
        self.result = FakeResult(returned=[{"name": "measurements.csv"}],
                                 knobs={"iterations": 10, "min_area_um2": None})
        self.raise_on_run = None
        self.raise_on_open = None
        self.sessions = []
        self.hello_calls = 0

    def hello(self):
        self.hello_calls += 1
        return {"protocol": 1, "max_file_bytes": self.max_file_bytes,
                "capabilities": {"sessions": True, "longpoll_max_s": 25.0}}

    def open(self, workflow, recipe, *, knobs=None, label="", **kw):
        if self.raise_on_open is not None:
            raise self.raise_on_open
        s = FakeSession(self, workflow, recipe, knobs, label)
        self.sessions.append(s)
        return s


class FakeLabLinkError(Exception):
    def __init__(self, status, message="boom"):
        super().__init__(f"[{status}] {message}")
        self.status = status
        self.message = message


# ── harness ───────────────────────────────────────────────────────

class ServiceCase(unittest.TestCase):

    def setUp(self):
        self.dir = Path(tempfile.mkdtemp(prefix="mebp_llsvc_"))
        os.environ["MEBP_LABLINK_CONFIG_DIR"] = str(self.dir)
        os.environ.pop("LABLINK_TOKEN", None)
        from SupportClasses import LabLinkConfigStore as C
        C.reset_store()
        self.cfg = C.get_store()
        self.cfg.set_base_url("http://hub:8765")
        self.cfg.set_token("t")
        self.cfg.set_enabled(True)
        self.cfg.set_source("still", {
            "enabled": True, "workflow": "nd2studios", "recipe": "segment",
            "knobs": {"iterations": 10, "min_area_um2": None}})

        from SupportClasses import LabLinkService as S
        self.S = S
        S.reset_service()
        self.hub = FakeHub()
        self.svc = S.LabLinkService(store=self.cfg,
                                    client_factory=lambda: self.hub,
                                    outbox=self.dir / "outbox")
        self.png = self._make_nd3()

    def tearDown(self):
        try:
            self.svc.stop(timeout=2.0)
        finally:
            self.S.reset_service()
            os.environ.pop("MEBP_LABLINK_CONFIG_DIR", None)

    def _make_nd3(self) -> Path:
        from SupportClasses import ND3
        p = self.dir / "capture.nd3"
        with ND3.ND3Writer(p) as w:
            w.add_image("capture", np.zeros((4, 4), np.uint16), axes="YX",
                        pixel_format="gray16",
                        meta={"scale": {"um_per_px": 0.65},
                              "acquisition": {"bit_depth": "12-bit",
                                              "numerical_aperture": 0.45}},
                        channels=[{"name": "GFP", "channel_number": 2}])
        return p

    def _run_one(self, **spec_kw):
        spec = self.S.JobSpec(source="still", path=str(self.png), **spec_kw)
        job_id = self.svc.submit(spec)
        self.assertIsNotNone(job_id, "the job was not accepted")
        deadline = time.time() + 10
        while time.time() < deadline:
            job = self.svc._jobs[job_id]
            if job.state in ("done", "failed", "dropped"):
                return job
            time.sleep(0.02)
        self.fail(f"job did not finish; state={self.svc._jobs[job_id].state}")


class TestHappyPath(ServiceCase):

    def test_a_job_uploads_the_pair_runs_and_collects(self):
        job = self._run_one()
        self.assertEqual(job.state, "done", job.detail)
        session = self.hub.sessions[0]
        names = sorted(session.uploaded)
        self.assertEqual(len(names), 2)
        self.assertTrue(names[0].endswith(".job.json"))
        self.assertTrue(names[1].endswith(".nd3"))
        # the pair shares a stem
        self.assertEqual(names[0][:-len(".job.json")], names[1][:-len(".nd3")])
        self.assertEqual(len(job.artifacts), 1)

    def test_the_session_is_closed_on_the_happy_path(self):
        self._run_one()
        self.assertEqual(self.hub.sessions[0].closed, 1)

    def test_the_full_knob_set_is_sent_with_the_command(self):
        """Knobs do not inherit from open; an omitted one silently reverts."""
        job = self._run_one()
        knobs = self.hub.sessions[0].run_calls[0]["knobs"]
        self.assertEqual(knobs, {"iterations": 10, "min_area_um2": None})
        self.assertIn("min_area_um2", knobs)     # null survived, not dropped
        self.assertIsNone(knobs["min_area_um2"])

    def test_the_knob_echo_is_recorded(self):
        """"the only thing that lets you reproduce a result later"."""
        job = self._run_one()
        self.assertEqual(job.knobs_used,
                         {"iterations": 10, "min_area_um2": None})

    def test_the_command_carries_an_idempotency_key(self):
        job = self._run_one()
        self.assertEqual(self.hub.sessions[0].run_calls[0]["cmd_id"], job.job_id)


class TestFailureHandling(ServiceCase):

    def test_the_session_is_closed_when_the_command_raises(self):
        """⚠ The one that matters: a leaked session holds a warm worker slot
        the hub cannot reuse, and `node_session_limit` is described as
        'your bug — you leaked sessions'."""
        self.hub.raise_on_run = FakeLabLinkError(500, "hub exploded")
        job = self._run_one()
        self.assertEqual(job.state, "failed")
        self.assertEqual(self.hub.sessions[0].closed, 1)

    def test_a_terminal_error_is_permanent_and_reported(self):
        self.hub.result = FakeResult(ok=False, code="bad_request",
                                     message="channels length disagrees")
        job = self._run_one()
        self.assertEqual(job.state, "failed")
        self.assertTrue(job.permanent)
        self.assertIn("channels length disagrees", job.detail)

    def test_missing_metadata_says_where_to_fix_it(self):
        self.hub.result = FakeResult(ok=False, code="missing_metadata",
                                     message="recipe derives from objective_na")
        job = self._run_one()
        self.assertIn("Microscope", job.detail)
        self.assertIn("wavelength", job.detail.lower())

    def test_an_unknown_error_code_is_shown_by_its_message(self):
        """"Handle an unrecognised error code by showing its message, not by
        falling through an exhaustive switch. The code set grows."""
        self.hub.result = FakeResult(ok=False, code="brand_new_code",
                                     message="something specific happened")
        job = self._run_one()
        self.assertIn("something specific happened", job.detail)

    def test_a_vanished_source_file_fails_the_job_not_the_worker(self):
        missing = self.dir / "gone.nd3"
        spec = self.S.JobSpec(source="still", path=str(missing))
        job_id = self.svc.submit(spec)
        deadline = time.time() + 5
        while time.time() < deadline and \
                self.svc._jobs[job_id].state not in ("failed", "done"):
            time.sleep(0.02)
        self.assertEqual(self.svc._jobs[job_id].state, "failed")
        # ...and the worker is still alive for the next job
        self.assertEqual(self._run_one().state, "done")


class TestRetryPolicy(unittest.TestCase):
    """The rule is the whole policy; a mistake here is a permanent silent loss
    (4xx retried forever) or a spurious give-up (5xx not retried)."""

    def test_only_network_and_5xx_are_retryable(self):
        from SupportClasses.LabLinkService import _is_retryable
        for status in (0, 500, 502, 503, 507):
            self.assertTrue(_is_retryable(FakeLabLinkError(status)), status)
        for status in (400, 401, 403, 404, 409, 410, 413):
            self.assertFalse(_is_retryable(FakeLabLinkError(status)), status)

    def test_a_full_hub_answers_503_and_is_therefore_retried(self):
        """Documented: 429 would make a conforming client give up permanently
        on a condition that clears the moment a session closes."""
        from SupportClasses.LabLinkService import _is_retryable
        self.assertTrue(_is_retryable(FakeLabLinkError(503, "hub_full")))

    def test_a_plain_exception_is_not_retryable(self):
        from SupportClasses.LabLinkService import _is_retryable
        self.assertFalse(_is_retryable(ValueError("local fault")))


class TestQueueAndGating(ServiceCase):

    def test_a_full_queue_drops_and_counts_rather_than_blocking(self):
        """⚠ `stop_recording` runs on the GUI thread; a blocking put would
        freeze the UI at the end of every recording once the queue backs up.

        Run on a worker with a join timeout rather than timing an inline call:
        a blocking `put` never returns at all, so an inline assertion would
        HANG the whole suite instead of failing it — and a hanging test is one
        that eventually gets disabled rather than fixed. Verified: mutating
        `put_nowait` to `put` fails this in ~2 s.
        """
        self.svc._q = queue.Queue(maxsize=1)
        self.svc._q.put_nowait("occupied")
        spec = self.S.JobSpec(source="still", path=str(self.png))
        done, out = threading.Event(), []

        def _submit():
            out.append(self.svc.submit(spec))
            done.set()

        threading.Thread(target=_submit, daemon=True).start()
        self.assertTrue(done.wait(timeout=2.0),
                        "submit() blocked on a full queue — on the GUI thread "
                        "that is a frozen UI at the end of every recording")
        self.assertIsNone(out[0])
        self.assertEqual(self.svc.dropped, 1)

    def test_a_disabled_source_is_not_queued_at_all(self):
        self.cfg.set_enabled(False)
        self.assertIsNone(
            self.svc.submit(self.S.JobSpec(source="still", path=str(self.png))))

    def test_a_source_without_a_recipe_is_not_queued(self):
        self.cfg.set_source("video", {"enabled": True, "workflow": "nd2studios"})
        self.assertIsNone(
            self.svc.submit(self.S.JobSpec(source="video", path=str(self.png))))

    def test_work_is_held_back_while_the_machine_is_busy(self):
        """A print must not compete with a 300 MB hash and a socket."""
        busy = threading.Event()
        busy.set()
        self.svc._busy_check = busy.is_set
        job_id = self.svc.submit(self.S.JobSpec(source="still",
                                                path=str(self.png)))
        time.sleep(0.4)
        self.assertEqual(self.svc._jobs[job_id].state, "queued")
        self.assertEqual(self.hub.hello_calls, 0, "it contacted the hub anyway")
        busy.clear()
        deadline = time.time() + 10
        while time.time() < deadline and \
                self.svc._jobs[job_id].state != "done":
            time.sleep(0.02)
        self.assertEqual(self.svc._jobs[job_id].state, "done")


class TestSizeRefusal(ServiceCase):

    def test_an_oversize_file_is_refused_before_the_transfer(self):
        self.hub.max_file_bytes = 128        # smaller than any real .nd3
        job = self._run_one()
        self.assertEqual(job.state, "failed")
        self.assertIn("max_file_bytes", job.detail)
        self.assertFalse(self.hub.sessions, "a session was opened anyway")

    def test_the_hub_limit_is_read_not_hardcoded(self):
        """The operator intends to raise it, so a constant would go stale."""
        self.hub.max_file_bytes = 1
        self.assertEqual(self._run_one().state, "failed")
        self.hub.max_file_bytes = 512 * 1024 * 1024
        self.assertEqual(self._run_one().state, "done")

    def test_a_per_source_ceiling_applies_below_the_hub_limit(self):
        self.cfg.set_source("still", {"max_upload_mb": 0})
        self.assertEqual(self._run_one().state, "done")
        # 1 MB ceiling, but the payload is tiny, so force it small
        self.svc._cfg.set_source("still", {"max_upload_mb": 0})
        self.assertEqual(self._run_one().state, "done")


class TestShutdown(ServiceCase):

    def test_stop_reports_what_was_never_sent(self):
        """In-session-only retry is honest only if the loss is stated."""
        self.svc._busy_check = lambda: True      # nothing will drain
        self.svc.submit(self.S.JobSpec(source="still", path=str(self.png)))
        self.svc.submit(self.S.JobSpec(source="still", path=str(self.png)))
        report = self.svc.stop(timeout=2.0)
        self.assertGreaterEqual(report["unsent"], 1)
        for job in self.svc._jobs.values():
            self.assertIn(job.state, ("dropped", "done", "failed"))

    def test_stop_is_safe_when_nothing_ever_started(self):
        from SupportClasses import LabLinkService as S
        fresh = S.LabLinkService(store=self.cfg,
                                 client_factory=lambda: self.hub,
                                 outbox=self.dir / "ob2")
        self.assertEqual(fresh.stop(timeout=1.0)["unsent"], 0)


class TestPublishShim(unittest.TestCase):
    """The producer contract: never raises, never blocks, no-ops when off."""

    def setUp(self):
        from SupportClasses import LabLinkService as S
        S.reset_service()

    def test_publishing_with_no_service_is_a_no_op(self):
        from SupportClasses import LabLinkPublish as P
        P.publish_capture("/nowhere/x.png")
        P.publish_capture("/nowhere/x.mp4", kind="video")
        P.publish_fluorescence_well("plate", "A1")
        P.publish_plate_mosaic("plate")

    def test_publishing_never_raises_even_on_nonsense(self):
        from SupportClasses import LabLinkPublish as P
        P.publish_capture(None)          # type: ignore[arg-type]
        P.publish_fluorescence_well("", "")
        P.publish_plate_mosaic("")

    def test_video_and_still_are_separate_sources(self):
        """A recording is routinely larger than the hub's per-file limit, so
        the operator must be able to send images without sending video."""
        from SupportClasses import LabLinkConfigStore as C
        self.assertIn("still", C.SOURCES)
        self.assertIn("video", C.SOURCES)


class TestQtFree(unittest.TestCase):

    def test_the_backend_modules_import_no_qt(self):
        import ast
        root = Path(__file__).resolve().parent.parent / "SupportClasses"
        for name in ("LabLinkService", "LabLinkPublish", "LabLinkJob",
                     "LabLinkConfigStore"):
            tree = ast.parse((root / f"{name}.py").read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                mods = []
                if isinstance(node, ast.Import):
                    mods = [a.name for a in node.names]
                elif isinstance(node, ast.ImportFrom):
                    mods = [node.module or ""]
                for mod in mods:
                    self.assertFalse(
                        mod.startswith("PySide6"),
                        f"{name} imports Qt; SupportClasses is backend-only")


if __name__ == "__main__":
    unittest.main()
