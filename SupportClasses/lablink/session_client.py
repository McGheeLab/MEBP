"""The node's side of a session: open, send data, run, collect, close.

    from lablink.session_client import SessionClient

    hub = SessionClient("http://hub:8765", token, node_id="scope-a")
    with hub.open("nd2studios", "segment-measure-2d",
                  knobs={"background_um": 6.0}) as s:      # blocks until ready
        ref = s.send_data(r"D:\\scans\\2026-08-03 A1 (well 3).nd2")
        for level in ("otsu", "li", "yen"):
            r = s.run(inputs=[ref], knobs={"level": level},
                      on_progress=lambda p: print("  " + p.text))
            print(f"{level}: {r.result.get('count')} objects "
                  f"in {r.duration_s:.2f}s{'  (cached)' if r.cached else ''}")
        s.pull()                       # anything the recipe held back
        s.fetch_all("inbox")

WHY THIS COMPOSES LabLinkClient RATHER THAN SUBCLASSING IT. LabLinkClient is
stateless — every method is one self-contained request, which is exactly why
"crash and retry" works — and it is what service.py, sync.py, lablink_cli.py,
lablink_connect.py and every QUICKSTART example depend on. A session is the
opposite: it has an id, a state, and an obligation to close. Putting close() on
LabLinkClient would make it genuinely ambiguous whether that closes a socket or a
session, and a subclass would inherit the wrong lifetime.

THE ONE RULE FOR A NODE PROGRAMMER: a session must be closed. Use the context
manager. A leaked warm instance holds one of the hub's session slots until the
idle reaper fires many minutes later, and from the next node's point of view that
is indistinguishable from a broken hub. __exit__ closes on the exception path too,
and never lets a close failure replace the original exception.

NOTHING HERE IS REQUIRED TO USE A SESSION. It is eight ordinary HTTP calls, and
QUICKSTART §10 shows the same flow in curl and in stdlib-only Python. This module
is the convenience, not the protocol.
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, field
from pathlib import Path

from .client import LabLinkClient, LabLinkError
from .fsutil import human_size, safe_name
from .protocol import DEFAULT_TIMEOUT_S, LONGPOLL_MAX_S

log = logging.getLogger("lablink.session_client")

#: A session that never reaches "ready" in this many seconds is a failure to
#: report, not a wait to extend. It deliberately exceeds the hub's own default
#: WORKER_OPEN_TIMEOUT_S (120) so the hub's message — which says WHY — wins the
#: race and the node does not report a bare timeout over a readable diagnosis.
READY_TIMEOUT_S = 180.0

#: Consecutive failed polls tolerated while a command runs. The command is
#: running on the hub and does not care about our socket, so a dropped poll must
#: not lose it; this only bounds a hub that has genuinely gone away.
POLL_RETRIES = 5

LIVE_STATES = frozenset({"ready", "running"})
TERMINAL_STATES = frozenset({"closed", "failed", "cancelled", "lost", "expired"})


# ---------------------------------------------------------------------------
# errors
# ---------------------------------------------------------------------------
class SessionError(Exception):
    """A session or command went wrong. NOT a LabLinkError.

    Deliberately outside the LabLinkError hierarchy, because LabLinkError.retryable
    is a claim about the TRANSPORT: "the request did not get through, send it
    again". These failures got through perfectly. "Your recipe found no cells" and
    "the worker was killed" are both non-retryable in the transport sense while
    needing completely different responses from the caller, so folding them into
    a boolean would only mislead. Catch SessionError for "the work failed" and
    LabLinkError for "the hub is unreachable"; they are separate questions.
    """

    def __init__(self, code: str, message: str, *, session: str = "",
                 cmd_seq: int | None = None, detail: str = ""):
        super().__init__(message)
        self.code = code
        self.message = message
        self.session = session
        self.cmd_seq = cmd_seq
        self.detail = detail


class CommandFailed(SessionError):
    """A command finished in a non-done state. The session may still be usable."""


class SessionLost(SessionError):
    """The session itself is gone: failed, expired, cancelled, or lost."""


def _gone(exc: LabLinkError, session: str) -> None:
    """Re-raise 410 as SessionLost; anything else unchanged.

    The hub answers 410 rather than 404 for a finished session so a node does not
    conclude it typo'd the id and retry the same wrong string forever. That is a
    fact about the SESSION, so it should not surface here as an HTTP status the
    caller then has to interpret.
    """
    if exc.status == 410:
        raise SessionLost("session_gone", exc.message, session=session) from None
    raise exc


# ---------------------------------------------------------------------------
# small records
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class InputRef:
    """A file that is on the hub, by the name the hub actually stored it under.

    `name` may differ from `original`: the exchange's name rules refuse characters
    that appear in real microscopy filenames, and two different files may want the
    same name in one session. Pass the ref itself to run(); never re-derive the
    name, because the ref carries the sha256 the hub verifies against.
    """

    name: str
    sha256: str
    size: int
    original: str
    status: str = "created"        # "created" or "duplicate"

    @property
    def renamed(self) -> bool:
        return self.name != self.original

    def to_wire(self) -> dict:
        return {"name": self.name, "sha256": self.sha256}

    def __str__(self) -> str:
        via = f" (sent as {self.name})" if self.renamed else ""
        return f"{self.original}{via} {human_size(self.size)}"


@dataclass(frozen=True)
class Progress:
    """One progress observation, with the display string already decided.

    `text` exists so a caller cannot accidentally invent a percentage. The obvious
    callback signature — `on_progress(fraction, message)` — breaks the moment a
    node reports no total, which is the normal case for a learned segmenter: the
    fraction is None and the caller's own `f"{p:.0%}"` raises TypeError in the
    middle of a run that was going fine. So the fraction is here for anyone who
    wants it, and `text` is the answer for everyone who does not.
    """

    done: int
    total: int
    fraction: float | None
    label: str
    note: str
    determinate: bool
    cached: int
    computed: int

    @property
    def text(self) -> str:
        step = f"{self.done}/{self.total}" if self.total else f"{self.done}/?"
        if self.determinate and self.fraction is not None:
            head = f"{step} {self.label} {self.fraction * 100:3.0f}%"
        else:
            head = f"{step} {self.label}"
        return f"{head} — {self.note}" if self.note else head

    @classmethod
    def from_doc(cls, p: dict) -> "Progress":
        return cls(
            done=int(p.get("done") or 0),
            total=int(p.get("total") or 0),
            fraction=(p.get("fraction") if isinstance(p.get("fraction"), (int, float))
                      else None),
            label=str(p.get("current_label") or p.get("current") or "working"),
            note=str(p.get("note") or ""),
            determinate=bool(p.get("determinate")),
            cached=int(p.get("cached") or 0),
            computed=int(p.get("computed") or 0),
        )

    def _key(self):
        return (self.done, self.label, self.note, self.fraction)


@dataclass(frozen=True)
class CommandResult:
    """One finished command. Mirrors worker_link.WorkerReply's shape on purpose."""

    cmd_seq: int
    cmd_id: str
    command: str
    state: str
    result: dict = field(default_factory=dict)
    artifacts: list = field(default_factory=list)
    error: dict | None = None
    cached: bool = False
    duration_s: float = 0.0
    session: str = ""
    progress: dict = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.state == "done"

    @property
    def code(self) -> str:
        return str((self.error or {}).get("code") or ("ok" if self.ok else self.state))

    @property
    def message(self) -> str:
        return str((self.error or {}).get("message") or "")

    @property
    def returned(self) -> list:
        """Names of artifacts that reached the out channel, ready to fetch."""
        # returned_as, not name: the exchange's name rules may have forced a
        # substitution, and the substituted name is the one that can be fetched.
        return [a.get("returned_as") or a.get("name") for a in self.artifacts
                if a.get("published")]

    @property
    def held(self) -> list:
        """Artifacts the recipe kept on the hub until pull() asks for them."""
        return [a.get("name") for a in self.artifacts
                if not a.get("published") and a.get("policy") == "pull"]

    def raise_for_status(self) -> "CommandResult":
        if self.ok:
            return self
        raise CommandFailed(self.code, self.message or f"command {self.state}",
                            session=self.session, cmd_seq=self.cmd_seq)

    def __str__(self) -> str:
        tail = "  (cached)" if self.cached else ""
        if self.ok:
            return f"cmd {self.cmd_seq} done in {self.duration_s:.3f}s{tail}"
        return f"cmd {self.cmd_seq} {self.state}: {self.code} {self.message}"


# ---------------------------------------------------------------------------
# the client
# ---------------------------------------------------------------------------
class SessionClient:
    """Opens sessions on a hub. Stateless itself; the state lives in HubSession."""

    def __init__(self, base_url: str, token: str, *, node_id: str | None = None,
                 timeout: float = DEFAULT_TIMEOUT_S):
        self.client = LabLinkClient(base_url, token, timeout=timeout,
                                    node_id=node_id)

    @classmethod
    def from_client(cls, client: LabLinkClient) -> "SessionClient":
        """Wrap an already-configured LabLinkClient (its token, timeout, node id)."""
        self = cls.__new__(cls)
        self.client = client
        return self

    @property
    def base_url(self) -> str:
        return self.client.base_url

    # ------------------------------------------------------------------ discovery
    def hello(self) -> dict:
        return self.client.hello()

    def supports_sessions(self) -> bool:
        """Ask before assuming: a plain file exchange answers 404 on /s."""
        return bool(self.hello().get("capabilities", {}).get("sessions"))

    def workflows(self) -> list:
        """What this hub offers, with each recipe's knob schema and bounds."""
        return self.client._json("GET", f"{self.base_url}/workflows").get(
            "workflows", [])

    def sessions(self) -> list:
        return self.client._json("GET", f"{self.base_url}/s").get("sessions", [])

    # ------------------------------------------------------------------ open
    def open(self, workflow: str, recipe: str, *, knobs: dict | None = None,
             label: str = "", graph=None, wait: bool = True,
             ready_timeout: float = READY_TIMEOUT_S) -> "HubSession":
        """Open a session and, by default, block until the worker says it is ready.

        `wait=False` returns immediately in state "opening" — useful when a node
        wants to upload its data while the worker is still starting, which is the
        one genuinely useful overlap in the whole flow.
        """
        body = {"workflow": workflow, "recipe": recipe}
        if knobs:
            body["knobs"] = knobs
        if label:
            body["label"] = label
        if graph is not None:
            body["graph"] = graph
        doc = self._post("/s", body)
        sess = HubSession(self, doc)
        if wait:
            sess.wait_ready(timeout=ready_timeout)
        return sess

    def attach(self, session_id: str) -> "HubSession":
        """Reattach to a session by id — after a node-side crash, say.

        The session outlives the process that opened it, which is why the id is
        worth writing down: the alternative is a warm instance nobody can reach
        and nobody can close.
        """
        try:
            doc = self.client._json("GET", f"{self.base_url}/s/{session_id}")
        except LabLinkError as exc:
            _gone(exc, session_id)
            raise                          # pragma: no cover - _gone re-raises
        return HubSession(self, doc)

    # ------------------------------------------------------------------ plumbing
    def _post(self, path: str, body: dict | None = None) -> dict:
        data = json.dumps(body or {}).encode("utf-8")
        return self.client._json(
            "POST", f"{self.base_url}{path}", data=data,
            headers={"Content-Type": "application/json",
                     "Content-Length": str(len(data))})


# ---------------------------------------------------------------------------
# one session
# ---------------------------------------------------------------------------
class HubSession:
    """A node's handle to one session running on the hub.

    Named HubSession, not Session, because lablink.sessions.Session is the hub-side
    object that owns the worker process. Two classes called Session in one package
    is a trap for whoever greps next.
    """

    def __init__(self, hub: SessionClient, doc: dict):
        self.hub = hub
        self.doc = dict(doc)
        self.id = str(doc["id"])
        self.in_channel = str(doc["in_channel"])
        self.out_channel = str(doc["out_channel"])
        self.workflow = doc.get("workflow")
        self.recipe = doc.get("recipe")
        self.cursor = int(doc.get("event_seq") or 0)
        self._closed = False
        self._sent: dict = {}          # sha256 -> InputRef, for free re-sends

    # ------------------------------------------------------------------ state
    @property
    def state(self) -> str:
        return str(self.doc.get("state") or "unknown")

    @property
    def alive(self) -> bool:
        return self.state not in TERMINAL_STATES

    def status(self, *, wait: float = 0.0, since_event: int | None = None) -> dict:
        """Fetch current state. `wait` long-polls for up to that many seconds."""
        query = {}
        if wait:
            query["wait"] = self._clamp_wait(wait)
            query["since_event"] = (self.cursor if since_event is None
                                    else since_event)
        try:
            doc = self.hub.client._json(
                "GET", self.hub.client._url("s", self.id, query=query or None))
        except LabLinkError as exc:
            _gone(exc, self.id)
            raise                          # pragma: no cover - _gone re-raises
        self.doc = doc
        self.cursor = int(doc.get("event_seq") or self.cursor)
        return doc

    def _clamp_wait(self, wait: float) -> float:
        """Never ask the hub to hold a request longer than we will wait for it.

        LONGPOLL_MAX_S (25) already sits under the 30 s that is simultaneously
        Handler.timeout and the default client timeout. But a caller may have
        constructed the client with a SHORTER timeout, in which case a legitimate
        long poll would look like an unreachable hub — so the client's own timeout
        gets the final say.
        """
        ceiling = min(LONGPOLL_MAX_S, max(1.0, self.hub.client.timeout - 5.0))
        return round(min(float(wait), ceiling), 3)

    def events(self, since: int | None = None) -> list:
        """Ledger entries newer than `since` (default: everything we have not seen).

        Reading events does NOT move the long-poll cursor used by wait_ready and
        run — those track their own position, so a diagnostic call here cannot make
        a waiter miss a transition.
        """
        doc = self.hub.client._json(
            "GET", self.hub.client._url(
                "s", self.id,
                query={"since_event": 0 if since is None else since}))
        return doc.get("events") or []

    def wait_ready(self, timeout: float = READY_TIMEOUT_S) -> "HubSession":
        """Block until the worker is ready. Raises SessionLost if it never is."""
        deadline = time.monotonic() + float(timeout)
        while True:
            if self.state in LIVE_STATES:
                return self
            if self.state in TERMINAL_STATES:
                raise SessionLost(
                    str(self.doc.get("reason") or self.state),
                    f"session {self.id} is {self.state}: "
                    f"{self.doc.get('detail') or self.doc.get('reason') or ''}",
                    session=self.id, detail=str(self.doc.get("detail") or ""))
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SessionLost(
                    "open_timeout",
                    f"session {self.id} was still {self.state} after "
                    f"{timeout:.0f}s. The hub reports: "
                    f"{self.doc.get('detail') or '(nothing yet)'}",
                    session=self.id)
            self.status(wait=min(remaining, LONGPOLL_MAX_S))

    # ------------------------------------------------------------------ data in
    def send_data(self, path, name: str | None = None, meta: dict | None = None,
                  *, retries: int = 4) -> InputRef:
        """Upload a file to this session's input channel. Returns a ref for run().

        Resume, checksum verification and deduplication come from the ordinary
        upload path, unchanged. Two things are added here:

        * The name is repaired with fsutil.safe_name, because real microscopy
          filenames routinely carry characters the exchange refuses and a 400 at
          this point is a puzzle rather than a problem. The original travels in the
          file's metadata, which is what tells a person which file this is.
        * A genuine same-name-different-content collision is disambiguated with a
          `dNNNN-` prefix instead of failing. Iterative sessions naturally re-send
          `frame.tif`; identical content stays a free duplicate.
        """
        path = Path(path)
        original = name or path.name
        base = safe_name(original)
        meta = dict(meta or {})
        if base != original:
            meta.setdefault("original_name", original)
        client = self.hub.client
        send_as = base
        for attempt in range(20):
            try:
                rec = client.upload(self.in_channel, path, name=send_as,
                                    meta=meta or None, retries=retries)
            except LabLinkError as exc:
                if exc.status != 409 or attempt == 19:
                    raise
                # Same name, different content. The hub is right to refuse; pick a
                # name that says which send this was.
                send_as = safe_name(f"d{attempt + 1:04d}-{base}")
                continue
            ref = InputRef(name=str(rec["name"]), sha256=str(rec["sha256"]),
                           size=int(rec.get("size") or 0), original=original,
                           status=str(rec.get("status") or "created"))
            self._sent[ref.sha256] = ref
            if ref.renamed:
                log.info("session %s: sent %r as %r", self.id, original, ref.name)
            return ref
        raise SessionError("name_collision",
                           f"could not find a free name for {original!r} on "
                           f"{self.in_channel}", session=self.id)

    def inputs(self) -> list:
        """What is currently on the input channel, as the hub sees it."""
        return self.hub.client.list_files(self.in_channel).get("files") or []

    # ------------------------------------------------------------------ run
    def run(self, command: str = "run", *, inputs=None, knobs: dict | None = None,
            cmd_id: str = "", on_progress=None, timeout: float | None = None,
            check: bool = True, poll_retries: int = POLL_RETRIES) -> CommandResult:
        """Run one command and wait for it. Returns a CommandResult.

        `timeout=None` waits as long as the hub is willing to run it. That is
        deliberate: a legitimate segmentation batch takes forty minutes, and a
        wall clock here would abandon exactly the expensive work the session
        exists to do. The hub's silence_timeout_s is what kills a WEDGED worker,
        which is a different question and already answered on the correct side.

        `check=True` raises CommandFailed on anything but "done". Pass
        `check=False` when a failure is an expected outcome ("no cells found") and
        you want to inspect `.error` yourself.

        `cmd_id` makes the send idempotent: if the response is lost, sending the
        same id again returns the running command instead of 409 busy. Supply one
        for any command you cannot cheaply repeat.
        """
        body: dict = {"command": command}
        if inputs:
            body["inputs"] = [i.to_wire() if isinstance(i, InputRef)
                              else ({"name": i} if isinstance(i, str) else dict(i))
                              for i in inputs]
        if knobs:
            body["knobs"] = knobs
        if cmd_id:
            body["cmd_id"] = cmd_id
        doc = self.hub._post(f"/s/{self.id}/cmd", body)
        return self.await_command(int(doc["cmd_seq"]), on_progress=on_progress,
                                  timeout=timeout, check=check,
                                  poll_retries=poll_retries)

    def start(self, command: str = "run", *, inputs=None, knobs: dict | None = None,
              cmd_id: str = "") -> int:
        """Send a command and return its cmd_seq without waiting. See await_command."""
        body: dict = {"command": command}
        if inputs:
            body["inputs"] = [i.to_wire() if isinstance(i, InputRef)
                              else ({"name": i} if isinstance(i, str) else dict(i))
                              for i in inputs]
        if knobs:
            body["knobs"] = knobs
        if cmd_id:
            body["cmd_id"] = cmd_id
        return int(self.hub._post(f"/s/{self.id}/cmd", body)["cmd_seq"])

    def await_command(self, cmd_seq: int, *, on_progress=None,
                      timeout: float | None = None, check: bool = True,
                      poll_retries: int = POLL_RETRIES) -> CommandResult:
        """Long-poll one command to completion, reporting progress as it moves."""
        deadline = None if timeout is None else time.monotonic() + float(timeout)
        last_key = None
        stalled = 0
        while True:
            wait = self._clamp_wait(LONGPOLL_MAX_S)
            if deadline is not None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise SessionError(
                        "wait_timeout",
                        f"command {cmd_seq} on session {self.id} was still "
                        f"running after {timeout:.0f}s. It is still running on "
                        f"the hub; cancel() it or await_command() again.",
                        session=self.id, cmd_seq=cmd_seq)
                wait = self._clamp_wait(min(wait, remaining))
            try:
                doc = self.hub.client._json(
                    "GET", self.hub.client._url("s", self.id, "cmd", str(cmd_seq),
                                                query={"wait": wait}))
                stalled = 0
            except LabLinkError as exc:
                # The command does not care about our socket. Only a hub that has
                # genuinely gone away should end the wait.
                if exc.status == 410:
                    _gone(exc, self.id)
                if not exc.retryable or stalled >= poll_retries:
                    raise
                stalled += 1
                log.warning("session %s: poll %d failed (%s); retrying",
                            self.id, stalled, exc)
                time.sleep(min(2 ** (stalled - 1), 8))
                continue

            if on_progress is not None:
                prog = Progress.from_doc(doc.get("progress") or {})
                if prog._key() != last_key:
                    last_key = prog._key()
                    on_progress(prog)

            if doc.get("longpoll_degraded") and str(doc.get("state")) == "running":
                # The hub had no waiter slot, so it answered IMMEDIATELY - and a
                # loop that just asks again turns one busy hub into a client
                # hammering it with a fresh TCP connection per millisecond,
                # which on a busy day exhausts the CLIENT's ephemeral ports
                # (errno 49/99) before it dents the hub. Found by the stress
                # harness, whose twenty degraded pollers did exactly that.
                time.sleep(1.0)

            if str(doc.get("state")) != "running":
                result = CommandResult(
                    cmd_seq=int(doc.get("cmd_seq") or cmd_seq),
                    cmd_id=str(doc.get("cmd_id") or ""),
                    command=str(doc.get("command") or ""),
                    state=str(doc.get("state") or "unknown"),
                    result=dict(doc.get("result") or {}),
                    artifacts=list(doc.get("artifacts") or []),
                    error=doc.get("error"),
                    cached=bool(doc.get("cached")),
                    duration_s=float(doc.get("duration_s") or 0.0),
                    session=self.id,
                    progress=dict(doc.get("progress") or {}))
                self.doc["state"] = doc.get("session_state") or self.state
                return result.raise_for_status() if check else result

    # ------------------------------------------------------------------ results
    def results(self, since_seq: int | None = None) -> list:
        """Files on the out channel: name, size, sha256, seq, meta."""
        return self.hub.client.list_files(
            self.out_channel, since_seq=since_seq).get("files") or []

    def pull(self, names=None) -> list:
        """Publish artifacts the recipe held back. Returns the names published.

        A recipe declares per output whether it returns automatically or waits to
        be asked; a 3 GB label volume nobody looks at should not cross the network
        by default. With no argument, everything currently held is published.
        """
        body = {"names": list(names)} if names else {}
        doc = self.hub._post(f"/s/{self.id}/pull", body)
        return list(doc.get("published") or [])

    def fetch(self, name: str, dest_dir, expected_sha: str | None = None) -> Path:
        """Download one result, with resume and checksum verification."""
        if expected_sha is None:
            match = next((f for f in self.results() if f.get("name") == name), None)
            expected_sha = (match or {}).get("sha256")
        return self.hub.client.download(self.out_channel, name, dest_dir,
                                        expected_sha=expected_sha)

    def fetch_all(self, dest_dir, *, since_seq: int | None = None) -> list:
        """Download everything on the out channel. Returns the local paths."""
        out = []
        for rec in self.results(since_seq=since_seq):
            out.append(self.hub.client.download(
                self.out_channel, rec["name"], dest_dir,
                expected_sha=rec.get("sha256")))
        return out

    # ------------------------------------------------------------------ control
    def cancel(self) -> dict:
        """Ask the hub to stop. Cooperative first, then enforced.

        Returns the hub's own reply, which states the guarantee rather than
        implying one: a compute engine's inner loop usually has no cancellation
        point, so this lands at the next boundary the worker checks, and the hub
        escalates to a kill if it never checks one.
        """
        return self.hub._post(f"/s/{self.id}/cancel")

    def close(self, *, discard: bool = False) -> dict:
        """Close the session. Idempotent from this side.

        `discard=True` also drops the session's channels, including any result
        still sitting on them. The default keeps them: the common mistake is
        closing before fetching, and losing data to a convenience flag is worse
        than leaving two directories for the reaper.
        """
        if self._closed:
            return {"session": self.id, "state": self.state, "already_closed": True}
        self._closed = True
        query = {"discard": "1"} if discard else None
        try:
            doc = self.hub.client._json(
                "DELETE", self.hub.client._url("s", self.id, query=query))
        except LabLinkError as exc:
            if exc.status == 410:
                # Already terminal — reaped, cancelled, or the hub restarted.
                # Nothing is owed and nothing is leaking.
                return {"session": self.id, "state": "gone", "note": exc.message}
            raise
        self.doc.update({k: v for k, v in doc.items() if k in ("state", "reason")})
        return doc

    # ------------------------------------------------------------------ context
    def __enter__(self) -> "HubSession":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        try:
            self.close()
        except Exception as err:                       # noqa: BLE001
            # Never let a failed close replace the caller's exception: the
            # original is the one that explains what happened, and a close error
            # on the way out of a failure is usually a consequence of it.
            log.warning("session %s: close failed on exit (%s)", self.id, err)
        return False

    def __repr__(self) -> str:
        return (f"<HubSession {self.id} {self.state} "
                f"{self.workflow}/{self.recipe}>")
