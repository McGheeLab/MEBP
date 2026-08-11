"""Wire-protocol constants — the single source of truth for LabLink.

Header names, URL shapes, naming rules and limits used by the store, the
HTTP server, the client, the sync agent and the tests. Stdlib only; imports
nothing from the rest of the package.
"""

from __future__ import annotations

import re

PROTOCOL_VERSION = 1
SERVICE_NAME = "lablink"

# ---------------------------------------------------------------------------
# Custom headers — one consistent prefix so a wire trace is self-describing.
# ---------------------------------------------------------------------------
H_TOKEN = "X-Lablink-Token"
H_SHA = "X-Lablink-Sha256"
H_META = "X-Lablink-Meta"
# The node_id whose token is being presented. Absent means the SITE token, so
# every existing client, script and QUICKSTART example keeps working unchanged.
H_NODE = "X-Lablink-Node"

# ---------------------------------------------------------------------------
# Limits
# ---------------------------------------------------------------------------
MAX_META_BYTES = 8192            # X-Lablink-Meta header, ASCII JSON
DEFAULT_MAX_FILE_MB = 512        # server refuses larger uploads (413)
DISK_FREE_MARGIN_MB = 256        # server refuses uploads that would leave less free (507)
CHUNK = 64 * 1024                # streaming chunk size, both directions
DEFAULT_TIMEOUT_S = 30           # socket timeout, server handler and client
DEFAULT_PORT = 8765

# ---------------------------------------------------------------------------
# Name rules — applied to BOTH channel names and file names.
#
# Single path segment; first char alphanumeric; then letters, digits, dot,
# underscore, space, hyphen; max 128 chars. Spaces are allowed because real
# instrument filenames are full of them (percent-encoding handles transport).
# Explicit extra rejections cover what the regex alone cannot express:
# trailing dot/space (Windows strips them silently) and Windows reserved
# device names.
# ---------------------------------------------------------------------------
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._ -]{0,127}$")

WINDOWS_RESERVED = (
    {"CON", "PRN", "AUX", "NUL"}
    | {f"COM{i}" for i in range(1, 10)}
    | {f"LPT{i}" for i in range(1, 10)}
)

# ---------------------------------------------------------------------------
# Sessions and node identity
#
# A session's data rides ORDINARY channels, named from its id, so uploads and
# downloads keep resume, checksum verification, deduplication and atomic
# visibility with no new durability code. The prefix is reserved: the polling
# service refuses to watch or reply to one, because it would otherwise find a
# session's own files and run a lab tool on them.
#
# Ids are 16 hex characters from secrets.token_hex(8) — short enough to read in a
# log line and type into a console, 64 bits so guessing one is not a route into
# another node's session.
# ---------------------------------------------------------------------------
SESSION_CH_PREFIX = "s-"
SESSION_ID_RE = re.compile(r"^[a-f0-9]{16}$")

# A node id is a public label: it appears in a header, in the console and in the
# ledger. Lower-case and hyphenated so it cannot collide by case on a
# case-insensitive filesystem, where the registry lives one file per node.
NODE_ID_RE = re.compile(r"^[a-z0-9][a-z0-9-]{0,31}$")

# A node-chosen idempotency key for a command, so a lost RESPONSE can be retried
# without the node being unable to tell "my command started" from "it didn't".
CMD_ID_RE = re.compile(r"^[A-Za-z0-9._-]{1,64}$")

# A node that leaks sessions must not be able to eat the hub. This is the right
# DEFAULT and the wrong RULE: a node that legitimately parallelises raises it in
# hub config, which is the operator's decision to make, not the node's.
MAX_SESSIONS_PER_NODE = 1

# Long-poll ceiling. 25 s is chosen to sit UNDER DEFAULT_TIMEOUT_S, which is
# simultaneously Handler.timeout and the default LabLinkClient timeout — so a
# wait of 30 s or more would trip both ends of the same request and a node would
# read a perfectly good long poll as "hub unreachable".
LONGPOLL_MAX_S = 25.0

# Concurrent long-poll waiters. A quarter of MAX_CONNECTIONS, because that budget
# is SHARED with uploads and artifact downloads: uncapped waiters would let idle
# nodes lock out the very transfers they are waiting for, and the symptom is a hub
# that answers every status request while never moving a byte. Over the cap a
# request answers immediately with the current state instead of failing, so the
# node's code path is identical either way and there is no second one to test.
MAX_WAITERS = 16

# Ledger entries returned in one status response, so a long-running session's
# history cannot make a single reply unbounded.
MAX_EVENTS_PER_RESPONSE = 200

REAPER_INTERVAL_S = 30.0

# How long a FAILED/LOST session stays addressable by id after it dies, before
# the reaper archives it. Long enough that a node's in-flight poll reads WHY it
# failed as a 200 with the reason; short enough that corpses do not linger in
# anyone's view. After this, the id answers 410 session_gone carrying the final
# state — the behaviour QUICKSTART's status table documents. (The first
# two-machine stress run found failed sessions retained FOREVER: counted as
# open in /hello, polluting the leak detector, and making the playbook's
# "no live sessions" gate unsatisfiable once the failure ladder had run.)
TERMINAL_LINGER_S = 120.0

# Finished session directories kept for inspection. The ledger outlives them.
KEEP_SESSION_DIRS = 20

# ---------------------------------------------------------------------------
# LWP/1 — the hub<->instance protocol
#
# Newline-delimited JSON over the worker's stdin/stdout. Not HTTP, and not a
# socket: no port to allocate, no auth to get wrong, no firewall rule, and the
# link's lifetime is the process's lifetime, so killing the process IS closing
# the connection.
#
#   hub -> worker   {"id": 7, "cmd": "run", "target": "meas"}
#   worker -> hub   {"seq": 88, "id": 7, "ev": "node", "state": "progress", ...}
#
# `id` is an integer chosen by the hub, monotonic from 1. An integer rather than
# a string because a JSON parser that gives every number back as a float — most of
# them outside Python — makes a worker comparing it to a string silently never
# match. `id: null` means unsolicited.
#
# stdout carries the control stream and NOTHING else. stderr is free-form and is
# captured to a ring buffer, which is the pressure valve that lets a worker use
# ordinary logging without corrupting the channel.
#
# Bulk data NEVER rides the pipe — only filesystem paths under the session dir.
# That keeps lines small, keeps hub memory flat regardless of dataset size, and
# keeps the worker contract implementable by anything that can read a file.
# ---------------------------------------------------------------------------
WORKER_PROTOCOL_VERSION = 1

# One line's ceiling. Over it the hub cannot know where the JSON ended, so the
# stream is no longer trustworthy: the worker is killed rather than resynced.
MAX_WORKER_LINE_BYTES = 1024 * 1024

# stderr kept per worker. Matches runner.MAX_CAPTURED_CHARS for the same reason:
# a chatty tool must not fill the disk or make a session record unreadable.
WORKER_STDERR_RING_CHARS = 64_000

# Tolerance for stdout lines that are not JSON objects. Scientific libraries print
# banners at import that cannot be suppressed — TensorFlow and CUDA are the usual
# culprits — so noise is counted and warned about, never fatal: a worker that works
# must not be killed for being chatty.
WORKER_NOISE_LINE_LIMIT = 1000
WORKER_NOISE_BYTE_LIMIT = 1024 * 1024

# Timeouts. Three of them, never conflated:
#   open     no `hello` at all — wrong interpreter, missing dependency, import storm
#   silence  no event OF ANY KIND while a command runs. THE load-bearing one: a
#            flat wall clock on a legitimately 40-minute segmentation batch is
#            wrong, while a silence timeout kills a wedged worker and never a
#            working one. This is why progress events are not cosmetic.
#   hard     absolute ceiling, off by default, for a recipe that must not overrun
WORKER_OPEN_TIMEOUT_S = 120.0
WORKER_SILENCE_TIMEOUT_S = 300.0
WORKER_HARD_TIMEOUT_S = 0.0          # 0 = no absolute ceiling
WORKER_IDLE_TIMEOUT_S = 1800.0       # warm instance with no commands

# The cancel ladder. Nothing in a compute engine is guaranteed interruptible, so
# cooperation is asked for first and enforced second.
WORKER_CANCEL_GRACE_S = 10.0         # cooperative cancel -> SIGTERM
WORKER_KILL_GRACE_S = 5.0            # SIGTERM -> SIGKILL

WORKER_COMMANDS = (
    "open", "set", "input", "run", "pull", "cancel", "reset", "ping", "close",
)

WORKER_EVENTS = (
    "hello", "node", "progress", "beat", "log", "artifact", "result", "error",
)

# The five states of a `node` event map 1:1 onto a compute engine's observer
# callback, so an adapter is one dict merge and nothing is renamed or lost.
NODE_STATES = ("start", "cached", "progress", "done", "error")

# A closed set, because operators read these. `worker_exited` is synthesised by
# the hub when a process dies without sending a terminal message.
# `missing_metadata` is distinct from `missing_input` on purpose, and the
# distinction is the client's remediation, not our taxonomy: `missing_input`
# means a named file is not there (re-upload it), while `missing_metadata` means
# the file IS there and valid but omits fields this recipe derives from — so the
# fix is to regenerate the sidecar with those fields, which a pusher can often do
# automatically by reading them back out of the source acquisition. Folding it
# into `missing_input` would leave a client string-matching the message to tell
# "upload something" from "add fields to what you uploaded". It is also a
# category error: the input is not missing, a field within it is.
WORKER_ERROR_CODES = frozenset({
    "unsupported", "protocol_error", "bad_request", "bad_recipe", "not_ready",
    "no_such_knob", "knob_out_of_range", "missing_input", "missing_metadata",
    "input_error", "missing_dependency", "compute_error", "cancelled",
    "resource_exhausted", "internal", "worker_exited", "silence_timeout",
    "hard_timeout", "open_timeout",
})
