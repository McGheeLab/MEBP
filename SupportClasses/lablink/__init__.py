"""LabLink client — VENDORED, stdlib-only. See VENDORED.md.

MEBP is a LabLink *node*: it opens a session, uploads an image plus its
``.job.json`` sidecar, runs a hub-declared recipe and collects the results. It
is never a hub and never a worker.

⚠ THIS FILE IS THE ONE EXCEPTION TO BYTE-IDENTICAL VENDORING, deliberately.
Upstream's ``__init__`` re-exports ``.recipes`` (recipe *authoring* — for
software that generates a hub manifest). MEBP does not author recipes: per
``docs/RECIPE-AUTHORING.md`` installing one is an offline act with a person in
the loop, not a wire operation. Vendoring ``recipes.py`` to satisfy an import
nothing here calls would widen the surface for nothing, so this module carries
only what MEBP uses. ``tests/test_v717_lablink_vendor.py`` pins the other four
files byte-for-byte and asserts this one does not import the modules that were
left out.

Nothing else from upstream may be copied in — in particular NOT ``runner.py``
or ``config.py``, which are the subprocess executor and the local declarations
of the commands it runs. Those belong on the machine that owns the analysis
tool, never inside an application that drives a needle over a glass plate.
"""

from .protocol import PROTOCOL_VERSION
from .client import LabLinkClient, LabLinkError
from .session_client import (
    CommandFailed,
    CommandResult,
    HubSession,
    InputRef,
    Progress,
    SessionClient,
    SessionError,
    SessionLost,
    TERMINAL_STATES,
)

#: Upstream commit these files were taken from; asserted by the vendor test.
UPSTREAM_COMMIT = "7123061c81128be3bec9d7ec72d6e84c80f9438a"

__all__ = [
    "PROTOCOL_VERSION", "UPSTREAM_COMMIT",
    "LabLinkClient", "LabLinkError",
    "SessionClient", "HubSession", "CommandResult", "CommandFailed",
    "SessionError", "SessionLost", "InputRef", "Progress", "TERMINAL_STATES",
]
