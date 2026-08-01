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
# underscore, space, hyphen; max 128 chars. Spaces are allowed because
# MATLAB/ImageJ users produce them (percent-encoding handles transport).
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
