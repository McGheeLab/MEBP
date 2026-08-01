#!/usr/bin/env python3
"""Run the LabLink folder-sync agent: outbox -> channel, channel -> inbox.

Usage::

    python lablink_sync.py --make-config C:\\lablink\\sync.json
    #   ...edit url, token and folder paths...
    python lablink_sync.py --config C:\\lablink\\sync.json
    python lablink_sync.py --config C:\\lablink\\sync.json --once

Drop a file in the outbox folder and it is uploaded once it stops changing;
anything new in the inbox channel is downloaded into the inbox folder.
Start and stop it whenever you like — nothing is lost or duplicated.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lablink.sync import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
