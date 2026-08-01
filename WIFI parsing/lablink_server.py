#!/usr/bin/env python3
"""Start the LabLink file-exchange server.

Usage::

    python "WIFI parsing\\lablink_server.py" --token SECRET
    python lablink_server.py --root D:\\lablink_data --token SECRET --ttl-hours 168

Run with --help for all options. The startup banner prints the exact URL
clients should use and the firewall rule to allow the port.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lablink.server import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
