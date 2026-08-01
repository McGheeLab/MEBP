"""Test package bootstrap.

The project folder ("WIFI parsing") contains a space, so it is never on
sys.path by accident; add it explicitly so `import lablink` works whether
tests are run from the repo root or from inside the folder:

    cd "WIFI parsing" && python -m unittest
    python -m unittest discover -s "WIFI parsing/tests" -t "WIFI parsing"
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PROJECT = os.path.dirname(_HERE)
if _PROJECT not in sys.path:
    sys.path.insert(0, _PROJECT)
