"""store_fixture.py — leak-proof temporary PrintTimingCalibrationStore for tests.

``PrintTimingCalibrationStore`` is reached through a process-wide singleton
(``get_store()`` → module global ``_store``). A test that swaps that global and
forgets to restore it leaks a temp-dir store into *every* test that runs after it
in the same process — which is exactly what happened in
``test_v75x_xy_challenge.py`` (it restored ``TCS._STORE``, a name that does not
exist) and ``test_v75x_xy_challenge_panel.py`` (it never restored at all).

Use one of these instead; both restore the previous global unconditionally.

    class T(unittest.TestCase):
        def setUp(self):
            self.store = use_temp_store(self)      # auto-restored via addCleanup

    # or, scoped:
    with temp_store() as store:
        ...
"""

from __future__ import annotations

import os
import tempfile
from contextlib import contextmanager

from SupportClasses import PrintTimingCalibrationStore as TCS


def make_temp_store(path: str | None = None):
    """A fresh store backed by a throwaway file (does NOT touch the global)."""
    if path is None:
        path = os.path.join(tempfile.mkdtemp(), "tc.json")
    return TCS.PrintTimingCalibrationStore(path)


@contextmanager
def temp_store(path: str | None = None):
    """Swap ``get_store()`` for a throwaway store, restoring on exit."""
    prev = TCS._store
    store = make_temp_store(path)
    TCS._store = store
    try:
        yield store
    finally:
        TCS._store = prev


def use_temp_store(testcase, path: str | None = None):
    """Swap in a throwaway store for the duration of ``testcase``.

    Registers an ``addCleanup`` so the previous global is restored even if the
    test errors — the property the two hand-rolled versions lacked.
    """
    prev = TCS._store
    store = make_temp_store(path)
    TCS._store = store

    def _restore():
        TCS._store = prev

    testcase.addCleanup(_restore)
    return store
