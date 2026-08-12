"""
tools — standalone bring-up / diagnostic utilities.

Packages under ``tools/`` are NOT part of the MEBP application. Each one is
a self-contained utility run directly from the repo root, e.g.::

    python tools/incubator/run.py

They may import hardware-agnostic helpers from ``SupportClasses`` and the
theme modules from ``gui``, but nothing in the main app imports them.
"""

__all__ = []
