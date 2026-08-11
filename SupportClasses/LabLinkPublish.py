"""
LabLinkPublish.py — the one call a producer makes to offer an output to LabLink.

v7.17. Three functions, one contract:

    **Never raises. Never blocks. Returns immediately.**

A producer is in the middle of finishing a scan, a mosaic or a recording. None
of them can afford an exception or a wait, and one of them
(`CaptureController.stop_recording`) runs on the **GUI thread** — which is why
the service enqueues with `put_nowait` and counts drops rather than blocking.
Everything degrades to a no-op, the pattern `CaptureContext` already follows:
feature off, never configured, service never started, config unreadable — all
of them simply do nothing.

Producers import THIS, never `LabLinkService`: it keeps the heavy imports
(h5py, the vendored client) off every producer's import path, and it means a
test can exercise a producer without a service in sight.

⚠ **Per-artifact only, from code that knows what the artifact is.** Never a
directory watcher: `resolve_output_dir` honours an arbitrary `output_dir`, so
a watcher pointed at a data drive would upload a year of unrelated experiments
to a shared channel over a link with no TLS.
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


def _offer(source: str, **fields) -> None:
    """Hand one output to the service, if there is one and it wants it."""
    try:
        from SupportClasses.LabLinkService import peek_service, JobSpec
        service = peek_service()
        if service is None:
            return                      # never turned on: cost is one import
        service.submit(JobSpec(source=source, **fields))
    except Exception:
        # A publish must never be able to damage the thing that produced the
        # output. Logged at debug: an operator with LabLink switched off
        # should not see warnings about it on every capture.
        logger.debug("LabLink: publish of a %s output failed", source,
                     exc_info=True)


def publish_capture(path: str, note: str = "", *, kind: str = "still") -> None:
    """A still or a recording that has just been written to disk.

    ``kind`` is ``"still"`` or ``"video"`` — they are separate sources so the
    operator can send images without sending every recording, which matters
    when a recording is far larger than the hub's per-file limit.
    """
    if not path:
        return
    _offer("video" if kind == "video" else "still", path=str(path), note=note)


def publish_fluorescence_well(plate_key: str, well: str) -> None:
    """A completed multi-channel well scan.

    ⚠ Called once per RUN, from `_finish_run` — never per channel. One `.nd3`
    carrying every channel is the bundle the operator asked for; publishing
    from `_on_channel_finished` would send N partial files instead.
    """
    if not plate_key or not well:
        return
    _offer("fluorescence_well", plate_key=str(plate_key), well=str(well))


def publish_plate_mosaic(plate_key: str) -> None:
    """A completed full-plate mosaic (the active scan)."""
    if not plate_key:
        return
    _offer("plate_mosaic", plate_key=str(plate_key))
