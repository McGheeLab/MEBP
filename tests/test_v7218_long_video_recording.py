"""
v7.21.8 — video recordings are no longer capped at 10 minutes.

Operator: *"For the video captures, I would for them to be longer than 10
minutes."*

`CAPTURE_DEFAULTS["video_max_seconds"]` was **600**, and the encoder loop stops
the recording the moment `elapsed_s` crosses it. The value was already
operator-editable (0–86400 s, 0 = no limit) and already persisted — but this rig
has no saved `capture` section, so every recording ran on the default and
stopped ten minutes in.

The time cap never protected anything the SIZE cap does not protect better: a
runaway recording is a disk problem, and bytes fill a disk, minutes do not. So
the default becomes 0 (no limit) and `video_max_gb` is left as the guard.

Removing the time cap is also what makes a multi-GB file REACHABLE, which turns
one latent container problem into a live one: AVI is RIFF, whose offsets are
32-bit. `container_byte_limit` therefore clamps AVI's effective size cap, so a
long AVI stops cleanly at a size the container can still describe rather than
running on into a file that looks finished and will not play.

Pinned here:
  1. the default really is "no limit", and 0 really means unlimited end-to-end;
  2. a stored value still wins (the operator can put a limit back);
  3. `container_byte_limit` only ever LOWERS the operator's number, and only
     for AVI;
  4. the clamp is actually applied by the encoder, and NOT to a raw time-lapse
     (a directory of TIFFs is not a RIFF container);
  5. the dialog reports which limit binds FIRST — quoting a limit that is not
     the one that will stop the recording is worse than quoting none.
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import unittest

from SupportClasses.CaptureSpec import (
    CAPTURE_DEFAULTS, merged_settings, estimated_video_mb_per_min,
)
from SupportClasses.CaptureVideoWriter import (
    CONTAINER_MAX_BYTES, container_byte_limit,
)

# The v7.15 suite already owns a recording harness whose camera is a REAL
# QObject emitting a REAL QImage (its own docstring records why a stub proved
# nothing there). Reused rather than re-stubbed, so these tests run the same
# ingress the production path does.
from tests.test_v715_recording import _Base as _RecBase       # noqa: E402

GB = 1024 ** 3


# ══════════════════════════════════════════════════════════════════════
#  1. The default
# ══════════════════════════════════════════════════════════════════════

class TestNoTimeCapByDefault(unittest.TestCase):

    def test_default_is_unlimited(self):
        """The whole request, in one assertion."""
        self.assertEqual(CAPTURE_DEFAULTS["video_max_seconds"], 0)

    def test_the_old_ten_minute_default_is_gone(self):
        """Guard-the-guard: 600 is the specific value the operator hit, so name
        it — a future 'tidy-up' that restores any non-zero default should fail
        loudly rather than quietly reinstating a surprise stop."""
        self.assertNotEqual(CAPTURE_DEFAULTS["video_max_seconds"], 600)

    def test_a_fresh_install_records_past_ten_minutes(self):
        """End-to-end on the value the encoder actually reads: an unset install
        must not stop at 600 s, or 3600 s, or at all."""
        cfg = merged_settings(None)
        max_s = float(cfg.get("video_max_seconds", 0) or 0)
        self.assertEqual(max_s, 0.0)
        # `if max_s and elapsed >= max_s` — falsy ⇒ the check never fires.
        self.assertFalse(max_s, "0 must be falsy so the duration check is skipped")

    def test_a_size_guard_is_still_in_place(self):
        """"No time limit" must not mean "nothing stops it". If someone ever
        zeroes the size default too, a forgotten recording fills the disk."""
        self.assertGreater(CAPTURE_DEFAULTS["video_max_gb"], 0.0)

    def test_a_stored_limit_still_wins(self):
        """The operator can put a limit back — the default changing must not
        make the setting inert."""
        cfg = merged_settings({"video_max_seconds": 300})
        self.assertEqual(cfg["video_max_seconds"], 300)

    def test_a_stored_zero_survives_the_round_trip(self):
        """0 is a real choice, and a falsy-value bug would silently restore a
        default. (`merged_settings` skips only None, not 0 — pinned.)"""
        self.assertEqual(
            merged_settings({"video_max_seconds": 0})["video_max_seconds"], 0)


# ══════════════════════════════════════════════════════════════════════
#  2. The container ceiling that replaces it
# ══════════════════════════════════════════════════════════════════════

class TestContainerByteLimit(unittest.TestCase):

    def test_mp4_gets_exactly_what_was_asked_for(self):
        self.assertEqual(container_byte_limit("mp4", 8 * GB), 8 * GB)

    def test_mp4_unlimited_stays_unlimited(self):
        self.assertEqual(container_byte_limit("mp4", 0), 0)

    def test_avi_is_clamped_down(self):
        self.assertEqual(container_byte_limit("avi", 8 * GB), 2 * GB)

    def test_avi_unlimited_still_gets_the_structural_ceiling(self):
        """The case that matters now: with no time cap AND no size cap, an AVI
        would otherwise run past what RIFF can index."""
        self.assertEqual(container_byte_limit("avi", 0), 2 * GB)

    def test_it_only_ever_lowers_the_operators_value(self):
        """It is a floor on trust, not a preference — a smaller configured cap
        must be honoured, not raised to the structural one."""
        self.assertEqual(container_byte_limit("avi", 1 * GB), 1 * GB)

    def test_unknown_container_is_left_alone(self):
        """Absent knowledge ⇒ do not invent a limit."""
        self.assertEqual(container_byte_limit("mkv", 8 * GB), 8 * GB)
        self.assertEqual(container_byte_limit("", 8 * GB), 8 * GB)
        self.assertEqual(container_byte_limit(None, 8 * GB), 8 * GB)

    def test_case_and_whitespace_tolerant(self):
        """The value comes from a combo's userData and could be stored in any
        case; a missed match silently disarms the guard."""
        for spelling in ("AVI", " avi ", "Avi"):
            with self.subTest(spelling=spelling):
                self.assertEqual(container_byte_limit(spelling, 0), 2 * GB)

    def test_the_table_is_the_single_source(self):
        self.assertEqual(CONTAINER_MAX_BYTES["avi"], 2 * GB)
        self.assertEqual(CONTAINER_MAX_BYTES["mp4"], 0)


# ══════════════════════════════════════════════════════════════════════
#  3. The encoder applies it
# ══════════════════════════════════════════════════════════════════════

class TestEncoderAppliesTheClamp(_RecBase):
    """⭐ Drives the REAL encoder loop.

    An earlier version of this class re-implemented the loop's `max_b`
    resolution and checked THAT — and a mutation run proved it worthless: with
    the loop's `if not self._raw:` turned into `if False:` (i.e. the clamp
    removed from production entirely) every assertion still passed, because they
    were testing a copy. An AST check that the helper is *called somewhere* in
    the module was no better — the call node survives inside a dead branch.

    So these start a real recording and report a byte count from the fake, then
    assert on whether the session actually stops. The configured cap (100 GB) is
    far above the reported size, so the ONLY thing that can stop an AVI here is
    the container clamp.
    """

    REPORTED = 2.5 * GB          # over the 2 GB AVI ceiling, under 100 GB

    def _run(self, **over):
        cfg = {"video_max_seconds": 0, "video_max_gb": 100.0}
        cfg.update(over)
        c = self._ctrl(**cfg)
        self.assertTrue(c.start_recording(), "recording did not start")
        rec = c._rec
        self._pump(2)
        rec._bytes_written = lambda: self.REPORTED
        self._pump(6)
        stopped = self._wait(lambda: not rec.active, timeout=3.0)
        return rec, stopped

    def test_an_avi_stops_at_the_container_ceiling(self):
        rec, stopped = self._run(video_container="avi")
        self.assertTrue(
            stopped,
            "an AVI ran past the 2 GB RIFF ceiling — the clamp is not applied "
            "in the encoder loop")
        self.assertIn("2.0 GB", rec._reason)

    def test_an_mp4_is_NOT_stopped_by_it(self):
        """Guard-the-guard: if this also stopped, the test above would pass for
        the wrong reason (something else ending every session)."""
        rec, stopped = self._run(video_container="mp4")
        self.assertFalse(
            stopped,
            f"an mp4 stopped at {self.REPORTED / GB:.1f} GB with a 100 GB cap "
            f"({rec._reason!r})")

    def test_a_raw_timelapse_is_not_a_container(self):
        """A raw time-lapse is a DIRECTORY of TIFFs — no RIFF index — so the AVI
        ceiling must not apply even when `video_container` still says avi from a
        previous session."""
        rec, stopped = self._run(video_container="avi",
                                 video_source="raw_timelapse")
        self.assertFalse(
            stopped,
            f"a raw time-lapse was stopped by the AVI container ceiling "
            f"({rec._reason!r})")


# ══════════════════════════════════════════════════════════════════════
#  4. The dialog names the limit that will actually bind
# ══════════════════════════════════════════════════════════════════════

class TestRuntimeNote(unittest.TestCase):

    @staticmethod
    def _note(cfg_over, mb_per_min, raw=False):
        from gui.dialogs.capture_settings_dialog import (
            VideoRecordingSettingsDialog as V)
        return V._runtime_note(merged_settings(cfg_over), mb_per_min, raw)

    def test_no_limits_at_all_says_so(self):
        n = self._note({"video_max_seconds": 0, "video_max_gb": 0.0}, 40.0)
        self.assertIn("No automatic stop", n)

    def test_size_limit_is_reported_in_time(self):
        """8 GB at 40 MB/min ≈ 205 min ≈ 3.4 h — the operator's real question is
        "how long", so the answer is given in hours, not gigabytes."""
        n = self._note({"video_max_seconds": 0, "video_max_gb": 8.0}, 40.0)
        self.assertIn("3.4 h", n)
        self.assertIn("8.0 GB limit", n)

    def test_the_binding_limit_wins_when_time_is_shorter(self):
        n = self._note({"video_max_seconds": 600, "video_max_gb": 8.0}, 40.0)
        self.assertIn("10 min", n)
        self.assertIn("time limit", n)

    def test_the_binding_limit_wins_when_size_is_shorter(self):
        """A raw time-lapse at 609 MB/min hits 8 GB in ~13 min — far sooner than
        a generous time limit, and quoting the time limit there would be a
        promise the recording cannot keep."""
        n = self._note({"video_max_seconds": 7200, "video_max_gb": 8.0}, 609.0,
                       raw=True)
        self.assertIn("13 min", n)
        self.assertIn("GB limit", n)

    def test_an_avi_reports_the_clamped_number_not_the_configured_one(self):
        """The number shown must be the number that will happen. At 40 MB/min,
        8 GB would read as 3.4 h — but an AVI stops at 2 GB, i.e. ~51 min."""
        n = self._note({"video_max_seconds": 0, "video_max_gb": 8.0,
                        "video_container": "avi"}, 40.0)
        self.assertIn("51 min", n)
        self.assertIn("AVI 2 GB", n)
        self.assertNotIn("3.4 h", n)

    def test_it_survives_a_zero_rate(self):
        """`_camera_size` can report a placeholder before a camera is running;
        a divide-by-zero in a tooltip refresher would break the whole dialog."""
        n = self._note({"video_max_seconds": 0, "video_max_gb": 8.0}, 0.0)
        self.assertIsInstance(n, str)
        self.assertTrue(n)


# ══════════════════════════════════════════════════════════════════════
#  5. Sanity: what the new default is worth on this rig's cameras
# ══════════════════════════════════════════════════════════════════════

class TestRealisticRuntimes(unittest.TestCase):
    """Not a contract — a guard that the shipped defaults leave a genuinely
    long recording available on the cameras this rig actually has. If a future
    change to the estimator or the size default silently drops an ordinary
    recording back under an hour, this fails and says so."""

    def test_encoded_video_runs_for_hours_on_every_fitted_camera(self):
        cap_gb = CAPTURE_DEFAULTS["video_max_gb"]
        for name, (w, h) in {
            "Tucsen Libra 25 (2600×2048)": (2600, 2048),
            "Andor Zyla (2048×2048)": (2048, 2048),
            "Teslong needle cam (1280×720)": (1280, 720),
        }.items():
            with self.subTest(camera=name):
                mb = estimated_video_mb_per_min(w, h, 15.0, "display", 80)
                minutes = cap_gb * 1024 / mb
                self.assertGreater(
                    minutes, 60,
                    f"{name}: only {minutes:.0f} min at the default size cap")


if __name__ == "__main__":
    unittest.main()
