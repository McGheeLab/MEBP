"""
test_v715_settings_split.py — image settings and video settings are separate.

Operator: *"the settings for the snapshot should be separate from the video
settings, it's confusing when they are the same."*

The KEYS were already separate; the presentation was not, and it was shared in
ways that actively misled — one preview rendering both names with merged
unknown-token warnings, a video-only problem shown as a warning under
*Images*, and a "Metadata" box that governed only stills.

⚠ The load-bearing test here is ``test_editing_one_kind_preserves_the_other``.
``Settings.set_section`` REPLACES a whole section, so a dialog that wrote back
only its own keys would silently delete the other dialog's settings on OK.
"""

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.CaptureSpec import (
    CAPTURE_DEFAULTS, merged_settings, still_extension, validate,
    validate_still, validate_video, video_extension)
from gui.dialogs.capture_settings_dialog import (
    ImageCaptureSettingsDialog, VideoRecordingSettingsDialog)


class _Settings:
    """Minimal stand-in with the real replace-the-whole-section semantics."""

    def __init__(self, section=None):
        self._data = {"capture": dict(section or {})}
        self.saves = 0

    def get_section(self, name):
        return dict(self._data.get(name, {}))

    def set_section(self, name, data):
        self._data[name] = dict(data)      # REPLACES, as the real one does

    def save(self):
        self.saves += 1


class _Mgr:
    def get_hw_settings(self, i):
        return {"resolution": (640, 480)}


class TestSeparation(unittest.TestCase):

    def test_each_dialog_owns_one_template(self):
        img = ImageCaptureSettingsDialog(_Settings(), _Mgr(), 0)
        vid = VideoRecordingSettingsDialog(_Settings(), _Mgr(), 0)
        self.assertEqual(img.TEMPLATE_KEY, "still_template")
        self.assertEqual(vid.TEMPLATE_KEY, "video_template")

    def test_preview_shows_only_its_own_kind(self):
        """The shared preview printed both names and merged their warnings."""
        img = ImageCaptureSettingsDialog(_Settings(), _Mgr(), 0)
        img._tpl.setText("shot_{camera}")
        vid = VideoRecordingSettingsDialog(_Settings(), _Mgr(), 0)
        vid._tpl.setText("clip_{camera}")
        self.assertIn("shot_", img._preview.text())
        self.assertNotIn("clip_", img._preview.text())
        self.assertIn("clip_", vid._preview.text())
        self.assertNotIn("shot_", vid._preview.text())

    def test_preview_shows_the_right_extension(self):
        img = ImageCaptureSettingsDialog(_Settings(), _Mgr(), 0)
        self.assertIn(".png", img._preview.text())
        vid = VideoRecordingSettingsDialog(_Settings(), _Mgr(), 0)
        self.assertIn(".mp4", vid._preview.text())

    def test_unknown_token_is_attributed_to_the_right_template(self):
        img = ImageCaptureSettingsDialog(_Settings(), _Mgr(), 0)
        img._tpl.setText("{nonsense}")
        self.assertIn("nonsense", img._preview.text())
        vid = VideoRecordingSettingsDialog(_Settings(), _Mgr(), 0)
        self.assertNotIn("nonsense", vid._preview.text())

    def test_a_video_problem_is_not_shown_on_the_image_dialog(self):
        """It used to render under the *Images* group box."""
        st = _Settings({"video_fps": 0})
        img = ImageCaptureSettingsDialog(st, _Mgr(), 0)
        self.assertNotIn("frame rate", img._note.text().lower())
        vid = VideoRecordingSettingsDialog(st, _Mgr(), 0)
        self.assertIn("frame rate", vid._note.text().lower())

    def test_an_image_problem_is_not_shown_on_the_video_dialog(self):
        """Note: the image dialog PREVENTS raw+PNG (it forces TIFF on
        selection), so the problem is checked through the validator each
        dialog is wired to rather than through a state the UI won't enter."""
        bad_still = {"still_source": "raw", "still_format": "png",
                     "video_fps": 15}
        vid = VideoRecordingSettingsDialog(_Settings(), _Mgr(), 0)
        img = ImageCaptureSettingsDialog(_Settings(), _Mgr(), 0)
        self.assertEqual(vid._validate(bad_still), [])
        self.assertTrue(img._validate(bad_still))
        self.assertIn("TIFF", img._validate(bad_still)[0])
        # …and the video dialog's note is about video, never about formats.
        self.assertIn("per minute", vid._note.text())

    def test_metadata_controls_live_with_the_stills(self):
        """embed_metadata / write_sidecar are read on the still path."""
        img = ImageCaptureSettingsDialog(_Settings(), _Mgr(), 0)
        self.assertTrue(hasattr(img, "_embed"))
        vid = VideoRecordingSettingsDialog(_Settings(), _Mgr(), 0)
        self.assertFalse(hasattr(vid, "_embed"))


class TestPersistence(unittest.TestCase):

    def test_editing_one_kind_preserves_the_other(self):
        """⭐ set_section REPLACES. A partial write would delete the video
        settings the moment the image dialog was OK'd."""
        st = _Settings({"video_template": "MYVIDEO", "video_fps": 7.5,
                        "video_max_seconds": 42, "still_template": "MYSTILL"})
        img = ImageCaptureSettingsDialog(st, _Mgr(), 0)
        img._tpl.setText("changed_still")
        img.accept()
        after = st.get_section("capture")
        self.assertEqual(after["still_template"], "changed_still")
        self.assertEqual(after["video_template"], "MYVIDEO")
        self.assertEqual(after["video_fps"], 7.5)
        self.assertEqual(after["video_max_seconds"], 42)

    def test_editing_video_preserves_the_stills(self):
        st = _Settings({"still_template": "MYSTILL", "still_format": "tiff",
                        "still_raw_avg_frames": 9})
        vid = VideoRecordingSettingsDialog(st, _Mgr(), 0)
        vid._fps.setValue(24.0)
        vid.accept()
        after = st.get_section("capture")
        self.assertEqual(after["video_fps"], 24.0)
        self.assertEqual(after["still_template"], "MYSTILL")
        self.assertEqual(after["still_format"], "tiff")
        self.assertEqual(after["still_raw_avg_frames"], 9)

    def test_cancel_persists_nothing(self):
        st = _Settings({"still_template": "ORIGINAL"})
        img = ImageCaptureSettingsDialog(st, _Mgr(), 0)
        img._tpl.setText("edited")
        img.reject()
        self.assertEqual(st.saves, 0)
        self.assertEqual(st.get_section("capture")["still_template"],
                         "ORIGINAL")

    def test_video_max_gb_is_no_longer_discarded(self):
        """It was hard-coded to the default in values(), so a stored value was
        reset on every OK — and nothing enforced it either."""
        st = _Settings({"video_max_gb": 2.5})
        vid = VideoRecordingSettingsDialog(st, _Mgr(), 0)
        self.assertEqual(vid._max_gb.value(), 2.5)
        vid.accept()
        self.assertEqual(st.get_section("capture")["video_max_gb"], 2.5)

    def test_full_round_trip_of_every_key(self):
        """set_values(x) then values() must reproduce x for the keys each
        dialog owns — the old suite asserted only 4 of 19."""
        custom = dict(CAPTURE_DEFAULTS)
        custom.update({
            "output_dir": "/tmp/shots", "subfolder_by_date": False,
            "still_template": "a_{camera}", "still_source": "raw",
            "still_format": "tiff", "still_raw_avg_frames": 8,
            "still_full_res": True, "embed_metadata": False,
            "write_sidecar": False, "operator": "alex", "notes": "n",
            "video_template": "b_{well}", "video_source": "raw_timelapse",
            "video_fps": 7.0, "video_quality": 55, "video_container": "avi",
            "video_max_seconds": 33, "video_max_gb": 1.5,
            "raw_timelapse_interval_s": 2.5,
        })
        img = ImageCaptureSettingsDialog(_Settings(custom), _Mgr(), 0)
        got = img.values()
        for k in ("output_dir", "subfolder_by_date", "still_template",
                  "still_source", "still_format", "still_raw_avg_frames",
                  "still_full_res", "embed_metadata", "write_sidecar",
                  "operator", "notes"):
            self.assertEqual(got[k], custom[k], f"image dialog lost {k}")

        vid = VideoRecordingSettingsDialog(_Settings(custom), _Mgr(), 0)
        got = vid.values()
        for k in ("output_dir", "subfolder_by_date", "video_template",
                  "video_source", "video_fps", "video_quality",
                  "video_container", "video_max_seconds", "video_max_gb",
                  "raw_timelapse_interval_s"):
            self.assertEqual(got[k], custom[k], f"video dialog lost {k}")

    def test_raw_still_still_forces_tiff(self):
        img = ImageCaptureSettingsDialog(_Settings(), _Mgr(), 0)
        idx = img._src.findData("raw")
        img._src.setCurrentIndex(idx)
        self.assertEqual(img.values()["still_format"], "tiff")
        self.assertFalse(img._fmt.isEnabled())


class TestSpecHelpers(unittest.TestCase):

    def test_video_extension_follows_the_container(self):
        self.assertEqual(video_extension({"video_container": "avi"}), ".avi")
        self.assertEqual(video_extension({}), ".mp4")

    def test_still_extension_unchanged(self):
        self.assertEqual(still_extension({"still_format": "tiff"}), ".tif")
        self.assertEqual(still_extension({}), ".png")

    def test_combined_validate_is_the_union(self):
        cfg = {"still_source": "raw", "still_format": "png", "video_fps": 0}
        self.assertEqual(sorted(validate(cfg)),
                         sorted(validate_still(cfg) + validate_video(cfg)))
        self.assertEqual(len(validate(cfg)), 2)


class TestButtonWiring(unittest.TestCase):
    """Each button must open ITS OWN dialog — the whole point of the split."""

    def test_open_capture_settings_selects_by_kind(self):
        import ast
        import inspect
        from gui.widgets import camera_feed_view as cfv
        src = inspect.getsource(cfv.CameraFeedView.open_capture_settings)
        self.assertIn("VideoRecordingSettingsDialog", src)
        self.assertIn("ImageCaptureSettingsDialog", src)
        # and the two right-click handlers must pass different kinds
        full = inspect.getsource(cfv)
        self.assertIn('open_capture_settings("still")', full)
        self.assertIn('open_capture_settings("video")', full)
        ast.parse(full)


if __name__ == "__main__":
    unittest.main()
