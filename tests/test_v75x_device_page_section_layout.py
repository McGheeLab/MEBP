"""
test_v75x_device_page_section_layout.py

v7.5.x — Hardware Setup → Device (Stage) sections are collapsible and
user-reorderable, with the order + collapsed state persisted per machine.

Covers:
  * ReorderableSectionList: default order, move up/down (incl. no-op at the
    ends), collapse state, and set_order tolerance (unknown keys ignored,
    missing keys kept so a stale layout never drops a section).
  * StageHardwarePanel: the Device page wires all config sections through the
    list, and the order + collapsed state round-trip through Settings.
"""

import os
import json
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QLabel

_app = QApplication.instance() or QApplication([])

from gui.widgets.reorderable_sections import ReorderableSectionList
from gui.pages.hardware.stage_panel import StageHardwarePanel
from SupportClasses.Settings import Settings


class TestReorderableSectionList(unittest.TestCase):
    def _list(self):
        lst = ReorderableSectionList()
        for k in ("a", "b", "c"):
            lst.add_section(k, k.upper(), QLabel(k))
        return lst

    def test_default_order_is_insertion_order(self):
        self.assertEqual(self._list().order(), ["a", "b", "c"])

    def test_move_down_and_up(self):
        lst = self._list()
        moves = []
        lst.order_changed.connect(lambda o: moves.append(list(o)))
        lst._on_move("a", +1)
        self.assertEqual(lst.order(), ["b", "a", "c"])
        lst._on_move("c", -1)
        self.assertEqual(lst.order(), ["b", "c", "a"])
        self.assertEqual(len(moves), 2)  # emitted once per real move

    def test_move_at_ends_is_noop(self):
        lst = self._list()
        emitted = []
        lst.order_changed.connect(lambda o: emitted.append(o))
        lst._on_move("a", -1)   # already first
        lst._on_move("c", +1)   # already last
        self.assertEqual(lst.order(), ["a", "b", "c"])
        self.assertEqual(emitted, [])  # nothing changed, nothing emitted

    def test_collapse_state(self):
        lst = self._list()
        changed = []
        lst.collapsed_changed.connect(lambda k, v: changed.append((k, v)))
        lst._sections["b"].set_collapsed(True, emit=True)
        self.assertEqual(lst.collapsed_states(), {"a": False, "b": True, "c": False})
        self.assertEqual(changed, [("b", True)])

    def test_set_order_ignores_unknown_keeps_missing(self):
        lst = self._list()
        # 'x' is unknown → ignored; 'b' omitted → kept at the end, never dropped
        lst.set_order(["c", "x", "a"])
        self.assertEqual(lst.order(), ["c", "a", "b"])

    def test_apply_collapsed_states_tolerates_unknown(self):
        lst = self._list()
        lst.apply_collapsed_states({"a": True, "ghost": True})
        self.assertTrue(lst._sections["a"].is_collapsed())
        self.assertFalse(lst._sections["c"].is_collapsed())


class TestDevicePagePersistence(unittest.TestCase):
    def setUp(self):
        fd, self.path = tempfile.mkstemp(suffix=".json")
        os.close(fd)
        with open(self.path, "w") as f:
            f.write("{}")

    def tearDown(self):
        try:
            os.remove(self.path)
        except OSError:
            pass

    def _settings(self):
        s = Settings(self.path)
        s.load()
        return s

    def test_panel_wires_all_config_sections(self):
        panel = StageHardwarePanel()
        self.assertEqual(
            panel._section_list.order(),
            ["device_profile", "axis_mapping", "xy_cal", "steps_cal",
             "jog_safety", "override_pos", "zp_feedrates", "recal_reminders"],
        )

    def test_order_and_collapse_round_trip(self):
        s1 = self._settings()
        p1 = StageHardwarePanel()
        p1.set_settings(s1)
        p1._section_list._on_move("recal_reminders", -1)  # persists order
        p1._section_list._sections["zp_feedrates"].set_collapsed(
            True, emit=True)                                # persists collapsed

        on_disk = json.load(open(self.path))["device_page_layout"]
        self.assertEqual(on_disk["order"][-2:], ["recal_reminders", "zp_feedrates"])
        self.assertTrue(on_disk["collapsed"]["zp_feedrates"])

        # Fresh session restores both.
        s2 = self._settings()
        p2 = StageHardwarePanel()
        p2.set_settings(s2)
        self.assertEqual(p2._section_list.order(), on_disk["order"])
        self.assertTrue(p2._section_list._sections["zp_feedrates"].is_collapsed())

    def test_missing_layout_leaves_defaults(self):
        p = StageHardwarePanel()
        p.set_settings(self._settings())  # empty file, no device_page_layout
        self.assertEqual(p._section_list.order()[0], "device_profile")


if __name__ == "__main__":
    unittest.main()
