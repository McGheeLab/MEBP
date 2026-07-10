"""
v7.5.x — Well-type presets for rosette wells.

A ``WellType`` is a physical vessel geometry preset (e.g. a 0.1 mL PCR tube):
single opening diameter + depth + rim height (how far the tube top sits above
the plate — the value that drives needle travel clearance) + optional ink Z.
Presets are stamped onto a rosette sub-well from a picker in the plate designer.

Covers:
  * WellType         — to_dict/from_dict round-trip, type coercion, label.
  * WellTypeStore    — built-in load, user save_user shadow, delete_user, get/all.
  * Shipped built-in — the real config/hardware/well_types/builtin/pcr-tube-0.1ml.
  * PlateDesign.Well — well_type_id serialization round-trip; None = key absent
                       (byte-identical legacy output).
  * PlateDesignerWidget — well-type picker present on a rosette sub-well card;
                       _apply_well_type stamps geometry / "(custom)" clears it.
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QComboBox  # noqa: E402

_app = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.WellTypeStore import (  # noqa: E402
    WellType, WellTypeStore, safe_id,
)
from SupportClasses.PlateDesign import (  # noqa: E402
    PlateDesign, Well, _entity_to_dict, _entity_from_dict,
)

_BUILTIN_PCR_ID = "pcr-tube-0.1ml"


def _write_type(directory: Path, **kw) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    data = {
        "id": kw["id"],
        "display_name": kw.get("display_name", kw["id"]),
        "diameter_mm": kw.get("diameter_mm", 6.0),
        "well_depth_mm": kw.get("well_depth_mm", 20.0),
        "rim_height_mm": kw.get("rim_height_mm", 20.0),
        "ink_z_mm": kw.get("ink_z_mm"),
        "volume_uL": kw.get("volume_uL"),
        "builtin": kw.get("builtin", True),
    }
    with open(directory / f"{kw['id']}.json", "w") as f:
        json.dump(data, f)


class TestWellTypeDataclass(unittest.TestCase):
    def test_round_trip(self):
        wt = WellType(
            id="tube-x", display_name="Tube X", diameter_mm=5.0,
            well_depth_mm=12.0, rim_height_mm=8.0, ink_z_mm=-3.0,
            volume_uL=50.0, builtin=True)
        wt2 = WellType.from_dict(wt.to_dict())
        self.assertEqual(wt2.id, "tube-x")
        self.assertEqual(wt2.display_name, "Tube X")
        self.assertAlmostEqual(wt2.diameter_mm, 5.0)
        self.assertAlmostEqual(wt2.well_depth_mm, 12.0)
        self.assertAlmostEqual(wt2.rim_height_mm, 8.0)
        self.assertAlmostEqual(wt2.ink_z_mm, -3.0)
        self.assertAlmostEqual(wt2.volume_uL, 50.0)
        self.assertTrue(wt2.builtin)

    def test_type_coercion_from_json_strings(self):
        wt = WellType.from_dict({
            "id": 42, "diameter_mm": "6.0", "well_depth_mm": "20",
            "rim_height_mm": "not-a-number", "ink_z_mm": "bad",
            "volume_uL": "100"})
        self.assertEqual(wt.id, "42")
        self.assertAlmostEqual(wt.diameter_mm, 6.0)
        self.assertAlmostEqual(wt.well_depth_mm, 20.0)
        self.assertAlmostEqual(wt.rim_height_mm, 0.0)   # bad → 0.0
        self.assertIsNone(wt.ink_z_mm)                  # bad → None
        self.assertAlmostEqual(wt.volume_uL, 100.0)

    def test_label_prefers_display_name(self):
        self.assertEqual(
            WellType(id="x", display_name="Nice Name").label, "Nice Name")

    def test_label_falls_back_with_volume(self):
        wt = WellType(id="x", manufacturer="Acme", model="T1", volume_uL=200.0)
        self.assertIn("Acme", wt.label)
        self.assertIn("200", wt.label)


class _TempStoreMixin:
    def _mk_store(self, builtin=(), user=()) -> WellTypeStore:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name)
        self._bdir = root / "builtin"
        self._udir = root / "user"
        for d in builtin:
            _write_type(self._bdir, builtin=True, **d)
        for d in user:
            _write_type(self._udir, builtin=False, **d)
        return WellTypeStore(builtin_dir=self._bdir, user_dir=self._udir)


class TestWellTypeStore(_TempStoreMixin, unittest.TestCase):
    def test_builtin_loads(self):
        store = self._mk_store(builtin=[{"id": "a"}, {"id": "b"}])
        self.assertIsNotNone(store.get("a"))
        self.assertTrue(store.get("a").builtin)
        self.assertEqual({w.id for w in store.all()}, {"a", "b"})

    def test_missing_id_returns_none(self):
        store = self._mk_store()
        self.assertIsNone(store.get(None))
        self.assertIsNone(store.get("nope"))

    def test_user_shadows_builtin(self):
        store = self._mk_store(
            builtin=[{"id": "a", "rim_height_mm": 20.0}])
        store.save_user(WellType(id="a", rim_height_mm=5.0, builtin=True))
        got = store.get("a")
        self.assertFalse(got.builtin)          # user always wins, builtin=False
        self.assertAlmostEqual(got.rim_height_mm, 5.0)
        # Fresh store re-reading the dirs sees the same shadow.
        store2 = WellTypeStore(builtin_dir=self._bdir, user_dir=self._udir)
        self.assertFalse(store2.get("a").builtin)
        self.assertAlmostEqual(store2.get("a").rim_height_mm, 5.0)

    def test_delete_user_resurfaces_builtin(self):
        store = self._mk_store(
            builtin=[{"id": "a", "rim_height_mm": 20.0}])
        store.save_user(WellType(id="a", rim_height_mm=5.0))
        self.assertAlmostEqual(store.get("a").rim_height_mm, 5.0)
        self.assertTrue(store.delete_user("a"))
        self.assertTrue(store.get("a").builtin)          # built-in returns
        self.assertAlmostEqual(store.get("a").rim_height_mm, 20.0)

    def test_delete_user_no_file_is_false(self):
        store = self._mk_store(builtin=[{"id": "a"}])
        self.assertFalse(store.delete_user("a"))         # no user file
        self.assertFalse(store.delete_user(""))

    def test_save_user_missing_id_rejected(self):
        store = self._mk_store()
        self.assertFalse(store.save_user(WellType(id="")))

    def test_safe_id_sanitizes(self):
        self.assertEqual(safe_id("0.1 mL PCR"), "0.1_mL_PCR")


class TestShippedBuiltin(unittest.TestCase):
    def test_pcr_tube_builtin_present(self):
        store = WellTypeStore()   # real config/hardware/well_types dirs
        wt = store.get(_BUILTIN_PCR_ID)
        self.assertIsNotNone(wt, "shipped PCR-tube built-in should load")
        self.assertTrue(wt.builtin)
        self.assertAlmostEqual(wt.diameter_mm, 6.0)
        self.assertAlmostEqual(wt.well_depth_mm, 20.0)
        self.assertAlmostEqual(wt.rim_height_mm, 20.0)
        self.assertAlmostEqual(wt.volume_uL, 100.0)


class TestWellTypeIdSerialization(unittest.TestCase):
    def test_none_omits_key_legacy_identical(self):
        w = Well(id=7, name="a", diameter=1.0)          # well_type_id None
        dd = _entity_to_dict(w)
        self.assertNotIn("well_type_id", dd)
        self.assertIsNone(_entity_from_dict(dd).well_type_id)

    def test_set_id_round_trips(self):
        w = Well(id=7, name="a", diameter=1.0)
        w.well_type_id = _BUILTIN_PCR_ID
        dd = _entity_to_dict(w)
        self.assertEqual(dd["well_type_id"], _BUILTIN_PCR_ID)
        self.assertEqual(_entity_from_dict(dd).well_type_id, _BUILTIN_PCR_ID)

    def test_plate_design_round_trip(self):
        d = PlateDesign.from_standard_format(24)
        w0 = d.get_wells()[0]
        # Legacy: no well_type_id on any serialized well entity.
        entities = d.to_dict()["entities"].values()
        wells = [e for e in entities if e.get("type") == "Well"]
        self.assertTrue(wells)
        self.assertTrue(all("well_type_id" not in e for e in wells))
        # Set + round-trip.
        w0.well_type_id = _BUILTIN_PCR_ID
        d2 = PlateDesign.from_dict(d.to_dict())
        w0b = next(w for w in d2.get_wells() if w.name == w0.name)
        self.assertEqual(w0b.well_type_id, _BUILTIN_PCR_ID)


class TestDesignerPicker(unittest.TestCase):
    """Offscreen smoke: the well-type picker on a rosette sub-well card, and
    _apply_well_type stamping. GUI interaction still needs real verification."""

    def _rosette_subwell_widget(self):
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        from gui.widgets.plate_designer_canvas import Tool
        w = PlateDesignerWidget(mode="rosette")
        w.load_plate(24)
        a1 = w._design.get_wells()[0]
        w._canvas.well_drill_requested.emit(a1.id)      # → _edit_context set
        self.assertIsNotNone(w._edit_context)
        w._canvas.set_tool(Tool.DRAW_SINGLE_WELL)
        w._canvas._handle_draw_well((2.0, 0.0))
        sub = w._design.get_wells()[0]
        return w, sub

    def test_well_card_has_type_picker(self):
        w, sub = self._rosette_subwell_widget()
        card = w._build_well_card(sub)
        combos = card.findChildren(QComboBox)
        labels = {c.itemText(i) for c in combos for i in range(c.count())}
        self.assertIn("(custom)", labels)
        self.assertIn("0.1 mL PCR tube", labels)

    def test_apply_well_type_stamps_geometry(self):
        w, sub = self._rosette_subwell_widget()
        w._apply_well_type(sub.id, _BUILTIN_PCR_ID)
        self.assertEqual(sub.well_type_id, _BUILTIN_PCR_ID)
        self.assertAlmostEqual(sub.diameter, 6.0)
        self.assertAlmostEqual(sub.well_depth_mm, 20.0)
        self.assertAlmostEqual(sub.rim_height_mm, 20.0)

    def test_apply_custom_clears_link_keeps_geometry(self):
        w, sub = self._rosette_subwell_widget()
        w._apply_well_type(sub.id, _BUILTIN_PCR_ID)
        w._apply_well_type(sub.id, None)                 # "(custom)"
        self.assertIsNone(sub.well_type_id)
        self.assertAlmostEqual(sub.diameter, 6.0)        # geometry unchanged
        self.assertAlmostEqual(sub.rim_height_mm, 20.0)


if __name__ == "__main__":
    unittest.main()
