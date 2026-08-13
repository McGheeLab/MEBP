"""
Structural guards for the v7.18 optics work.

These pin things a behavioural test cannot see:

* ``OpticsRegistry`` must stay importable from anywhere — it is imported BY
  ``ObjectiveCalibration`` and ``MicroscopeConfigStore``, so a Qt or ``gui.``
  import inside it would drag the GUI into every store in the app and create
  cycles that only show up as an ImportError at startup.
* ``MountedOptic`` must keep having ``label`` and NOT ``name``. The fluorescence
  workflow read ``getattr(optic, "name", "")``, which was silently always empty —
  the whole class of bug that a ``getattr`` default hides. If someone later adds a
  ``name`` field, the guard here should be revisited deliberately rather than
  letting two spellings coexist.
* The legacy channel→number table must not be reachable from anything that could
  command the filter turret. On this rig its "mCherry → 3" names a cube that is
  not mCherry and its "Bright Field → 5" names an EMPTY slot.
"""

import ast
import os
import subprocess
import sys
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

REPO = Path(__file__).resolve().parent.parent
PURE_MODULES = ("SupportClasses/OpticsRegistry.py",)


def _imported_names(path: Path) -> set:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    out = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            out.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            out.add(node.module)
    return out


class TestTheCoreIsPure(unittest.TestCase):
    def test_no_qt_or_gui_imports(self):
        for rel in PURE_MODULES:
            names = _imported_names(REPO / rel)
            for bad in ("PySide6", "PyQt5", "PyQt6"):
                self.assertNotIn(bad, {n.split(".")[0] for n in names},
                                 f"{rel} must not import {bad}")
            for n in names:
                self.assertFalse(n.startswith("gui"),
                                 f"{rel} must not import {n}")

    def test_no_repo_imports_at_all(self):
        """Zero repo imports is what makes it safe to import from any store."""
        for rel in PURE_MODULES:
            for n in _imported_names(REPO / rel):
                self.assertFalse(
                    n.startswith("SupportClasses"),
                    f"{rel} imports {n}; keeping it dependency-free is what "
                    f"prevents a cycle with ObjectiveCalibration / "
                    f"MicroscopeConfigStore, which both import IT.")

    def test_it_imports_with_pyside_absent(self):
        """Proven in a clean interpreter, not by inspecting this one."""
        code = (
            "import sys\n"
            "for m in list(sys.modules):\n"
            "    if m.split('.')[0] in ('PySide6','PyQt5','PyQt6'):\n"
            "        del sys.modules[m]\n"
            "class _Blocker:\n"
            "    def find_module(self, name, path=None):\n"
            "        if name.split('.')[0] in ('PySide6','PyQt5','PyQt6'):\n"
            "            raise ImportError(name)\n"
            "sys.meta_path.insert(0, _Blocker())\n"
            "from SupportClasses.OpticsRegistry import find_slot, resolve_slots\n"
            "assert find_slot((), 'x', kind='filter').ok is False\n"
            "print('OK')\n")
        res = subprocess.run([sys.executable, "-c", code], cwd=str(REPO),
                             capture_output=True, text=True, timeout=120)
        self.assertIn("OK", res.stdout, res.stderr[-2000:])


class TestMountedOpticFieldNames(unittest.TestCase):
    def test_it_has_label_and_no_name(self):
        from SupportClasses.MicroscopeControl import MountedOptic
        fields = {f.name for f in MountedOptic.__dataclass_fields__.values()}
        self.assertIn("label", fields)
        self.assertNotIn(
            "name", fields,
            "The fluorescence workflow used to read optic.name, which does not "
            "exist — a getattr default made it silently empty forever. If a "
            "'name' field is added, re-check every optic.label reader.")

    def test_the_fluorescence_workflow_no_longer_reads_name(self):
        """Walked by AST, NOT by substring.

        A substring guard fails here for the reason this repo keeps recording: the
        first version of this test matched the explanatory COMMENT that names the
        old bug, so it failed against correct code. Only the parsed calls count.
        """
        path = REPO / "gui/pages/workflows/fluorescence_mosaic_workflow.py"
        tree = ast.parse(path.read_text(encoding="utf-8"))
        reads = set()
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                    and node.func.id == "getattr"
                    and len(node.args) >= 2
                    and isinstance(node.args[0], ast.Name)
                    and node.args[0].id == "optic"
                    and isinstance(node.args[1], ast.Constant)):
                reads.add(node.args[1].value)
        self.assertNotIn("name", reads,
                         "MountedOptic has no 'name' field — this read was "
                         "always empty and silently fell back.")
        self.assertIn("label", reads, "the guard found no optic reads at all, "
                                      "so it would pass vacuously")


class TestChannelOrdinalIsNotASlotMap(unittest.TestCase):
    def test_the_ordinal_table_is_unchanged_in_VALUE(self):
        """Renaming the concept must not silently renumber acquisition order —
        LabLinkJob orders a sidecar's channels by it."""
        from SupportClasses.FluorescenceMosaicStore import CHANNEL_ORDINALS
        self.assertEqual(CHANNEL_ORDINALS,
                         {"DAPI": 1, "FITC": 2, "mCherry": 3, "Cy5": 4,
                          "Bright Field": 5})

    def test_the_legacy_name_still_reads_for_on_disk_sidecars(self):
        from SupportClasses.FluorescenceMosaicStore import (
            CHANNEL_NUMBERS, CHANNEL_ORDINALS, channel_number, channel_ordinal)
        self.assertIs(CHANNEL_NUMBERS, CHANNEL_ORDINALS)
        self.assertEqual(channel_number("mCherry"), channel_ordinal("mCherry"))

    def test_it_is_documented_as_NOT_a_turret_slot(self):
        """The original comment claimed it told the operator which turret
        position to select. That was never true of any particular rig."""
        src = (REPO / "SupportClasses/FluorescenceMosaicStore.py"
               ).read_text(encoding="utf-8")
        head = src[:src.index("def default_color")]
        self.assertIn("NOT", head)
        self.assertIn("TURRET SLOT", head.upper())

    def test_channel_slot_refuses_rather_than_using_the_ordinal(self):
        """The real resolver must not fall back to the table it replaced."""
        from types import SimpleNamespace

        from SupportClasses.FluorescenceMosaicStore import channel_slot

        class _Cfg:
            def filter_labels(self):
                return {1: "DAPI", 2: "FITC", 3: "TxRed", 4: "Cy5"}

            def filter_slots(self):
                return 6

            def filter_optics(self):
                return {}

            def optic_aliases(self, kind):
                return {}

        st = SimpleNamespace(
            filter_count=6, filter_position=1,
            native_filter_names=("DAPI", "FITC", "TxRed", "Cy5", "-----", "-----"),
            mounted_filters=tuple(
                SimpleNamespace(position=i, present=(i <= 4), label=lbl, code="")
                for i, lbl in enumerate(
                    ("DAPI", "FITC", "TxRed", "Cy5", "-----", "-----"), start=1)))

        m = channel_slot("mCherry", scope_state=st, config_store=_Cfg())
        self.assertFalse(m.ok, "mCherry must not resolve to TxRed unaided")
        self.assertNotEqual(m.position, 3)
        bf = channel_slot("Bright Field", scope_state=st, config_store=_Cfg())
        self.assertFalse(bf.ok)
        self.assertNotEqual(bf.position, 5, "slot 5 is empty on this rig")
        ok = channel_slot("FITC", scope_state=st, config_store=_Cfg())
        self.assertTrue(ok.ok)
        self.assertEqual(ok.position, 2)


class TestOneImplementationOfTheWalk(unittest.TestCase):
    def test_objective_ladder_delegates_optic_at(self):
        from SupportClasses import OpticsRegistry
        from SupportClasses.ObjectiveLadder import _optic_at
        from types import SimpleNamespace
        st = SimpleNamespace(mounted_objectives=(
            SimpleNamespace(position=2, label="10x"),))
        self.assertIs(_optic_at(st, 2),
                      OpticsRegistry.optic_at(st, 2, OpticsRegistry.OBJECTIVE))

    def test_resolve_ladder_uses_the_shared_resolver(self):
        src = (REPO / "SupportClasses/ObjectiveLadder.py").read_text(
            encoding="utf-8")
        tree = ast.parse(src)
        fn = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == "resolve_ladder")
        called = {c.func.id for c in ast.walk(fn)
                  if isinstance(c, ast.Call) and isinstance(c.func, ast.Name)}
        self.assertIn("resolve_slots", called,
                      "resolve_ladder must consume the shared join, not "
                      "re-implement the position->name walk.")


if __name__ == "__main__":
    unittest.main()
