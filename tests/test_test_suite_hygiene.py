"""test_test_suite_hygiene.py — structural checks on the test suite itself.

These guard against defects that are INVISIBLE to a normal test run, because the
affected tests simply never execute. A suite that silently drops a third of a
file's coverage still reports "OK", so only a structural check catches it.

Motivating incident (v7.9): ``test_v79_per_bore_cell_targeting.py`` had

    if __name__ == "__main__":
        unittest.main()

sitting in the MIDDLE of the file, above ``TestSimultaneousPrep`` and
``TestSimultaneousPostClean``. Run as a script the interpreter reached
``unittest.main()`` before those classes were defined, so 9 of 45 tests were
never collected — and they were the ONLY coverage of the simultaneous-prep
feature, which was consequently dead in production for weeks while its suite
reported green. The recorded test count in the update plan (36) was exactly the
number a runner honouring that guard collects, which is how the discrepancy hid
in plain sight.

Deliberately AST-based, not import-based: these files import Qt and hardware
modules, and this check must stay cheap and side-effect-free.
"""

from __future__ import annotations

import ast
import unittest
from pathlib import Path

TESTS_DIR = Path(__file__).resolve().parent


def _test_files() -> list[Path]:
    return sorted(p for p in TESTS_DIR.glob("test_*.py") if p.is_file())


def _main_guard_lineno(tree: ast.Module) -> int | None:
    """Line of the top-level ``if __name__ == "__main__":`` block, if any."""
    for node in tree.body:
        if not isinstance(node, ast.If):
            continue
        t = node.test
        if (isinstance(t, ast.Compare)
                and isinstance(t.left, ast.Name) and t.left.id == "__name__"
                and len(t.comparators) == 1
                and isinstance(t.comparators[0], ast.Constant)
                and t.comparators[0].value == "__main__"):
            return node.lineno
    return None


def _base_name(base: ast.expr) -> str | None:
    """``unittest.TestCase`` → "TestCase"; ``_PageCase`` → "_PageCase"."""
    if isinstance(base, ast.Attribute):
        return base.attr
    if isinstance(base, ast.Name):
        return base.id
    return None


def _testcase_class_names(tree: ast.Module) -> set[str]:
    """Every top-level class in the module that is a TestCase subclass.

    ⚠ Resolved TRANSITIVELY through module-local base classes. The first cut of
    this check matched only a base *literally* named ``…TestCase``, which made it
    blind to the dominant pattern in this suite — one shared local base
    (``class _PageCase(unittest.TestCase)``) that every real test class derives
    from. It therefore passed while two files carried the exact defect it exists
    to catch, stranding 41 tests between them. Any rule that inspects base names
    must chase local bases or it is checking the wrong files.
    """
    classes = [n for n in tree.body if isinstance(n, ast.ClassDef)]
    known: set[str] = set()
    changed = True
    while changed:                      # fixpoint: bases may be defined in any order
        changed = False
        for node in classes:
            if node.name in known:
                continue
            for base in node.bases:
                name = _base_name(base)
                if name and (name.endswith("TestCase") or name in known):
                    known.add(node.name)
                    changed = True
                    break
    return known


def _is_testcase_class(node: ast.AST, known: set[str] | None = None) -> bool:
    if not isinstance(node, ast.ClassDef):
        return False
    if known is not None:
        return node.name in known
    for base in node.bases:
        name = _base_name(base)
        if name and name.endswith("TestCase"):
            return True
    return False


class TestNoTestCaseAfterMainGuard(unittest.TestCase):
    """A TestCase defined after ``unittest.main()`` never runs as a script."""

    def test_every_test_file_defines_all_testcases_before_its_main_guard(self):
        offenders: list[str] = []
        for path in _test_files():
            try:
                tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
            except SyntaxError as exc:            # a broken file is its own failure
                offenders.append(f"{path.name}: unparseable ({exc})")
                continue
            guard = _main_guard_lineno(tree)
            if guard is None:
                continue                          # no guard, nothing to strand
            known = _testcase_class_names(tree)
            stranded = [n.name for n in tree.body
                        if _is_testcase_class(n, known) and n.lineno > guard]
            if stranded:
                offenders.append(
                    f"{path.name}: {len(stranded)} TestCase class(es) defined "
                    f"AFTER the __main__ guard on line {guard} and therefore "
                    f"never collected when the file is run as a script: "
                    f"{', '.join(stranded)}")
        self.assertEqual(
            offenders, [],
            "Move the `if __name__ == \"__main__\": unittest.main()` block to "
            "the END of each file listed:\n  " + "\n  ".join(offenders))

    def test_nothing_at_all_is_defined_after_the_main_guard(self):
        """The stronger rule, needing no base-class resolution to be right.

        Whatever a class derives from, a top-level ``class``/``def`` after the
        guard is unreachable when the file runs as a script. Checking *that*
        cannot be defeated by a naming convention, so it backstops the
        TestCase-specific check above rather than duplicating it.
        """
        offenders: list[str] = []
        for path in _test_files():
            try:
                tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
            except SyntaxError:
                continue                          # reported by the check above
            guard = _main_guard_lineno(tree)
            if guard is None:
                continue
            after = [f"{type(n).__name__} {getattr(n, 'name', '')}".strip()
                     for n in tree.body
                     if isinstance(n, (ast.ClassDef, ast.FunctionDef,
                                       ast.AsyncFunctionDef))
                     and n.lineno > guard]
            if after:
                offenders.append(f"{path.name}: {', '.join(after)} "
                                 f"(guard on line {guard})")
        self.assertEqual(
            offenders, [],
            "The `__main__` guard must be the LAST statement in the file:\n  "
            + "\n  ".join(offenders))

    def test_the_v79_per_bore_file_collects_every_class_it_defines(self):
        """Regression pin for the specific file that carried the defect."""
        path = TESTS_DIR / "test_v79_per_bore_cell_targeting.py"
        self.assertTrue(path.exists())
        tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
        classes = [n.name for n in tree.body if _is_testcase_class(n)]
        guard = _main_guard_lineno(tree)
        self.assertIsNotNone(guard, "file should still have a __main__ guard")
        self.assertIn("TestSimultaneousPrep", classes)
        self.assertIn("TestSimultaneousPostClean", classes)
        for n in tree.body:
            if _is_testcase_class(n):
                self.assertLess(
                    n.lineno, guard,
                    f"{n.name} is defined after the __main__ guard")


class TestSanityOfTheHygieneCheckItself(unittest.TestCase):
    """A structural check that cannot fail is worse than no check."""

    def test_it_detects_a_stranded_testcase(self):
        src = (
            "import unittest\n"
            "class TestA(unittest.TestCase):\n    pass\n"
            'if __name__ == "__main__":\n    unittest.main()\n'
            "class TestB(unittest.TestCase):\n    pass\n"
        )
        tree = ast.parse(src)
        guard = _main_guard_lineno(tree)
        stranded = [n.name for n in tree.body
                    if _is_testcase_class(n) and n.lineno > guard]
        self.assertEqual(stranded, ["TestB"])

    def test_it_detects_a_stranded_testcase_behind_a_LOCAL_base_class(self):
        """The case the first cut of this check missed entirely.

        ``_PageCase`` does not end in "TestCase", so base-name matching alone
        classified every class in ``test_v79_cell_targeting_setup_page.py`` as
        "not a test" and the file sailed through with 27 tests stranded.
        """
        src = (
            "import unittest\n"
            "class _PageCase(unittest.TestCase):\n    pass\n"
            "class TestA(_PageCase):\n    pass\n"
            'if __name__ == "__main__":\n    unittest.main()\n'
            "class TestB(_PageCase):\n    pass\n"
        )
        tree = ast.parse(src)
        known = _testcase_class_names(tree)
        self.assertIn("TestB", known, "local base not resolved")
        guard = _main_guard_lineno(tree)
        stranded = [n.name for n in tree.body
                    if _is_testcase_class(n, known) and n.lineno > guard]
        self.assertEqual(stranded, ["TestB"])

    def test_it_resolves_a_local_base_declared_after_its_subclass(self):
        """Order-independent: the resolver iterates to a fixpoint."""
        src = (
            "import unittest\n"
            "class TestA(_Mid):\n    pass\n"
            "class _Mid(_PageCase):\n    pass\n"
            "class _PageCase(unittest.TestCase):\n    pass\n"
        )
        known = _testcase_class_names(ast.parse(src))
        self.assertEqual(known, {"TestA", "_Mid", "_PageCase"})

    def test_it_passes_a_correctly_ordered_file(self):
        src = (
            "import unittest\n"
            "class TestA(unittest.TestCase):\n    pass\n"
            "class TestB(unittest.TestCase):\n    pass\n"
            'if __name__ == "__main__":\n    unittest.main()\n'
        )
        tree = ast.parse(src)
        guard = _main_guard_lineno(tree)
        stranded = [n.name for n in tree.body
                    if _is_testcase_class(n) and n.lineno > guard]
        self.assertEqual(stranded, [])

    def test_it_finds_the_real_test_files(self):
        files = _test_files()
        self.assertGreater(len(files), 50, "expected the full suite on disk")
        self.assertIn("test_v79_per_bore_cell_targeting.py",
                      [p.name for p in files])


if __name__ == "__main__":
    unittest.main()
