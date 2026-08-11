"""v7.17 — the vendored LabLink client must stay what upstream shipped.

`SupportClasses/lablink/` is a copy of four stdlib-only modules from the
private lablink repo. A copy rots: the older fork at `WIFI parsing/lablink/`
has ALREADY drifted from canonical on all three of the files it shares, which
is the empirical argument for pinning rather than trusting.

Four guards, in increasing strength:

1. a pinned sha256 per file, so a local edit fails here rather than at the bench;
2. AST purity, so nothing repo-shaped creeps into a module that must stay
   vendorable;
3. a NEGATIVE list, because the dangerous failure is not editing a vendored
   file but adding one — `runner.py` executes subprocess commands and
   `config.py` declares them, and neither belongs in an application that drives
   a needle;
4. an env-gated byte diff against a live upstream checkout, which is the only
   check that can see upstream moving.

Guards 1-3 need no network and no sibling repo, so they run everywhere.
"""

from __future__ import annotations

import ast
import hashlib
import os
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
VENDOR = REPO / "SupportClasses" / "lablink"

# sha256 of each byte-identical file, from upstream 7123061.
# Regenerate ONLY when re-vendoring — see SupportClasses/lablink/VENDORED.md.
PINNED = {
    "protocol.py":
        "ad251c977a82f1155d55be60dcb3c814990fa40360d5589c74aa4bdf83d95189",
    "fsutil.py":
        "0b56842ff5d281d645b9420944681b2bb6d3ea46cd27de698b1ba0c1249eb4ae",
    "client.py":
        "bc9386c77f6ffaf35087a319e00478bc0ae3ceabef477d98c254a007b894f9a3",
    "session_client.py":
        "6d39962c9c9acffbc932c0d13e1cb260e26bd6a23a83eaf6c32751be0a6cba93",
}

UPSTREAM_COMMIT = "7123061c81128be3bec9d7ec72d6e84c80f9438a"

#: Ours, not upstream's — upstream re-exports recipe AUTHORING, which MEBP
#: does not do. Excluded from the sha pin, still covered by purity.
OURS = {"__init__.py"}

#: Upstream modules that must NEVER appear here. The first two are the whole
#: reason this list is a test: they are the tool machine's command executor and
#: the local declaration of the commands it runs.
FORBIDDEN = {
    "runner.py", "config.py", "server.py", "service.py", "sessions.py",
    "session_http.py", "console_http.py", "hubconfig.py", "nodes.py",
    "recipes.py", "report.py", "store.py", "sync.py", "worker_link.py",
    "__main__.py",
}

STDLIB = set(getattr(sys, "stdlib_module_names", ())) or {
    "json", "os", "re", "sys", "time", "hashlib", "logging", "pathlib",
    "urllib", "dataclasses", "typing", "shutil", "tempfile", "threading",
}


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class TestPinnedBytes(unittest.TestCase):
    """A vendored file that changed locally is a lie about its provenance."""

    def test_every_pinned_file_matches_its_hash(self):
        for name, want in PINNED.items():
            path = VENDOR / name
            self.assertTrue(path.is_file(), f"{name} is missing from the vendor dir")
            self.assertEqual(
                _sha(path), want,
                f"\n{name} no longer matches upstream {UPSTREAM_COMMIT[:8]}.\n"
                f"NEVER patch a vendored file in place — fix it upstream in\n"
                f"c:/dev/lablink and re-vendor (SupportClasses/lablink/"
                f"VENDORED.md), then update PINNED here.")

    def test_the_vendor_dir_holds_only_what_is_declared(self):
        found = {p.name for p in VENDOR.glob("*.py")}
        self.assertEqual(
            found, set(PINNED) | OURS,
            "an undeclared .py appeared in the vendor dir; add it to PINNED "
            "(byte-identical) or OURS, and check it is not on FORBIDDEN")

    def test_the_recorded_commit_agrees_with_the_package(self):
        from SupportClasses.lablink import UPSTREAM_COMMIT as pkg_commit
        self.assertEqual(pkg_commit, UPSTREAM_COMMIT)


class TestNoHubSideCode(unittest.TestCase):
    """The executor half of LabLink must never land in the bioprinter app."""

    def test_forbidden_modules_are_absent(self):
        for name in sorted(FORBIDDEN):
            self.assertFalse(
                (VENDOR / name).exists(),
                f"{name} was vendored. MEBP is a LabLink NODE, never a hub or "
                f"a worker. runner.py/config.py in particular are a subprocess "
                f"executor and its command declarations.")

    def test_nothing_vendored_can_spawn_a_subprocess(self):
        """A cheap, direct check of the property FORBIDDEN exists to protect."""
        for path in sorted(VENDOR.glob("*.py")):
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                if isinstance(node, (ast.Import, ast.ImportFrom)):
                    mods = ([a.name for a in node.names]
                            if isinstance(node, ast.Import)
                            else [node.module or ""])
                    for mod in mods:
                        root = mod.split(".")[0]
                        self.assertNotIn(
                            root, {"subprocess", "multiprocessing", "ctypes"},
                            f"{path.name} imports {mod!r}")


class TestPurity(unittest.TestCase):
    """Vendored code must stay vendorable: stdlib + its own siblings only."""

    def test_imports_resolve_to_stdlib_or_a_sibling(self):
        siblings = {p.stem for p in VENDOR.glob("*.py")}
        for path in sorted(VENDOR.glob("*.py")):
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom):
                    if node.level:                       # from . import x
                        self.assertIn(
                            (node.module or "").split(".")[0] or "", siblings | {""},
                            f"{path.name}: relative import outside the package")
                        continue
                    roots = [(node.module or "").split(".")[0]]
                else:
                    if not isinstance(node, ast.Import):
                        continue
                    roots = [a.name.split(".")[0] for a in node.names]
                for root in roots:
                    if not root or root in siblings:
                        continue
                    self.assertIn(
                        root, STDLIB,
                        f"{path.name} imports {root!r}, which is neither "
                        f"stdlib nor a vendored sibling. The vendored client "
                        f"must stay dependency-free.")

    def test_our_init_does_not_reach_for_an_excluded_module(self):
        """The one file we wrote must not import what we chose not to vendor."""
        tree = ast.parse((VENDOR / "__init__.py").read_text(encoding="utf-8"))
        excluded = {Path(n).stem for n in FORBIDDEN}
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.level:
                self.assertNotIn(
                    (node.module or "").split(".")[0], excluded,
                    "__init__ imports a module that is not vendored — that is "
                    "exactly why it is ours rather than upstream's copy")


class TestUsableSurface(unittest.TestCase):
    """The names MEBP builds against must actually be importable."""

    def test_the_session_api_is_present(self):
        from SupportClasses import lablink
        for name in ("SessionClient", "HubSession", "CommandResult",
                     "CommandFailed", "SessionLost", "LabLinkClient",
                     "LabLinkError", "TERMINAL_STATES"):
            self.assertTrue(hasattr(lablink, name), f"missing {name}")

    def test_protocol_constants_are_the_ones_the_docs_state(self):
        from SupportClasses.lablink import protocol as p
        self.assertEqual(p.NAME_RE.pattern, r"^[A-Za-z0-9][A-Za-z0-9._ -]{0,127}$")
        self.assertEqual(p.MAX_META_BYTES, 8192)
        self.assertEqual(p.PROTOCOL_VERSION, 1)
        # Load-bearing: the socket timeout must sit ABOVE this, or a working
        # long poll turns into a retry storm.
        self.assertGreater(p.LONGPOLL_MAX_S, 0)

    def test_terminal_states_include_lost(self):
        """`lost` means the hub restarted; it must be terminal so the service
        surfaces it instead of waiting forever."""
        from SupportClasses.lablink import TERMINAL_STATES
        for state in ("closed", "failed", "cancelled", "lost", "expired"):
            self.assertIn(state, TERMINAL_STATES)


class TestUpstreamDiff(unittest.TestCase):
    """Armed only when an upstream git repo is pointed at.

    ⚠ Compares against the PINNED COMMIT via `git show`, never against files
    on disk. The sibling clone's working tree sits on whatever branch someone
    left it on — it was five commits behind `origin/main` when this was
    vendored — so a file comparison would report drift that is really just
    somebody else's checkout state, and the guard would be dismissed as noisy
    exactly when it needs to be believed.
    """

    def test_bytes_match_the_pinned_upstream_commit(self):
        import subprocess
        up = os.environ.get("MEBP_LABLINK_UPSTREAM")
        if not up:
            self.skipTest("set MEBP_LABLINK_UPSTREAM=<path to the lablink repo> "
                          "to arm the upstream comparison")
        repo = Path(up)
        # Tolerate being pointed at either the repo root or the package dir.
        if repo.name == "lablink" and (repo.parent / ".git").is_dir():
            repo = repo.parent
        if not (repo / ".git").is_dir():
            self.skipTest(f"not a git repo: {repo}")

        def show(rel):
            return subprocess.run(
                ["git", "-C", str(repo), "show", f"{UPSTREAM_COMMIT}:{rel}"],
                capture_output=True)

        probe = show(f"lablink/{next(iter(PINNED))}")
        if probe.returncode != 0:
            self.skipTest(
                f"commit {UPSTREAM_COMMIT[:8]} is not in {repo} — run "
                f"`git -C {repo} fetch origin` first")

        for name in PINNED:
            got = show(f"lablink/{name}")
            self.assertEqual(
                got.returncode, 0,
                f"{name} is not present at {UPSTREAM_COMMIT[:8]} — it may have "
                f"been renamed; read the diff before re-vendoring")
            self.assertEqual(
                hashlib.sha256(got.stdout).hexdigest(), PINNED[name],
                f"{name} does not match upstream {UPSTREAM_COMMIT[:8]}. The "
                f"vendored copy was edited, or PINNED was updated without "
                f"re-copying the file.")


if __name__ == "__main__":
    unittest.main()
