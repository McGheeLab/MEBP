# Vendored LabLink client

| | |
|---|---|
| **Upstream** | `https://github.com/McGheeLab/lablink` (private) |
| **Commit** | `7123061c81128be3bec9d7ec72d6e84c80f9438a` |
| **Vendored** | 2026-08-08, for MEBP v7.17 |
| **Contract** | `docs/INTEGRATING-WITH-LABLINK.md`, `QUICKSTART.md` §10, `docs/IMAGE-JOB-FORMAT.md` |

## What is here, and what is deliberately not

| file | state |
|---|---|
| `protocol.py` | **byte-identical** to upstream |
| `fsutil.py` | **byte-identical** |
| `client.py` | **byte-identical** |
| `session_client.py` | **byte-identical** |
| `__init__.py` | **ours** — see the note in that file |

Everything else in upstream's `lablink/` package is **excluded on purpose**:
`server.py`, `sessions.py`, `session_http.py`, `console_http.py`, `console/`,
`hubconfig.py`, `nodes.py`, `recipes.py`, `report.py`, `runner.py`,
`service.py`, `store.py`, `sync.py`, `worker_link.py`, `config.py`,
`__main__.py`.

Two of those matter more than the rest. **`runner.py` executes subprocess
commands and `config.py` declares them** — they are the tool machine's half of
LabLink. Copying either into an application that drives a needle over a glass
plate would put a command executor inside it for no benefit whatever, because
MEBP never runs a recipe: it asks a hub to. `tests/test_v717_lablink_vendor.py`
asserts their absence rather than trusting this paragraph.

`recipes.py` is excluded for a milder reason: it is recipe *authoring*, and
installing a recipe is an offline act with a person in the loop
(`docs/RECIPE-AUTHORING.md`), not something MEBP does.

## Why vendor at all

`QUICKSTART.md` says *"No client library exists or is needed"*, so a hand-written
client would be sanctioned. We vendor because the download-resume and
hash-precedence logic is the subtle, security-relevant part — QUICKSTART §8
records that each of its three client rules *"was a real bug here"*, and
`client.py` already encodes the fixes:

* a short partial is **resumed**, not discarded, while a complete-but-wrong one
  is refetched from zero;
* the **caller's** pinned hash wins over the one the response offers, so a
  hostile or spoofed server cannot substitute content and hash it to match;
* an unverified file is never renamed into place.

Re-deriving those invites re-introducing the bugs they fix.

Not `pip install`: upstream is a private repo with no published package, and
MEBP ships as a PyInstaller bundle. A `sys.path` insert into `WIFI parsing/`
cannot work either — the space makes it non-importable and PyInstaller's static
analysis would never follow it. That copy has, separately, **already drifted**
from canonical, which is the empirical argument for pinning rather than trusting.

## Re-vendoring

```bash
cd c:/dev/lablink && git fetch origin
for f in protocol.py fsutil.py client.py session_client.py; do
  git show origin/main:lablink/$f > c:/dev/MEBP/SupportClasses/lablink/$f
done
git -C c:/dev/lablink rev-parse origin/main   # -> update UPSTREAM_COMMIT + the table above
python -m unittest tests.test_v717_lablink_vendor   # regenerate the sha manifest it prints
```

Then re-read `session_client.py`'s diff before trusting it: a protocol change on
the hub side is exactly what a silent vendor refresh would hide.

**Never patch a vendored file in place.** Fix it upstream and re-vendor —
otherwise the sha manifest becomes a record of a lie.

## Drift detection

`tests/test_v717_lablink_vendor.py` enforces four things:

1. a pinned sha256 per file — a local edit fails the suite;
2. AST purity — every import resolves to the standard library or a sibling
   vendored module (no `SupportClasses.*`, no PySide6, no numpy, no cv2);
3. the negative list above;
4. an **env-gated byte diff** against a live upstream checkout. Set
   `MEBP_LABLINK_UPSTREAM=c:/dev/lablink/lablink` to arm it; it skips otherwise,
   so the suite still runs on a machine with no sibling repo.
