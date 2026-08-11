# MEBP v7.17 — LabLink imaging integration

## Objective

Push MEBP imaging outputs to a **LabLink hub** so a recipe on another machine
(ND2 Studios) can process them — deconvolution, segmentation — and bring the
results back. Operator: *"we want to be able integrate lablink to any imaging
workflows… if we make a flourescence scan we should push the scan results to
lablink for a nd2studios recipe to do things like deconvolution or other
recipes. lets make a workflow page that sets this up. when we turn it on with
this workflow page, it automatically sends outputs of various types to lablink
for processing. flourescent mosiaic scans should for sure be able to go for
this processing. images and video that are captured should also go for
processing."*

Before this, MEBP had **zero** LabLink integration and **zero** networking code
(one `socket.gethostname()` in a log line).

---

## 🔴 Security finding, fixed — but the token must still be rotated

`WIFI parsing/lablink_connect.py:34-35` was **tracked and pushed to GitHub**
with a live hub address and site token:

```python
DEFAULT_URL   = "http://<hub-ip>:8765"     # a real private address
DEFAULT_TOKEN = "<redacted — 20-char site token>"
```

(The literal values are deliberately **not** repeated here. They are already
published in history under `c2821bf`, and re-quoting them in a new file only
widens the exposure without telling a reader anything the description does not.
Recover them from that commit if the rotation needs to identify the old token.)

Upstream fixed exactly this in commit `b9edb79` ("Remove the site token and a
real address from committed defaults"); the MEBP fork never received it and it
sits in history under `c2821bf`. Now purged to match upstream — both `""`, with
the explanatory comment; the file already supported `LABLINK_URL`/
`LABLINK_TOKEN` and already gitignored `lablink_config.json`, so nothing else
changed.

**Purging does not un-publish it.** LabLink's own QUICKSTART §8 is explicit
that the token *"does not stop an attacker: it is a single shared secret, and
every holder can read, overwrite and delete everything in every channel"*, and
there is no TLS. **Rotate it on the server.**

A private IP remains in `WIFI parsing/plan 1.md:21` — a network-survey note,
far lower risk, left alone.

---

## ⚠ The first plan was built on a stale clone

`c:\dev\lablink` was **5 commits behind**. A `git fetch` brought down
`docs/INTEGRATING-WITH-LABLINK.md`, `docs/IMAGE-JOB-FORMAT.md`,
`docs/SEND-A-CELL-SEGMENTATION-JOB.md`, `docs/RECIPE-AUTHORING.md` and an
entire **session API** absent locally. Recorded because the wrong version was
convincing:

| first cut assumed | reality |
|---|---|
| MEBP cannot select a recipe, only a channel + filename | **MEBP names workflow + recipe + knobs** via `POST /s`. The channel/fnmatch model is the older *file-exchange* half. |
| fire-and-forget push; poll a reply channel | **sessions**: open → upload to the hub-returned `in_channel` → `POST /s/{id}/cmd` → long-poll → collect from `out_channel` → `DELETE`. |
| operator configures channel + name prefix + format | **"Discover, do not hardcode"** — recipes, knob bounds, channel names, size limits and timeouts all read at run time. |
| poll every ~5 s | **long-poll** `?wait=25`, socket timeout **above** `capabilities.longpoll_max_s`. |
| correlate on `input_sha256` in the reply meta | session + `cmd_seq`; artifacts carry `meta` naming session/command/recipe. |

Survived unchanged: the 409/unique-name hazard, `NAME_RE`, "retry iff status 0
or ≥ 500", `max_file_bytes` from `/hello`, "a successful upload never means
your sidecar is valid", not reusing `attach_processed`, and a Qt-free service
with the page as a control surface.

**The security boundary still holds, one level up:** MEBP picks a *declared*
workflow and recipe and adjusts *published* knobs within *published* bounds.
`POST /s` carrying a `graph` is refused `403 graph_not_allowed` unconditionally.

## Operator decisions (AskUserQuestion)

1. **Push and retrieve results** — bidirectional.
2. **Send `.nd3`** (+ its `.job.json` sidecar). **✅ The hub already reads it** —
   see the correction below.
3. **Retry in-session only** — a restart drops unsent work and must say so.
4. **Metadata = scientific context only** — operator name and free-text notes
   stripped.
5. **Limits come from the hub, never a constant** (*"we may be increasing this
   ceiling later… for tailscale links we would need to limit and refuse"*),
   plus a per-source ceiling for slow links.
6. **Wavelengths live in `MicroscopeConfigStore`**, from the rig's real filter
   specs.

## ⚠ Correction — `.nd3` IS accepted; I read the wrong field

An earlier revision of this document, and the first version of the workflow
page's banner, both stated that the hub cannot read `.nd3` and that *"a job sent
today is expected to be refused"*. **That was wrong.**

The mistake was reading the recipe manifest's **top-level `match`**, which is
`null`, and concluding the format was unsupported. The field that governs
acceptance is **`inputs[].match`**, and every `nd2studios` recipe lists `*.nd3`
there:

```
recipes/nd2studios/cell-segmentation/recipe.json
  inputs: [{role: "image", node: "load", required: true,
            match: ["*.tif", "*.tiff", "*.nd2", "*.nd3"]}]
```

`docs/IMAGE-JOB-FORMAT.md` goes further and documents the pairing rule for the
format by name — *"`scan.nd3` beside `scan.job.json`"* — and cites
`MEBP/SupportClasses/LabLinkJob.py` as the sidecar builder. So the two repos were
already designed against each other; `ND3_NOT_YET_ACCEPTED` is correctly `False`.

The banner now states **what is sent and why the sidecar decides**, and escalates
to a warning only if that flag is flipped back for a hub older than commit
`7123061`. A standing warning about a working format trains the operator to
ignore the banner, and a test forbids re-adding one.

## Files

| File | Status | Purpose |
|---|---|---|
| `WIFI parsing/lablink_connect.py` | edit | 🔴 token/address purged |
| `SupportClasses/lablink/` | new | Vendored client: `protocol`/`fsutil`/`client`/`session_client` **byte-identical**, `__init__` ours |
| `SupportClasses/lablink/VENDORED.md` | new | Upstream commit + re-vendor procedure |
| `SupportClasses/LabLinkConfigStore.py` | new | Per-machine hub config + per-source job settings |
| `SupportClasses/LabLinkJob.py` | new | Upload naming + the `lablink.imagejob/1` sidecar |
| `SupportClasses/LabLinkService.py` | new | Session worker: build → open → upload → run → collect → close |
| `SupportClasses/LabLinkPublish.py` | new | Producer shim; never raises, never blocks |
| `SupportClasses/MicroscopeConfigStore.py` | edit | `filter_optics` — per-cube emission/excitation; `WAVELENGTH_BAND_NM`/`clean_wavelength` made public so the UI has no second copy of the band |
| `gui/pages/workflows/lablink_workflow.py` | new | The control surface: connection, discovery, per-source recipe + knobs, queue |
| `gui/widgets/lablink_bridge.py` | new | Worker-thread notification → GUI-thread Signal, payload-free |
| `gui/pages/workflows/workflow_picker.py` | edit | 🔗 **LabLink Processing** tile |
| `gui/pages/workflows_mode.py` | edit | Dispatch for the new tile |
| `gui/pages/hardware/microscope_setup_panel.py` | edit | Em/Ex columns on the filter-cube table |
| `gui/widgets/capture_controller.py` | edit | `publish_capture` beside both `captured.emit` (still + video) |
| `gui/pages/workflows/fluorescence_mosaic_workflow.py` | edit | `publish_fluorescence_well` in `_finish_run` |
| `gui/pages/calibration.py` | edit | `publish_plate_mosaic` in `_ploc_on_mosaic_finished` (plate branch) |
| `gui/app.py` | edit | `closeEvent` stops the service and reports what was unsent |
| `MEBP.spec` | edit | hiddenimports for `h5py`, ND3, the vendored client |
| `.gitignore` | edit | `config/hardware/lablink*.json` — **same commit as the store** |
| `tests/test_v717_lablink_vendor.py` | new | 11 — drift, purity, negative list |
| `tests/test_v717_lablink_job.py` | new | 34 — config, optics, naming, sidecar |
| `tests/test_v717_lablink_service.py` | new | 26 — sessions against a fake hub |
| `tests/test_v717_lablink_page.py` | new | 70 — page, knob tri-state, `applies_when`, bridge, wavelength UI |

## What was built

**Vendoring, pinned.** Four stdlib-only modules copied byte-identical from
upstream `7123061`, with a sha manifest, AST purity check, and an env-gated
byte diff **against the pinned commit via `git show`** — not against files on
disk, because the sibling clone sits on whatever branch someone left it on and
a file comparison would report drift that is really somebody's checkout state.
`__init__.py` is ours: upstream's re-exports `recipes` (recipe *authoring*),
which MEBP does not do. ⚠ **`runner.py` and `config.py` are never vendored** —
a subprocess executor and its command declarations have no place in an app that
drives a needle; a negative-list test asserts their absence.

**Naming.** `{prefix}_{hint}_{stamp}_{nonce}`. MEBP's store filenames are
deterministic and a re-scan overwrites them, and the hub answers **409** to
same-name-different-content — 4xx, therefore never retried — so a name derived
from them would upload the first scan of a well and permanently reject every
one after. The hint is `[A-Za-z0-9]`-only and **nothing parses it back**;
identity lives in the job record.

**The sidecar is derived by reading the written `.nd3` back**, never from the
values that produced it, so it cannot describe something other than the bytes
being sent. Refuses rather than guesses: no `um_per_px`; images at different
pixel sizes (one `pixel_size_um` silently picked would make every µm² figure
wrong while looking plausible); unknown sensor bit depth (**the array dtype is
the container depth — the invented value the sidecar exists to override — and
is never a fallback**).

**Wavelengths** live on `MicroscopeConfigStore.filter_optics`, keyed by cube
**name** rather than turret slot (the optics travel with the cube, and a scan
labels its channels by name), matched case-insensitively, refused rather than
clamped outside 200–1600 nm. Absent stays absent all the way to the wire: the
hub answers `missing_metadata` naming the fields, which is recoverable, whereas
a fabricated value is not — **measured on real data, supplying NA + emission
moved a segmented object count from 2855 to 2660 with no warning from any
layer.**

**The service** owns one session per job and closes it in a `finally` on every
exit path including an exception (a leaked session holds a warm worker slot the
hub cannot reuse). Retry is **iff `status == 0 or >= 500`** — a full hub answers
503 rather than 429 precisely so that rule stays correct. The socket timeout
sits **above** `longpoll_max_s`. Size is checked against `/hello`'s reported
`max_file_bytes` before a session is opened. `put_nowait` + a drop counter,
never a blocking put. Transfers are held while a print or scan is live.

## Bugs found and fixed en route

- **🐞 My own nonce was too small.** The first cut used 2 bytes; its own test
  measured **729 collisions in 10 000 same-second names** against a predicted
  763. A collision is a 409, i.e. exactly the failure a unique name exists to
  prevent. Now 4 bytes (0.012 expected), with the arithmetic recorded beside
  the constant and a bound calibrated to fail at 2 or 3 bytes.
- **🐞 Sidecar channels would have been alphabetical.**
  `ND3Reader.image_ids()` returns `sorted(...)`, but the sidecar's `channels`
  array must be in **acquisition order** because *"it is how channel indices
  resolve"*. A DAPI/FITC/Cy5 well would have been declared Cy5, DAPI, FITC and
  a recipe indexing by position would analyse the wrong channel and return a
  result that looks entirely normal. Now sorted by `channel_number` (the
  microscope's filter position), ids as the tie-break.
- **⚠ One of my own tests hung instead of failing.** The full-queue test timed
  an inline `submit()`, so the blocking-put mutation hung the whole suite
  rather than failing it — and a hanging test is one that gets disabled rather
  than fixed. Rewritten to submit on a worker with a join timeout; the mutation
  now fails in ~2 s with a diagnostic.

## The UI half

**The page configures and observes; the service owns the work**, so sending
continues when the operator navigates away — which is the normal case, because a
scan finishes and they move on. `LabLinkBridge` carries the worker's
notification to the GUI thread as a **payload-free** Signal; the page re-reads
`snapshot()`, which takes the lock and deep-copies, rather than being handed the
live `JobRecord` the worker is still mutating. The bridge unsubscribes on
destruction, since the service is a singleton that outlives every page.

**Discovery is surfaced, and nothing is cached to disk.** One button runs
`GET /hello` + `GET /workflows` **off the GUI thread** (a socket timeout above
`longpoll_max_s` means tens of seconds against an unreachable host) and renders
the hub's own recipes, knob bounds, size limit and long-poll window. A hub that
answers but has no `capabilities.sessions` is reported as a file exchange rather
than discovered from a confusing 404 one upload later, and unusable recipes are
shown with the hub's reason.

**Knobs render as their three real states.** Each knob row is a mode combo —
*Recipe default* (omit the key) / *Derive from file* (an explicit `null`) /
*Set to* (a pinned value) — plus a value widget bounded by the hub's published
`min`/`max`/`enum`/`max_len`. `applies_when` is evaluated against the sibling
knobs and **disables** the control, because a knob nothing will read is refused
rather than ignored; an inapplicable knob is never sent.

**⚠ Discovery deliberately does not choose a recipe.** The first cut
auto-selected the first one, which left the combo *displaying* an analysis while
the store held `""` — so a source read as armed with a recipe named and silently
sent nothing. It also decides which analysis runs on the data, and a
deconvolution is not a segmentation. There is now an explicit empty choice, what
the page shows always equals what is stored, and an armed source with no recipe
says so in the panel.

**Filter wavelengths** are two spin columns on the existing filter-cube table,
keyed by cube **name** (the optics travel with the cube), `0` rendered as `—`
for "not known", bounded by `WAVELENGTH_BAND_NM` read from the store — made
public precisely so the UI holds no second copy of the band. Out of band is
**refused, not rounded**, and the whole table is replaced on save so clearing a
row actually clears it.

## Testing

**145 tests green** (11 vendor + 34 job/config + 26 service + **73 page/bridge/
wavelength**). The fake hub exercises the production service end to end — build,
sidecar, open, upload, run, collect, close — with no socket; the page tests drive
`_on_probe_finished` with a document whose field names are taken from
`lablink/recipes.py::knob_wire`/`recipe_wire`, so passing means MEBP parses what
the hub actually publishes.

**17/17 mutations confirmed CAUGHT** across both halves, sources restored from a
scratchpad copy (never `git checkout` — the tree carries other sessions'
uncommitted work).

Backend (8): nonce 4→2 bytes · channel order left alphabetical · bit depth
guessed from the container · retry rule → every 4xx · `put_nowait` → blocking
put · `close()` dropped from the `finally` · `max_file_bytes` unenforced · knobs
not sent per command.

UI (9): auto-pick the first recipe · knob map read with `.get()` · `applies_when`
ignored · inapplicable knob sent anyway · `recipe_metadata_requirements` fails
open · bridge never unsubscribes · out-of-band wavelength clamped instead of
refused · optics keyed by slot instead of cube name · an unset wavelength stored
as a real zero.

⚠ **Two of my own tests were too weak and the mutations caught them first.**
`test_pinned_zero_survives_a_round_trip` pinned `channel`, whose stored form is
`[0]` — a **non-empty list, therefore truthy** — so the `.get()` mutation sailed
straight past the one test whose docstring claimed to catch it. Replaced with a
pinned scalar `0.0` and a pinned `False`, the two genuinely falsy cases. And the
wavelength refusal test patched `QMessageBox.warning` inside itself, so a
mutation that moved the refusal into a *different* test opened a real modal and
**the whole run hung instead of failing** — a hanging test gets disabled rather
than fixed, so the patch is now class-wide, with a `test_a_valid_commit_shows_no_warning`
guarding the guard against silently swallowing every refusal.

Regression green per-suite: nikon-ti 101 · nd3-container 53 · nd3-export 37 ·
capture-core 47 · capture-ui 22 · fluorescence-mosaic 35 · workflow-settings 35 ·
full-print 20 · stress-test 21 · plate-location-toggle 21 · context-panel 21 ·
illumination 31 · fluor-capture 17 · responsive-context 15 · fluor-shift 11 ·
suite-hygiene 10, plus a `gui.app` import smoke, an `MEBP.spec` parse, and an
offscreen `WorkflowsModePage` opening the new tile and returning to the picker.

**One pre-existing failure PROVED not ours:** `test_v75x_quick_print_workflow`
(1 failure + 1 error — `_speed_pct_spin`, retired by v7.6, and a `_print_btn`
gate). Reproduced identically in a **pristine `git worktree` at HEAD** with none
of this work present; neither test touches a file this change edits.

## Status

- [x] 🔴 Token/address purged
- [x] `MEBP.spec` hiddenimports
- [x] Vendored client + drift tests
- [x] `LabLinkConfigStore` + `.gitignore`
- [x] Sidecar builder + filter wavelengths
- [x] `LabLinkService` + `LabLinkPublish`
- [x] **Producer wiring** — `capture_controller.py` beside both
      `captured.emit` (still + video) · `fluorescence_mosaic_workflow._finish_run`
      (whole run, **not** `_on_channel_finished`, which would send N partial
      files) · `calibration._ploc_on_mosaic_finished` (plate branch only, after
      the store's synchronous save)
- [x] **`LabLinkWorkflowPage`** + tile + dispatch + `LabLinkBridge` +
      `MainWindow.closeEvent` teardown (`WorkflowsModePage` has no shutdown
      fan-out, so the service is stopped directly)
- [x] **Hardware Setup → Microscope** wavelength fields
- [x] `GET /workflows` discovery surfaced in the page (recipes, knob bounds,
      `requires_metadata`, unusable recipes)
- [ ] `POST /enroll` node identity (the field is stored and passed; nothing
      mints one yet — the site token alone works)
- [ ] v7.16 `.nd3` is still **untracked** (5 files, 90 tests green) — land it
      first; a `git add -A` would otherwise sweep it into this commit
- [ ] Bench verification below (no hardware or hub reached this session)

## Needs verification on ME3B V1, in order

1. `GET /hello` — `protocol`, `capabilities.sessions`, `longpoll_max_s`,
   `max_file_bytes`. Confirm Tailscale is **direct, not relayed** (relayed is
   1.5 MB/s, so 512 MB ≈ 6 minutes).
2. `GET /workflows` — the recipe and knob bounds MEBP will send against.
3. Choose a workflow + recipe per source and confirm the panel's own gate
   clears — an armed source with no recipe must say so, not sit silent.
4. **Push one still end to end** (`.nd3` + sidecar → run → collect) and confirm
   the `knobs` echo is recorded. If it comes back `unsupported_input`, the hub
   predates the `*.nd3` recipe change — check `inputs[].match`, **not** the
   manifest's top-level `match`, which is null on every recipe.
5. **Re-scan one well twice — both jobs must run.** The 409 kill; the first
   scan working proves nothing.
6. Oversize file → local refusal quoting the hub's own limit, before transfer.
7. Change one knob and re-run → milliseconds, `cached: true`.
8. Close MEBP with work in flight → honest report, sessions closed.
9. **No upload during a print**, and a print with jobs pending completes with
   no ZP disconnect.

## Issues & Decisions

- **Vendor rather than hand-write**, though QUICKSTART says *"No client library
  exists or is needed"*: the download-resume and hash-precedence logic is the
  security-relevant part, §8 records that each of its three client rules *"was
  a real bug here"*, and the shipped client already encodes the fixes.
- **Never `HardwareConfig`** — the CAMERA_CAL_PERSIST_STORE lesson, *and* it
  holds a bearer token while setup files are the ones meant to travel between
  rigs. `as_dict()` redacts by default because `logs/` is partly tracked.
- **Results are read-only artifacts.** Nothing consumes one automatically — no
  overlay, no `prefer_processed`, no detection or calibration input.
  `FluorescenceMosaicStore.attach_processed()` is deliberately NOT reused: v7.13
  owns it, `save_channel` drops the stale `_proc.png` on re-save,
  `_apply_post_to_captured` re-processes every channel from RAW (so one click
  would replace a deconvolution with a local denoise), and a counts CSV is not
  an image at all.
- **Per-artifact publishing only, never a directory watcher** — `output_dir`
  is arbitrary, so a watcher pointed at a data drive would upload a year of
  unrelated experiments over a link with no TLS.
- **Held artifacts are not pulled by default** — a multi-gigabyte label volume
  nobody opened should not cross a 1.5 MB/s link on its own.
- **Deliberately not done:** a durable on-disk byte queue (decision 3;
  half-implementing it is worse than not), transcoding video to `.nd3`, recipe
  authoring (`RECIPE-AUTHORING.md` is an offline act with a person in the
  loop), and the file-exchange/`SyncAgent` half.
