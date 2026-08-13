# MEBP v7.17 — Step 2's direct setters, Max Z retired, and the estimate/clamp split

## Objective

Operator, on the needle-calibration wizard's reference-heights step:

> *"we want the 5 critical offsets to have a direct setter button — replace z,
> safe z, plate top, (approximate plate bottom derived from plate top). remove
> max z we dont need it for anything. the approximate plate bottom should not set
> the z bottom for clamping, we need to find the actual plate bottom later in
> step 3 of the wizard."*

Three independent changes. The third is the safety-relevant one.

## Files Modified

| File | Change |
|---|---|
| `gui/widgets/needle_bore_wizard.py` | Replace-Z setter + shared capture helper; step-2 readouts; Max Z dropped from the autofill list; `_floor_advisory` now distinguishes estimated from absent; text |
| `SupportClasses/StageController.py` | NEW `print_floor_datum_zref()` + `NON_CLAMPING_PLATE_BOTTOM_SOURCES`; `_apply_print_floor_raw` resolves through it |
| `gui/pages/calibration.py` | Max Z removed (setter, grid row, label, refs dict, `_ALL_Z_REF_KEYS`, `_Z_REFERENCE_FIELDS`, persistence, needle-cam estimate, plate-type autofill, learn loop); writer docstrings state what does and does not arm the clamp |
| `gui/widgets/xz_side_view.py` | Max badge + default key removed |
| `gui/widgets/standard_jog_context.py` | Hardware-Info Max Z row removed |
| `gui/dialogs/workflow_settings_dialog.py` | Max Z row removed from the locations panel |
| `gui/pages/jog_control.py`, 4 × `gui/pages/workflows/*.py` | `max_z` dropped from the z-reference default dicts |
| `gui/pages/print_setup_legacy.py` | stops forwarding `max_z` onto `PrintSettings` |

## 1 · A direct setter for every reference

Step 2 had setters for plate top, fast-move (safe) Z and the derived bottom, but
**Replace Z existed only in the Advanced Z references group** — so a
wizard-driven operator could not set it at all without leaving the flow. Added
`Set replace Z = current Z`, with a per-reference readout beside each button and
all four names in the step's state line.

Both new setters route through one `_capture_reference(host_setter, host_attr,
label)`. It verifies the host ATTRIBUTE afterwards rather than trusting the call
to have worked — the host setters return `None` either way, so a disconnected Z
board would otherwise read as success.

## 2 · Max Z retired

Audited before removing: **`max_z` had no behavioural consumer anywhere.** Every
use was a readout (XZ badge, Hardware-Info row, workflow-settings locations
panel, calibration labels) plus one `setattr` onto `PrintSettings` that nothing
ever read. Removing it also removes one more zero-ref height to re-teach after a
Z re-datum — `max_z` was among the three references that came back 24–49 mm
below the machine's hard bottom in the v7.9.1 stale-reference incident.

Removed from 11 files. A test walks `gui/` and `SupportClasses/` and fails on any
surviving `max_z` / `_max_z` reference, so a partial sweep cannot leave a dead
readout behind.

**Deliberately NOT removed:** `StageController.estimate_plate_z_refs()` still
returns a `plate_max_z` key, and plate types on disk keep any stored `max`
offset. Nothing reads either now. That estimator is a general-purpose helper with
its own tests, and the learn loop MERGES offsets rather than replacing them, so
retiring a reference must not destroy stored data — a legacy `max` offset is left
untouched instead of being deleted from the operator's plate files.

## 3 · 🔴 An ESTIMATED plate bottom never arms the clamp

The step-2 bottom is `plate top − datasheet offset`: a planning number, not a
measurement. Arming the print-floor clamp with it is unsafe **in both
directions**, which is what makes "just use the guess until we measure" wrong:

* **Estimated too HIGH** (the real glass is lower than the datasheet implies) —
  the clamp stops the needle above the glass and therefore **blocks the very
  touch-off that would measure the true bottom.** The operator cannot calibrate
  their way out, because the guess is what is stopping them. This is the
  operator's case.
* **Estimated too LOW** — the clamp passes a Z that punches through the glass:
  *false* protection, worse than none, because the caller believes it is
  guarded. (The same reasoning that made `set_print_floor_active` refcounted.)

Implemented as NEW `StageController.print_floor_datum_zref()` — the plate-bottom
scalar **only when it may serve as a floor** — with `_apply_print_floor_raw`
resolving its datum through it. One place, so a second caller cannot re-arm a
guess by accident; an AST test pins that the clamp calls it.

Three properties that make the rule hold:

* **`get_plate_bottom_z()` is unchanged.** Print heights, the step-3 survey
  clearance and every readout still get the best available guess — only the
  CLAMP insists on a measurement.
* **The tag survives `app.py`'s untagged re-push.** `set_plate_bottom_z` only
  overwrites `source` when non-None, and `_update_print_floor_datum` re-pushes
  untagged on every `calibration_data_changed`, so the estimate cannot be
  silently promoted. Pinned by test.
* **An absent/legacy `source` (None) still clamps.** Only an explicit
  `"estimated"` disarms, so no pre-v7.17 path loses its floor. Audited every
  production writer: contact touch-off → `"taught"`, step-4 optical →
  `"optical"`, restore → the saved source or `"taught"`, needle-cam guess and
  step 2 → `"estimated"`. Only the last two are excluded.

**Disclosed, not hidden:** an operator who does step 2 and stops now has no
armed floor where previously they had an (unreliable) one. `_floor_advisory()`
says so inline on step 3, and distinguishes the two cases — because an estimate
LOOKS taught (the readout shows a number) and is the case most likely to be
over-trusted.

## Testing Notes

NEW `tests/test_v717_step2_setters_and_estimate_floor.py` — **30 pass**.

**5/5 mutations CAUGHT**, sources restored byte-identical:

| Mutation | Result |
|---|---|
| Clamp reads `_plate_bottom_z_zref` directly again (the original bug) | 3 failures |
| Rule over-broad — an untagged/legacy bottom stops clamping | 1 failure |
| Step 2 applies its bottom as `"taught"` (arming a guess) | 1 failure |
| Advisory stops distinguishing estimate from absent | 1 failure |
| `"optical"` added to the non-clamping list (silent floor loss on a measurement) | 2 failures |

Regression **522 green** in one run across bore-wizard / step2-plate-bottom /
needle-loc-tab-layout / bore-dot-overlay / print-floor-refcount /
bore-gate-live-refresh / optical-needle-datum / cal-z-envelope /
xz-custom-z / plate-location-z-side-view / needle-offset-z-side-view /
jog-navigation / context-panel / workflow-settings-popout /
last-known-calibration / plate-types / plate-z-autocal-tab /
print-z-plate-bottom / plate-bottom-plane-controller / plate-bottom-wizard /
plate-bottom-worker / host-accessor-contract / suite-hygiene, plus a `gui.app`
import smoke.

**Offscreen walkthrough of the real `CalibrationPage`:** all four setters
present; Max Z absent from the label, `get_z_references()` and the XZ badge
list; the three direct setters capture; the applied bottom reads back as
`get_plate_bottom_z() = 1.0` with `source = "estimated"` while
`print_floor_datum_zref()` is **None**; and a descent **0.8 mm past the
estimate passes through unclamped** — the touch-off can reach the real glass.

### Existing suites updated (contracts genuinely changed)

* `test_v75x_plate_z_autocal_tab` — `_zoff_lbl_max_z` no longer exists.
* `test_v75x_plate_types` — `plate_max_z` is no longer an autofill target; the
  learn loop no longer WRITES a `max` offset (a legacy one on disk is asserted
  to survive, since the save merges).
* `test_v75x_cal_z_envelope_no_clobber` — the two `_zoff_set_max_z` tests are
  gone (removed feature); the invariant they guarded (no Z capture writes the
  device envelope) is unchanged and still covered. ⚠ **Its fake page had been
  raising AttributeError since v7.9.1** (`_zoff_read_stage_xy_um` /
  `_zoff_push_plate_bottom_to_controller` were added to the production setter
  and never here), so that test had been testing nothing — repaired, plus a new
  assertion that the contact touch-off tags itself `"taught"`.

### Disclosures

* **One pre-existing failure PROVED not ours:**
  `test_v75x_quick_print_pick_and_place::TestSetupStatus::test_all_set_is_clean`
  ("Stage motion not characterised"). Reproduced identically in a clean
  `git worktree` at HEAD.
* ⚠ **Process incident.** A `git stash push` used to prove that failure
  pre-existed swept up this tree's large body of unrelated uncommitted work, and
  the pop conflicted on 19 runtime artifacts (`settings.json`, `logs/*.log`,
  `last_calibration.json`, well-training PNGs). Source applied cleanly —
  conflicts are per-path — and the index was resolved by re-staging each file's
  on-disk content, restoring the `A` state they had at session start; verified by
  `git ls-files -u` empty, valid `settings.json`, `compileall`, and a `gui.app`
  import. **Do not use `git stash` in this repo** while it carries uncommitted
  work: use a `git worktree` at HEAD (which is how the failure was finally
  confirmed) or file copies. The mutation checks in this change used file copies
  for the same reason.

## Needs GUI/HW verification on ME3B V1, IN ORDER

1. Step 2 shows four setter buttons — plate top, fast-move, replace, apply
   bottom — each with its own readout, and **no Max Z anywhere** (also check the
   XZ strip badges and the Jog page's Hardware Info card).
2. Capture each of the three directly and confirm the number matches the live Z.
3. Apply the derived bottom: the reference reads with `(estimated)` and the
   step-3 status carries the ⚠ inactive-clamp advisory.
4. **The safety check, and the reason for the change:** with only the estimate
   applied, jog the needle DOWN past the estimated bottom — it must NOT be
   clamped short. (Watch the needle, not the screen, and start well above the
   glass.)
5. Run the step-4 touch-off. Afterwards the source reads `optical` (or `taught`
   for the contact fallback) and the advisory clears.
6. **Confirm the floor is now armed:** attempt to drive below the measured
   bottom and confirm it clamps, with the "would punch through the plate bottom"
   line in the log.
7. Restart and confirm the restored bottom still clamps (a saved measured value
   re-pushes as `taught`/its own source, not `estimated`).
8. A print still refuses / warns appropriately when the bottom is only estimated.
