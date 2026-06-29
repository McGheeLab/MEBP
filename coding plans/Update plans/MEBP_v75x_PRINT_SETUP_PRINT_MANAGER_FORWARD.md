# MEBP v7.5.x — Printing-mode Start fixed: forward `print_manager` from the wizard shell

## Objective

Fix a regression that blocked printing from **Printing mode** (Print Setup →
Monitor → **Start**). Pressing Start did nothing; the log filled with:

```
[E] gui.app: No print_manager on setup page
```

(once per Start press). The print never launched.

## Root Cause

The v7.5.0 Print Builder refactor made the Printing-mode setup page the
`WizardPrintSetupPage` shell (`gui/pages/print_setup/page.py`), which composes the
legacy `PrintSetupPage` internally as `self._legacy`. The `PrintManager` is
created and owned by the **legacy** page (`gui/pages/print_setup_legacy.py:104`,
`self.print_manager = PrintManager(controller)`).

`gui/app.py` reaches the manager as `setup_page.print_manager` — where
`setup_page = self._printing_mode.setup_page` is the wizard shell — from five
sites:

| Site | Purpose |
|------|---------|
| `_on_monitor_start` (app.py:1180) | Start the print (the visible failure). |
| `_wire_print_manager_to_monitor` (app.py:982/986) | Recorder + progress/state signal bridge to the Monitor. |
| `_on_monitor_pause` (app.py:1414) | Pause. |
| `_on_monitor_resume` (app.py:1420) | Resume. |
| `_on_monitor_abort` (app.py:1426) | Abort. |

The wizard shell **did not expose `print_manager`** (no property, no
`__getattr__` forward). So `hasattr(setup_page, "print_manager")` was `False`:
`_on_monitor_start` logged the error and returned, and Pause/Resume/Abort/wiring
silently no-op'd via their `hasattr` guards.

> Scope note: only **Printing mode** was affected. The **Quick Print** workflow
> builds and owns its own `PrintManager` and never goes through this path, so it
> kept working — it was the usable print path while this was broken.

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/print_setup/page.py` | New read-only `print_manager` property on `WizardPrintSetupPage` forwarding to `self._legacy.print_manager` (under "External contract (matches legacy PrintSetupPage)"). |
| `tests/test_v75x_print_setup_print_manager_forward.py` | New — regression guard. |

## Implementation Steps

- [x] Add `print_manager` forwarding property to `WizardPrintSetupPage`
      (`return getattr(self._legacy, "print_manager", None)`).
- [x] Regression test: wizard page exposes `print_manager`, it resolves to the
      legacy manager and is not None, AND the real access path
      `PrintingModePage.setup_page.print_manager` resolves.

## Testing Notes

- `python -m unittest tests.test_v75x_print_setup_print_manager_forward` → 4 passed.
- `python -m unittest tests.test_v75x_quick_print_workflow tests.test_v75x_print_setup_routine tests.test_hybrid_execution` → 22 passed (no regression).
- Headless smoke: `PrintSetupPage(...)` and `PrintingModePage(...).setup_page`
  both report `hasattr(..., "print_manager") == True` with a non-None manager.
- Manual bench (pending, ME3B V1): in Printing mode, build a job → Monitor →
  Start launches the print (no "No print_manager" error); Pause/Resume/Abort work.

## Issues & Decisions

- Chose a **read-only property forward** over copying the manager onto the wizard
  or adding a broad `__getattr__`: the legacy page owns the manager's lifecycle,
  the forward returns the live object (so `setup.print_manager.recorder = ...` at
  app.py:749 mutates the real manager), and it fixes all five call sites at once
  without widening the wizard's surface area.
- `getattr(..., None)` fallback keeps `hasattr` truthy and degrades to `None`
  (rather than raising) in the unlikely case the legacy page hasn't built a
  manager; in practice the legacy `__init__` always creates one.
