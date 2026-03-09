# MEBP Patching Best Practices — Lessons Learned

**Version:** 1.0 | **Updated:** March 2026
**Scope:** Rules and patterns for writing deployment scripts that modify the MEBP codebase

---
## 0. Ignore some files

Do not use any .bak file to understand this code.
If you find any .bak files ignore them.

## 1. Core Principle: Never Trust Assumed File State

The #1 cause of patch failures across v7.2.4, v7.2.5, and v7.2.6 was **assuming the file content matches what project knowledge shows**. Project knowledge contains snapshots from different points in time — the actual file on disk may have been modified by prior patches, manual edits, or failed partial applications.

**Rule:** Every deployment script must read the actual file at runtime and make decisions based on what it finds, not what project knowledge says should be there.

---

## 2. Failure Taxonomy — Every Way Patches Break

### 2.1 Exact String Anchor Mismatches

**What happens:** Patch uses `content.replace(old_string, new_string)` but `old_string` doesn't exist verbatim in the file.

**Common causes:**
- Whitespace differences (spaces vs tabs, trailing spaces, blank lines)
- Comment dashes: `# ── 7. Emit signals ──` vs `# ── 8. Emit signals ──` (numbering changed by prior patch)
- Method has extra code from a prior version's patch (e.g., v7.2.4 added `_ctx_validity_label` handling inside `_on_config_changed`, so v7.2.6's anchor for the simple version doesn't match)
- Prior `.bak` file version was used as reference instead of current file

**Prevention:**
- Use regex with `re.DOTALL` to match method signatures, not full method bodies
- Match on structural patterns: `def method_name(self` + first unique line, not entire blocks
- Never include comment separator lines in anchors (dash counts vary)
- Never include step numbers in anchors (they get renumbered)

### 2.2 Guard Logic Sees References, Not Definitions

**What happens:** Guard checks `if "method_name" not in content` to decide whether to add the method. But the method name appears in a `.connect(self.method_name)` call from an earlier step, so the guard skips insertion. The method definition is never added but a UI widget references it → AttributeError at runtime.

**The v7.2.6 example:** Step 1C replaced `_build_summary_section` with code containing `self._staging_add_current` in a button connect. Step 1D's guard `if "_staging_add_current" not in content` then saw the reference and skipped adding the actual method definitions.

**Prevention:**
- Always check for the **definition**, not just the name: `if "def _staging_add_current" not in content:`
- For attributes: check `"self._staging_print_names: list" not in content` (the type annotation makes it unique to the definition)
- Better yet: check for the method's docstring which is unique to the definition

### 2.3 String Concatenation Across Lines

**What happens:** Python interprets a `+` operator on a new line as unary positive, not string concatenation.

```python
# BROKEN — Python sees line 3 as a separate statement: unary +(content[...])
content = content[:match.end()] + \
    "some string\n"
    + content[match.end():]  # TypeError: bad operand type for unary +

# FIXED — use parentheses or a variable
inject = ("some string\n")
content = content[:match.end()] + inject + content[match.end():]
```

**Prevention:**
- Never rely on `\` line continuation for multi-part string concatenation
- Assign injected text to a named variable first, then do a single-line concatenation
- Or use parenthesized expressions: `content = (content[:x] + "text" + content[x:])`

### 2.4 Method Replacement Hits Wrong Match

**What happens:** `re.search(r'def method_name\(self\)', content)` finds the first occurrence, but there are multiple methods with similar names or the pattern appears in a comment/docstring.

**Prevention:**
- Use `find_method()` helper that matches `def name(self` at the correct indentation level (4 spaces for class methods)
- Include the full signature: `def method_name(self, arg1, arg2):`
- After finding the match, verify the next `def ` boundary to get the full method body
- Use `re.MULTILINE` with `^    def ` to anchor to line start at class indentation

### 2.5 Insertion Position Off-by-One

**What happens:** Code is inserted inside a string literal, inside a comment, or between a decorator and its function.

**Prevention:**
- After finding an insertion point, verify the character before and after
- Always insert at a `\n` boundary
- After insertion, verify the result with `ast.parse()` before writing

### 2.6 Widget References Without Widget Creation

**What happens:** Code references `self._wash_check.isChecked()` but that widget was never created in `_build_ui()`. This is a pre-existing bug in the codebase that only manifests when new code calls the broken path.

**The v7.2.6 example:** `_get_plan_preferences()` referenced `self._wash_check` which was planned for a UI section that was never implemented. Our `validate()` → `_generate_plan()` → `_get_plan_preferences()` hit it.

**Prevention:**
- Before calling any method, search project knowledge for its full implementation
- Use `getattr(self, '_widget_name', None)` for any widget that might not exist
- Wrap entire method calls in try/except when they touch UI state that may be incomplete
- For preferences/config methods, use safe defaults: `getattr(getattr(self, '_spin', None), 'value', lambda: 0.0)()`

### 2.7 File Already Partially Modified

**What happens:** A prior run crashed midway. The file has some changes applied but not others. Re-running the script applies changes in the wrong order or duplicates insertions.

**Prevention:**
- Every change must have an idempotency guard that checks for the RESULT, not the INPUT
- Check for a unique marker in the new code (e.g., `"v7.2.6: Staging list"`)
- Use `if "v7.2.6: marker text" not in content:` as the primary guard, not input-pattern matching
- Never append without checking if the content is already there

### 2.8 AST-Valid But Semantically Broken

**What happens:** `ast.parse()` passes (syntax is valid Python) but the code doesn't work because:
- A method was inserted at the module level instead of inside the class
- Indentation is wrong (method body at wrong level)
- An import was added but the imported name conflicts with a local variable

**Prevention:**
- After AST parse, also verify the function count matches expectations
- Check that inserted methods are inside the correct class by verifying indentation
- Log all inserted methods and their indentation level

### 2.9 Signal/Slot Feedback Loops

**What happens:** Adding a signal connection creates an infinite loop. For example, connecting `config_changed` → `set_hardware_config` → `_on_config_changed` → `config_changed.emit()`.

**Prevention:**
- Always use a re-entrancy guard (`self._restoring = True`) when doing bulk config restore
- Block signals on widgets during programmatic updates: `widget.blockSignals(True)`
- Skip the source page when propagating config changes
- Document the signal flow in comments

### 2.10 Import Conflicts

**What happens:** Adding an import like `from SupportClasses.PrintPlanOfAction import validate_well_setup` fails at runtime because the module has a circular import or hasn't been created yet.

**Prevention:**
- Use lazy imports inside method bodies: `try: from Module import Class except ImportError: return default`
- Never add top-level imports in deployment scripts — always inject lazy imports inside the methods that need them

---

## 3. Deployment Script Architecture

### 3.1 Required Structure

Every deployment script must follow this pattern:

```python
#!/usr/bin/env python3
"""One-line description."""

import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

def find_root() -> Path:
    """Find MEBP project root by looking for SupportClasses/ + gui/."""
    ...

def safe_read(path: Path) -> str:
    """Read file, return empty string if missing."""
    ...

def safe_write(path: Path, content: str, label: str) -> bool:
    """AST-verify → backup → write. Returns False on AST failure."""
    ...

def find_method(content: str, name: str):
    """Find method boundaries using regex. Returns match or None."""
    pattern = re.compile(
        rf'^(    def {re.escape(name)}\(self.*?\n)'
        rf'(.*?)'
        rf'(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)
```

### 3.2 Change Pattern

Every individual change must:

1. **Guard:** Check if already applied (look for unique marker in NEW code)
2. **Find:** Locate the target using regex or structural search
3. **Modify:** Apply the change
4. **Report:** Print what happened (applied / skipped / failed)

```python
# GOOD pattern
if "v7.2.6: descriptive marker" not in content:
    match = re.search(r'(def target_method\(self\))', content)
    if match:
        content = content[:match.start()] + new_code + content[match.end():]
        print("  ✓ Applied: description")
    else:
        print("  ✗ MISS: target_method not found")
else:
    print("  ○ SKIP: already applied")
```

### 3.3 AST Verification

**Always** AST-parse before writing:

```python
try:
    ast.parse(content)
except SyntaxError as e:
    print(f"AST FAIL — not writing: {e}")
    return False  # Never write broken code
```

### 3.4 Backup Strategy

DO not create backup files

## 4. What to Do Instead of String-Match Patches

### 4.1 Method-Level Replacement

Replace entire methods rather than splicing lines:

```python
m = find_method(content, "_get_plan_preferences")
if m:
    new_method = '''    def _get_plan_preferences(self):
        """Safe version with defaults."""
        return PlanPreferences(
            max_ink=getattr(getattr(self, '_spin', None), 'value', lambda: 50)(),
        )

'''
    content = content[:m.start()] + new_method + content[m.end():]
```

### 4.2 Safe Widget Access Pattern

When referencing UI widgets that may not exist:

```python
# BAD — crashes if widget doesn't exist
value = self._wash_check.isChecked()

# GOOD — safe default
value = getattr(getattr(self, '_wash_check', None), 'isChecked', lambda: True)()

# ALSO GOOD — explicit check
if hasattr(self, '_wash_check') and self._wash_check is not None:
    value = self._wash_check.isChecked()
else:
    value = True  # safe default
```

### 4.3 Safe Imports Inside Methods

```python
def validate(self):
    try:
        from SupportClasses.PrintPlanOfAction import validate_well_setup
        return validate_well_setup(...)
    except ImportError:
        return False, ["Module not available"]
    except Exception as e:
        return False, [f"Error: {e}"]
```

### 4.4 Injection Rather Than Replacement

When adding new functionality, inject new methods rather than modifying existing ones:

```python
# Instead of modifying _refresh_well_colors, add a wrapper:
if "def _refresh_well_colors_v726" not in content:
    # Add new method
    # Then change calls from _refresh_well_colors → _refresh_well_colors_v726
```

---

## 5. Pre-Deployment Checklist

Before delivering any deployment script:

- [ ] Every string concatenation uses parentheses or named variables (no `\` + newline + `+`)
- [ ] Every guard checks for the **definition** (`"def method_name"`) not just a name reference
- [ ] Every inserted method uses correct indentation (4 spaces for class methods)
- [ ] Every widget access uses `getattr` or `hasattr` safety
- [ ] Every import inside a method body is wrapped in `try/except ImportError`
- [ ] `ast.parse()` runs on the final content before writing
- [ ] Backup is created before any write
- [ ] Each change has a unique `v7.X.Y:` marker for idempotency
- [ ] The script handles the "already partially applied" case gracefully
- [ ] Method boundaries are found with regex, not exact string matching
- [ ] No anchors depend on comment dash counts, step numbers, or trailing whitespace

---

## 6. Testing Protocol

After applying any deployment:

1. **AST check:** `python3 -c "import ast; ast.parse(open('file.py').read())"`
2. **Import check:** `python3 -c "from gui.pages.module import ClassName"`
3. **Launch check:** `python main.py` — verify no crash on startup
4. **Smoke test:** Exercise the changed functionality manually
5. **Edge test:** Try the operation with missing data (no HW config, no prints, empty wells)

---

## 7. Project-Specific Gotchas

### 7.1 Multiple File Versions in Project Knowledge

Project knowledge contains both current files AND `.bak_*` versions. Always prefer the file WITHOUT `.bak` in the name. Search results may return stale backup content.

### 7.2 v7.2.5 Applied Features That Look Missing

The v7.2.5 patches added `_build_role_bar`, `_build_role_options`, `_active_role`, and `_role_options_stack` to `print_well_setup.py`. These may look like they need to be added, but they already exist. Always check before adding.

### 7.3 The `_on_config_changed` Guard

`hardware_setup.py._on_config_changed()` is called by dozens of widget signals. Any method that programmatically sets widget values during restore MUST either:
- Block signals on each widget: `widget.blockSignals(True)` / `False`
- Or set a `self._restoring = True` flag and check it at the top of `_on_config_changed`

### 7.4 Signal Emission Order

`config_changed.emit(self._config)` must happen AFTER `_rebuild_config()` completes. If emitted during restore, downstream pages receive a half-built config.

### 7.5 Camera Detection

`detect_cameras()` blocks the thread for up to 3 seconds per camera index. It must never be called on the main thread during startup. Use `detect_cameras_async()` with a QTimer callback.

### 7.6 Console Splitter

The QSplitter between page stack and console MUST have:
- `setChildrenCollapsible(False)` — prevents collapse to 0 height
- Minimum heights on both children — prevents resize crash
- The console's QTextEdit must also have a minimum height

### 7.7 PrintPlanOfAction Dependencies

`_generate_plan()` calls `_get_plan_preferences()` which reads from UI widgets. If those widgets were never created (partial UI implementation), it crashes. Always use `getattr` with defaults.