"""
page.py — WizardPrintSetupPage orchestrator (v7.5.0).

The new wizard-style entry point for Print Setup. Composes the legacy
``PrintSetupPage`` (from ``gui/pages/print_setup_legacy.py``)
internally and reparents its four tab widgets into wizard steps. The
legacy page continues to own its hardware-config forwarding, Finalize
helpers (_build_execution_config, _generate_print, _send_to_monitor),
PrintManager, and PrintQueue.

This composition keeps behavior parity with v7.4.2 while delivering
the wizard shell + side panel + validation panel.

External contract — drop-in compatible with the legacy class so
``printing_mode.py`` and ``app.py`` need no changes:
    set_hardware_config(config)
    set_xy_position_scale(value)
    get_context_widget() -> QWidget
    on_status_update()
    get_page_title() -> str
    get_page_subtitle() -> str

Forwarded signals:
    workspace_updated(WorkspaceConfig)
    navigate_to_page(int)
    job_ready(PrintJob)
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QHBoxLayout, QLabel, QPushButton, QSplitter, QStackedWidget,
    QVBoxLayout, QWidget,
)

from gui.scaling import s as _s, scaled_font_size as _sf
from gui.styles import COLORS



from .chrome import STEP_DESCRIPTIONS, build_step_frame
from .context_panel import PrintSetupContextPanel, WizardStepStrip
from .models import PrintObjectsModel, WellAssignmentModel
from .side_panel_objects import PrintObjectsSidePanel
from .step_objects import ObjectsStep
from .step_plan import PlanStep
from .step_wells import WellsStep
from .step_workspace import WorkspaceStep
from .stepper import WizardStepper
from .validation import ValidationIssue, ValidationSeverity, aggregate
from .validation_panel import ValidationPanel
from .wizard_step_base import WizardStepBase

logger = logging.getLogger(__name__)


STEP_TITLES = ["Print Objects", "Wells & Roles", "Plan & Run"]


class WizardPrintSetupPage(QWidget):
    """v7.5.0 wizard shell around the legacy print-setup tab widgets."""

    # Forwarded from the legacy page
    workspace_updated = Signal(object)
    navigate_to_page = Signal(int)
    job_ready = Signal(object)

    def __init__(self, controller, settings=None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings

        # ── Compose legacy page to harvest its tab widgets ────────
        from gui.pages.print_setup_legacy import PrintSetupPage as _LegacyPage
        self._legacy = _LegacyPage(controller, settings, parent=None)
        # We never add ``self._legacy`` to a layout — its tabs are
        # reparented below and the surrounding QTabWidget is left
        # dangling so the legacy helpers (print_manager,
        # _build_execution_config, _generate_print, _send_to_monitor …)
        # keep working. Explicitly hide so Qt never paints it as a
        # floating top-level window, and lock it out of the screen
        # backing-store as a belt-and-suspenders.
        self._legacy.hide()
        self._legacy.setAttribute(Qt.WA_DontShowOnScreen, True)

        # Forward signals from the legacy page
        self._legacy.workspace_updated.connect(self.workspace_updated.emit)
        self._legacy.navigate_to_page.connect(self.navigate_to_page.emit)
        self._legacy.job_ready.connect(self._on_legacy_job_ready)

        # ── Shared models ─────────────────────────────────────────
        self._objects_model = PrintObjectsModel(self)
        self._wells_model = WellAssignmentModel(self)
        # NOTE: in this transitional commit the models are passive — the
        # tab widgets remain the authoritative state. A follow-up will
        # bind them bidirectionally so the side panel + step bodies
        # share one source of truth.

        # ── Wizard steps (thin wrappers around legacy tab widgets) ─
        self._steps: list[WizardStepBase] = []
        self._build_steps()

        # ── Tabbed left context panel (Hardware / Print List /
        # Print Settings). The legacy page's get_context_widget()
        # returns the Display Options body — we reparent that into the
        # third tab so users keep the same controls.
        # Built BEFORE _wire_signals so the wiring can inject panels.
        legacy_settings_body = None
        try:
            legacy_settings_body = self._legacy.get_context_widget()
        except Exception as e:
            logger.warning(f"legacy get_context_widget failed: {e}")
        self._context_panel = PrintSetupContextPanel(
            self._objects_model,
            print_settings_body=legacy_settings_body,
            parent=self,
            step_count=len(STEP_TITLES),
            step_titles=STEP_TITLES,
        )
        self._context_panel.edit_hw_requested.connect(
            lambda: self.navigate_to_page.emit(0)
        )

        # v7.5.4: wizard step strip is now its OWN widget, exposed via
        # ``get_left_nav_widget()``. The PrintingModePage mounts it
        # inside its tabbar under the mode icons so all left-edge
        # nav buttons live in the same column.
        self._step_strip = WizardStepStrip(
            step_count=len(STEP_TITLES),
            step_titles=STEP_TITLES,
            parent=None,
        )

        # ── Layout ────────────────────────────────────────────────
        self._build_layout()
        self._wire_signals()

        # Default — first step active
        self._stack.setCurrentIndex(0)
        self._context_panel.set_active_step(0)
        self._update_nav_state(0)

    # ════════════════════════════════════════════════════════════════
    #  External contract (matches legacy PrintSetupPage)
    # ════════════════════════════════════════════════════════════════

    def get_page_title(self) -> str:
        return "Print Setup"

    def get_page_subtitle(self) -> str:
        return "Workspace → objects → wells → plan"

    @property
    def print_manager(self):
        """Forward to the composed legacy page's PrintManager.

        v7.5.x: the Printing-mode setup page is this wizard shell, but the
        ``PrintManager`` is created/owned by the legacy page (``self._legacy``).
        ``gui/app.py`` reaches ``setup_page.print_manager`` from five sites —
        ``_on_monitor_start`` (Start), ``_wire_print_manager_to_monitor``
        (recorder/bridge wiring), and the Pause/Resume/Abort handlers. Without
        this forward, ``hasattr(setup_page, "print_manager")`` was False, so
        Start logged "No print_manager on setup page" and silently returned —
        the print never ran. Read-only on purpose; the legacy page owns the
        object's lifecycle. Returns None if the legacy page hasn't built one.
        """
        return getattr(self._legacy, "print_manager", None)

    def set_xy_position_scale(self, value: float) -> None:
        if hasattr(self._legacy, "set_xy_position_scale"):
            self._legacy.set_xy_position_scale(value)

    def set_calibration_data(self, plate, well_positions, safe_z=None) -> None:
        """v7.5.x: forward the calibrated taught well positions to the composed
        legacy page so the print path drives to the taught wells (not a
        geometric stage-origin grid). Mirrors the Jog/Workflows wiring; the
        legacy page owns the resolver state."""
        if hasattr(self._legacy, "set_calibration_data"):
            self._legacy.set_calibration_data(plate, well_positions, safe_z)

    def set_hardware_config(self, config) -> None:
        if hasattr(self._legacy, "set_hardware_config"):
            self._legacy.set_hardware_config(config)
        for step in self._steps:
            step.on_hardware_config_changed(config)
        # Forward to the left context panel's Hardware tab too —
        # that's the user-facing summary now that Step 1's
        # HardwareSummaryWidget is hidden.
        if hasattr(self, "_context_panel"):
            self._context_panel.set_hardware_config(config)

    def on_status_update(self) -> None:
        # Forward to the legacy page for any tab-level polling, then
        # to the currently-active step (mirrors today's behavior).
        if hasattr(self._legacy, "on_status_update"):
            try:
                self._legacy.on_status_update()
            except Exception as e:
                logger.debug(f"legacy on_status_update raised: {e}")
        cur = self._stack.currentWidget()
        if cur is not None and hasattr(cur, "on_status_update"):
            try:
                cur.on_status_update()
            except Exception:
                pass

    def get_context_widget(self) -> QWidget:
        return self._context_panel

    def get_left_nav_widget(self) -> QWidget:
        """v7.5.4: the wizard's step navigation strip (1/2/3 + Back/
        Next) lives in the PrintingModePage tabbar under the mode
        icons. ModePage queries this method during ``add_sub_page``
        and mounts the returned widget into its sub-nav stack."""
        return self._step_strip

    # ════════════════════════════════════════════════════════════════
    #  Construction
    # ════════════════════════════════════════════════════════════════

    def _build_steps(self) -> None:
        # Harvest the four tab widgets from the legacy page and
        # reparent them into wizard steps.

        tab_workspace = getattr(self._legacy, "tab_workspace", None)
        tab_objects = getattr(self._legacy, "tab_objects", None)
        tab_wells = getattr(self._legacy, "tab_wells", None)
        tab_finalize = getattr(self._legacy, "tab_finalize", None)

        # Hide every tab widget *before* removeTab() detaches it.
        # ``QTabWidget.removeTab`` leaves the removed child parentless
        # while preserving its visibility — a previously-current tab
        # would briefly become a floating top-level window, which Qt
        # may flash on screen as a "weird embedded window".
        for w in (tab_workspace, tab_objects, tab_wells, tab_finalize):
            if w is not None:
                w.hide()

        # Detach from the legacy QTabWidget.
        if hasattr(self._legacy, "tabs"):
            tabs_widget = self._legacy.tabs
            for _ in range(tabs_widget.count()):
                tabs_widget.removeTab(0)

        # v7.5.0: the standalone Workspace step is gone — the hardware
        # summary has moved to the left context panel. Keep
        # WorkspaceTab alive as a hidden child of the wizard so its
        # ``workspace_changed`` signal continues to drive the
        # downstream legacy wiring (Print Objects + Well Setup
        # listen for plate-format updates).
        if tab_workspace is not None:
            tab_workspace.setParent(self)
            tab_workspace.hide()
        self._tab_workspace = tab_workspace  # retained reference

        # Three real wizard steps remain. Each body is wrapped in a
        # polished step frame (header + subtitle + content card) for
        # a consistent professional look across the wizard.
        self.step_objects = _LegacyHostStep(
            step_index=1, step_title="Print Objects",
            body=tab_objects, parent=self,
        )
        self.step_wells = _LegacyHostStep(
            step_index=2, step_title="Wells & Roles",
            body=tab_wells, parent=self,
        )
        # v7.6.0: Plan & Run mirrors Steps 1-2 — the entire Finalize
        # form (Print Parameters + Plan of Action + Calculated +
        # Validate/Generate/Send) moves to the Step 3 left-context
        # Tools. The body becomes a visual preview (projection trio).
        self._tab_finalize = tab_finalize
        from gui.widgets.projection_canvas import create_horizontal_well_preview
        try:
            self._plan_preview = create_horizontal_well_preview()
        except Exception as e:
            logger.warning(f"plan preview unavailable: {e}")
            self._plan_preview = QWidget()
        framed_plan_body = build_step_frame(
            *STEP_DESCRIPTIONS["Plan & Run"],
            body=self._plan_preview,
        )
        self.step_plan = PlanStep(body_widget=framed_plan_body, parent=self)
        self.step_plan.step_index = 3

        self._steps = [
            self.step_objects,
            self.step_wells,
            self.step_plan,
        ]

    def _build_layout(self) -> None:
        # Give the wizard a subtle mantle background so the step card
        # inside each step body reads as elevated against it.
        self.setObjectName("wizardPrintSetupPage")
        self.setAutoFillBackground(True)
        self.setStyleSheet(
            f"#wizardPrintSetupPage {{"
            f"  background: {COLORS['mantle']};"
            f"}}"
        )

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # v7.5.3: the compact top stepper is gone. Step navigation
        # now lives in the LEFT CONTEXT pane's vertical step strip —
        # see ``PrintSetupContextPanel`` — and the body fills the
        # remainder of the height. The orchestrator keeps
        # ``self._stepper`` as None so existing helpers that look it
        # up can short-circuit, and ``self._prev_btn`` /
        # ``self._next_btn`` are unused.
        self._stepper = None
        self._prev_btn = None
        self._next_btn = None

        # ── Middle row: stacked steps + side panel ────────────────
        # v7.6.0: body + right "This Print" pane live in a draggable
        # QSplitter so the user can resize (or drag-collapse) the
        # right pane. The side panel keeps its own collapse chevron.
        self._stack = QStackedWidget()
        for step in self._steps:
            self._stack.addWidget(step)
            step.show()

        self._side_panel = PrintObjectsSidePanel(self._objects_model)

        self._body_splitter = QSplitter(Qt.Horizontal, self)
        self._body_splitter.setObjectName("wizardBodySplitter")
        self._body_splitter.setChildrenCollapsible(True)
        self._body_splitter.setHandleWidth(_s(4))
        self._body_splitter.addWidget(self._stack)
        self._body_splitter.addWidget(self._side_panel)
        self._body_splitter.setStretchFactor(0, 1)  # body stretches
        self._body_splitter.setStretchFactor(1, 0)  # side pane fixed-ish
        outer.addWidget(self._body_splitter, 1)

        # ── Bottom validation panel (starts collapsed at height 0) ─
        self._validation_panel = ValidationPanel(self)
        outer.addWidget(self._validation_panel, 0, Qt.AlignBottom)

    # ── Nav button factory ────────────────────────────────────────

    def _make_nav_button(self, text: str, primary: bool) -> QPushButton:
        """Build a Prev/Next button styled to sit alongside the
        compact stepper chips (~30 px tall to fit the 40 px row)."""
        btn = QPushButton(text)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setFixedHeight(_s(28))
        if primary:
            btn.setStyleSheet(
                f"QPushButton {{"
                f"  background: {COLORS['mauve']};"
                f"  color: {COLORS['base']};"
                f"  border: 1px solid {COLORS['mauve']};"
                f"  border-radius: {_s(6)}px;"
                f"  padding: {_s(2)}px {_s(14)}px;"
                f"  font-weight: 700;"
                f"  font-size: {_sf(10)}pt;"
                f"}}"
                f"QPushButton:hover {{"
                f"  background: {COLORS['pink']};"
                f"  border-color: {COLORS['pink']};"
                f"}}"
                f"QPushButton:disabled {{"
                f"  background: {COLORS['surface0']};"
                f"  color: {COLORS['overlay0']};"
                f"  border-color: {COLORS['surface0']};"
                f"}}"
            )
        else:
            btn.setStyleSheet(
                f"QPushButton {{"
                f"  background: transparent;"
                f"  color: {COLORS['subtext0']};"
                f"  border: 1px solid {COLORS['surface1']};"
                f"  border-radius: {_s(6)}px;"
                f"  padding: {_s(2)}px {_s(12)}px;"
                f"  font-weight: 600;"
                f"  font-size: {_sf(10)}pt;"
                f"}}"
                f"QPushButton:hover {{"
                f"  background: {COLORS['surface0']};"
                f"  color: {COLORS['text']};"
                f"}}"
                f"QPushButton:disabled {{"
                f"  color: {COLORS['overlay0']};"
                f"  border-color: {COLORS['surface0']};"
                f"}}"
            )
        return btn

    def _wire_signals(self) -> None:
        # v7.5.4: step navigation lives in ``self._step_strip``
        # (mounted by PrintingModePage into its tabbar). The
        # orchestrator subscribes to its signals and forwards body /
        # nav-button state.
        self._step_strip.step_clicked.connect(self._jump_to_step)
        self._step_strip.prev_clicked.connect(self._go_prev)
        self._step_strip.next_clicked.connect(self._go_next)
        self._stack.currentChanged.connect(self._update_nav_state)
        self._update_nav_state(0)

        # Side panel hooks — these are intentionally light in this
        # transitional commit. Full bidirectional binding to
        # PrintObjectsTab is part of the next round.
        self._side_panel.add_requested.connect(self._on_side_add)
        self._side_panel.edit_requested.connect(self._on_side_edit)
        self._side_panel.delete_requested.connect(self._on_side_delete)

        self._validation_panel.jump_requested.connect(self._jump_to_step_target)

        # Mirror prints_changed from the legacy Objects tab into our
        # side-panel model so the browser reflects current objects.
        tab_objects = getattr(self.step_objects, "_body", None)
        if tab_objects is not None and hasattr(tab_objects, "prints_changed"):
            tab_objects.prints_changed.connect(self._refresh_objects_model)
        self._refresh_objects_model([])

        # v7.5.1: hand the PrintObjectsTab's detached objects_panel
        # (Print List + Objects in This Print + Summary) to the side
        # panel so it can be shown when Step 1 is active.
        if tab_objects is not None and hasattr(tab_objects, "objects_panel"):
            self._side_panel.set_step1_objects_panel(
                tab_objects.objects_panel)
        # v7.5.3: bind the Print List section inside objects_panel to
        # the shared PrintObjectsModel so it reflects session state.
        if (tab_objects is not None
                and hasattr(tab_objects, "set_print_list_model")):
            tab_objects.set_print_list_model(self._objects_model)

        # v7.5.3: inject Step 1 Tools panels (Designer + Ink + Auto-
        # Layout + CSV Import) into the LEFT context pane's tools
        # stack so the body can stay focused on the projection trio.
        if tab_objects is not None:
            # v7.5.6: CSV Import is no longer a standalone card — it's
            # a "custom object" type selectable at the top of the
            # Object Designer's type list.
            step1_tools = [
                getattr(tab_objects, "designer_panel", None),
                getattr(tab_objects, "ink_material_panel", None),
                getattr(tab_objects, "auto_layout_panel", None),
            ]
            step1_tools = [w for w in step1_tools if w is not None]
            if step1_tools:
                self._context_panel.set_step_tools(0, step1_tools)

        # v7.6.0: inject Step 2 (Wells & Roles) Tools panels — role
        # bar, role options, save/load/auto-assign — so the body keeps
        # only the plate + assignment summary.
        tab_wells = getattr(self.step_wells, "_body", None)
        if tab_wells is not None:
            step2_tools = [
                getattr(tab_wells, "role_bar_panel", None),
                getattr(tab_wells, "role_options_panel", None),
                getattr(tab_wells, "well_actions_panel", None),
            ]
            step2_tools = [w for w in step2_tools if w is not None]
            if step2_tools:
                self._context_panel.set_step_tools(1, step2_tools)

        # v7.6.0: inject Step 3 (Plan & Run) Tools — the entire
        # Finalize form (Print Params + Plan of Action + Calculated +
        # Validate/Generate/Send). The body is the projection preview.
        if self._tab_finalize is not None:
            self._context_panel.set_step_tools(2, [self._tab_finalize])

        # v7.6.0: Step 3 right pane = lightweight live "Running"
        # status mirror.
        from .run_status_panel import RunStatusPanel
        self._run_status_panel = RunStatusPanel()
        self._side_panel.set_step3_run_status_panel(self._run_status_panel)
        # Mirror job_ready into the run-status queue so the user sees
        # the queued print without leaving Plan & Run.
        self.job_ready.connect(self._on_job_ready_status)

        # Step-aware sidebar: switch when the active stack index
        # changes. (The left context panel + step strip are updated
        # by ``_update_nav_state``.)
        self._stack.currentChanged.connect(self._side_panel.set_active_step)
        self._side_panel.set_active_step(self._stack.currentIndex())

    # ════════════════════════════════════════════════════════════════
    #  Navigation
    # ════════════════════════════════════════════════════════════════

    def _jump_to_step(self, index: int) -> None:
        if not (0 <= index < len(self._steps)):
            return
        self._stack.setCurrentIndex(index)

    def _go_prev(self) -> None:
        self._jump_to_step(self._stack.currentIndex() - 1)

    def _go_next(self) -> None:
        self._jump_to_step(self._stack.currentIndex() + 1)

    def _update_nav_state(self, index: int) -> None:
        """v7.5.4: update both the wizard step strip (in the tabbar)
        and the context panel (Tools content) when the wizard's stack
        changes."""
        self._step_strip.set_prev_enabled(index > 0)
        is_last = (index >= len(self._steps) - 1)
        self._step_strip.set_next_enabled(not is_last)
        self._step_strip.set_active_step(index)
        self._context_panel.set_active_step(index)

    def _jump_to_step_target(self, step: int, target_id: str) -> None:
        # ``step`` is 1-based in ValidationIssue
        idx = max(0, step - 1)
        self._jump_to_step(idx)
        step_widget = self._steps[idx] if 0 <= idx < len(self._steps) else None
        if step_widget is not None:
            try:
                step_widget.focus_target(target_id)
            except Exception:
                pass

    # ════════════════════════════════════════════════════════════════
    #  Validation
    # ════════════════════════════════════════════════════════════════

    def run_validation(self) -> list[ValidationIssue]:
        """Run every step's validate() and surface the aggregate in the
        slide-up panel. Called by the Plan step's Validate button or
        from a future global toolbar hook."""
        per_step: list[list[ValidationIssue]] = []
        for step in self._steps:
            try:
                per_step.append(step.validate())
            except NotImplementedError:
                per_step.append([])
            except Exception as e:
                logger.warning(f"validate() raised on {step}: {e}")
                per_step.append([])
        issues = aggregate(per_step)
        self._validation_panel.set_issues(issues)
        self._validation_panel.show_animated()
        # Mark per-step chip state
        per_step_index = {i: [] for i in range(1, len(self._steps) + 1)}
        for issue in issues:
            per_step_index.setdefault(issue.step, []).append(issue)
        for i, step in enumerate(self._steps):
            step_issues = per_step_index.get(i + 1, [])
            if any(iss.is_error for iss in step_issues):
                self._step_strip.set_step_state(i, "error")
            elif i < self._stack.currentIndex():
                self._step_strip.set_step_state(i, "done")
        return issues

    # ════════════════════════════════════════════════════════════════
    #  Side panel hooks (transitional)
    # ════════════════════════════════════════════════════════════════

    def _refresh_objects_model(self, _names: list[str]) -> None:
        """Re-sync the side panel from the legacy Objects tab's
        in-memory ``_objects`` list."""
        tab_objects = getattr(self.step_objects, "_body", None)
        if tab_objects is None:
            return
        entries = list(getattr(tab_objects, "_objects", []))
        self._objects_model.clear()
        for entry in entries:
            self._objects_model.add_object(entry)

    def _on_side_add(self) -> None:
        # Jump to the Objects step and trigger its "add" affordance.
        self._jump_to_step(1)
        tab_objects = getattr(self.step_objects, "_body", None)
        if tab_objects is not None and hasattr(tab_objects, "_open_object_editor"):
            try:
                tab_objects._open_object_editor()
            except Exception as e:
                logger.debug(f"_open_object_editor unavailable: {e}")

    def _on_side_edit(self, name: str) -> None:
        self._jump_to_step(1)
        tab_objects = getattr(self.step_objects, "_body", None)
        if tab_objects is None:
            return
        for i, obj in enumerate(getattr(tab_objects, "_objects", [])):
            if obj.get("name") == name:
                if hasattr(tab_objects, "_open_object_editor"):
                    try:
                        tab_objects._open_object_editor(index=i)
                    except Exception:
                        pass
                break

    def _on_side_delete(self, name: str) -> None:
        tab_objects = getattr(self.step_objects, "_body", None)
        if tab_objects is None:
            return
        objs = getattr(tab_objects, "_objects", None)
        if objs is None:
            return
        for i, obj in enumerate(objs):
            if obj.get("name") == name:
                del objs[i]
                if hasattr(tab_objects, "_refresh_object_list"):
                    tab_objects._refresh_object_list()
                if hasattr(tab_objects, "_emit_prints_changed"):
                    tab_objects._emit_prints_changed()
                break

    # ════════════════════════════════════════════════════════════════
    #  Misc
    # ════════════════════════════════════════════════════════════════

    def _on_legacy_job_ready(self, job) -> None:
        # Mark Plan step as generated and forward the job out.
        self.step_plan.mark_generated(job)
        self.job_ready.emit(job)

    def _on_job_ready_status(self, job) -> None:
        """v7.6.0: push a queued-job summary into the Step 3 right-pane
        Run-status mirror so the user sees it without leaving the
        wizard."""
        panel = getattr(self, "_run_status_panel", None)
        if panel is None:
            return
        try:
            name = getattr(job, "name", None) or getattr(
                job, "print_name", "Print")
            n_wells = len(getattr(job, "well_assignments", []) or [])
            detail = f"{n_wells} well(s)" if n_wells else "queued"
            panel.add_job(str(name), detail)
            panel.set_state("RUNNING")
            panel.set_progress(job=str(name))
        except Exception as e:
            logger.debug(f"run-status push failed: {e}")


# ═══════════════════════════════════════════════════════════════════
# Helper — generic wizard step that hosts an existing widget
# ═══════════════════════════════════════════════════════════════════


class _LegacyHostStep(WizardStepBase):
    """Lightweight WizardStepBase that hosts an existing legacy tab
    widget inside the shared polished step frame (header card +
    content card)."""

    def __init__(self, step_index: int, step_title: str,
                 body: QWidget | None, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.step_index = step_index
        self.step_title = step_title
        self._body = body

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        if body is not None:
            title, subtitle = STEP_DESCRIPTIONS.get(
                step_title, (step_title, ""))
            frame = build_step_frame(title, subtitle, body, parent=self)
            lay.addWidget(frame)
            # The tab widgets were inside a QTabWidget which calls
            # setVisible(False) on non-current tabs; explicitly show
            # so reparenting into the wizard step doesn't leave them
            # hidden.
            body.show()

    # The legacy widgets keep their own signal wiring; we don't need
    # to surface state_changed here.

    def is_valid(self) -> bool:
        return True

    def validate(self) -> list[ValidationIssue]:
        return []

    def get_state(self) -> dict:
        return {}

    def set_state(self, state: dict) -> None:
        return None

    def on_hardware_config_changed(self, hw_config) -> None:
        # Forwarding is handled by the legacy page's
        # set_hardware_config(), which the orchestrator already calls
        # — no double-wiring needed.
        return None
