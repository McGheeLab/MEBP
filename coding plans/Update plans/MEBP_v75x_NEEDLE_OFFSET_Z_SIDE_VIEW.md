# MEBP v7.5.x — Z side view on the Needle Offset reference-Z card

## Objective
`changes_needed.md` item 3: "on the needle offset calibration page on the reference
z heights card, I want to add a z side view next to it so I can move the z around
quickly."

## Approach
Reuse the existing `gui/widgets/xz_side_view.py::XZSideView` exactly as the Plate
Location tab does (`MEBP_v75x_PLATE_LOCATION_Z_SIDE_VIEW.md`), including its
`_on_ploc_go_to_z` handler — the green "Safe" badge retracts to Fast Move Z; the
other badges drive to each reference height.

## Files Modified
- `gui/pages/calibration.py`
  - `_build_z_offset_tab`: the "Reference Z heights" group now hosts a horizontal
    `QSplitter` — the capture-button grid (left ≈2) beside a new `self._zoff_xz_view`
    (right ≈1) + a hint. Wired: `set_z_display_sign(controller.z_up_sign())`,
    `set_safety_limits`, `set_needle(od)`, `set_z_references(get_z_references())`,
    `go_to_z_requested.connect(self._on_ploc_go_to_z)`.
  - `on_status_update`: pushes `set_zero_offset_x/z` + `set_position(zx_um, z_zr)` +
    `set_z_references(get_z_references())` to `_zoff_xz_view` each tick (mirrors the
    Plate Location feed), guarded with `getattr`.

## Implementation Steps
- [x] Restructure z_group into grid | XZ-view splitter + wire the view
- [x] Live status feed in `on_status_update`
- [x] Reuse `_on_ploc_go_to_z` for badge clicks
- [x] Tests `tests/test_v75x_needle_offset_z_side_view.py` (5)

## Testing
`tests/test_v75x_needle_offset_z_side_view.py` — view present after build; status
update feeds it without error; "Safe" badge → `move_z_absolute(z, from_zero_ref=True)`;
no-op when ZP disconnected; reference dict feed. Plate-Location Z-side-view +
autocal-tab suites stay green.

## Needs real-HW verification on ME3B V1
Badge clicks move Z to each reference; the live needle tracks while setting heights.
