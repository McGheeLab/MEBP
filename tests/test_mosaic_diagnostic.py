#!/usr/bin/env python3
"""
Mosaic Stitching Diagnostic Tool

Creates a known reference image, slices it into tiles as a simulated raster
scan would, reconstructs it via MosaicBuilder, and compares the result to
the ground truth to identify stitching errors.

This isolates the stitching pipeline from all camera/stage/GUI variables.
"""

import sys
import os
import math
import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.MosaicBuilder import (
    MosaicBuilder, _compute_feather_weights, FrameRecord
)


# ═══════════════════════════════════════════════════════════════════
# Reference image generation
# ═══════════════════════════════════════════════════════════════════

def make_reference_image(width_um=20000, height_um=15000, um_per_px=3.34):
    """Create a reference image with circles (simulating wells) and a grid.

    Returns (image_bgr, um_per_px, (width_um, height_um))
    """
    w_px = int(width_um / um_per_px)
    h_px = int(height_um / um_per_px)
    img = np.full((h_px, w_px, 3), (10, 10, 10), dtype=np.uint8)

    # Draw a grid of circles (simulating wells)
    well_spacing_um = 4000  # 4mm spacing
    well_radius_um = 1500   # 1.5mm radius
    well_r_px = int(well_radius_um / um_per_px)
    spacing_px = int(well_spacing_um / um_per_px)

    # Offset from edges
    margin_px = spacing_px

    circles_drawn = 0
    for y_px in range(margin_px, h_px - margin_px // 2, spacing_px):
        for x_px in range(margin_px, w_px - margin_px // 2, spacing_px):
            # Well fill (reddish, like SimulatedCamera)
            cv2.circle(img, (x_px, y_px), well_r_px, (80, 80, 230), -1)
            # Well edge
            cv2.circle(img, (x_px, y_px), well_r_px, (40, 40, 180), 2)
            circles_drawn += 1

    # Draw crosshair markers at known positions for alignment verification
    for y_px in range(0, h_px, h_px // 4):
        cv2.line(img, (0, y_px), (w_px, y_px), (0, 80, 0), 1)
    for x_px in range(0, w_px, w_px // 4):
        cv2.line(img, (x_px, 0), (x_px, h_px), (0, 80, 0), 1)

    print(f"Reference image: {w_px}x{h_px} px, {circles_drawn} circles")
    return img, um_per_px, (width_um, height_um)


# ═══════════════════════════════════════════════════════════════════
# Tile extraction (simulates camera capture at each raster position)
# ═══════════════════════════════════════════════════════════════════

def extract_tile(reference, stage_x_um, stage_y_um, frame_w_px, frame_h_px,
                 um_per_px, origin_x_um=0, origin_y_um=0):
    """Extract a tile from the reference image at a given stage position.

    stage_x_um, stage_y_um: CENTER of the camera FOV in world coords.
    origin_x_um, origin_y_um: World coords of the reference image's top-left pixel.
    """
    fov_w_um = frame_w_px * um_per_px
    fov_h_um = frame_h_px * um_per_px

    # Top-left of FOV in world coords
    tl_x_um = stage_x_um - fov_w_um / 2.0
    tl_y_um = stage_y_um - fov_h_um / 2.0

    # Convert to pixel coordinates in the reference image
    px_x = int((tl_x_um - origin_x_um) / um_per_px)
    px_y = int((tl_y_um - origin_y_um) / um_per_px)

    ref_h, ref_w = reference.shape[:2]

    # Create output tile (black where outside reference)
    tile = np.zeros((frame_h_px, frame_w_px, 3), dtype=np.uint8)

    # Compute overlap between tile and reference
    src_x1 = max(0, px_x)
    src_y1 = max(0, px_y)
    src_x2 = min(ref_w, px_x + frame_w_px)
    src_y2 = min(ref_h, px_y + frame_h_px)

    dst_x1 = src_x1 - px_x
    dst_y1 = src_y1 - px_y
    dst_x2 = dst_x1 + (src_x2 - src_x1)
    dst_y2 = dst_y1 + (src_y2 - src_y1)

    if src_x2 > src_x1 and src_y2 > src_y1:
        tile[dst_y1:dst_y2, dst_x1:dst_x2] = reference[src_y1:src_y2, src_x1:src_x2]

    return tile


# ═══════════════════════════════════════════════════════════════════
# Diagnostic runner
# ═══════════════════════════════════════════════════════════════════

def run_diagnostic():
    """Full diagnostic: reference → tile → reconstruct → compare."""

    # Parameters matching typical scan config
    frame_w, frame_h = 916, 686
    um_per_px = 3.34
    overlap = 0.10
    width_um = 20000.0
    height_um = 15000.0

    print("=" * 70)
    print("MOSAIC STITCHING DIAGNOSTIC")
    print("=" * 70)

    # Step 1: Create reference image
    print("\n[1] Creating reference image...")
    reference, _, _ = make_reference_image(width_um, height_um, um_per_px)

    # Step 2: Generate raster positions
    print("\n[2] Generating raster grid...")
    builder = MosaicBuilder(
        frame_size_px=(frame_w, frame_h),
        micron_per_pixel=um_per_px,
        overlap=overlap,
        target_mosaic_px=2000,
    )

    bounds = (0.0, 0.0, width_um, height_um)
    positions = builder.generate_raster_positions(bounds, overlap=overlap)
    print(f"    Grid: {builder._grid_cols}x{builder._grid_rows} = {len(positions)} tiles")
    print(f"    FOV: {frame_w * um_per_px:.0f} x {frame_h * um_per_px:.0f} µm")
    print(f"    Canvas extent: {builder.canvas_extent_um}")
    print(f"    Canvas origin: {builder._canvas_origin_um}")

    # Step 3: Extract tiles and feed to builder (simulating scan)
    print("\n[3] Extracting tiles and stitching incrementally...")
    for i, (sx, sy) in enumerate(positions):
        tile = extract_tile(reference, sx, sy, frame_w, frame_h, um_per_px,
                            origin_x_um=0, origin_y_um=0)
        builder.add_raster_frame(tile, sx, sy, index=i)
        builder.stitch_incremental()

    incremental = builder.composite
    print(f"    Incremental composite: {incremental.shape}")

    # Step 4: Analyze the incremental composite
    print("\n[4] Analyzing incremental composite...")
    analyze_composite(reference, incremental, builder, "INCREMENTAL", um_per_px)

    # Step 5: Build with registration
    print("\n[5] Building with phase correlation registration...")
    registered = builder.build_mosaic()
    print(f"    Registered composite: {registered.shape}")
    analyze_composite(reference, registered, builder, "REGISTERED", um_per_px)

    # Step 6: Detailed tile placement analysis
    print("\n[6] Tile placement analysis...")
    analyze_tile_placement(builder, um_per_px)

    # Step 7: Feather weight analysis
    print("\n[7] Feather weight analysis...")
    analyze_feather_weights(builder)

    # Step 8: Save diagnostic images
    out_dir = os.path.join(os.path.dirname(__file__), "diagnostic_output")
    os.makedirs(out_dir, exist_ok=True)

    cv2.imwrite(os.path.join(out_dir, "01_reference.png"), reference)
    cv2.imwrite(os.path.join(out_dir, "02_incremental.png"), incremental)
    cv2.imwrite(os.path.join(out_dir, "03_registered.png"), registered)

    # Save difference image (amplified)
    ref_resized = resize_reference_to_composite(reference, builder)
    if ref_resized is not None:
        # Ensure same dimensions for absdiff
        rh, rw = ref_resized.shape[:2]
        ih, iw = incremental.shape[:2]
        if (rh, rw) != (ih, iw):
            ref_for_inc = cv2.resize(ref_resized, (iw, ih), interpolation=cv2.INTER_AREA)
        else:
            ref_for_inc = ref_resized
        diff = cv2.absdiff(ref_for_inc, incremental)
        diff_amplified = np.clip(diff.astype(np.float32) * 5, 0, 255).astype(np.uint8)
        cv2.imwrite(os.path.join(out_dir, "04_diff_incremental_5x.png"), diff_amplified)

        rrh, rrw = registered.shape[:2]
        if (rh, rw) != (rrh, rrw):
            ref_for_reg = cv2.resize(ref_resized, (rrw, rrh), interpolation=cv2.INTER_AREA)
        else:
            ref_for_reg = ref_resized
        diff_reg = cv2.absdiff(ref_for_reg, registered)
        diff_reg_amp = np.clip(diff_reg.astype(np.float32) * 5, 0, 255).astype(np.uint8)
        cv2.imwrite(os.path.join(out_dir, "05_diff_registered_5x.png"), diff_reg_amp)

    # Save a single tile for inspection
    if positions:
        sx, sy = positions[0]
        tile0 = extract_tile(reference, sx, sy, frame_w, frame_h, um_per_px)
        cv2.imwrite(os.path.join(out_dir, "06_tile_0.png"), tile0)

    # Save feather weight visualization
    if builder._feather_weights is not None:
        fw_vis = (builder._feather_weights * 255).astype(np.uint8)
        cv2.imwrite(os.path.join(out_dir, "07_feather_weights.png"), fw_vis)

    print(f"\n    Diagnostic images saved to: {out_dir}/")
    print("=" * 70)


def resize_reference_to_composite(reference, builder):
    """Resize the reference image to match the composite canvas dimensions."""
    if builder._display_cache is None:
        return None

    extent = builder.canvas_extent_um
    if extent is None:
        return None

    ch, cw = builder._display_cache.shape[:2]
    um_per_px = builder._um_per_px

    # The composite canvas covers canvas_extent in world space.
    # The reference image covers (0, 0) to (width_um, height_um).
    # We need to extract the region of the reference that corresponds to
    # the canvas extent, then resize it to canvas pixel dimensions.

    min_x_um, min_y_um, max_x_um, max_y_um = extent
    ref_h, ref_w = reference.shape[:2]

    # Convert canvas extent to reference pixel coords
    rx1 = int(min_x_um / um_per_px)
    ry1 = int(min_y_um / um_per_px)
    rx2 = int(max_x_um / um_per_px)
    ry2 = int(max_y_um / um_per_px)

    # Clip to reference bounds
    crop_x1 = max(0, rx1)
    crop_y1 = max(0, ry1)
    crop_x2 = min(ref_w, rx2)
    crop_y2 = min(ref_h, ry2)

    # Create padded region matching canvas extent
    region_w = rx2 - rx1
    region_h = ry2 - ry1
    padded = np.zeros((region_h, region_w, 3), dtype=np.uint8)

    # Copy the valid portion
    pad_x1 = crop_x1 - rx1
    pad_y1 = crop_y1 - ry1
    pad_x2 = pad_x1 + (crop_x2 - crop_x1)
    pad_y2 = pad_y1 + (crop_y2 - crop_y1)
    if crop_x2 > crop_x1 and crop_y2 > crop_y1:
        padded[pad_y1:pad_y2, pad_x1:pad_x2] = reference[crop_y1:crop_y2, crop_x1:crop_x2]

    # Resize to composite dimensions
    resized = cv2.resize(padded, (cw, ch), interpolation=cv2.INTER_AREA)
    return resized


def analyze_composite(reference, composite, builder, label, um_per_px):
    """Compare a composite against the reference."""
    ref_resized = resize_reference_to_composite(reference, builder)
    if ref_resized is None:
        print(f"    [{label}] Could not resize reference for comparison")
        return

    # Handle different shapes between incremental and registered
    ch_comp, cw_comp = composite.shape[:2]
    ch_ref, cw_ref = ref_resized.shape[:2]
    if (ch_comp, cw_comp) != (ch_ref, cw_ref):
        # Resize reference to match this composite
        ref_resized = cv2.resize(ref_resized, (cw_comp, ch_comp), interpolation=cv2.INTER_AREA)

    # Only compare where composite has data (non-black)
    comp_gray = cv2.cvtColor(composite, cv2.COLOR_BGR2GRAY)
    ref_gray = cv2.cvtColor(ref_resized, cv2.COLOR_BGR2GRAY)
    mask = comp_gray > 0

    if not np.any(mask):
        print(f"    [{label}] Composite is entirely black!")
        return

    # Pixel-wise error
    diff = np.abs(composite.astype(np.float64) - ref_resized.astype(np.float64))
    masked_diff = diff[mask]

    mae = np.mean(masked_diff)
    max_err = np.max(masked_diff)
    rmse = np.sqrt(np.mean(masked_diff ** 2))

    # Structural similarity on gray
    comp_masked = comp_gray.copy()
    ref_masked = ref_gray.copy()
    comp_masked[~mask] = 0
    ref_masked[~mask] = 0

    # Coverage
    total_px = composite.shape[0] * composite.shape[1]
    covered_px = np.count_nonzero(mask)

    print(f"    [{label}] Coverage:  {covered_px}/{total_px} ({100*covered_px/total_px:.1f}%)")
    print(f"    [{label}] MAE:       {mae:.2f} (lower=better, 0=perfect)")
    print(f"    [{label}] RMSE:      {rmse:.2f}")
    print(f"    [{label}] Max error: {max_err:.0f}")

    # Check for seam artifacts: high-error bands at tile boundaries
    # Look at error in overlap zones vs non-overlap zones
    weight_sum = builder._weight_sum
    if weight_sum is not None:
        wh, ww = weight_sum.shape
        if (wh, ww) == (ch_comp, cw_comp):
            overlap_mask = weight_sum > 1.01  # pixels with contributions from 2+ tiles
            single_mask = (weight_sum > 0) & (weight_sum <= 1.01)

            if np.any(overlap_mask):
                overlap_err = np.mean(diff[overlap_mask])
                n_overlap = np.count_nonzero(overlap_mask)
            else:
                overlap_err = 0
                n_overlap = 0

            if np.any(single_mask):
                single_err = np.mean(diff[single_mask])
                n_single = np.count_nonzero(single_mask)
            else:
                single_err = 0
                n_single = 0

            print(f"    [{label}] Overlap zone error: {overlap_err:.2f} ({n_overlap} px)")
            print(f"    [{label}] Single tile error:  {single_err:.2f} ({n_single} px)")
            if overlap_err > single_err * 2 and n_overlap > 0:
                print(f"    [{label}] *** SEAM ARTIFACTS DETECTED: overlap error >> single tile error ***")


def analyze_tile_placement(builder, um_per_px):
    """Check if tiles are being placed at the correct canvas positions."""
    if not builder._records:
        print("    No records")
        return

    fov_w_um = builder._frame_size_px[0] * um_per_px
    fov_h_um = builder._frame_size_px[1] * um_per_px
    scale = builder._mosaic_scale
    ox, oy = builder._canvas_origin_um

    tile_w = int(fov_w_um * scale)
    tile_h = int(fov_h_um * scale)

    print(f"    FOV: {fov_w_um:.0f} x {fov_h_um:.0f} µm")
    print(f"    Scale: {scale:.6f} px/µm")
    print(f"    Tile on canvas: {tile_w} x {tile_h} px")
    print(f"    Canvas origin: ({ox:.0f}, {oy:.0f}) µm")

    # Check first few tiles
    for i, rec in enumerate(builder._records[:4]):
        tl_x_um = rec.stage_x_um - fov_w_um / 2.0
        tl_y_um = rec.stage_y_um - fov_h_um / 2.0
        px = int((tl_x_um - ox) * scale)
        py = int((tl_y_um - oy) * scale)

        # What world position does this canvas pixel correspond to?
        back_x_um = px / scale + ox
        back_y_um = py / scale + oy

        err_x = abs(back_x_um - tl_x_um)
        err_y = abs(back_y_um - tl_y_um)

        print(f"    Tile {i}: stage=({rec.stage_x_um:.0f}, {rec.stage_y_um:.0f}) "
              f"→ canvas=({px}, {py}), "
              f"roundtrip err=({err_x:.1f}, {err_y:.1f}) µm")

    # Check tile stepping
    if len(builder._records) >= 2:
        r0 = builder._records[0]
        r1 = builder._records[1]
        actual_step_x = r1.stage_x_um - r0.stage_x_um
        actual_step_y = r1.stage_y_um - r0.stage_y_um
        expected_step_x = fov_w_um * (1 - builder._overlap)
        expected_step_y = fov_h_um * (1 - builder._overlap)
        print(f"    Step 0→1: actual=({actual_step_x:.0f}, {actual_step_y:.0f}) µm, "
              f"expected=({expected_step_x:.0f}, {expected_step_y:.0f}) µm")

        # In canvas pixels
        step_px = actual_step_x * scale
        print(f"    Step 0→1 in canvas px: {step_px:.1f} (tile_w={tile_w}, "
              f"ratio={step_px/tile_w:.4f}, expected={1-builder._overlap:.2f})")


def analyze_feather_weights(builder):
    """Check feather weight properties."""
    fw = builder._feather_weights
    if fw is None:
        print("    No feather weights")
        return

    print(f"    Shape: {fw.shape}")
    print(f"    Range: [{fw.min():.4f}, {fw.max():.4f}]")
    print(f"    Center value: {fw[fw.shape[0]//2, fw.shape[1]//2]:.4f}")
    print(f"    Corner value: {fw[0, 0]:.4f}")
    print(f"    Edge midpoints: top={fw[0, fw.shape[1]//2]:.4f}, "
          f"left={fw[fw.shape[0]//2, 0]:.4f}")

    # Check that the sum of overlapping feather weights ≈ 1
    # For 10% overlap, two adjacent tiles should sum to ~1 in the overlap zone
    overlap_px = int(fw.shape[1] * builder._overlap)
    if overlap_px > 0:
        right_edge = fw[:, -overlap_px:]  # right edge of tile A
        left_edge = fw[:, :overlap_px]    # left edge of tile B

        # At each horizontal position in the overlap, the sum should ≈ 1
        combined = right_edge + left_edge[:, ::-1]  # mirror the left edge
        print(f"    Overlap zone ({overlap_px} px):")
        print(f"      Weight sum range: [{combined.min():.4f}, {combined.max():.4f}]")
        print(f"      Weight sum at center: {combined[fw.shape[0]//2, overlap_px//2]:.4f}")

        if combined.min() < 0.8:
            print(f"      *** WARNING: Weight sum drops below 0.8 — causes dark seams! ***")


if __name__ == "__main__":
    run_diagnostic()
