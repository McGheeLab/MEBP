import math
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

########################
# User-Defined Parameters
########################

input_image_path = "ua_stack_rgb_black_10.png"  # Provide your input image path

# Tool parameters
tool_diameter = 100.0   # microns (example: 5 mm = 5000 microns)
path_overlap = 0.8       # fraction of tool diameter to overlap between passes

# Image scale
pixels_per_micron = 0.3  # how many pixels per micron
                          # Adjust according to your image scaling.

# Feedrate parameters
base_feedrate = 300.0   # microns per second for straight moves
corner_slowdown_factor = 0.5  # slow down at sharp corners
corner_angle_threshold = 89.0 # degrees

# Raster parameters
threshold = 128          # pixel intensity threshold: <= this is "black"
step_over = tool_diameter * path_overlap

# Output waypoint file
output_file = "toolpath_waypoints.csv"

########################
# Load and Process Image
########################

img = Image.open(input_image_path).convert('L')
pixels = np.array(img)

height, width = pixels.shape

microns_per_pixel = 1.0 / pixels_per_micron

# Determine how many pixels we move down each pass
pixels_per_pass = step_over * pixels_per_micron
pixels_per_pass = max(int(math.ceil(pixels_per_pass)), 1)

########################
# Generate Toolpath
########################

path_points = []

y_px = 0
direction = 1  # 1: left-to-right, -1: right-to-left
while y_px < height:
    current_line = pixels[y_px, :]
    
    black_indices = np.where(current_line <= threshold)[0]
    if black_indices.size == 0:
        # no black pixel in this row
        y_px += pixels_per_pass
        direction *= -1
        continue
    
    # Merge consecutive pixels into segments
    segments = []
    start_idx = black_indices[0]
    for i in range(1, len(black_indices)):
        if black_indices[i] > black_indices[i-1] + 1:
            segments.append((start_idx, black_indices[i-1]))
            start_idx = black_indices[i]
    segments.append((start_idx, black_indices[-1]))
    
    if direction == -1:
        segments = segments[::-1]
    
    current_y_microns = y_px * microns_per_pixel
    
    for seg in segments:
        seg_start_px, seg_end_px = seg
        if direction == -1:
            seg_start_px, seg_end_px = seg_end_px, seg_start_px
        
        seg_start_x_microns = seg_start_px * microns_per_pixel
        seg_end_x_microns = seg_end_px * microns_per_pixel
        
        # Move to start of segment
        path_points.append((seg_start_x_microns, current_y_microns))
        # Cut through segment
        path_points.append((seg_end_x_microns, current_y_microns))
    
    y_px += pixels_per_pass
    direction *= -1

########################
# Assign Times and Velocities
########################

def distance(p1, p2):
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

def angle_between_vectors(v1, v2):
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    mag1 = math.sqrt(v1[0]**2+v1[1]**2)
    mag2 = math.sqrt(v2[0]**2+v2[1]**2)
    if mag1*mag2 == 0:
        return 0.0
    cos_angle = dot/(mag1*mag2)
    cos_angle = max(min(cos_angle,1),-1)
    angle = math.degrees(math.acos(cos_angle))
    return angle

if len(path_points) == 0:
    print("No toolpath generated (no black pixels found).")
    exit()

times = [0.0]
waypoints = [(path_points[0][0], path_points[0][1], 0.0)]

for i in range(1, len(path_points)):
    p_prev = path_points[i-1]
    p_cur = path_points[i]
    dist = distance(p_prev, p_cur)
    
    if i < len(path_points)-1:
        p_next = path_points[i+1]
        v_in = (p_cur[0]-p_prev[0], p_cur[1]-p_prev[1])
        v_out = (p_next[0]-p_cur[0], p_next[1]-p_cur[1])
        corner_angle = angle_between_vectors(v_in, v_out)
        if corner_angle < corner_angle_threshold:
            feedrate = base_feedrate * corner_slowdown_factor
        else:
            feedrate = base_feedrate
    else:
        feedrate = base_feedrate
    
    dt = dist / feedrate
    t_new = times[-1] + dt
    times.append(t_new)
    waypoints.append((p_cur[0], p_cur[1], t_new))

########################
# Save Waypoints to CSV
########################

with open(output_file, 'w') as f:
    for wp in waypoints:
        f.write(f"{wp[0]},{wp[1]},0,0,0,0,{wp[2]}\n")

print("Toolpath waypoints saved to", output_file)

########################
# Visualization: Overlay Waypoints on Image
########################

# Convert waypoints from microns back to pixel coordinates
waypoints_px = []
for (x_mic, y_mic, t) in waypoints:
    x_px = x_mic / microns_per_pixel
    y_px = y_mic / microns_per_pixel
    waypoints_px.append((x_px, y_px))

waypoints_px = np.array(waypoints_px)

plt.figure(figsize=(10, 8))
plt.imshow(pixels, cmap='gray', origin='upper')
plt.plot(waypoints_px[:,0], waypoints_px[:,1], 'r-', linewidth=1)
plt.title('Toolpath Overlay (0,0 Start)')
plt.xlabel('X (pixels)')
plt.ylabel('Y (pixels)')
plt.show()
