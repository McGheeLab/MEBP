import math
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

########################
# User-Defined Parameters
########################

input_image_path = "ua_stack_rgb_black_10.png"       # base image for toolpath
pump_image_paths = ["Color 1.png", "Color 2.png", "Color 3.png"]  # one image per pump

# Tool parameters
tool_diameter = 100.0   # microns
path_overlap = 0.8       # fraction of tool diameter to overlap between passes

# Image scale
pixels_per_micron = 0.2  # how many pixels per micron

# Feedrate parameters
base_feedrate = 1000.0   # microns/second for straight moves
corner_slowdown_factor = 0.5
corner_angle_threshold = 120.0 # degrees

# Raster parameters
threshold = 128          # black pixel threshold
step_over = tool_diameter * path_overlap

# Pump extrusion parameters
flow_factor = 0.01       # volume per micron traveled
z_pos = 0.0              # constant Z height for all waypoints

# Output waypoint file
output_file = "toolpath_waypoints.csv"

########################
# Load and Process Images
########################

img = Image.open(input_image_path).convert('L')
pixels = np.array(img)

pump_images = []
for p_path in pump_image_paths:
    p_img = Image.open(p_path).convert('L')
    p_pixels = np.array(p_img)
    if p_pixels.shape != pixels.shape:
        raise ValueError("Pump image size must match the base image size.")
    pump_images.append(p_pixels)

height, width = pixels.shape
microns_per_pixel = 1.0 / pixels_per_micron

# Determine vertical spacing in pixels
pixels_per_pass = step_over * pixels_per_micron
pixels_per_pass = max(int(math.ceil(pixels_per_pass)), 1)

def get_pump_states_at_pixel(x_px, y_px):
    # Clamp coordinates
    x_px = min(max(x_px, 0), width-1)
    y_px = min(max(y_px, 0), height-1)
    return [p_img[y_px, x_px] <= threshold for p_img in pump_images]

########################
# Generate Toolpath With Pump State Boundaries in Zigzag
########################

path_points = []
y_px = 0
direction = 1  # 1: left-to-right, -1: right-to-left

while y_px < height:
    current_line = pixels[y_px, :]
    black_indices = np.where(current_line <= threshold)[0]
    
    if black_indices.size == 0:
        # No black segments on this line, move down
        y_px += pixels_per_pass
        direction *= -1
        continue
    
    # Identify black segments left-to-right
    segments = []
    start_idx = black_indices[0]
    for i in range(1, len(black_indices)):
        if black_indices[i] > black_indices[i-1] + 1:
            segments.append((start_idx, black_indices[i-1]))
            start_idx = black_indices[i]
    segments.append((start_idx, black_indices[-1]))
    
    # If direction is right-to-left, reverse and invert segments
    if direction == -1:
        # Reverse the order of segments
        segments = segments[::-1]
        # Invert each segment to handle them right-to-left
        # That means (start, end) -> (end, start) because we'll traverse them backward
        segments = [(seg_end, seg_start) for (seg_start, seg_end) in segments]
    
    current_y_microns = y_px * microns_per_pixel
    
    # Process each segment according to direction
    for (seg_start_px, seg_end_px) in segments:
        if direction == 1:
            # left-to-right
            px_range = range(seg_start_px, seg_end_px + 1)
        else:
            # right-to-left
            px_range = range(seg_start_px, seg_end_px - 1, -1)
        
        px_list = list(px_range)
        
        # Initialize pump states at first pixel
        prev_states = get_pump_states_at_pixel(px_list[0], y_px)
        first_x = px_list[0] * microns_per_pixel
        first_y = current_y_microns
        
        if not path_points or (path_points[-1] != (first_x, first_y)):
            path_points.append((first_x, first_y))
        
        # Check for state changes
        for idx in range(1, len(px_list)):
            cur_px = px_list[idx]
            current_states = get_pump_states_at_pixel(cur_px, y_px)
            if current_states != prev_states:
                # State changed between px_list[idx-1] and px_list[idx]
                prev_px = px_list[idx-1]
                prev_x = prev_px * microns_per_pixel
                prev_y = current_y_microns
                if not path_points or path_points[-1] != (prev_x, prev_y):
                    path_points.append((prev_x, prev_y))
                
                # Add waypoint at current pixel
                cur_x = cur_px * microns_per_pixel
                cur_y = current_y_microns
                path_points.append((cur_x, cur_y))
                prev_states = current_states
        
        # Ensure segment end is a waypoint
        last_px = px_list[-1]
        last_x = last_px * microns_per_pixel
        last_y = current_y_microns
        if not path_points or path_points[-1] != (last_x, last_y):
            path_points.append((last_x, last_y))
    
    y_px += pixels_per_pass
    direction *= -1

########################
# Compute Times, Velocities, Pump Displacements
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
pump_displacements = [0.0, 0.0, 0.0]

def get_pump_states_mic(x_mic, y_mic):
    x_px = int(round(x_mic / microns_per_pixel))
    y_px = int(round(y_mic / microns_per_pixel))
    x_px = min(max(x_px, 0), width-1)
    y_px = min(max(y_px, 0), height-1)
    return [p_img[y_px, x_px] <= threshold for p_img in pump_images]

waypoints = []
init_states = get_pump_states_mic(path_points[0][0], path_points[0][1])
waypoints.append((path_points[0][0], path_points[0][1], z_pos,
                  pump_displacements[0], pump_displacements[1], pump_displacements[2], 0.0))
pump_states = [init_states]

for i in range(1, len(path_points)):
    p_prev = path_points[i-1]
    p_cur = path_points[i]
    dist = distance(p_prev, p_cur)
    
    # Compute feedrate
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
    
    current_states = get_pump_states_mic(p_cur[0], p_cur[1])
    pump_states.append(current_states)
    
    # Increment pump displacement if pump is on
    for j, on_state in enumerate(current_states):
        if on_state:
            pump_displacements[j] += dist * flow_factor
    
    waypoints.append((p_cur[0], p_cur[1], z_pos,
                      pump_displacements[0], pump_displacements[1], pump_displacements[2], t_new))

########################
# Save Waypoints to CSV
########################

with open(output_file, 'w') as f:
    f.write("X (microns),Y (microns),Z (microns),Pump0_disp,Pump1_disp,Pump2_disp,Time (s)\n")
    for wp in waypoints:
        f.write(f"{wp[0]},{wp[1]},{wp[2]},{wp[3]},{wp[4]},{wp[5]},{wp[6]}\n")

print("Toolpath waypoints with pump displacement and internal pump state waypoints (robust zigzag) saved to", output_file)

########################
# Visualization
########################

waypoints_px = np.array([[wp[0]/microns_per_pixel, wp[1]/microns_per_pixel] for wp in waypoints])

# Define pump colors (RGB): Pump0=Red, Pump1=Green, Pump2=Blue
def get_segment_color(state):
    r = 1.0 if state[0] else 0.0
    g = 1.0 if state[1] else 0.0
    b = 1.0 if state[2] else 0.0
    if r==0 and g==0 and b==0:
        return (1.0, 1.0, 1.0) # white if no pumps
    return (r, g, b)

plt.figure(figsize=(10, 8))
plt.imshow(pixels, cmap='gray', origin='upper')

# Plot segments with colors based on pump states at start of each segment
for i in range(len(waypoints)-1):
    start_pt = waypoints_px[i]
    end_pt = waypoints_px[i+1]
    color = get_segment_color(pump_states[i])
    plt.plot([start_pt[0], end_pt[0]], [start_pt[1], end_pt[1]], color=color, linewidth=2)

# Plot waypoints as filled black circles
plt.scatter(waypoints_px[:,0], waypoints_px[:,1], c='magenta', s=20, zorder=5)

plt.title('Robust Zigzag Toolpath with Pump State Boundaries')
plt.xlabel('X (pixels)')
plt.ylabel('Y (pixels)')
plt.show()
