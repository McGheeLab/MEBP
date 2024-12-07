import math
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class ToolpathGenerator:
    def __init__(self,
                 threshold=128,
                 tool_diameter=300.0,
                 path_overlap=0.8,
                 pixels_per_micron=0.09,
                 base_feedrate=1000.0,
                 corner_slowdown_factor=0.5,
                 corner_angle_threshold=120.0,
                 flow_factor=0.01,
                 initial_z=0.0,
                 z_increment=10.0):
        self.threshold = threshold
        self.tool_diameter = tool_diameter
        self.path_overlap = path_overlap
        self.pixels_per_micron = pixels_per_micron
        self.base_feedrate = base_feedrate
        self.corner_slowdown_factor = corner_slowdown_factor
        self.corner_angle_threshold = corner_angle_threshold
        self.flow_factor = flow_factor
        self.z_pos = initial_z
        self.z_increment = z_increment

        self.main_images = []
        self.pump_images_layers = []
        self.pump_colors = []
        self.waypoints = []
        self.pump_states_all = []
        
    def load_layer_images(self, main_image_path, pump_image_paths):
        """Load a single layer of images: one main and multiple pump images."""
        main_img = Image.open(main_image_path).convert('L')
        main_pixels = np.array(main_img)
        if len(self.main_images) == 0:
            # First layer defines dimensions
            self.height, self.width = main_pixels.shape
        else:
            # Check dimensions
            if main_pixels.shape != (self.height, self.width):
                raise ValueError("All images must have the same dimensions.")
        
        pump_pixels_list = []
        for p_path in pump_image_paths:
            p_img = Image.open(p_path).convert('L')
            p_pixels = np.array(p_img)
            if p_pixels.shape != (self.height, self.width):
                raise ValueError("Pump image size must match the main image size.")
            pump_pixels_list.append(p_pixels)
        
        self.main_images.append(main_pixels)
        self.pump_images_layers.append(pump_pixels_list)
        
    def set_pump_colors(self, colors):
        """Set the pump colors as a list of (R,G,B) tuples, one per pump."""
        self.pump_colors = colors

    def _microns_per_pixel(self):
        return 1.0 / self.pixels_per_micron

    def _distance(self, p1, p2):
        return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

    def _angle_between_vectors(self, v1, v2):
        dot = v1[0]*v2[0] + v1[1]*v2[1]
        mag1 = math.sqrt(v1[0]**2+v1[1]**2)
        mag2 = math.sqrt(v2[0]**2+v2[1]**2)
        if mag1*mag2 == 0:
            return 0.0
        cos_angle = dot/(mag1*mag2)
        cos_angle = max(min(cos_angle,1),-1)
        angle = math.degrees(math.acos(cos_angle))
        return angle

    def _get_pump_states_at_pixel(self, pump_images, x_px, y_px):
        x_px = min(max(x_px, 0), self.width-1)
        y_px = min(max(y_px, 0), self.height-1)
        return [p_img[y_px, x_px] <= self.threshold for p_img in pump_images]

    def _get_pump_states_mic(self, pump_images, x_mic, y_mic):
        microns_per_pixel = self._microns_per_pixel()
        x_px = int(round(x_mic / microns_per_pixel))
        y_px = int(round(y_mic / microns_per_pixel))
        return self._get_pump_states_at_pixel(pump_images, x_px, y_px)

    def _generate_layer_toolpath(self, main_pixels, pump_images):
        """Generate toolpath for a single layer."""
        # Determine vertical spacing in pixels
        step_over = self.tool_diameter * self.path_overlap
        pixels_per_pass = step_over * self.pixels_per_micron
        pixels_per_pass = max(int(math.ceil(pixels_per_pass)), 1)
        microns_per_pixel = self._microns_per_pixel()

        path_points = []
        direction = 1
        y_px = 0

        while y_px < self.height:
            current_line = main_pixels[y_px, :]
            black_indices = np.where(current_line <= self.threshold)[0]
            if black_indices.size == 0:
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

            # Reverse and invert segments if right-to-left
            if direction == -1:
                segments = segments[::-1]
                segments = [(seg_end, seg_start) for (seg_start, seg_end) in segments]

            current_y_microns = y_px * microns_per_pixel

            # Process each segment
            for (seg_start_px, seg_end_px) in segments:
                if direction == 1:
                    px_range = range(seg_start_px, seg_end_px + 1)
                else:
                    px_range = range(seg_start_px, seg_end_px - 1, -1)

                px_list = list(px_range)

                prev_states = self._get_pump_states_at_pixel(pump_images, px_list[0], y_px)
                first_x = px_list[0] * microns_per_pixel
                first_y = current_y_microns
                if not path_points or path_points[-1] != (first_x, first_y):
                    path_points.append((first_x, first_y))

                # Check for state changes along the segment
                for idx in range(1, len(px_list)):
                    cur_px = px_list[idx]
                    current_states = self._get_pump_states_at_pixel(pump_images, cur_px, y_px)
                    if current_states != prev_states:
                        prev_px = px_list[idx-1]
                        prev_x = prev_px * microns_per_pixel
                        prev_y = current_y_microns
                        if not path_points or path_points[-1] != (prev_x, prev_y):
                            path_points.append((prev_x, prev_y))

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

        return path_points

    def _compute_waypoints(self, path_points, pump_images):
        """From a given set of path points, compute times, feedrates, pump displacement, and states."""
        if len(path_points) == 0:
            return [], []

        times = [0.0]
        pump_displacements = [0.0 for _ in pump_images]
        waypoints = []
        init_states = self._get_pump_states_mic(pump_images, path_points[0][0], path_points[0][1])
        waypoints.append((path_points[0][0], path_points[0][1], self.z_pos,
                          *pump_displacements, 0.0))
        pump_states = [init_states]

        for i in range(1, len(path_points)):
            p_prev = path_points[i-1]
            p_cur = path_points[i]
            dist = self._distance(p_prev, p_cur)

            # Compute feedrate
            if i < len(path_points)-1:
                p_next = path_points[i+1]
                v_in = (p_cur[0]-p_prev[0], p_cur[1]-p_prev[1])
                v_out = (p_next[0]-p_cur[0], p_next[1]-p_cur[1])
                corner_angle = self._angle_between_vectors(v_in, v_out)
                if corner_angle < self.corner_angle_threshold:
                    feedrate = self.base_feedrate * self.corner_slowdown_factor
                else:
                    feedrate = self.base_feedrate
            else:
                feedrate = self.base_feedrate

            dt = dist / feedrate
            t_new = times[-1] + dt
            times.append(t_new)

            current_states = self._get_pump_states_mic(pump_images, p_cur[0], p_cur[1])
            pump_states.append(current_states)

            # Increment pump displacement if on
            for j, on_state in enumerate(current_states):
                if on_state:
                    pump_displacements[j] += dist * self.flow_factor

            waypoints.append((p_cur[0], p_cur[1], self.z_pos,
                              *pump_displacements, t_new))

        return waypoints, pump_states

    def _reorder_path_to_closest(self, path_points, last_endpoint):
        """Reorder path_points so that it starts from the waypoint closest to last_endpoint."""
        if not path_points or last_endpoint is None:
            return path_points
        # Find closest point index
        distances = [self._distance(last_endpoint, p) for p in path_points]
        min_idx = np.argmin(distances)
        # Rotate the path_points so that path_points[min_idx] becomes the first
        reordered = path_points[min_idx:] + path_points[:min_idx]
        return reordered

    def generate_layers_toolpath(self, main_image_paths, pump_image_paths_list, pump_colors, z_increment=10.0):
        """Generate toolpaths for multiple layers, alternating direction each layer, 
        and incrementing Z after each layer."""
        self.pump_colors = pump_colors
        self.z_increment = z_increment

        # Clear previous data
        self.waypoints.clear()
        self.pump_states_all.clear()
        self.main_images.clear()
        self.pump_images_layers.clear()
        self.z_pos = 0.0

        last_endpoint = None
        for layer_idx, (m_path, p_paths) in enumerate(zip(main_image_paths, pump_image_paths_list)):
            # Load layer images for this layer
            self.main_images = []
            self.pump_images_layers = []
            self.load_layer_images(m_path, p_paths)

            # Generate the layer's raw path points
            path_points = self._generate_layer_toolpath(self.main_images[-1],
                                                        self.pump_images_layers[-1])

            # First handle the direction alternation
            # Even layers (0, 2, 4, ...) -> ascending order (no change)
            # Odd layers (1, 3, 5, ...) -> descending order (reverse path)
            if layer_idx % 2 == 1:
                path_points = path_points[::-1]

            # Now that the path is in the correct final direction,
            # reorder it if it's not the first layer to start from the closest point
            # to the last endpoint of the previous layer.
            if layer_idx > 0 and last_endpoint is not None and path_points:
                path_points = self._reorder_path_to_closest(path_points, last_endpoint)

            # Compute waypoints for this layer
            layer_waypoints, layer_pump_states = self._compute_waypoints(path_points, self.pump_images_layers[-1])

            # Merge layer waypoints into global list
            if layer_idx == 0:
                self.waypoints = layer_waypoints
                self.pump_states_all = layer_pump_states
            else:
                if self.waypoints and layer_waypoints:
                    # Adjust times so that this layer starts after previous layer ended
                    time_offset = self.waypoints[-1][-1]  # last waypoint's time
                    adjusted_waypoints = []
                    for wp in layer_waypoints:
                        new_wp = list(wp)
                        new_wp[-1] += time_offset
                        adjusted_waypoints.append(tuple(new_wp))

                    self.waypoints.extend(adjusted_waypoints)
                    self.pump_states_all.extend(layer_pump_states)
                else:
                    self.waypoints.extend(layer_waypoints)
                    self.pump_states_all.extend(layer_pump_states)

            # Update last endpoint and increment Z
            if self.waypoints:
                last_endpoint = (self.waypoints[-1][0], self.waypoints[-1][1])
            self.z_pos += self.z_increment

    def save_waypoints(self, output_file):
        with open(output_file, 'w') as f:
            # write the waypoint file
            for wp in self.waypoints:
                f.write(",".join(map(str, wp)) + "\n")

    def plot_toolpath_3d(self):
        """Plot the entire toolpath in 3D, coloring segments according to pump states."""
        if not self.waypoints:
            print("No waypoints to plot.")
            return

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Extract coordinates
        xs = [wp[0] for wp in self.waypoints]
        ys = [wp[1] for wp in self.waypoints]
        zs = [wp[2] for wp in self.waypoints]

        # Define function for segment color
        def get_segment_color(state):
            r = g = b = 0.0
            for s, c in zip(state, self.pump_colors):
                if s:
                    r += c[0]
                    g += c[1]
                    b += c[2]
            if r == 0 and g == 0 and b == 0:
                return (1.0, 1.0, 1.0) # white if no pumps
            r = min(r,1.0)
            g = min(g,1.0)
            b = min(b,1.0)
            return (r, g, b)

        # Plot segments with colors
        for i in range(len(self.waypoints)-1):
            seg_x = [xs[i], xs[i+1]]
            seg_y = [ys[i], ys[i+1]]
            seg_z = [zs[i], zs[i+1]]
            color = get_segment_color(self.pump_states_all[i])
            ax.plot(seg_x, seg_y, seg_z, color=color, linewidth=2)

        # Plot waypoints as black points
        ax.scatter(xs, ys, zs, c='black', s=20, depthshade=True)

        ax.set_title('Toolpath in 3D with Pump State Boundaries')
        ax.set_xlabel('X (microns)')
        ax.set_ylabel('Y (microns)')
        ax.set_zlabel('Z (microns)')
        plt.show()


if __name__ == "__main__":
    # make a toolpath generator
    generator = ToolpathGenerator()


    main_image_paths = ["ua_stack_rgb_black_10.png", "BME Logo path.png"]  # multiple layers
    pump_image_paths_list = [
     ["Color 1.png", "Color 2.png", "Color 3.png"],
     ["BME Logo 2-04.png", "BME Logo blank.png", "BME Logo blank.png"],
    ]   
    pump_colors = [(1,0,0), (0,1,0), (0,0,1)]  # RGB colors for pumps
    generator.generate_layers_toolpath(main_image_paths, pump_image_paths_list, pump_colors, z_increment=10.0)
    generator.save_waypoints("multi_layer_toolpath.csv")
    generator.plot_toolpath_3d()
