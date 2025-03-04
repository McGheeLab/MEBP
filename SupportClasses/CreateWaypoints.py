import csv
import math

# Parameters for the spiral
num_points = 100     # Number of waypoints
a = 0.0              # Starting radius
b = 0.1              # Rate of increase of radius per radian
angle_step = 0.2     # Angle step in radians
time_step = 5      # Time increment per waypoint

with open("spiral.csv", "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    # Write header
    writer.writerow(["x", "y", "z", "p1", "p2", "p3", "t"])
    
    for i in range(num_points):
        theta = i * angle_step
        r = a + b * theta
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        z = 0.0      # Constant z; change if you want a 3D spiral
        p1 = 0       # Example value for p1 (change as needed)
        p2 = 0       # Example value for p2
        p3 = 0       # Example value for p3
        t = i * time_step
        writer.writerow([x, y, z, p1, p2, p3, t])

print("spiral.csv has been generated.")
