import numpy as np

lidar_height = 1.0
actual_lateral_distance = 1.2
angle_min = -138*np.pi/180
angle_max =  138*np.pi/180
angle_increment = 0.25*np.pi/180
ranges = range(1104)
grid_height_min = 0.4
grid_height_max = 2.0

grid_height_step = (grid_height_max - grid_height_min) / 3  # 3 height levels
grid_heights = [
    grid_height_min + i * grid_height_step 
    for i in range(4)  # 4 boundaries for 3 zones
]

# LaserScan data: scan.angle_min, scan.angle_increment, scan.ranges
# Angles of each scan
angles = angle_min + np.arange(len(ranges)/2) * angle_increment
print(f"Angles: {angles*180/np.pi}")

# Hights of each laser ray at actual_lateral_distance
heights = lidar_height - actual_lateral_distance/np.tan(angles)
print(f"Höhen: {heights}")

# Indices for each zone
zone_indices = [[] for _ in range(len(grid_heights)-1)]

for i, h in enumerate(heights):
    for z in range(len(grid_heights)-1):
        if grid_heights[z] <= h < grid_heights[z+1]:
            zone_indices[z].append(i)
            break
for i in range(len(zone_indices)):
    print(f"Indices of zone: {i}")
    print(f"Length zone_indices all: {len(zone_indices)}")
    print(f"Length zone_indices individually: {len(zone_indices[i])}")
    print(f"Interval [{zone_indices[i][0]}; {zone_indices[i][-1]}]")