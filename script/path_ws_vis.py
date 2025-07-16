import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Arrow
import numpy as np
from matplotlib import cm

# Function to compute global corners from local corners, position, and orientation
def get_global_corners(x, y, theta, local_corners):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    global_corners = []
    for lx, ly in local_corners:
        gx = x + lx * cos_theta - ly * sin_theta
        gy = y + lx * sin_theta + ly * cos_theta
        global_corners.append((gx, gy))
    return global_corners

# Robot dimensions
l_front = 0.38
l_back = 0.12
w = 0.29
local_corners_robot = [
    (l_front, w/2),
    (l_front, -w/2),
    (-l_back, -w/2),
    (-l_back, w/2)
]

# Object dimensions (square with side length 0.15m)
s = 0.15
half_s = s / 2
local_corners_object = [
    (half_s, half_s),
    (half_s, -half_s),
    (-half_s, -half_s),
    (-half_s, half_s)
]

# Path points
path = [
[3.57752, 4.28899, 1.52412],
[3.56555, 4.0326, 1.52412],
[3.55357, 3.77622, 1.52412],
[3.53297, 4.03136, 1.77863],
[3.44879, 4.2731, 2.03313],
[3.30646, 4.48586, 2.28763],
[3.11516, 4.65593, 2.54213],
[2.8872, 4.77236, 2.79664],
[2.63726, 4.82765, 3.05114],
[2.38146, 4.81823, 3.30564],
[2.13627, 4.74472, 3.56014],
[1.91749, 4.61184, 3.81464],
[1.7392, 4.42816, 4.06915],
[1.61291, 4.20552, 4.32365],
[1.54673, 3.95824, 4.57815],
[1.54494, 3.70227, 4.83265],
[1.60766, 3.4541, 5.08715],
[1.70161, 3.21525, 5.08715],
[1.82478, 2.99086, 5.34166],
[2.00048, 2.80471, 5.59616],
[2.21739, 2.66878, 5.85066],
[2.46153, 2.59185, 6.10516],
[2.71717, 2.57886, 0.0764793],
[2.96785, 2.63065, 0.330982],
[3.19742, 2.74388, 0.585484],
[3.39108, 2.91127, 0.839986],
[3.50434, 3.06461, 1.02929],
[3.40131, 2.83028, 1.28379],
[3.36059, 2.57757, 1.5383],
[3.36951, 2.40708, 1.70778],
[3.30265, 2.65417, 1.96228],
[3.17574, 2.87647, 2.21679],
[2.99696, 3.05966, 2.47129],
[2.77781, 3.19193, 2.72579],
[2.53242, 3.26477, 2.98029],
[2.45418, 3.27438, 3.05847],
]

# Objects in the workspace
objects = [
    {'name': 'b9', 'x': 1.9649452797179674, 'y': 3.3151443159946345, 'theta': -0.08311932780525295},
    {'name': 'd2', 'x': 3.6065165996551514, 'y': 3.4514808654785156, 'theta': 0.7358155250549316},
    {'name': 'b3', 'x': 1.6911420683512188, 'y': 2.6256094907822454, 'theta': -0.0920688504985566},
    {'name': 'd8', 'x': 2.1348116397857666, 'y': 0.29120028018951416, 'theta': -0.009176576510071754},
    {'name': 'd5', 'x': 0.6259856820106506, 'y': 4.80679988861084, 'theta': -0.09028522670269012},
    {'name': 'b7', 'x': 0.26406906043677236, 'y': 2.8881048785158967, 'theta': 0.8094720847761377},
    {'name': 'd10', 'x': 3.6437253952026367, 'y': 1.717552661895752, 'theta': 0.7029879689216614},
    {'name': 'd4', 'x': 3.5595109462738037, 'y': 0.2788912355899811, 'theta': 0.7621512413024902},
    {'name': 'd6', 'x': 3.600428342819214, 'y': 4.779384136199951, 'theta': -0.04667137563228607},
    {'name': 'd1', 'x': 0.5892226696014404, 'y': 3.4944515228271484, 'theta': 0.7799423336982727},
]

# Boundary of the workspace
boundary_corners = [(0, 0), (4, 0), (4, 5.2), (0, 5.2)]

# Goal pose data
goal_x = 2.4541770761155917
goal_y = 3.2743857902003124
goal_theta = 3.05847332578454
arrow_length = 0.1

# Create figure and axis
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-0.2, 4.2)
ax.set_ylim(-0.2, 5.4)
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_title('Workspace with Robot Path and Goal Pose')

# Plot boundary
boundary_polygon = Polygon(boundary_corners, closed=True, fill=False, edgecolor='k', linewidth=2, label='Boundary')
ax.add_patch(boundary_polygon)

# Plot objects
for obj in objects:
    x = obj['x']
    y = obj['y']
    theta = obj['theta']
    name = obj['name']
    global_corners = get_global_corners(x, y, theta, local_corners_object)
    object_polygon = Polygon(global_corners, closed=True, fill=False, edgecolor='b')
    ax.add_patch(object_polygon)
    ax.text(x, y, name, ha='center', va='center', color='b')

# Plot path polygons with different colors
cmap = cm.get_cmap('viridis', len(path))
for i, p in enumerate(path):
    x, y, theta = p
    global_corners = get_global_corners(x, y, theta, local_corners_robot)
    color = cmap(i)
    polygon = Polygon(global_corners, closed=True, fill=False, edgecolor=color, alpha=0.7)
    ax.add_patch(polygon)

# Plot path line
path_x = [p[0] for p in path]
path_y = [p[1] for p in path]
ax.plot(path_x, path_y, 'k--', label='Path')

# Plot start and end markers
ax.plot(path_x[0], path_y[0], 'go', label='Start')
ax.plot(path_x[-1], path_y[-1], 'ro', label='End')

# Plot goal pose robot geometry
global_corners_goal = get_global_corners(goal_x, goal_y, goal_theta, local_corners_robot)
goal_polygon = Polygon(global_corners_goal, closed=True, fill=False, edgecolor='g', linestyle='--', label='Goal Robot')
ax.add_patch(goal_polygon)
ax.text(goal_x, goal_y, 'Goal', ha='center', va='center', color='g')

# Plot goal pose arrow
dx = arrow_length * np.cos(goal_theta)
dy = arrow_length * np.sin(goal_theta)
goal_arrow = Arrow(goal_x, goal_y, dx, dy, width=0.05, color='g', label='Goal Pose')
ax.add_patch(goal_arrow)

# Add legend
ax.legend()

# Show the plot
plt.show()