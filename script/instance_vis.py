import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Arrow
import numpy as np

def get_global_corners(x, y, theta, local_corners):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    global_corners = []
    for lx, ly in local_corners:
        gx = x + lx * cos_theta - ly * sin_theta
        gy = y + lx * sin_theta + ly * cos_theta
        global_corners.append((gx, gy))
    return global_corners

# Robot data
robot_x = 3.57752443625251
robot_y = 4.2889920287029435
robot_theta = 1.5241249511626105
l_front = 0.38
l_back = 0.12
w = 0.29
local_corners_robot = [
    (l_front, w/2),
    (l_front, -w/2),
    (-l_back, -w/2),
    (-l_back, w/2)
]

# Goal pose data
goal_x = 2.4541770761155917
goal_y = 3.2743857902003124
goal_theta = 3.05847332578454
arrow_length = 0.1

# Objects data
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
    {'name': 'd1', 'x': 0.5892226696014404, 'y': 3.4944525228271484, 'theta': 0.7799423336982727},
]

# Boundary data (4m x 5.2m rectangle)
boundary_corners = [
    (0, 0),
    (4, 0),
    (4, 5.2),
    (0, 5.2)
]

# Create figure and axis
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-0.2, 4.2)
ax.set_ylim(-0.2, 5.4)
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_title('Workspace Visualization with Boundary and Goal Pose')

# Plot boundary
boundary_polygon = Polygon(boundary_corners, closed=True, fill=False, edgecolor='k', linewidth=2, label='Boundary')
ax.add_patch(boundary_polygon)

# Plot robot
global_corners_robot = get_global_corners(robot_x, robot_y, robot_theta, local_corners_robot)
robot_polygon = Polygon(global_corners_robot, closed=True, fill=False, edgecolor='r', label='Robot')
ax.add_patch(robot_polygon)
ax.text(robot_x, robot_y, 'Robot', ha='center', va='center', color='r')

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

# Plot objects
s = 0.15
half_s = s / 2
local_corners_object = [
    (half_s, half_s),
    (half_s, -half_s),
    (-half_s, -half_s),
    (-half_s, half_s)
]

for obj in objects:
    x = obj['x']
    y = obj['y']
    theta = obj['theta']
    name = obj['name']
    global_corners = get_global_corners(x, y, theta, local_corners_object)
    object_polygon = Polygon(global_corners, closed=True, fill=False, edgecolor='b')
    ax.add_patch(object_polygon)
    ax.text(x, y, name, ha='center', va='center', color='b')

# Add legend
ax.legend()

plt.show()