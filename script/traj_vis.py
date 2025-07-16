#!/usr/bin/env python3
import sys
import re
import math
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
from matplotlib.widgets import Slider

def parse_waypoints(file_path):
    waypoints = []
    pat = re.compile(
        r"Waypoint\s+\d+:\s+\(x=([-\d\.eE]+),\s*y=([-\d\.eE]+),\s*yaw=([-\d\.eE]+),\s*ref_vel=([-\d\.eE]+),\s*time=([-\d\.eE]+)\)"
    )
    with open(file_path, "r") as f:
        for line in f:
            m = pat.search(line)
            if m:
                x, y, yaw, ref_vel, time = map(float, m.groups())
                waypoints.append(dict(x=x, y=y, yaw=yaw, ref_vel=ref_vel, time=time))
    return waypoints

def draw_frame(ax, waypoints, idx, xlim, ylim):
    ax.clear()
    ax.set_title(f"Robot Trajectory - Frame {idx}/{len(waypoints)-1}")
    ax.set_aspect('equal')
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    # Plot trajectory up to idx
    xs = [wp['x'] for wp in waypoints[:idx+1]]
    ys = [wp['y'] for wp in waypoints[:idx+1]]
    ax.plot(xs, ys, 'b-', linewidth=2, label="Trajectory")
    # Draw robot arrow at current pose
    wp = waypoints[idx]
    length = 0.25
    dx = length * math.cos(wp['yaw'])
    dy = length * math.sin(wp['yaw'])
    ax.arrow(wp['x'], wp['y'], dx, dy, head_width=0.12, head_length=0.12, fc='r', ec='k', linewidth=2)
    ax.scatter(wp['x'], wp['y'], c='r', s=50)
    # Info text
    text = f"x={wp['x']:.2f}, y={wp['y']:.2f}\nyaw={wp['yaw']:.2f}\nv={wp['ref_vel']:.2f}, t={wp['time']:.2f}"
    ax.text(0.05, 0.95, text, transform=ax.transAxes, fontsize=10, va='top', ha='left', bbox=dict(facecolor='w', alpha=0.5))
    ax.legend()

def main(filepath):
    waypoints = parse_waypoints(filepath)
    if not waypoints:
        print("No waypoints found.")
        sys.exit(1)

    xs = [wp['x'] for wp in waypoints]
    ys = [wp['y'] for wp in waypoints]
    margin = 0.5
    xlim = (min(xs)-margin, max(xs)+margin)
    ylim = (min(ys)-margin, max(ys)+margin)

    fig, ax = plt.subplots(figsize=(8, 8))
    plt.subplots_adjust(bottom=0.18)
    ax_slider = plt.axes([0.15, 0.05, 0.7, 0.03])
    slider = Slider(ax_slider, "Frame", 0, len(waypoints)-1, valinit=0, valstep=1)

    def update(val):
        idx = int(slider.val)
        draw_frame(ax, waypoints, idx, xlim, ylim)
        fig.canvas.draw_idle()

    slider.on_changed(update)
    draw_frame(ax, waypoints, 0, xlim, ylim)
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python visualize_waypoints.py path/to/waypoints.txt")
        sys.exit(1)
    main(sys.argv[1])
